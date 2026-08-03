/* =============================================================================
 *  BLDC_Control.c — BLDC 六步换向控制
 *
 *  架构:
 *    位置环(PID) → 速度环(PID) → 电流环(PID) → PWM
 *    Hall 中断采集转速/位置, 控制任务周期消费
 *
 *  PID 使用位置式实现, 带积分限幅和退积分反馈抗饱和,
 *  防止限幅时积分继续累积造成超调。
 * ==========================================================================*/

/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"
#include <math.h>

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;
Hall_Info_t Hall_Info =
{
    .HallFirstEdge = 1
};

BLDCMosCom_t gComFwd[8] =
{
    [0] = {PHASE_NONE, PHASE_NONE},
    [1] = {PHASE_U, PHASE_W},
    [2] = {PHASE_V, PHASE_U},
    [3] = {PHASE_V, PHASE_W},
    [4] = {PHASE_W, PHASE_V},
    [5] = {PHASE_U, PHASE_V},
    [6] = {PHASE_W, PHASE_U},
    [7] = {PHASE_NONE, PHASE_NONE},
};
BLDCMosCom_t gComRev[8] =
{
    [0] = {PHASE_NONE, PHASE_NONE},
    [1] = {PHASE_W, PHASE_U},
    [2] = {PHASE_U, PHASE_V},
    [3] = {PHASE_W, PHASE_V},
    [4] = {PHASE_V, PHASE_W},
    [5] = {PHASE_V, PHASE_U},
    [6] = {PHASE_U, PHASE_W},
    [7] = {PHASE_NONE, PHASE_NONE},
};
BLDCMosCom_t *pHallTable = NULL;

/* local helpers -------------------------------------------------------------*/

/* 关闭全部功率管，避免上下桥直通 */
static void prvDisableAllMos( void )
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_RESET);
}

/* 浮点数钳位 */
static float prvClampf( float value, float min, float max )
{
    if ( value < min )
    {
        return min;
    }
    if ( value > max )
    {
        return max;
    }
    return value;
}

/* =============================================================================
 *  PID 实现 — 位置式 + 积分限幅 + 退积分反馈抗饱和
 *
 *  error = ref - fdb
 *  积分累积并限幅到 IntLimit
 *  输出 = Kp*error + SumError + Kd*(error - PreError)/Dt
 *  输出限幅到 OutLimit
 *  若限幅后输出与限幅前不一致，说明积分饱和，用 Kb 把差值退回去
 * ==========================================================================*/
float BLDC_PID_Calc( BLDC_PID_Pos_t *pPID, float ref, float fdb )
{
    float err = ref - fdb;
    float deriv = 0.0f;

    /* 微分项 */
    if ( pPID->Dt > 0.0f )
    {
        deriv = pPID->Kd * (err - pPID->PreError) / pPID->Dt;
    }

    /* 积分累积 + 积分限幅 */
    pPID->SumError += pPID->Ki * err * pPID->Dt;
    pPID->SumError = prvClampf(pPID->SumError, -pPID->IntLimit, pPID->IntLimit);

    /* 限幅前输出 */
    float u_unlim = pPID->Kp * err + pPID->SumError + deriv;

    /* 限幅后输出 */
    float u_sat = prvClampf(u_unlim, -pPID->OutLimit, pPID->OutLimit);

    /* 退积分反馈: 输出饱和时把多余积分退回去 */
    float diff = u_sat - u_unlim;
    if ( pPID->Kb > 0.0f )
    {
        pPID->SumError += pPID->Kb * diff * pPID->Dt;
    }

    pPID->PreError = err;
    pPID->PrevOut  = u_sat;
    pPID->Output   = u_sat;
    return u_sat;
}

/* 初始化速度环 PID */
static void prvBLDC_PIDSpeedInit( BLDC_PID_Pos_t *pPID )
{
    pPID->Kp = BLDC_SPEED_PID_KP;
    pPID->Ki = BLDC_SPEED_PID_KI;
    pPID->Kd = BLDC_SPEED_PID_KD;
    pPID->PreError = 0.0f;
    pPID->SumError = 0.0f;
    pPID->Output   = 0.0f;
    pPID->OutLimit = BLDC_MAX_CUR_TARGET_mA;
    pPID->IntLimit = BLDC_MAX_CUR_TARGET_mA * 0.5f;
    pPID->Kb       = 0.5f;
    pPID->PrevOut  = 0.0f;
    pPID->Dt       = 0.005f;    /* 5ms 控制周期 */
}

/* 初始化电流环 PID */
static void prvBLDC_PIDCurrentInit( BLDC_PID_Pos_t *pPID )
{
    pPID->Kp = BLDC_CURRENT_PID_KP;
    pPID->Ki = BLDC_CURRENT_PID_KI;
    pPID->Kd = BLDC_CURRENT_PID_KD;
    pPID->PreError = 0.0f;
    pPID->SumError = 0.0f;
    pPID->Output   = 0.0f;
    pPID->OutLimit = (float)BLDC_PWM_MAX_DUTY;
    pPID->IntLimit = (float)BLDC_PWM_MAX_DUTY * 0.5f;
    pPID->Kb       = 0.5f;
    pPID->PrevOut  = 0.0f;
    pPID->Dt       = 0.005f;    /* 5ms 控制周期 */
}

/* 初始化位置环 PID */
static void prvBLDC_PIDPositionInit( BLDC_PID_Pos_t *pPID )
{
    pPID->Kp = BLDC_POSITION_PID_KP;
    pPID->Ki = BLDC_POSITION_PID_KI;
    pPID->Kd = BLDC_POSITION_PID_KD;
    pPID->PreError = 0.0f;
    pPID->SumError = 0.0f;
    pPID->Output   = 0.0f;
    pPID->OutLimit = BLDC_MAX_CUR_TARGET_mA;
    pPID->IntLimit = BLDC_MAX_CUR_TARGET_mA * 0.5f;
    pPID->Kb       = 0.5f;
    pPID->PrevOut  = 0.0f;
    pPID->Dt       = 0.005f;    /* 5ms 控制周期 */
}

/* 复位单个 PID 的历史状态 */
static void prvBLDC_PIDReset( BLDC_PID_Pos_t *pPID )
{
    pPID->PreError = 0.0f;
    pPID->SumError = 0.0f;
    pPID->Output   = 0.0f;
    pPID->PrevOut  = 0.0f;
}

/* 切换控制模式时清空各环路积分和历史状态 */
static void prvBLDC_ResetAllLoopState( BLDC_Info_t *pBLDC )
{
    prvBLDC_PIDReset(&pBLDC->PIDPos_SpeedLoop);
    prvBLDC_PIDReset(&pBLDC->PID_CurrentLoop);
    prvBLDC_PIDReset(&pBLDC->PIDPos_PositionLoop);
    pBLDC->ExpectedRPM_Ramp = 0.0f;
}

/* 速度目标斜坡，限制加速度 */
static void prvBLDC_RampTargetRPM( BLDC_Info_t *pBLDC )
{
    if ( pBLDC->ExpectedRPM_Ramp < pBLDC->ExpectedRPM )
    {
        pBLDC->ExpectedRPM_Ramp += BLDC_RPM_RAMP_STEP;
        if ( pBLDC->ExpectedRPM_Ramp > pBLDC->ExpectedRPM )
        {
            pBLDC->ExpectedRPM_Ramp = pBLDC->ExpectedRPM;
        }
    }
    else if ( pBLDC->ExpectedRPM_Ramp > pBLDC->ExpectedRPM )
    {
        pBLDC->ExpectedRPM_Ramp -= BLDC_RPM_RAMP_STEP;
        if ( pBLDC->ExpectedRPM_Ramp < pBLDC->ExpectedRPM )
        {
            pBLDC->ExpectedRPM_Ramp = pBLDC->ExpectedRPM;
        }
    }
}

/* 取三相电流绝对值中的最大值，作为电流环和保护反馈 */
static float prvBLDC_GetCurrentMagnitude( const BLDC_Info_t *pBLDC )
{
    float iu = fabsf(pBLDC->CurrentPhase.U_PhaseCurrent);
    float iv = fabsf(pBLDC->CurrentPhase.V_PhaseCurrent);
    float iw = fabsf(pBLDC->CurrentPhase.W_PhaseCurrent);
    float peak = iu;

    if ( iv > peak )
    {
        peak = iv;
    }
    if ( iw > peak )
    {
        peak = iw;
    }

    return peak;
}

/* 更新当前 PWM 相的占空比 */
static void prvBLDC_UpdateActiveDuty( uint16_t duty )
{
    if ( BLDC_Info.ActivePwmPhase == PHASE_U )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty);
    }
    else if ( BLDC_Info.ActivePwmPhase == PHASE_V )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);
    }
    else if ( BLDC_Info.ActivePwmPhase == PHASE_W )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
    }
}

/* 三个数的中值 */
static uint32_t prvMedian3( uint32_t a, uint32_t b, uint32_t c )
{
    uint32_t temp;

    if ( a > b )
    {
        temp = a; a = b; b = temp;
    }
    if ( b > c )
    {
        temp = b; b = c; c = temp;
    }
    if ( a > b )
    {
        temp = a; a = b; b = temp;
    }

    return b;
}

/* Hall 周期滤波：3 点中值 + 一阶 IIR */
static uint32_t prvHallPeriodFilter_Update( Hall_Info_t *pHall, uint32_t rawValue )
{
    HallSpeedFilter_t *pFilter = &pHall->HallSpeedFilter;
    uint32_t median = rawValue;

    if ( rawValue == 0U )
    {
        return 0U;
    }

    pFilter->HallTickBuf[pFilter->Index] = rawValue;
    pFilter->Index = (pFilter->Index + 1U) % 3U;
    if ( pFilter->ValidCnt < 3U )
    {
        pFilter->ValidCnt++;
    }

    if ( pFilter->Inited == 0U )
    {
        pFilter->LastFilter = rawValue;
        pFilter->Inited = 1U;
        return rawValue;
    }

    if ( pFilter->ValidCnt >= 3U )
    {
        uint8_t i0 = pFilter->Index;
        uint8_t i1 = (pFilter->Index + 1U) % 3U;
        uint8_t i2 = (pFilter->Index + 2U) % 3U;
        median = prvMedian3(pFilter->HallTickBuf[i0],
                            pFilter->HallTickBuf[i1],
                            pFilter->HallTickBuf[i2]);
    }

    pFilter->LastFilter = pFilter->LastFilter
                        + ((int32_t)median - (int32_t)pFilter->LastFilter) / 8;
    return pFilter->LastFilter;
}

/* 判断 Hall 状态跳变方向：正转 +1，反转 -1 */
static int8_t prvBLDC_GetHallStepDelta( uint8_t previousHall, uint8_t currentHall )
{
    static const uint8_t hallForwardSeq[6] = {1U, 5U, 4U, 6U, 2U, 3U};
    int8_t previousIndex = -1;
    int8_t currentIndex = -1;
    uint8_t i;

    for ( i = 0U; i < 6U; i++ )
    {
        if ( hallForwardSeq[i] == previousHall )
        {
            previousIndex = (int8_t)i;
        }
        if ( hallForwardSeq[i] == currentHall )
        {
            currentIndex = (int8_t)i;
        }
    }

    if ( previousIndex < 0 || currentIndex < 0 )
    {
        return 0;
    }

    if ( hallForwardSeq[(previousIndex + 1) % 6] == currentHall )
    {
        return 1;
    }
    if ( hallForwardSeq[(previousIndex + 5) % 6] == currentHall )
    {
        return -1;
    }

    return 0;
}

/* 消费最新一次 Hall 沿：更新转速估算，再刷新当前换相 */
static void prvBLDC_HallCyclic( void )
{
    uint8_t hall;
    uint32_t rawDelta;
    uint32_t filtDelta;

    if ( Hall_Info.HallEdgeFlag == 0U )
    {
        return;
    }

    Hall_Info.HallEdgeFlag = 0U;
    hall = Hall_Info.HallStateShadow;
    BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

    rawDelta = Hall_Info.HallTickCnt;
    if ( rawDelta > 0U )
    {
        filtDelta = prvHallPeriodFilter_Update(&Hall_Info, rawDelta);
        if ( filtDelta > 0U )
        {
            /* 每个 Hall 沿间隔为 60 电角度，折算机械转速 */
            BLDC_Info.RPM = 60.0f * (float)BLDC_HALL_TIMER_HZ
                          / ((float)filtDelta * 6.0f * (float)BLDC_POLE_PAIRS);
        }
    }

    if ( hall >= 1U && hall <= 6U &&
         pHallTable[hall].PwmPhase != PHASE_NONE &&
         pHallTable[hall].LowPhase != PHASE_NONE )
    {
        BLDC_ChangeMOSstate(pHallTable[hall].PwmPhase,
                            pHallTable[hall].LowPhase,
                            BLDC_Info.Pulse);
    }
}

/* public functions ----------------------------------------------------------*/

/* 初始化全部控制环 PID */
void BLDC_PIDInit( BLDC_Info_t *pBLDC )
{
    prvBLDC_PIDSpeedInit(&pBLDC->PIDPos_SpeedLoop);
    prvBLDC_PIDCurrentInit(&pBLDC->PID_CurrentLoop);
    prvBLDC_PIDPositionInit(&pBLDC->PIDPos_PositionLoop);
}

/* 复位控制状态 */
void BLDC_ResetControlState( BLDC_Info_t *pBLDC )
{
    pBLDC->RPM = 0.0f;
    pBLDC->CurrentMagnitude = 0.0f;
    pBLDC->ExpectedCurrent = 0.0f;
    pBLDC->Pulse = 0U;
    pBLDC->MotorStalling = 0U;
    pBLDC->MotorRunning = 0U;
    pBLDC->ExpectedRPM_Ramp = 0.0f;
    prvBLDC_PIDReset(&pBLDC->PIDPos_SpeedLoop);
    prvBLDC_PIDReset(&pBLDC->PID_CurrentLoop);
    prvBLDC_PIDReset(&pBLDC->PIDPos_PositionLoop);
    pBLDC->ActivePwmPhase = PHASE_NONE;
    pBLDC->ActiveLowPhase = PHASE_NONE;
}

/* 复位位置基准 */
void BLDC_PositionReset( BLDC_Info_t *pBLDC )
{
    pBLDC->CurrentAngleDeg = 0.0f;
    pBLDC->ExpectedAngleDeg = 0.0f;
    pBLDC->HallStepCount = 0;
    pBLDC->CtrlMode = BLDC_CTRL_SPEED;
    pBLDC->Direction = MOTOR_FWD;
    pBLDC->PositionCmdActive = 0U;
}

/* 设置速度目标 */
void BLDC_SetExpectedRPM( float expectedRPM )
{
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_SPEED )
    {
        prvBLDC_ResetAllLoopState(&BLDC_Info);
    }
    BLDC_Info.ExpectedRPM = prvClampf(expectedRPM, 0.0f, BLDC_MAX_RPM_TARGET);
    BLDC_Info.CtrlMode = BLDC_CTRL_SPEED;
    BLDC_Info.PositionCmdActive = 0U;
}

float BLDC_GetExpectedRPM( void )
{
    return BLDC_Info.ExpectedRPM;
}

/* 设置电流目标 */
void BLDC_SetExpectedCurrent( float expectedCurrent )
{
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_CURRENT )
    {
        prvBLDC_ResetAllLoopState(&BLDC_Info);
    }
    BLDC_Info.ExpectedCurrent = prvClampf(expectedCurrent, 0.0f, BLDC_MAX_CUR_TARGET_mA);
    BLDC_Info.CtrlMode = BLDC_CTRL_CURRENT;
    BLDC_Info.PositionCmdActive = 0U;
}

float BLDC_GetExpectedCurrent( void )
{
    return BLDC_Info.ExpectedCurrent;
}

/* 设置绝对位置目标 */
void BLDC_SetExpectedAngle( float expectedAngleDeg )
{
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION )
    {
        prvBLDC_ResetAllLoopState(&BLDC_Info);
    }
    BLDC_Info.ExpectedAngleDeg = expectedAngleDeg;
    BLDC_Info.CtrlMode = BLDC_CTRL_POSITION;
    BLDC_Info.PositionCmdActive = 1U;
}

/* 增量修改位置目标 */
void BLDC_AddExpectedAngle( float deltaAngleDeg )
{
    float baseAngle = BLDC_Info.ExpectedAngleDeg;

    /* 位置环未激活时，以当前反馈角度作为增量起点 */
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION || BLDC_Info.PositionCmdActive == 0U )
    {
        baseAngle = BLDC_Info.CurrentAngleDeg;
    }

    BLDC_SetExpectedAngle(baseAngle + deltaAngleDeg);
}

float BLDC_GetExpectedAngle( void )
{
    return BLDC_Info.ExpectedAngleDeg;
}

float BLDC_GetCurrentAngle( void )
{
    return BLDC_Info.CurrentAngleDeg;
}

void BLDC_PositionTask( void )
{
    if ( BLDC_Info.PositionCmdActive == 0U )
    {
        return;
    }
}

/* 关闭功率级 */
void BLDC_Disable( void )
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    prvDisableAllMos();
    BLDC_SD_DISABLE();
}

/* 使能功率级 */
void BLDC_Enable( void )
{
    BLDC_SD_ENABLE();
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

/* 重新启动功率级，保留当前外环目标 */
void BLDC_Start( void )
{
    uint16_t startPulse = BLDC_Info.Pulse;
    float expectedRpm = BLDC_Info.ExpectedRPM;
    float expectedCurrent = BLDC_Info.ExpectedCurrent;
    float expectedAngle = BLDC_Info.ExpectedAngleDeg;
    BLDC_CtrlMode_t ctrlMode = BLDC_Info.CtrlMode;
    MotorDir_t direction = BLDC_Info.Direction;
    uint8_t positionCmdActive = BLDC_Info.PositionCmdActive;

    BLDC_Info.MotorStalling = 0U;
    BLDC_ResetControlState(&BLDC_Info);
    BLDC_Info.ExpectedRPM = expectedRpm;
    BLDC_Info.ExpectedCurrent = expectedCurrent;
    BLDC_Info.ExpectedAngleDeg = expectedAngle;
    BLDC_Info.CtrlMode = ctrlMode;
    BLDC_Info.Direction = direction;
    BLDC_Info.PositionCmdActive = positionCmdActive;
    if ( startPulse < BLDC_STARTUP_DUTY )
    {
        startPulse = BLDC_STARTUP_DUTY;
    }
    BLDC_Info.Pulse = startPulse;
    BLDC_Info.MotorRunning = 1U;
    BLDC_Enable();
    Hall_Start();
}

/* 设置 PWM 占空比 */
void BLDC_SetPulse( int32_t duty )
{
    if ( duty < 0 )
    {
        duty = 0;
    }
    if ( duty > (int32_t)BLDC_PWM_MAX_DUTY )
    {
        duty = (int32_t)BLDC_PWM_MAX_DUTY;
    }
    BLDC_Info.Pulse = (uint16_t)duty;
    if ( BLDC_Info.MotorRunning != 0U )
    {
        prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
    }
}

/* =============================================================================
 *  控制任务 — 位置环 / 速度环 / 电流环级联
 * ==========================================================================*/
void BLDC_ControlTask( void )
{
    float positionCurCmd = 0.0f;
    float speedCurCmd = 0.0f;
    float currentTarget = 0.0f;
    float angleError = BLDC_Info.ExpectedAngleDeg - BLDC_Info.CurrentAngleDeg;
    float currentFeedback = prvBLDC_GetCurrentMagnitude(&BLDC_Info);
    uint16_t dutyCmd;

    BLDC_Info.CurrentMagnitude = currentFeedback;

    switch ( BLDC_Info.CtrlMode )
    {
        case BLDC_CTRL_POSITION:
            /* 位置环先判断方向，再输出带符号电流需求 */
            if ( BLDC_Info.PositionCmdActive == 0U )
            {
                return;
            }

            if ( fabsf(angleError) <= BLDC_POSITION_DEADBAND_DEG )
            {
                BLDC_Info.ExpectedCurrent = 0.0f;
                BLDC_Info.ExpectedRPM = 0.0f;
                BLDC_Info.ExpectedRPM_Ramp = 0.0f;
                BLDC_Info.PositionCmdActive = 0U;
                BLDC_Stop();
                return;
            }

            if ( angleError > 0.0f )
            {
                BLDC_SetDirection(MOTOR_FWD);
                positionCurCmd = BLDC_PID_Calc(&BLDC_Info.PIDPos_PositionLoop,
                                               BLDC_Info.ExpectedAngleDeg,
                                               BLDC_Info.CurrentAngleDeg);
            }
            else
            {
                BLDC_SetDirection(MOTOR_REV);
                positionCurCmd = -BLDC_PID_Calc(&BLDC_Info.PIDPos_PositionLoop,
                                                BLDC_Info.ExpectedAngleDeg,
                                                BLDC_Info.CurrentAngleDeg);
            }

            currentTarget = fabsf(positionCurCmd);
            if ( currentTarget > 0.0f && currentTarget < BLDC_POSITION_MIN_CUR_mA )
            {
                currentTarget = BLDC_POSITION_MIN_CUR_mA;
            }
            break;

        case BLDC_CTRL_SPEED:
            /* 速度环在斜坡处理后的转速给定基础上输出电流需求 */
            prvBLDC_RampTargetRPM(&BLDC_Info);
            speedCurCmd = BLDC_PID_Calc(&BLDC_Info.PIDPos_SpeedLoop,
                                        BLDC_Info.ExpectedRPM_Ramp,
                                        BLDC_Info.RPM);
            currentTarget = speedCurCmd;
            break;

        case BLDC_CTRL_CURRENT:
            /* 电流模式直接绕过外环 */
            currentTarget = BLDC_Info.ExpectedCurrent;
            break;

        default:
            currentTarget = 0.0f;
            break;
    }

    currentTarget = prvClampf(currentTarget, 0.0f, BLDC_MAX_CUR_TARGET_mA);
    BLDC_Info.ExpectedCurrent = currentTarget;

    if ( currentTarget <= 0.0f )
    {
        /* 电流目标为 0 时释放功率级 */
        if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION )
        {
            BLDC_Stop();
        }
        return;
    }

    /* 电流内环把电流误差换算成 PWM 占空比指令 */
    float dutyFloat = BLDC_PID_Calc(&BLDC_Info.PID_CurrentLoop,
                                    currentTarget,
                                    currentFeedback);
    dutyFloat = prvClampf(dutyFloat, 0.0f, (float)BLDC_PWM_MAX_DUTY);
    dutyCmd = (uint16_t)dutyFloat;
    if ( dutyCmd < BLDC_STARTUP_DUTY )
    {
        dutyCmd = BLDC_STARTUP_DUTY;
    }

    BLDC_Info.Pulse = dutyCmd;
    if ( BLDC_Info.MotorRunning == 0U )
    {
        BLDC_Start();
    }
    else
    {
        prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
    }
}

uint16_t BLDC_GetPulse( void )
{
    return BLDC_Info.Pulse;
}

void BLDC_SetDirection( MotorDir_t dir )
{
    BLDC_Info.Direction = dir;
}

void BLDC_Stop( void )
{
    BLDC_ResetControlState(&BLDC_Info);
    BLDC_Info.ExpectedRPM = 0.0f;
    BLDC_Info.ExpectedRPM_Ramp = 0.0f;
    BLDC_Info.PositionCmdActive = 0U;
    BLDC_Disable();
    Hall_Disable();
}

void BLDC_TripStop( void )
{
    BLDC_ResetControlState(&BLDC_Info);
    BLDC_Info.ExpectedRPM = 0.0f;
    BLDC_Info.ExpectedRPM_Ramp = 0.0f;
    BLDC_Info.PositionCmdActive = 0U;
    BLDC_Info.MotorStalling = 1U;
    BLDC_Disable();
    Hall_Disable();
}

/* 启动 Hall 采集 */
void Hall_Start( void )
{
    uint8_t hall;

    Hall_Info.HallFirstEdge = 1U;
    Hall_Info.HallEdgeFlag = 0U;
    Hall_Info.HallTickCnt = 0U;
    Hall_Info.HallStateShadow = Hall_GetState();
    Hall_Info.HallLastEdgeMs = SystemRunTime_1ms;
    Hall_Info.HallSectorStartMs = SystemRunTime_1ms;
    Hall_Info.HallSectorPeriodMs = 0U;
    Hall_Info.HallSpeedFilter.Index = 0U;
    Hall_Info.HallSpeedFilter.ValidCnt = 0U;
    Hall_Info.HallSpeedFilter.LastFilter = 0U;
    Hall_Info.HallSpeedFilter.Inited = 0U;
    Hall_Info.HallSpeedFilter.HallTickBuf[0] = 0U;
    Hall_Info.HallSpeedFilter.HallTickBuf[1] = 0U;
    Hall_Info.HallSpeedFilter.HallTickBuf[2] = 0U;

    HAL_TIMEx_HallSensor_Start_IT(&htim5);
    hall = Hall_Info.HallStateShadow;
    BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

    if ( hall >= 1U && hall <= 6U )
    {
        BLDC_ChangeMOSstate(pHallTable[hall].PwmPhase,
                            pHallTable[hall].LowPhase,
                            BLDC_Info.Pulse);
    }
}

void Hall_enable( void )
{
    Hall_Start();
}

void Hall_Disable( void )
{
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Stop(&htim5);
}

/* 读取三路 Hall 状态，编码为 1~6 */
uint8_t Hall_GetState( void )
{
    uint8_t state = 0U;

    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 0;
    }
    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 1;
    }
    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 2;
    }
    return state;
}

/* 每个有效 Hall 跳变对应机械角度前进一步，同时记录扇区时间用于插值 */
void BLDC_OnHallTransition( uint8_t previousHall, uint8_t currentHall )
{
    int8_t stepDelta = prvBLDC_GetHallStepDelta(previousHall, currentHall);
    uint32_t nowMs = SystemRunTime_1ms;

    if ( stepDelta == 0 )
    {
        if ( previousHall == currentHall || currentHall < 1U || currentHall > 6U )
        {
            return;
        }
        stepDelta = (BLDC_Info.Direction == MOTOR_FWD) ? 1 : -1;
    }

    /* 记录当前扇区用时，并更新扇区起点时刻 */
    Hall_Info.HallSectorPeriodMs = nowMs - Hall_Info.HallSectorStartMs;
    Hall_Info.HallSectorStartMs = nowMs;

    BLDC_Info.HallStepCount += stepDelta;
    BLDC_Info.CurrentAngleDeg = (float)BLDC_Info.HallStepCount * BLDC_MECH_DEG_PER_SECTOR;
}

/* 切换 MOSFET 状态 */
void BLDC_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty )
{
    prvDisableAllMos();

    switch ( PwmPhase )
    {
        case PHASE_U:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Duty);
            break;
        case PHASE_V:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Duty);
            break;
        case PHASE_W:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Duty);
            break;
        default:
            return;
    }

    switch ( LowPhase )
    {
        case PHASE_U:
            HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_SET);
            break;
        case PHASE_V:
            HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_SET);
            break;
        case PHASE_W:
            HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_SET);
            break;
        default:
            return;
    }

    BLDC_Info.ActivePwmPhase = PwmPhase;
    BLDC_Info.ActiveLowPhase = LowPhase;
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);
}

/* 选择正转/反转换向表 */
void BLDC_HallTableSelect( MotorDir_t Dir )
{
    if ( Dir == MOTOR_FWD )
    {
        pHallTable = gComFwd;
    }
    else
    {
        pHallTable = gComRev;
    }
}

MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC )
{
    return pBLDC->Direction;
}

/* 软限流逐步减小占空比，硬过流立即停机 */
void BLDC_CurrentProtect( void )
{
    float peak = prvBLDC_GetCurrentMagnitude(&BLDC_Info);

    if ( peak >= BLDC_CURRENT_TRIP_mA )
    {
        BLDC_TripStop();
        return;
    }

    if ( peak > BLDC_CURRENT_SOFT_LIMIT_mA )
    {
        float delta = peak - BLDC_CURRENT_SOFT_LIMIT_mA;
        uint16_t reduction = (uint16_t)(delta * BLDC_CURRENT_LIMIT_KP);

        if ( reduction < 1U )
        {
            reduction = 1U;
        }
        if ( BLDC_Info.Pulse > (BLDC_PWM_MIN_DUTY + reduction) )
        {
            BLDC_Info.Pulse -= reduction;
        }
        else
        {
            BLDC_Info.Pulse = BLDC_PWM_MIN_DUTY;
        }
        prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
    }
}

/* 主循环：先处理 Hall 反馈，再跑级联控制，最后做电流保护 */
void BLDC_Cyclic( void )
{
    /* FOC 模式跳过六步换向，由 FOC_Update 在 1ms 任务中处理 */
    if ( BLDC_Info.CtrlMode == BLDC_CTRL_FOC )
    {
        return;
    }

    if ( BLDC_Info.MotorStalling != 0U )
    {
        BLDC_TripStop();
        return;
    }

    if ( BLDC_Info.MotorRunning != 0U )
    {
        prvBLDC_HallCyclic();
    }

    BLDC_ControlTask();
    if ( BLDC_Info.MotorRunning != 0U )
    {
        BLDC_CurrentProtect();
    }
}
