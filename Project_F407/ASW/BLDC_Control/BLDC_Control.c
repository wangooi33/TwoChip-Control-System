/* =============================================================================
 *  BLDC_Control.c — BLDC 电机控制核心
 *
 *  职责:
 *    位置环 / 速度环 / 电流环 PID 级联, 目标设定, 启停与保护。
 *
 *  分工:
 *    Hall.c    — 霍尔传感器采集 / 角度 / 测速
 *    SixStep.c — 六步换相 / MOSFET 驱动
 *    FOC.c     — FOC 矢量控制
 *
 *  FOC 模式下本模块只做状态管理, 不调用六步换相。
 * ==========================================================================*/

/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"
#include "Hall.h"
#include "SixStep.h"
#include "FOC.h"
#include <math.h>

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;

/* local helpers -------------------------------------------------------------*/

/* 初始化速度环 PID */
static void BLDC_PIDSpeedInit( PID_t *pPID )
{
    PID_Init(pPID, BLDC_SPEED_PID_KP, BLDC_SPEED_PID_KI, BLDC_SPEED_PID_KD,
             BLDC_MAX_CUR_TARGET_mA, BLDC_MAX_CUR_TARGET_mA * 0.5f, 0.5f, 0.005f);
}

/* 初始化电流环 PID */
static void BLDC_PIDCurrentInit( PID_t *pPID )
{
    PID_Init(pPID, BLDC_CURRENT_PID_KP, BLDC_CURRENT_PID_KI, BLDC_CURRENT_PID_KD,
             (float)BLDC_PWM_MAX_DUTY, (float)BLDC_PWM_MAX_DUTY * 0.5f, 0.5f, 0.005f);
}

/* 初始化位置环 PID */
static void BLDC_PIDPositionInit( PID_t *pPID )
{
    PID_Init(pPID, BLDC_POSITION_PID_KP, BLDC_POSITION_PID_KI, BLDC_POSITION_PID_KD,
             BLDC_MAX_CUR_TARGET_mA, BLDC_MAX_CUR_TARGET_mA * 0.5f, 0.5f, 0.005f);
}

/* 切换控制模式时清空各环路积分和历史状态 */
static void BLDC_ResetAllLoopState( BLDC_Info_t *pBLDC )
{
    PID_Reset(&pBLDC->PIDPos_SpeedLoop);
    PID_Reset(&pBLDC->PID_CurrentLoop);
    PID_Reset(&pBLDC->PIDPos_PositionLoop);
    pBLDC->ExpectedRPM_Ramp = 0.0f;
}

/* 速度目标斜坡，限制加速度 */
static void BLDC_RampTargetRPM( BLDC_Info_t *pBLDC )
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
static float BLDC_GetCurrentMagnitude( const BLDC_Info_t *pBLDC )
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

/* public functions ----------------------------------------------------------*/

/* 初始化全部控制环 PID */
void BLDC_PIDInit( BLDC_Info_t *pBLDC )
{
    BLDC_PIDSpeedInit(&pBLDC->PIDPos_SpeedLoop);
    BLDC_PIDCurrentInit(&pBLDC->PID_CurrentLoop);
    BLDC_PIDPositionInit(&pBLDC->PIDPos_PositionLoop);
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
    PID_Reset(&pBLDC->PIDPos_SpeedLoop);
    PID_Reset(&pBLDC->PID_CurrentLoop);
    PID_Reset(&pBLDC->PIDPos_PositionLoop);
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
        BLDC_ResetAllLoopState(&BLDC_Info);
    }
    BLDC_Info.ExpectedRPM = Clampf(expectedRPM, 0.0f, BLDC_MAX_RPM_TARGET);
    BLDC_Info.CtrlMode = BLDC_CTRL_SPEED;
    BLDC_Info.PositionCmdActive = 0U;
}

/* 设置电流目标 */
void BLDC_SetExpectedCurrent( float expectedCurrent )
{
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_CURRENT )
    {
        BLDC_ResetAllLoopState(&BLDC_Info);
    }
    BLDC_Info.ExpectedCurrent = Clampf(expectedCurrent, 0.0f, BLDC_MAX_CUR_TARGET_mA);
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
        BLDC_ResetAllLoopState(&BLDC_Info);
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

/* 关闭功率级 */
void BLDC_Disable( void )
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    SixStep_DisableAllMos();
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

        /* FOC 模式由 FOC_Enable 接管, 六步换相完全不参与 */
    if ( ctrlMode == BLDC_CTRL_FOC )
    {
        BLDC_Info.MotorStalling = 0U;
        BLDC_ResetControlState(&BLDC_Info);
        BLDC_Info.CtrlMode = BLDC_CTRL_FOC;
        BLDC_Info.Direction = direction;
        BLDC_Info.MotorRunning = 1U;
        BLDC_SD_ENABLE();
        FOC_Enable();
        return;
    }

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
    SixStep_Start();
}

/* 设置 PWM 占空比 (仅六步模式生效) */
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
        SixStep_UpdateActiveDuty(BLDC_Info.Pulse);
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
    float currentFeedback = BLDC_GetCurrentMagnitude(&BLDC_Info);
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
                positionCurCmd = PID_Update(&BLDC_Info.PIDPos_PositionLoop,
                                            BLDC_Info.ExpectedAngleDeg,
                                            BLDC_Info.CurrentAngleDeg);
            }
            else
            {
                BLDC_SetDirection(MOTOR_REV);
                positionCurCmd = -PID_Update(&BLDC_Info.PIDPos_PositionLoop,
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
            BLDC_RampTargetRPM(&BLDC_Info);
            speedCurCmd = PID_Update(&BLDC_Info.PIDPos_SpeedLoop,
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

    currentTarget = Clampf(currentTarget, 0.0f, BLDC_MAX_CUR_TARGET_mA);
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
    float dutyFloat = PID_Update(&BLDC_Info.PID_CurrentLoop,
                                 currentTarget,
                                 currentFeedback);
    dutyFloat = Clampf(dutyFloat, 0.0f, (float)BLDC_PWM_MAX_DUTY);
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
        SixStep_UpdateActiveDuty(BLDC_Info.Pulse);
    }
}

/* 切换到 FOC 矢量控制模式 */
void BLDC_SetFOCMode( void )
{
    BLDC_ResetAllLoopState(&BLDC_Info);
    BLDC_Info.CtrlMode = BLDC_CTRL_FOC;
    BLDC_Info.PositionCmdActive = 0U;
}
void BLDC_SetDirection( MotorDir_t dir )
{
    BLDC_Info.Direction = dir;
}

MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC )
{
    return pBLDC->Direction;
}

void BLDC_Stop( void )
{
    BLDC_ResetControlState(&BLDC_Info);
    BLDC_Info.ExpectedRPM = 0.0f;
    BLDC_Info.ExpectedRPM_Ramp = 0.0f;
    BLDC_Info.PositionCmdActive = 0U;

    /* FOC 模式关闭矢量控制, 六步模式关闭换相驱动 */
    if ( BLDC_Info.CtrlMode == BLDC_CTRL_FOC )
    {
        FOC_Disable();
    }
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_FOC )
    {
        SixStep_Stop();
    }
    BLDC_Disable();
}

void BLDC_TripStop( void )
{
    BLDC_ResetControlState(&BLDC_Info);
    BLDC_Info.ExpectedRPM = 0.0f;
    BLDC_Info.ExpectedRPM_Ramp = 0.0f;
    BLDC_Info.PositionCmdActive = 0U;
    BLDC_Info.MotorStalling = 1U;

    if ( BLDC_Info.CtrlMode == BLDC_CTRL_FOC )
    {
        FOC_Disable();
    }
    if ( BLDC_Info.CtrlMode != BLDC_CTRL_FOC )
    {
        SixStep_Stop();
    }
    BLDC_Disable();
}

/* 软限流逐步减小占空比，硬过流立即停机 */
void BLDC_CurrentProtect( void )
{
    float peak = BLDC_GetCurrentMagnitude(&BLDC_Info);

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
        SixStep_UpdateActiveDuty(BLDC_Info.Pulse);
    }
}

/* 主循环: 六步模式处理 Hall 换相; FOC 模式完全跳过 */
void BLDC_Cyclic( void )
{
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
        SixStep_HallCyclic();
    }

    BLDC_ControlTask();
    if ( BLDC_Info.MotorRunning != 0U )
    {
        BLDC_CurrentProtect();
    }
}
