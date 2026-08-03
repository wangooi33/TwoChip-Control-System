/* =============================================================================
 *  FOC.c — 磁场定向控制 (Field Oriented Control)
 *
 *  架构:
 *    1. SVPWM (扇区判定 + T1/T2 + 比较值)
 *    2. 位置检测 (3 路霍尔 + 扇区内线性插值)
 *    3. 电流反馈复用 w_adc 采样链路 (ADC_to_Current + BLDC_Info 零偏)
 *    4. 三环级联 (电流-速度-位置)
 *    5. 1ms 任务调度
 *
 *  Park 变换物理含义:
 *    d 轴(磁场方向) — 仅励磁, 不做功, 控制 Id = 0
 *    q 轴(切线方向) — 输出力矩, 由外环给定 Iq
 * ==========================================================================*/

/* includes ------------------------------------------------------------------*/
#include "FOC.h"
#include "BLDC_Control.h"
#include "Hall.h"
#include "w_adc.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
FOC_Info_t FOC_Info;

/* =============================================================================
 *  Park 变换 (αβ -> dq)
 *
 *  d 轴(磁场方向) — 励磁分量, 控制为 0
 *  q 轴(切线方向) — 转矩分量, 输出力矩
 * ==========================================================================*/
static void Park( float alpha, float beta, float theta,
                  float *pId, float *pIq )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *pId =  alpha * c + beta * s;
    *pIq = -alpha * s + beta * c;
}

/* 逆 Park 变换 (dq -> αβ) */
static void InvPark( float vd, float vq, float theta,
                     float *pValpha, float *pVbeta )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *pValpha = vd * c - vq * s;
    *pVbeta  = vd * s + vq * c;
}

/* =============================================================================
 *  SVPWM — 扇区判定 + T1/T2 + 比较值
 * ==========================================================================*/

/* 判定扇区: 利用三个参考值的符号编码 */
static uint8_t CalcSector( float valpha, float vbeta )
{
    float vr1 = vbeta;
    float vr2 = (sqrtf(3.0f) * 0.5f) * valpha - 0.5f * vbeta;
    float vr3 = -(sqrtf(3.0f) * 0.5f) * valpha - 0.5f * vbeta;
    uint8_t n = 0;

    if ( vr1 > 0.0f ) n |= 1U;
    if ( vr2 > 0.0f ) n |= 2U;
    if ( vr3 > 0.0f ) n |= 4U;

    /* N -> 扇区映射表 */
    static const uint8_t map[8] = {0, 2, 6, 1, 4, 3, 5, 0};
    return map[n];
}

/* 计算相邻两个有效矢量的作用时间 */
static void CalcT1T2( float va, float vb, uint8_t sector,
                      float ts, float vdc, float *pT1, float *pT2 )
{
    float x = sqrtf(3.0f) * vb * ts / vdc;
    float y = (sqrtf(3.0f) * vb + 3.0f * va) * ts / (2.0f * vdc);
    float z = (sqrtf(3.0f) * vb - 3.0f * va) * ts / (2.0f * vdc);

    switch ( sector )
    {
        case 1:  *pT1 = -z; *pT2 = x;  break;
        case 2:  *pT1 =  z; *pT2 = y;  break;
        case 3:  *pT1 =  x; *pT2 = -y; break;
        case 4:  *pT1 = -x; *pT2 = z;  break;
        case 5:  *pT1 = -y; *pT2 = -z; break;
        case 6:  *pT1 =  y; *pT2 = -x; break;
        default: *pT1 = 0.0f; *pT2 = 0.0f; break;
    }
}

/* 按扇区把三个比较值分配给 A/B/C 三相 */
static void SectorToCmp( uint8_t sector, uint16_t c1, uint16_t c2, uint16_t c3,
                         uint16_t *pTa, uint16_t *pTb, uint16_t *pTc )
{
    switch ( sector )
    {
        case 1:  *pTa = c1; *pTb = c2; *pTc = c3; break;
        case 2:  *pTa = c2; *pTb = c1; *pTc = c3; break;
        case 3:  *pTa = c2; *pTb = c3; *pTc = c1; break;
        case 4:  *pTa = c3; *pTb = c2; *pTc = c1; break;
        case 5:  *pTa = c3; *pTb = c1; *pTc = c2; break;
        case 6:  *pTa = c1; *pTb = c3; *pTc = c2; break;
        default: *pTa = 0;  *pTb = 0;  *pTc = 0;  break;
    }
}

/* SVPWM 主函数: 输入实际电压 Vα/Vβ [V], 输出三相 CCR 计数值 */
static void SVPWM( float valpha, float vbeta, uint32_t *duty )
{
    float vdc = BLDC_Info.PowerVoltage;

    /* 母线电压异常时使用安全默认值，避免除零 */
    if ( vdc < 1.0f )
    {
        vdc = 24.0f;
    }

    /* 中心对齐模式 PWM 周期 [s] */
    float ts = 2.0f * (float)FOC_PWM_PERIOD / FOC_TIM_CLK_HZ;

    uint8_t sector = CalcSector(valpha, vbeta);

    float t1 = 0.0f;
    float t2 = 0.0f;
    CalcT1T2(valpha, vbeta, sector, ts, vdc, &t1, &t2);

    /* 过调制限制 */
    if ( (t1 + t2) > ts )
    {
        float k = ts / (t1 + t2);
        t1 *= k;
        t2 *= k;
    }

    /* 中心对齐模式比较值 */
    float t0 = ts - t1 - t2;
    uint16_t c1 = (uint16_t)(t0 * 0.25f * FOC_TIM_CLK_HZ);
    uint16_t c2 = c1 + (uint16_t)(t1 * 0.5f * FOC_TIM_CLK_HZ);
    uint16_t c3 = c2 + (uint16_t)(t2 * 0.5f * FOC_TIM_CLK_HZ);

    if ( c1 > FOC_PWM_PERIOD ) c1 = FOC_PWM_PERIOD;
    if ( c2 > FOC_PWM_PERIOD ) c2 = FOC_PWM_PERIOD;
    if ( c3 > FOC_PWM_PERIOD ) c3 = FOC_PWM_PERIOD;

    uint16_t ta = 0;
    uint16_t tb = 0;
    uint16_t tc = 0;
    SectorToCmp(sector, c1, c2, c3, &ta, &tb, &tc);

    duty[0] = ta;
    duty[1] = tb;
    duty[2] = tc;
}

/* =============================================================================
 *  位置检测 — 3 路霍尔 + 扇区内线性插值
 * ==========================================================================*/
static void HallPositionUpdate( void )
{
    uint32_t nowMs = SystemRunTime_1ms;
    float progress = 0.0f;

    /* 扇区用时: HallTickCnt 为 1MHz 计数, 换算为 ms */
    float periodMs = (float)Hall_Info.HallTickCnt / 1000.0f;

    if ( periodMs > 1.0f )
    {
        uint32_t elapsed = nowMs - Hall_Info.HallLastEdgeMs;
        progress = (float)elapsed / periodMs;
        progress = Clampf(progress, 0.0f, 1.0f);
    }

    /* 连续机械角: 每个 Hall 步进 = 30° 机械角 */
    float mechDeg = ((float)BLDC_Info.HallStepCount + progress)
                  * HALL_DEG_PER_SECTOR;
    FOC_Info.PositionDeg = mechDeg;

    /* 连续电角度 = 机械角 * 极对数 */
    float thetaDeg = mechDeg * (float)BLDC_POLE_PAIRS;
    FOC_Info.ThetaElec_Rad = thetaDeg * DEG_TO_RAD;

    if ( FOC_Info.ThetaElec_Rad >= 6.2831853f )
    {
        FOC_Info.ThetaElec_Rad -= 6.2831853f;
    }
    if ( FOC_Info.ThetaElec_Rad < 0.0f )
    {
        FOC_Info.ThetaElec_Rad += 6.2831853f;
    }

    /* 电角速度与转速 */
    if ( periodMs > 1.0f )
    {
        float periodS = periodMs * 0.001f;
        FOC_Info.OmegaElec_RadPs = 1.04719755f / periodS;   /* 60° = PI/3 rad */
        FOC_Info.SpeedRPM = FOC_Info.OmegaElec_RadPs * 60.0f
                          / (6.2831853f * (float)BLDC_POLE_PAIRS);
    }
    else
    {
        FOC_Info.OmegaElec_RadPs = 0.0f;
        FOC_Info.SpeedRPM = 0.0f;
    }
}

/* =============================================================================
 *  三相电流读取 — 复用 w_adc 的 ADC_to_Current 与 BLDC_Info 零偏校准
 *  返回 mA 并做一阶低通滤波
 * ==========================================================================*/
static void UpdateFOCCurrents( void )
{
    float rawU = ADC_to_Current(gADC3CaptureBuffer[BLDC_U_Current],
                                BLDC_Info.CurrZeroOffsetV.U_PhaseSetV) * 1000.0f;
    float rawV = ADC_to_Current(gADC3CaptureBuffer[BLDC_V_Current],
                                BLDC_Info.CurrZeroOffsetV.V_PhaseSetV) * 1000.0f;
    float rawW = ADC_to_Current(gADC3CaptureBuffer[BLDC_W_Current],
                                BLDC_Info.CurrZeroOffsetV.W_PhaseSetV) * 1000.0f;

    FOC_Info.Iu_mA = 0.8f * FOC_Info.Iu_mA + 0.2f * rawU;
    FOC_Info.Iv_mA = 0.8f * FOC_Info.Iv_mA + 0.2f * rawV;
    FOC_Info.Iw_mA = 0.8f * FOC_Info.Iw_mA + 0.2f * rawW;
}

/* =============================================================================
 *  TIM8 FOC 模式重配置
 * ==========================================================================*/
static void TimerFOCConfig( void )
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);

    /* PH13/14/15: GPIO -> TIM8_CH1N/CH2N/CH3N (AF3) */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOH, &gpio);

    HAL_TIM_PWM_DeInit(&htim8);

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 1;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period = (uint32_t)FOC_PWM_PERIOD;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);

    TIM_MasterConfigTypeDef masterCfg = {0};
    masterCfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
    masterCfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim8, &masterCfg);

    TIM_OC_InitTypeDef ocCfg = {0};
    ocCfg.OCMode = TIM_OCMODE_PWM1;
    ocCfg.Pulse = 0;
    ocCfg.OCPolarity = TIM_OCPOLARITY_HIGH;
    ocCfg.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ocCfg.OCFastMode = TIM_OCFAST_DISABLE;
    ocCfg.OCIdleState = TIM_OCIDLESTATE_RESET;
    ocCfg.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim8, &ocCfg, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim8, &ocCfg, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim8, &ocCfg, TIM_CHANNEL_3);

    TIM_BreakDeadTimeConfigTypeDef bdtCfg = {0};
    bdtCfg.OffStateRunMode = TIM_OSSR_ENABLE;
    bdtCfg.OffStateIDLEMode = TIM_OSSI_ENABLE;
    bdtCfg.LockLevel = TIM_LOCKLEVEL_OFF;
    bdtCfg.DeadTime = (uint8_t)FOC_DEADTIME_CYCLES;
    bdtCfg.BreakState = TIM_BREAK_DISABLE;
    bdtCfg.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdtCfg.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim8, &bdtCfg);

    HAL_TIM_MspPostInit(&htim8);
}

static void PWMStart( void )
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

static void PWMStop( void )
{
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;

    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
}

/* =============================================================================
 *  三环级联
 * ==========================================================================*/

/* 位置环: 输出转速目标 (10Hz / 100ms) */
static void PositionLoop( void )
{
    float rpmRef = PID_Update(&FOC_Info.PID_Position,
                              FOC_Info.PositionRef_Deg,
                              FOC_Info.PositionDeg);
    FOC_Info.SpeedRef_RPM = Clampf(rpmRef,
                                   -FOC_POSITION_LIMIT_RPM,
                                   FOC_POSITION_LIMIT_RPM);
}

/* 速度环: 输出 q 轴电流目标 (100Hz / 10ms) */
static void SpeedLoop( void )
{
    float iqRef = PID_Update(&FOC_Info.PID_Speed,
                             FOC_Info.SpeedRef_RPM,
                             FOC_Info.SpeedRPM);
    FOC_Info.Iq_Ref_mA = Clampf(iqRef,
                                -FOC_SPEED_LIMIT_mA,
                                FOC_SPEED_LIMIT_mA);
    FOC_Info.Id_Ref_mA = 0.0f;   /* Id = 0 控制 */
}

/* 电流环: 输出 dq 电压 (1kHz / 1ms) */
static void CurrentLoop( void )
{
    float vd = PID_Update(&FOC_Info.PID_Id,
                          FOC_Info.Id_Ref_mA,
                          FOC_Info.Id_mA);
    float vq = PID_Update(&FOC_Info.PID_Iq,
                          FOC_Info.Iq_Ref_mA,
                          FOC_Info.Iq_mA);
    FOC_Info.Vd = vd;
    FOC_Info.Vq = vq;
}

/* =============================================================================
 *  公共函数
 * ==========================================================================*/

void FOC_Init( void )
{
    PID_Init(&FOC_Info.PID_Id, FOC_KP_CURRENT, FOC_KI_CURRENT, 0.0f,
             FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA * 0.5f, 0.5f, FOC_CTRL_PERIOD_S);
    PID_Init(&FOC_Info.PID_Iq, FOC_KP_CURRENT, FOC_KI_CURRENT, 0.0f,
             FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA * 0.5f, 0.5f, FOC_CTRL_PERIOD_S);
    PID_Init(&FOC_Info.PID_Speed, FOC_KP_SPEED, FOC_KI_SPEED, 0.0f,
             FOC_SPEED_LIMIT_mA, FOC_SPEED_LIMIT_mA * 0.5f, 0.5f,
             (float)FOC_SPEED_LOOP_MS * 0.001f);
    PID_Init(&FOC_Info.PID_Position, FOC_KP_POSITION, FOC_KI_POSITION, FOC_KD_POSITION,
             FOC_POSITION_LIMIT_RPM, FOC_POSITION_LIMIT_RPM * 0.5f, 0.5f,
             (float)FOC_POSITION_LOOP_MS * 0.001f);

    /* 清零状态 */
    FOC_Info.ThetaElec_Rad = 0.0f;
    FOC_Info.OmegaElec_RadPs = 0.0f;
    FOC_Info.PositionDeg = 0.0f;
    FOC_Info.SpeedRPM = 0.0f;
    FOC_Info.Iu_mA = 0.0f;
    FOC_Info.Iv_mA = 0.0f;
    FOC_Info.Iw_mA = 0.0f;
    FOC_Info.Id_mA = 0.0f;
    FOC_Info.Iq_mA = 0.0f;
    FOC_Info.Id_Ref_mA = 0.0f;
    FOC_Info.Iq_Ref_mA = 0.0f;
    FOC_Info.SpeedRef_RPM = 0.0f;
    FOC_Info.PositionRef_Deg = 0.0f;
    FOC_Info.LoopCounter = 0U;
    FOC_Info.LoopMode = FOC_LOOP_CURRENT;
    FOC_Info.Calibrated = 0U;
    FOC_Info.Enabled = 0U;

    /* 电流零偏由 main 的 Motor_CurrentOffsetCalibrate 完成, 这里直接标记可用 */
    FOC_Info.Calibrated = 1U;

    /* 读取初始 Hall 状态 -> 初始电角度 */
    uint8_t hall = Hall_GetState();
    if ( hall >= 1U && hall <= 6U )
    {
        /* 根据霍尔真值表:
         *正转序列 hallForwardSeq = {1, 5, 4, 6, 2, 3}
         * 序列第 N 位 -> 电角度中心 30 + N*60 度:
         *   Hall=1 -> 30°, Hall=5 -> 90°, Hall=4 -> 150°
         *   Hall=6 -> 210°, Hall=2 -> 270°, Hall=3 -> 330°
         */
        static const float hallCenterDeg[8] = {
            0.0f,  30.0f, 270.0f, 330.0f,
            150.0f,  90.0f, 210.0f, 0.0f
        };
        FOC_Info.ThetaElec_Rad = hallCenterDeg[hall] * DEG_TO_RAD;
    }
}

void FOC_Enable( void )
{
    if ( !FOC_Info.Calibrated )
    {
        return;
    }

    Hall_Start();       /* FOC 位置反馈依赖霍尔 */
    TimerFOCConfig();
    PWMStart();

    FOC_Info.Enabled = 1U;
}

void FOC_Disable( void )
{
    FOC_Info.Enabled = 0U;

    PWMStop();
    Hall_Disable();
    PID_Reset(&FOC_Info.PID_Id);
    PID_Reset(&FOC_Info.PID_Iq);
    PID_Reset(&FOC_Info.PID_Speed);
    PID_Reset(&FOC_Info.PID_Position);
}

/* =============================================================================
 *  FOC_Update() — 1kHz 控制主循环
 * ==========================================================================*/
void FOC_Update( void )
{
    if ( !FOC_Info.Enabled )
    {
        return;
    }

    FOC_Info.LoopCounter++;

    /* 1. 三相电流 (复用 w_adc 换算) */
    UpdateFOCCurrents();

    /* 2. Hall 位置插值 */
    HallPositionUpdate();

    float ia = FOC_Info.Iu_mA;
    float ib = FOC_Info.Iv_mA;
    float theta = FOC_Info.ThetaElec_Rad;

    /* 3. Clarke: abc -> αβ (复用 w_adc 通用变换) */
    Clarke(ia, ib, &FOC_Info.Ialpha_mA, &FOC_Info.Ibeta_mA);

    /* 4. Park: αβ -> dq */
    Park(FOC_Info.Ialpha_mA, FOC_Info.Ibeta_mA, theta,
         &FOC_Info.Id_mA, &FOC_Info.Iq_mA);

    /* 5. 外环: 位置环(100ms) -> 速度环(10ms) */
    if ( FOC_Info.LoopMode == FOC_LOOP_POSITION &&
         (FOC_Info.LoopCounter % FOC_POSITION_LOOP_MS) == 0U )
    {
        PositionLoop();
    }

    if ( (FOC_Info.LoopMode == FOC_LOOP_SPEED ||
          FOC_Info.LoopMode == FOC_LOOP_POSITION) &&
         (FOC_Info.LoopCounter % FOC_SPEED_LOOP_MS) == 0U )
    {
        SpeedLoop();
    }

    /* 6. 电流环 */
    CurrentLoop();

    /* 7. 电压归一化 + 逆 Park */
    float vdc = BLDC_Info.PowerVoltage;		/* ADC采样的母线电压 */
    if ( vdc < 1.0f )
    {
        vdc = 24.0f;
    }
	//把电流环输出归一化到 [-1, 1]
    float vdNorm = FOC_Info.Vd / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
    float vqNorm = FOC_Info.Vq / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
	//换算成实际相电压
    vdNorm = Clampf(vdNorm, -1.0f, 1.0f);
    vqNorm = Clampf(vqNorm, -1.0f, 1.0f);
	//把归一化值映射到真实电压.乘 vdc * 0.5 是因为在中心点电压参考系下,相电压最大幅值约等于母线电压的一半
    float vdReal = vdNorm * vdc * 0.5f;
    float vqReal = vqNorm * vdc * 0.5f;

    InvPark(vdReal, vqReal, theta,
            &FOC_Info.Valpha, &FOC_Info.Vbeta);

    /* 8. SVPWM */
    uint32_t duty[3];
    SVPWM(FOC_Info.Valpha, FOC_Info.Vbeta, duty);

    /* 9. 更新 TIM8 CCR */
    TIM8->CCR1 = duty[0];
    TIM8->CCR2 = duty[1];
    TIM8->CCR3 = duty[2];
}

/* =============================================================================
 *  目标给定 API
 * ==========================================================================*/

void FOC_SetIqRef( float iq_mA )
{
    FOC_Info.Iq_Ref_mA = Clampf(iq_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
    FOC_Info.LoopMode = FOC_LOOP_CURRENT;
}

void FOC_SetIdRef( float id_mA )
{
    FOC_Info.Id_Ref_mA = Clampf(id_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
}

void FOC_SetSpeedRef( float rpm )
{
    FOC_Info.SpeedRef_RPM = rpm;
    FOC_Info.LoopMode = FOC_LOOP_SPEED;
}

float FOC_GetSpeedRPM( void )
{
    return FOC_Info.SpeedRPM;
}

void FOC_SetPositionRef( float deg )
{
    FOC_Info.PositionRef_Deg = deg;
    FOC_Info.LoopMode = FOC_LOOP_POSITION;
}

float FOC_GetPositionDeg( void )
{
    return FOC_Info.PositionDeg;
}
