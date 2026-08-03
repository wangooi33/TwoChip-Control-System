/* =============================================================================
 *  FOC.c — 磁场定向控制 (Field Oriented Control)
 *
 *  按照整合版流程实现:
 *    1. PID 控制器 (退积分反馈抗饱和)
 *    2. SVPWM (扇区判定 + T1/T2 + 比较值)
 *    3. 位置检测 (3 路霍尔传感器 + 扇区内线性插值)
 *    4. 电流采样 + Clarke + Park
 *    5. 三环级联 (电流-速度-位置)
 *    6. 1ms 任务调度
 *
 *  位置反馈: 霍尔传感器 (60° 分辨率) -> 扇区内按时间比例线性插值
 *  电流反馈: ADC3 DMA (PF6/PF7/PF8, 三路同时采样)
 *  PWM 输出: TIM8 中心对齐 + 互补输出 + 死区
 *
 *  Park 变换物理含义:
 *    d 轴(磁场方向) — 仅励磁, 不做功, 控制 Id = 0
 *    q 轴(切线方向) — 输出力矩, 由外环给定 Iq
 * ==========================================================================*/

/* includes ------------------------------------------------------------------*/
#include "FOC.h"
#include "BLDC_Control.h"
#include "adc.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
FOC_Info_t FOC_Info;

/* local helpers -------------------------------------------------------------*/

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
 *  PID 实现 — 退积分反馈抗饱和
 *
 *  error = ref - fdb
 *  integral += Ki * error * dt (限幅到 IntLimit)
 *  output = Kp*error + integral + Kd*(error - PrevErr)/dt
 *  output 限幅到 Limit
 *  若限幅前后不一致, 将差值 * Kb 反馈回积分器, 自动退积分
 * ==========================================================================*/
static void prvPID_Init( FOC_PID_t *pPID, float Kp, float Ki, float Kd,
                         float limit, float intLimit, float Kb, float dt )
{
    pPID->Kp       = Kp;
    pPID->Ki       = Ki;
    pPID->Kd       = Kd;
    pPID->Integral = 0.0f;
    pPID->Output   = 0.0f;
    pPID->Limit    = limit;
    pPID->IntLimit = intLimit;
    pPID->Kb       = Kb;
    pPID->PrevErr  = 0.0f;
    pPID->PrevOut  = 0.0f;
    pPID->Dt       = dt;
}

static void prvPID_Reset( FOC_PID_t *pPID )
{
    pPID->Integral = 0.0f;
    pPID->Output   = 0.0f;
    pPID->PrevErr  = 0.0f;
    pPID->PrevOut  = 0.0f;
}

static float prvPID_Calc( FOC_PID_t *pPID, float ref, float fdb )
{
    float error = ref - fdb;
    float dt = pPID->Dt;
    float deriv = 0.0f;

    if ( dt > 0.0f )
    {
        deriv = pPID->Kd * (error - pPID->PrevErr) / dt;
    }

    /* 积分累积 + 积分限幅 */
    pPID->Integral += pPID->Ki * error * dt;
    pPID->Integral  = prvClampf(pPID->Integral, -pPID->IntLimit, pPID->IntLimit);

    /* 限幅前输出 */
    float outUnlim = pPID->Kp * error + pPID->Integral + deriv;

    /* 限幅后输出 */
    float outSat = prvClampf(outUnlim, -pPID->Limit, pPID->Limit);

    /* 退积分反馈: 输出饱和时把多余积分退回去 */
    float diff = outSat - outUnlim;
    if ( pPID->Kb > 0.0f )
    {
        pPID->Integral += pPID->Kb * diff * dt;
    }

    pPID->PrevErr = error;
    pPID->PrevOut = outSat;
    pPID->Output  = outSat;
    return outSat;
}

/* =============================================================================
 *  Clarke 变换 (abc -> αβ)
 *
 *  Iα = Ia
 *  Iβ = (Ia + 2 * Ib) / √3
 *  前提: Ia + Ib + Ic = 0 (星形连接)
 * ==========================================================================*/
static void prvClarke( float ia, float ib, float *pAlpha, float *pBeta )
{
    *pAlpha = ia;
    *pBeta  = (ia + 2.0f * ib) * INV_SQRT3;
}

/* =============================================================================
 *  Park 变换 (αβ -> dq)
 *
 *  d 轴(磁场方向) — 励磁分量, 控制为 0
 *  q 轴(切线方向) — 转矩分量, 输出力矩
 * ==========================================================================*/
static void prvPark( float alpha, float beta, float theta,
                     float *pId, float *pIq )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *pId =  alpha * c + beta * s;
    *pIq = -alpha * s + beta * c;
}

/* 逆 Park 变换 (dq -> αβ) */
static void prvInvPark( float vd, float vq, float theta,
                        float *pValpha, float *pVbeta )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *pValpha = vd * c - vq * s;
    *pVbeta  = vd * s + vq * c;
}

/* =============================================================================
 *  SVPWM — 扇区判定 + T1/T2 + 比较值
 *
 *  流程:
 *    1. calc_sector: 由 Vα, Vβ 判定扇区 1~6
 *    2. calc_T1_T2:  计算相邻两个有效矢量作用时间
 *    3. 过调制处理:  T1 + T2 > Ts 时等比缩小
 *    4. 转换为定时器比较值 (中心对齐模式)
 *    5. sector_to_cmp: 按扇区分配三相比较值
 *    6. 写入 TIM8 CCR
 * ==========================================================================*/

/* 判定扇区: 利用三个参考值的符号编码 */
static uint8_t prvCalcSector( float valpha, float vbeta )
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
static void prvCalcT1T2( float va, float vb, uint8_t sector,
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
static void prvSectorToCmp( uint8_t sector, uint16_t c1, uint16_t c2, uint16_t c3,
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
static void prvSVPWM( float valpha, float vbeta, uint32_t *duty )
{
    float vdc = BLDC_Info.PowerVoltage;

    /* 母线电压异常时使用安全默认值，避免除零 */
    if ( vdc < 1.0f )
    {
        vdc = 24.0f;
    }

    /* 中心对齐模式 PWM 周期 [s] */
    float ts = 2.0f * (float)FOC_PWM_PERIOD / FOC_TIM_CLK_HZ;

    /* 1. 判定扇区 */
    uint8_t sector = prvCalcSector(valpha, vbeta);

    /* 2. 计算 T1, T2 */
    float t1 = 0.0f;
    float t2 = 0.0f;
    prvCalcT1T2(valpha, vbeta, sector, ts, vdc, &t1, &t2);

    /* 3. 过调制限制 */
    if ( (t1 + t2) > ts )
    {
        float k = ts / (t1 + t2);
        t1 *= k;
        t2 *= k;
    }

    /* 4. 中心对齐模式比较值 */
    float t0 = ts - t1 - t2;
    uint16_t c1 = (uint16_t)(t0 * 0.25f * FOC_TIM_CLK_HZ);
    uint16_t c2 = c1 + (uint16_t)(t1 * 0.5f * FOC_TIM_CLK_HZ);
    uint16_t c3 = c2 + (uint16_t)(t2 * 0.5f * FOC_TIM_CLK_HZ);

    /* 钳位到 ARR 范围 */
    if ( c1 > FOC_PWM_PERIOD ) c1 = FOC_PWM_PERIOD;
    if ( c2 > FOC_PWM_PERIOD ) c2 = FOC_PWM_PERIOD;
    if ( c3 > FOC_PWM_PERIOD ) c3 = FOC_PWM_PERIOD;

    /* 5. 按扇区分配三相比较值 */
    uint16_t ta = 0;
    uint16_t tb = 0;
    uint16_t tc = 0;
    prvSectorToCmp(sector, c1, c2, c3, &ta, &tb, &tc);

    /* 6. 输出 */
    duty[0] = ta;
    duty[1] = tb;
    duty[2] = tc;
}

/* =============================================================================
 *  位置检测 — 3 路霍尔 + 扇区内线性插值
 *
 *  Hall 跳变间隔为 60 电角度, 中断里记录:
 *    HallStepCount     — 累计扇区索引
 *    HallSectorStartMs — 进入当前扇区时刻
 *    HallSectorPeriodMs— 上一个扇区用时
 *
 *  每次控制周期:
 *    progress = (now - SectorStart) / SectorPeriod
 *    theta_elec = (HallStepCount + progress) * 60°
 *    机械角 = (HallStepCount + progress) * 30°
 *
 *  优点: 上电即可知绝对扇区, 无需对齐; 扇区内线性插值得到连续角度
 *  缺点: 低速时插值误差大, 位置分辨率仅 60° 电角度
 * ==========================================================================*/
static void prvHallPositionUpdate( void )
{
    uint32_t nowMs = SystemRunTime_1ms;
    float progress = 0.0f;

    /* 扇区用时: HallTickCnt 为 1MHz 计数, 换算为 ms */
    float periodMs = (float)Hall_Info.HallTickCnt / 1000.0f;

    if ( periodMs > 1.0f )
    {
        uint32_t elapsed = nowMs - Hall_Info.HallLastEdgeMs;
        progress = (float)elapsed / periodMs;
        progress = prvClampf(progress, 0.0f, 1.0f);
    }

    /* 连续机械角: 每个 Hall 步进 = 30° 机械角 */
    float mechDeg = ((float)BLDC_Info.HallStepCount + progress)
                  * BLDC_MECH_DEG_PER_SECTOR;
    FOC_Info.PositionDeg = mechDeg;

    /* 连续电角度 = 机械角 * 极对数 */
    float thetaDeg = mechDeg * (float)BLDC_POLE_PAIRS;
    FOC_Info.ThetaElec_Rad = thetaDeg * DEG_TO_RAD;

    /* 归一化 [0, 2*PI) */
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
 *  三相电流读取
 *
 *  ADC3: CH4(PF6)=U, CH5(PF7)=V, CH6(PF8)=W
 *  采样电阻 20mΩ, 运放增益 8x
 *  I_mA = (V_adc - V_offset) / (0.02 * 8) * 1000
 * ==========================================================================*/
static void prvReadPhaseCurrents( void )
{
    float rawU, rawV, rawW;

    /* ADC 原始值 -> 电压 */
    float vU = ((float)gADC3CaptureBuffer[BLDC_U_Current] * 3.3f) / 4095.0f;
    float vV = ((float)gADC3CaptureBuffer[BLDC_V_Current] * 3.3f) / 4095.0f;
    float vW = ((float)gADC3CaptureBuffer[BLDC_W_Current] * 3.3f) / 4095.0f;

    /* 去偏置 -> 电流 [mA] */
    rawU = (vU - FOC_Info.Iu_OffsetV) / (8.0f * 0.02f) * 1000.0f;
    rawV = (vV - FOC_Info.Iv_OffsetV) / (8.0f * 0.02f) * 1000.0f;
    rawW = (vW - FOC_Info.Iw_OffsetV) / (8.0f * 0.02f) * 1000.0f;

    /* 一阶低通滤波，抑制 PWM 开关噪声 */
    FOC_Info.Iu_mA = 0.8f * FOC_Info.Iu_mA + 0.2f * rawU;
    FOC_Info.Iv_mA = 0.8f * FOC_Info.Iv_mA + 0.2f * rawV;
    FOC_Info.Iw_mA = 0.8f * FOC_Info.Iw_mA + 0.2f * rawW;
}

/* 电流偏置校准: 电机静止时采样 200 次取平均 */
static void prvCalibrateOffsets( void )
{
    uint32_t sumU = 0;
    uint32_t sumV = 0;
    uint32_t sumW = 0;

    for ( uint32_t i = 0; i < 200U; i++ )
    {
        sumU += gADC3CaptureBuffer[BLDC_U_Current];
        sumV += gADC3CaptureBuffer[BLDC_V_Current];
        sumW += gADC3CaptureBuffer[BLDC_W_Current];
    }

    FOC_Info.Iu_OffsetV = ((float)sumU / 200.0f) * 3.3f / 4095.0f;
    FOC_Info.Iv_OffsetV = ((float)sumV / 200.0f) * 3.3f / 4095.0f;
    FOC_Info.Iw_OffsetV = ((float)sumW / 200.0f) * 3.3f / 4095.0f;

    FOC_Info.Iu_mA = 0.0f;
    FOC_Info.Iv_mA = 0.0f;
    FOC_Info.Iw_mA = 0.0f;

    FOC_Info.Calibrated = 1U;
}

/* =============================================================================
 *  TIM8 FOC 模式重配置
 *  六步换向 -> FOC:
 *    1. 计数模式: UP -> CENTERALIGNED1
 *    2. 互补输出: 使能 CH1N/CH2N/CH3N (PH13/14/15, AF3)
 *    3. 死区时间: FOC_DEADTIME_CYCLES
 * ==========================================================================*/
static void prvTimerFOCConfig( void )
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

    /* 重新初始化 TIM8 -> 中心对齐 */
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

static void prvPWMStart( void )
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

static void prvPWMStop( void )
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
static void prvPositionLoop( void )
{
    float rpmRef = prvPID_Calc(&FOC_Info.PID_Position,
                               FOC_Info.PositionRef_Deg,
                               FOC_Info.PositionDeg);
    FOC_Info.SpeedRef_RPM = prvClampf(rpmRef,
                                      -FOC_POSITION_LIMIT_RPM,
                                      FOC_POSITION_LIMIT_RPM);
}

/* 速度环: 输出 q 轴电流目标 (100Hz / 10ms) */
static void prvSpeedLoop( void )
{
    float iqRef = prvPID_Calc(&FOC_Info.PID_Speed,
                              FOC_Info.SpeedRef_RPM,
                              FOC_Info.SpeedRPM);
    FOC_Info.Iq_Ref_mA = prvClampf(iqRef,
                                   -FOC_SPEED_LIMIT_mA,
                                   FOC_SPEED_LIMIT_mA);
    FOC_Info.Id_Ref_mA = 0.0f;   /* Id = 0 控制 */
}

/* 电流环: 输出 dq 电压 (1kHz / 1ms) */
static void prvCurrentLoop( void )
{
    float vd = prvPID_Calc(&FOC_Info.PID_Id,
                           FOC_Info.Id_Ref_mA,
                           FOC_Info.Id_mA);
    float vq = prvPID_Calc(&FOC_Info.PID_Iq,
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
    /* 初始化三个环路 PID */
    prvPID_Init(&FOC_Info.PID_Id, FOC_KP_CURRENT, FOC_KI_CURRENT, 0.0f,
                FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA * 0.5f, 0.5f, FOC_CTRL_PERIOD_S);
    prvPID_Init(&FOC_Info.PID_Iq, FOC_KP_CURRENT, FOC_KI_CURRENT, 0.0f,
                FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA * 0.5f, 0.5f, FOC_CTRL_PERIOD_S);
    prvPID_Init(&FOC_Info.PID_Speed, FOC_KP_SPEED, FOC_KI_SPEED, 0.0f,
                FOC_SPEED_LIMIT_mA, FOC_SPEED_LIMIT_mA * 0.5f, 0.5f,
                (float)FOC_SPEED_LOOP_MS * 0.001f);
    prvPID_Init(&FOC_Info.PID_Position, FOC_KP_POSITION, FOC_KI_POSITION, FOC_KD_POSITION,
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

    /* 校准电流零点 (电机必须静止) */
    prvCalibrateOffsets();

    /* 读取初始 Hall 状态 -> 初始电角度 */
    uint8_t hall = Hall_GetState();
    if ( hall >= 1U && hall <= 6U )
    {
        static const float hallCenterDeg[8] = {
            0.0f,  30.0f,  90.0f, 150.0f,
            210.0f, 270.0f, 330.0f, 0.0f
        };
        FOC_Info.ThetaElec_Rad = hallCenterDeg[hall] * DEG_TO_RAD;
    }
}

void FOC_Enable( void )
{
    if ( !FOC_Info.Calibrated )
    {
        return;     /* 未校准, 拒绝启动 */
    }

    prvTimerFOCConfig();
    prvPWMStart();

    FOC_Info.Enabled = 1U;
}

void FOC_Disable( void )
{
    FOC_Info.Enabled = 0U;

    prvPWMStop();
    prvPID_Reset(&FOC_Info.PID_Id);
    prvPID_Reset(&FOC_Info.PID_Iq);
    prvPID_Reset(&FOC_Info.PID_Speed);
    prvPID_Reset(&FOC_Info.PID_Position);
}

/* =============================================================================
 *  FOC_Update() — 1kHz 控制主循环
 *
 *  1. 读取三相电流
 *  2. Hall 扇区线性插值, 更新位置/速度反馈
 *  3. Clarke 变换
 *  4. Park 变换 (d=磁场方向 Id=0, q=切线方向力矩)
 *  5. 按计数分频执行位置环/速度环
 *  6. 电流环 -> Vd/Vq
 *  7. 逆 Park + SVPWM
 *  8. 更新 TIM8 CCR
 * ==========================================================================*/
void FOC_Update( void )
{
    float dt = FOC_CTRL_PERIOD_S;

    if ( !FOC_Info.Enabled )
    {
        return;
    }

    FOC_Info.LoopCounter++;

    /* ---- 1. 读取三相电流 ---- */
    prvReadPhaseCurrents();

    /* ---- 2. Hall 位置插值 ---- */
    prvHallPositionUpdate();

    float ia = FOC_Info.Iu_mA;
    float ib = FOC_Info.Iv_mA;
    float theta = FOC_Info.ThetaElec_Rad;

    /* ---- 3. Clarke: abc -> αβ ---- */
    prvClarke(ia, ib, &FOC_Info.Ialpha_mA, &FOC_Info.Ibeta_mA);

    /* ---- 4. Park: αβ -> dq ---- */
    /*  d 轴 — 磁场方向, 控制 Id=0; q 轴 — 切线方向, 输出力矩 */
    prvPark(FOC_Info.Ialpha_mA, FOC_Info.Ibeta_mA, theta,
            &FOC_Info.Id_mA, &FOC_Info.Iq_mA);

    /* ---- 5. 外环: 位置环(100ms) -> 速度环(10ms) ---- */
    if ( FOC_Info.LoopMode == FOC_LOOP_POSITION &&
         (FOC_Info.LoopCounter % FOC_POSITION_LOOP_MS) == 0U )
    {
        prvPositionLoop();
    }

    if ( FOC_Info.LoopMode == FOC_LOOP_SPEED &&
         (FOC_Info.LoopCounter % FOC_SPEED_LOOP_MS) == 0U )
    {
        prvSpeedLoop();
    }

    if ( FOC_Info.LoopMode == FOC_LOOP_POSITION &&
         (FOC_Info.LoopCounter % FOC_SPEED_LOOP_MS) == 0U )
    {
        prvSpeedLoop();
    }

    /* ---- 6. 电流环 ---- */
    prvCurrentLoop();

    /* ---- 7. 电压归一化 + 逆 Park ---- */
    float vdc = BLDC_Info.PowerVoltage;
    if ( vdc < 1.0f )
    {
        vdc = 24.0f;
    }

    float vdNorm = FOC_Info.Vd / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
    float vqNorm = FOC_Info.Vq / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
    vdNorm = prvClampf(vdNorm, -1.0f, 1.0f);
    vqNorm = prvClampf(vqNorm, -1.0f, 1.0f);

    /* 归一化电压 -> 实际电压 [V] */
    float vdReal = vdNorm * vdc * 0.5f;
    float vqReal = vqNorm * vdc * 0.5f;

    prvInvPark(vdReal, vqReal, theta,
               &FOC_Info.Valpha, &FOC_Info.Vbeta);

    /* ---- 8. SVPWM ---- */
    uint32_t duty[3];
    prvSVPWM(FOC_Info.Valpha, FOC_Info.Vbeta, duty);

    /* ---- 9. 更新 TIM8 CCR ---- */
    TIM8->CCR1 = duty[0];
    TIM8->CCR2 = duty[1];
    TIM8->CCR3 = duty[2];
}

/* =============================================================================
 *  目标给定 API
 * ==========================================================================*/

void FOC_SetIqRef( float iq_mA )
{
    FOC_Info.Iq_Ref_mA = prvClampf(iq_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
    FOC_Info.LoopMode = FOC_LOOP_CURRENT;
}

void FOC_SetIdRef( float id_mA )
{
    FOC_Info.Id_Ref_mA = prvClampf(id_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
}

float FOC_GetIqRef( void )
{
    return FOC_Info.Iq_Ref_mA;
}

float FOC_GetIdRef( void )
{
    return FOC_Info.Id_Ref_mA;
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
