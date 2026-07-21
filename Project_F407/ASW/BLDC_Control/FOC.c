/* includes ------------------------------------------------------------------*/
#include "FOC.h"
#include "BLDC_Control.h"
#include "adc.h"
#include "tim.h"

FOC_Info_t FOC_Info;


static float prvClampf( float value, float min, float max )
{
    if ( value < min ) return min;
    if ( value > max ) return max;
    return value;
}
static void prvPI_Init( FOC_PI_t *pPI, float Kp, float Ki, float limit )
{
    pPI->Kp       = Kp;
    pPI->Ki       = Ki;
    pPI->Integral = 0.0f;
    pPI->Output   = 0.0f;
    pPI->Limit    = limit;
}

static void prvPI_Reset( FOC_PI_t *pPI )
{
    pPI->Integral = 0.0f;
    pPI->Output   = 0.0f;
}

static float prvPI_Calc( FOC_PI_t *pPI, float error, float dt )
{
    float p_term  = pPI->Kp * error;
    pPI->Integral += pPI->Ki * error * dt;
    pPI->Integral  = prvClampf(pPI->Integral, -pPI->Limit, pPI->Limit);

    float out = p_term + pPI->Integral;
    out = prvClampf(out, -pPI->Limit, pPI->Limit);

    /* 输出饱和时撤销本次积分累加 */
    if ( out >= pPI->Limit || out <= -pPI->Limit )
    {
        pPI->Integral -= pPI->Ki * error * dt;
    }

    pPI->Output = out;
    return out;
}

/* =============================================================================
 *  Clarke 变换 (abc → αβ)
 *
 *  三相静止 → 两相静止
 *    Iα = Ia
 *    Iβ = (Ia + 2 * Ib) / √3
 *  前提: Ia + Ib + Ic = 0 (星形连接)
 *
 *  输入: Ia, Ib (mA)
 *  输出: *p_alpha = Iα, *p_beta = Iβ
 * ==========================================================================*/
static void prvClarke( float ia, float ib, float *p_alpha, float *p_beta )
{
    *p_alpha = ia;
    *p_beta  = (ia + 2.0f * ib) * INV_SQRT3;
}

/* =============================================================================
 *  Park 变换 (αβ → dq, 旋转坐标系)
 *
 *  两相静止 → 两相旋转 (随转子同步旋转)
 *    Id  =  Iα * cos(θ) + Iβ * sin(θ)   ← 磁场方向 (励磁, 控制为 0)
 *    Iq  = -Iα * sin(θ) + Iβ * cos(θ)   ← 切线方向 (转矩, 输出力矩)
 *
 *  输入: Iα, Iβ, θ (电角度 rad)
 *  输出: *p_id = Id, *p_iq = Iq
 * ==========================================================================*/
static void prvPark( float i_alpha, float i_beta, float theta,
                     float *p_id, float *p_iq )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *p_id =  i_alpha * c + i_beta * s;
    *p_iq = -i_alpha * s + i_beta * c;
}

/* =============================================================================
 *  逆 Park 变换 (dq → αβ)
 *
 *  旋转坐标系 → 静止坐标系 (合成电压矢量)
 *    Vα = Vd * cos(θ) - Vq * sin(θ)
 *    Vβ = Vd * sin(θ) + Vq * cos(θ)
 *
 *  输入: Vd, Vq (归一化 [-1, 1]), θ
 *  输出: *p_valpha, *p_vbeta
 * ==========================================================================*/
static void prvInvPark( float vd, float vq, float theta,
                        float *p_valpha, float *p_vbeta )
{
    float c = cosf(theta);
    float s = sinf(theta);
    *p_valpha = vd * c - vq * s;
    *p_vbeta  = vd * s + vq * c;
}

/* =============================================================================
 *  SVPWM (Space Vector PWM) — 三相标量 max-min 注入法
 *
 *  无需查扇区表, 计算简洁:
 *    1. 反 Clarke: (Vα, Vβ) → (Va, Vb, Vc)
 *    2. 计算共模分量 Vcom = (Vmax + Vmin) / 2
 *    3. 注入共模: dA/B/C = Vx - Vcom + 0.5
 *    4. 钳位到 [0, 1]
 *    5. 转换为 TIM8 计数值
 *
 *  输入: Valpha, Vbeta (归一化 [-1, 1])
 *  输出: duty[0..2] (TIM8 CCR 计数值)
 * ==========================================================================*/
static void prvSVPWM( float Valpha, float Vbeta, uint32_t *duty )
{
    /* 反 Clarke: αβ → 三相电压 */
    float Va =  Vbeta;
    float Vb = -0.5f * Vbeta + 0.86602540378f * Valpha;    /* √3/2 */
    float Vc = -0.5f * Vbeta - 0.86602540378f * Valpha;

    /* 共模注入: 叠加零序分量 */
    float Vmax = fmaxf(fmaxf(Va, Vb), Vc);
    float Vmin = fminf(fminf(Va, Vb), Vc);
    float Vcom = 0.5f * (Vmax + Vmin);

    float dA = (Va - Vcom) + 0.5f;
    float dB = (Vb - Vcom) + 0.5f;
    float dC = (Vc - Vcom) + 0.5f;

    /* 钳位 */
    dA = prvClampf(dA, 0.0f, 1.0f);
    dB = prvClampf(dB, 0.0f, 1.0f);
    dC = prvClampf(dC, 0.0f, 1.0f);

    /* 转换为定时器计数值 */
    duty[0] = (uint32_t)(dA * (float)FOC_PWM_PERIOD);
    duty[1] = (uint32_t)(dB * (float)FOC_PWM_PERIOD);
    duty[2] = (uint32_t)(dC * (float)FOC_PWM_PERIOD);
}

/* =============================================================================
 *  Hall-PLL 位置观测器
 *
 *  霍尔每转只给出 6 个离散位置 (60 度分辨率), 直接用于 FOC
 *  会导致转矩脉动。PLL 在两次 Hall 跳变之间插值出连续电角度。
 *
 *  Hall 状态 → 扇区中心角度表:
 *    Hall=1 →  30°   |  Hall=2 →  90°
 *    Hall=3 → 150°   |  Hall=4 → 210°
 *    Hall=5 → 270°   |  Hall=6 → 330°
 *
 *  PLL 更新:
 *    θ_meas = hall_center_deg[Hall]
 *    err = sin(θ_est - θ_meas)     ← sin 避免 0/360 跳变
 *    ω += Ki * err * dt            ← 积分锁定转速
 *    θ += (Kp * err + ω) * dt      ← 推算连续角度
 * ==========================================================================*/
static void prvHallPLLObserver( float dt )
{
    static const float hall_center_deg[8] = {
        0.0f,  30.0f,  90.0f, 150.0f,
        210.0f, 270.0f, 330.0f, 0.0f
    };

    uint8_t hall = Hall_GetState();
    float theta_meas_rad;

    /* 只处理有效 Hall 状态 */
    if ( hall >= 1U && hall <= 6U )
    {
        theta_meas_rad = hall_center_deg[hall] * DEG_TO_RAD;
    }
    else
    {
        return;
    }

    /* ---- PLL 更新 ---- */
    float err = sinf(FOC_Info.ThetaElec_Rad - theta_meas_rad);

    /* 角速度积分 */
    FOC_Info.OmegaElec_RadPs += FOC_PLL_KI * err * dt;
    FOC_Info.OmegaElec_RadPs  = prvClampf(FOC_Info.OmegaElec_RadPs,
                                           -10000.0f, 10000.0f);

    /* 角度更新 (比例 + 速度推算) */
    FOC_Info.ThetaElec_Rad += (FOC_PLL_KP * err + FOC_Info.OmegaElec_RadPs) * dt;

    /* 归一化 [0, 2*PI) */
    if ( FOC_Info.ThetaElec_Rad >= 6.2831853f )
        FOC_Info.ThetaElec_Rad -= 6.2831853f;
    if ( FOC_Info.ThetaElec_Rad < 0.0f )
        FOC_Info.ThetaElec_Rad += 6.2831853f;
}

/* =============================================================================
 *  三相电流读取
 *
 *  ADC3 通道映射:
 *    CH4 (PF6) = BLDC_U_Current (0)
 *    CH5 (PF7) = BLDC_V_Current (1)
 *    CH6 (PF8) = BLDC_W_Current (2)
 *
 *  采样电阻: 20 mOhm, 运放增益: 8x
 *  I_mA = (V_adc - V_offset) / (R_shunt * Gain) * 1000
 *        = (V_adc - V_offset) * 6250
 *
 *  一阶低通滤波: 截止频率 ≈ 32 Hz (1 kHz 采样率)
 * ==========================================================================*/
static void prvReadPhaseCurrents( void )
{
    float raw_u, raw_v, raw_w;

    /* ADC 值 → 电压 */
    float v_u = ((float)gADC3CaptureBuffer[BLDC_U_Current] * 3.3f) / 4095.0f;
    float v_v = ((float)gADC3CaptureBuffer[BLDC_V_Current] * 3.3f) / 4095.0f;
    float v_w = ((float)gADC3CaptureBuffer[BLDC_W_Current] * 3.3f) / 4095.0f;

    /* 去偏置 → 电流 (mA) */
    raw_u = (v_u - FOC_Info.Iu_OffsetV) / (8.0f * 0.02f) * 1000.0f;
    raw_v = (v_v - FOC_Info.Iv_OffsetV) / (8.0f * 0.02f) * 1000.0f;
    raw_w = (v_w - FOC_Info.Iw_OffsetV) / (8.0f * 0.02f) * 1000.0f;

    /* 一阶低通滤波 */
    FOC_Info.Iu_mA = 0.8f * FOC_Info.Iu_mA + 0.2f * raw_u;
    FOC_Info.Iv_mA = 0.8f * FOC_Info.Iv_mA + 0.2f * raw_v;
    FOC_Info.Iw_mA = 0.8f * FOC_Info.Iw_mA + 0.2f * raw_w;
}

/* =============================================================================
 *  电流偏置校准
 *
 *  电机静止时, ADC 采样值因运放偏置和 ADC 偏移而不为零。
 *  连续采样 200 次取平均, 记录偏移电压。
 *  后续电流采样始终减去该偏置。
 * ==========================================================================*/
static void prvCalibrateOffsets( void )
{
    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;

    for ( uint32_t i = 0; i < 200; i++ )
    {
        sum_u += gADC3CaptureBuffer[BLDC_U_Current];
        sum_v += gADC3CaptureBuffer[BLDC_V_Current];
        sum_w += gADC3CaptureBuffer[BLDC_W_Current];
    }

    FOC_Info.Iu_OffsetV = ((float)sum_u / 200.0f) * 3.3f / 4095.0f;
    FOC_Info.Iv_OffsetV = ((float)sum_v / 200.0f) * 3.3f / 4095.0f;
    FOC_Info.Iw_OffsetV = ((float)sum_w / 200.0f) * 3.3f / 4095.0f;

    FOC_Info.Iu_mA = 0.0f;
    FOC_Info.Iv_mA = 0.0f;
    FOC_Info.Iw_mA = 0.0f;

    FOC_Info.Calibrated = 1U;
}

/* =============================================================================
 *  TIM8 FOC 模式重配置
 *  六步换向 → FOC 模式:
 *    1. 计数模式: UP → CENTERALIGNED1
 *    2. 互补输出: 使能 CH1N/CH2N/CH3N (PH13/14/15, AF3)
 *    3. 死区时间: 0 → FOC_DEADTIME_CYCLES (约 119 ns)
 *    4. 自动重装载预装载: 使能
 * ==========================================================================*/
static void prvTimerFOCConfig( void )
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOH, &gpio);

    /* 重新初始化 TIM8 → 中心对齐模式 */
    HAL_TIM_PWM_DeInit(&htim8);

    htim8.Instance = TIM8;
    htim8.Init.Prescaler       = 1;
    htim8.Init.CounterMode     = TIM_COUNTERMODE_CENTERALIGNED1;
    htim8.Init.Period          = (uint32_t)FOC_PWM_PERIOD;
    htim8.Init.ClockDivision   = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim8);

    /* 主输出触发 = 更新事件 (用于触发 ADC 采样) */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);


    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = (uint8_t)FOC_DEADTIME_CYCLES;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

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

void FOC_Init( void )
{
    /* 初始化 PI 控制器 */
    prvPI_Init(&FOC_Info.PID_Id, FOC_KP_CURRENT, FOC_KI_CURRENT, FOC_CURRENT_LIMIT_mA);
    prvPI_Init(&FOC_Info.PID_Iq, FOC_KP_CURRENT, FOC_KI_CURRENT, FOC_CURRENT_LIMIT_mA);

    /* 清零状态 */
    FOC_Info.ThetaElec_Rad   = 0.0f;
    FOC_Info.OmegaElec_RadPs = 0.0f;
    FOC_Info.Iu_mA = 0.0f;
    FOC_Info.Iv_mA = 0.0f;
    FOC_Info.Iw_mA = 0.0f;
    FOC_Info.Id_mA = 0.0f;
    FOC_Info.Iq_mA = 0.0f;
    FOC_Info.Id_Ref_mA = 0.0f;
    FOC_Info.Iq_Ref_mA = 0.0f;
    FOC_Info.Calibrated = 0U;
    FOC_Info.Enabled    = 0U;

    /* 校准电流零点 (电机必须静止) */
    prvCalibrateOffsets();

    /* 读取初始 Hall → 初始电角度 */
    uint8_t hall = Hall_GetState();
    if ( hall >= 1U && hall <= 6U )
    {
        static const float hall_center_deg[8] = {
            0.0f,  30.0f,  90.0f, 150.0f,
            210.0f, 270.0f, 330.0f, 0.0f
        };
        FOC_Info.ThetaElec_Rad = hall_center_deg[hall] * DEG_TO_RAD;
    }
}

void FOC_SetIqRef( float iq_mA )
{
    FOC_Info.Iq_Ref_mA = prvClampf(iq_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
}

void FOC_SetIdRef( float id_mA )
{
    FOC_Info.Id_Ref_mA = prvClampf(id_mA, -FOC_CURRENT_LIMIT_mA, FOC_CURRENT_LIMIT_mA);
}

float FOC_GetIqRef( void ) { return FOC_Info.Iq_Ref_mA; }
float FOC_GetIdRef( void ) { return FOC_Info.Id_Ref_mA; }

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
    prvPI_Reset(&FOC_Info.PID_Id);
    prvPI_Reset(&FOC_Info.PID_Iq);
}

/* =============================================================================
 *    1. 读取三相电流 (ADC3 DMA)
 *    2. Hall-PLL 观测器 (估算连续电角度)
 *    3. Clarke 变换: abc → αβ
 *    4. Park 变换: αβ → dq
 *    5. PI 电流控制器: Id_ref-Id → Vd, Iq_ref-Iq → Vq
 *    6. 电压归一化: mA → [-1, 1]
 *    7. 逆 Park 变换: dq → αβ (合成电压矢量)
 *    8. SVPWM: αβ → 三相占空比
 *    9. 更新 TIM8 CCR 寄存器
 * ==========================================================================*/
void FOC_Update( void )
{
    float dt = FOC_CTRL_PERIOD_S;

    if ( !FOC_Info.Enabled )
    {
        return;
    }

    /* ---- 步骤 1: 读取三相电流 ---- */
    prvReadPhaseCurrents();

    /* ---- 步骤 2: Hall-PLL 角度观测 ---- */
    prvHallPLLObserver(dt);

    float ia    = FOC_Info.Iu_mA;
    float ib    = FOC_Info.Iv_mA;
    float theta = FOC_Info.ThetaElec_Rad;

    /* ---- 步骤 3: Clarke 变换: abc → αβ ---- */
    prvClarke(ia, ib, &FOC_Info.Ialpha_mA, &FOC_Info.Ibeta_mA);

    /* ---- 步骤 4: Park 变换: αβ → dq (使用电角度 θ) ---- */
    /*  d 轴 — 沿磁场方向 → 仅励磁, 控制 Id = 0                               */
    /*  q 轴 — 垂直磁场方向 → 切线力矩, 控制 Iq = 转矩目标                      */
    prvPark(FOC_Info.Ialpha_mA, FOC_Info.Ibeta_mA, theta,
            &FOC_Info.Id_mA, &FOC_Info.Iq_mA);

    /* ---- 步骤 5: PI 电流控制器 ---- */
    float vd = prvPI_Calc(&FOC_Info.PID_Id,
                          FOC_Info.Id_Ref_mA - FOC_Info.Id_mA, dt);
    float vq = prvPI_Calc(&FOC_Info.PID_Iq,
                          FOC_Info.Iq_Ref_mA - FOC_Info.Iq_mA, dt);
    FOC_Info.Vd = vd;
    FOC_Info.Vq = vq;

    /* ---- 步骤 6: 电压归一化 mA → [-1, 1] ---- */
    float vd_norm = vd / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
    float vq_norm = vq / FOC_CURRENT_LIMIT_mA * FOC_VOLTAGE_LIMIT;
    vd_norm = prvClampf(vd_norm, -1.0f, 1.0f);
    vq_norm = prvClampf(vq_norm, -1.0f, 1.0f);

    /* ---- 步骤 7: 逆 Park: dq → αβ (使用同一个 θ) ---- */
    prvInvPark(vd_norm, vq_norm, theta,
               &FOC_Info.Valpha, &FOC_Info.Vbeta);

    /* ---- 步骤 8: SVPWM 合成三相电压 ---- */
    uint32_t duty[3];
    prvSVPWM(FOC_Info.Valpha, FOC_Info.Vbeta, duty);

    /* ---- 步骤 9: 更新 TIM8 比较寄存器 ---- */
    TIM8->CCR1 = duty[0];
    TIM8->CCR2 = duty[1];
    TIM8->CCR3 = duty[2];
}
