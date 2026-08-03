/* =============================================================================
 *  w_adc.c — 母线电压 / 三相电流 / 温度采样链路
 *
 *  采样结果直接写入 BLDC_Info, 不再维护重复的采样结构。
 * ==========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "w_adc.h"
#include "beep.h"
#include <math.h>

/* =============================================================================
 *  ADC 原始值 -> 电流 [A]
 * ==========================================================================*/
float ADC_to_Current( uint16_t raw, float offsetV )
{
    float ampVoltage = (float)raw * ADC_V_PER_LSB;
    float ampSignal  = ampVoltage - offsetV;
    float current    = ampSignal / (ADC_AMP_GAIN * ADC_R_SHUNT);

    return current;
}

/* ADC 原始值 -> 母线电压 [V] */
float ADC_to_BusVoltage( uint16_t raw )
{
    float vbus  = (float)raw * ADC_V_PER_LSB;
    float power = vbus * ADC_DIV_RATIO;

    return power;
}

/* ADC 原始值 -> NTC 阻值 [Ohm] */
float ADC_to_Rt( uint16_t raw )
{
    float vTemp = (float)raw * ADC_V_PER_LSB;

    /* 开路保护 */
    if ( vTemp < 0.001f )
    {
        return 999999.0f;
    }

    return ADC_TEMP_R_FIXED * (ADC_REF_V / vTemp - 1.0f);
}

/* NTC 阻值 -> 温度 [C] */
float Rt_to_Temperature( float rt )
{
    /* 短路保护 */
    if ( rt <= 0.0f )
    {
        return -273.15f;
    }

    float ratio   = rt / ADC_TEMP_R0;
    float lnRatio = logf(ratio);
    float invT1   = lnRatio / ADC_TEMP_BETA + 1.0f / ADC_TEMP_T2;

    if ( invT1 <= 0.0f )
    {
        return -273.15f;
    }

    return 1.0f / invT1 - 273.15f;
}

/* 三相电流采样入口: 写入 BLDC_Info (mA) */
void ReadPhaseCurrents( void )
{
    float offsetU = BLDC_Info.CurrZeroOffsetV.U_PhaseSetV;
    float offsetV = BLDC_Info.CurrZeroOffsetV.V_PhaseSetV;
    float offsetW = BLDC_Info.CurrZeroOffsetV.W_PhaseSetV;

    BLDC_Info.CurrentPhase.U_PhaseCurrent =
        ADC_to_Current(gADC3CaptureBuffer[BLDC_U_Current], offsetU) * 1000.0f;
    BLDC_Info.CurrentPhase.V_PhaseCurrent =
        ADC_to_Current(gADC3CaptureBuffer[BLDC_V_Current], offsetV) * 1000.0f;
    BLDC_Info.CurrentPhase.W_PhaseCurrent =
        ADC_to_Current(gADC3CaptureBuffer[BLDC_W_Current], offsetW) * 1000.0f;

    /* 一阶低通滤波 */
    BLDC_Info.CurrFilt.U_CurrFilt = 0.9f * BLDC_Info.CurrFilt.U_CurrFilt
                                  + 0.1f * BLDC_Info.CurrentPhase.U_PhaseCurrent;
    BLDC_Info.CurrFilt.V_CurrFilt = 0.9f * BLDC_Info.CurrFilt.V_CurrFilt
                                  + 0.1f * BLDC_Info.CurrentPhase.V_PhaseCurrent;
    BLDC_Info.CurrFilt.W_CurrFilt = 0.9f * BLDC_Info.CurrFilt.W_CurrFilt
                                  + 0.1f * BLDC_Info.CurrentPhase.W_PhaseCurrent;

    BLDC_Info.CurrentPhase.U_PhaseCurrent = BLDC_Info.CurrFilt.U_CurrFilt;
    BLDC_Info.CurrentPhase.V_PhaseCurrent = BLDC_Info.CurrFilt.V_CurrFilt;
    BLDC_Info.CurrentPhase.W_PhaseCurrent = BLDC_Info.CurrFilt.W_CurrFilt;
}

/* 母线电压采样入口 */
void ReadBusVoltage( void )
{
    BLDC_Info.PowerVoltage = ADC_to_BusVoltage(gADC3CaptureBuffer[BLDC_PowerVoltage]);
}

/* 温度采样入口 */
void ReadTemperature( void )
{
    float rt    = ADC_to_Rt(gADC3CaptureBuffer[BLDC_MotorTemperature]);
    float tempC = Rt_to_Temperature(rt);

    BLDC_Info.MotorTemperature = tempC;
}

/* 过压/欠压保护 */
uint8_t CheckBusFault( void )
{
    if ( BLDC_Info.PowerVoltage > ADC_VBUS_OV_THRESH ||
         BLDC_Info.PowerVoltage < ADC_VBUS_UV_THRESH )
    {
        BLDC_TripStop();
        return 1U;
    }
    return 0U;
}

/* 过温保护 */
uint8_t CheckTempFault( void )
{
    if ( BLDC_Info.MotorTemperature >= ADC_TEMP_OT_THRESH )
    {
        BLDC_TripStop();
        return 1U;
    }
    return 0U;
}

/* SVPWM 模块取母线电压 (T1/T2 计算需要 Vdc) */
float GetBusVoltage( void )
{
    return BLDC_Info.PowerVoltage;
}

/* Clarke 变换: 三相电流 -> 两相正交电流 */
void Clarke( float iu, float iv, float *pAlpha, float *pBeta )
{
    *pAlpha = iu;
    *pBeta  = (iu + 2.0f * iv) * 0.57735f;
}

/* 电流零点校准: 电机静止时采样 200 次取平均 */
void Motor_CurrentOffsetCalibrate( BLDC_Info_t *pBLDC )
{
    uint32_t sumU = 0;
    uint32_t sumV = 0;
    uint32_t sumW = 0;

    BEEP_ON;
    for ( uint8_t i = 0; i < 200; i++ )
    {
        sumU += gADC3CaptureBuffer[BLDC_U_Current];
        sumV += gADC3CaptureBuffer[BLDC_V_Current];
        sumW += gADC3CaptureBuffer[BLDC_W_Current];
        HAL_Delay(2);
    }
    BEEP_OFF;

    pBLDC->CurrZeroOffsetV.U_PhaseSetV = ((float)sumU / 200.0f) * ADC_V_PER_LSB;
    pBLDC->CurrZeroOffsetV.V_PhaseSetV = ((float)sumV / 200.0f) * ADC_V_PER_LSB;
    pBLDC->CurrZeroOffsetV.W_PhaseSetV = ((float)sumW / 200.0f) * ADC_V_PER_LSB;
    pBLDC->CurrFilt.U_CurrFilt = 0.0f;
    pBLDC->CurrFilt.V_CurrFilt = 0.0f;
    pBLDC->CurrFilt.W_CurrFilt = 0.0f;
}

/* ADC 周期任务: 采样 -> 换算 -> 保护 */
void ADC_Cyclic( void )
{
    ReadPhaseCurrents();
    ReadBusVoltage();
    ReadTemperature();

    CheckBusFault();
    CheckTempFault();
}
