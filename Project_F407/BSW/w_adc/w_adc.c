/* Includes ------------------------------------------------------------------*/
#include "w_adc.h"
#include "beep.h"
#include <math.h>

/* local constants -----------------------------------------------------------*/
#define ADC_REF_VOLTAGE         (3.3f)           /* ADC参考电压。 */
#define ADC_MAX_COUNTS          (4095.0f)        /* 12位ADC满量程计数。 */
#define ADC1_VBUS_SCALE         (37.0f)          /* BDC/BLDC母线电压分压比例。 */
#define ADC1_CURR_GAIN          (8.0f * 0.02f)   /* BDC电流采样增益：放大倍数 * 分流电阻。 */
#define ADC3_CURR_GAIN          (8.0f * 0.02f)   /* BLDC相电流采样增益：放大倍数 * 分流电阻。 */
#define ADC_VBUS_OFFSET_V       (1.24f)          /* 母线电压采样通道的硬件偏置。 */
#define ADC_TEMP_PULLUP_OHM     (4700.0f)        /* NTC上拉电阻阻值。 */
#define ADC_TEMP_R25_OHM        (10000.0f)       /* NTC在25摄氏度时的阻值。 */
#define ADC_TEMP_BETA           (3950.0f)        /* NTC的Beta常数。 */

/* local helpers -------------------------------------------------------------*/
static float prvADC_RawToVolt( uint16_t raw )
{
	return ((float)raw * ADC_REF_VOLTAGE) / ADC_MAX_COUNTS;
}

static void prvBDC_ADCCollects( BDC_Info_t *pBDC )
{
	float vbus_v = prvADC_RawToVolt(gADC1CaptureBuffer[BDC_PowerVoltage]);
	float curr_v = prvADC_RawToVolt(gADC1CaptureBuffer[BDC_MotorCurrent]);
	float curr_mA = (curr_v - pBDC->CurrZeroOffsetV) / ADC1_CURR_GAIN * 1000.0f;

	/* 将ADC1原始采样值换算为母线电压和电机电流。 */
	pBDC->PowerVoltage = (vbus_v - ADC_VBUS_OFFSET_V) * ADC1_VBUS_SCALE;

	/* 做一次一阶低通，避免BDC电流反馈抖动过大。 */
	pBDC->CurrFilt = 0.9f * pBDC->CurrFilt + 0.1f * curr_mA;
	if ( pBDC->CurrFilt < 0.0f )
	{
		pBDC->CurrFilt = 0.0f;
	}
	pBDC->CurrentRealTime = pBDC->CurrFilt;
}

static float prvBLDC_TemperatureCal( void )
{
	float v_temp = prvADC_RawToVolt(gADC3CaptureBuffer[BLDC_MotorTemperature]);

	/* 根据NTC分压和Beta模型换算电机温度。 */
	float r_temp = (ADC_REF_VOLTAGE - v_temp) / (v_temp / ADC_TEMP_PULLUP_OHM);
	float t25 = 273.15f + 25.0f;
	float ka = 273.15f;

	return ADC_TEMP_BETA * t25 / (ADC_TEMP_BETA + logf(r_temp / ADC_TEMP_R25_OHM) * t25) - ka;
}

static void prvBLDC_ADCCollects( BLDC_Info_t *pBLDC )
{
	float vbus_v = prvADC_RawToVolt(gADC3CaptureBuffer[BLDC_PowerVoltage]);

	/* 这里把母线电压和三相电流原始值统一换算成工程量。 */
	pBLDC->PowerVoltage = (vbus_v - ADC_VBUS_OFFSET_V) * ADC1_VBUS_SCALE;

	pBLDC->CurrentPhase.U_PhaseCurrent =
		(prvADC_RawToVolt(gADC3CaptureBuffer[BLDC_U_Current]) - pBLDC->CurrZeroOffsetV.U_PhaseSetV) / ADC3_CURR_GAIN * 1000.0f;
	pBLDC->CurrentPhase.V_PhaseCurrent =
		(prvADC_RawToVolt(gADC3CaptureBuffer[BLDC_V_Current]) - pBLDC->CurrZeroOffsetV.V_PhaseSetV) / ADC3_CURR_GAIN * 1000.0f;
	pBLDC->CurrentPhase.W_PhaseCurrent =
		(prvADC_RawToVolt(gADC3CaptureBuffer[BLDC_W_Current]) - pBLDC->CurrZeroOffsetV.W_PhaseSetV) / ADC3_CURR_GAIN * 1000.0f;

	/* 相电流先滤波，再送入BLDC控制环路使用。 */
	pBLDC->CurrFilt.U_CurrFilt = 0.9f * pBLDC->CurrFilt.U_CurrFilt + 0.1f * pBLDC->CurrentPhase.U_PhaseCurrent;
	pBLDC->CurrFilt.V_CurrFilt = 0.9f * pBLDC->CurrFilt.V_CurrFilt + 0.1f * pBLDC->CurrentPhase.V_PhaseCurrent;
	pBLDC->CurrFilt.W_CurrFilt = 0.9f * pBLDC->CurrFilt.W_CurrFilt + 0.1f * pBLDC->CurrentPhase.W_PhaseCurrent;

	pBLDC->CurrentPhase.U_PhaseCurrent = pBLDC->CurrFilt.U_CurrFilt;
	pBLDC->CurrentPhase.V_PhaseCurrent = pBLDC->CurrFilt.V_CurrFilt;
	pBLDC->CurrentPhase.W_PhaseCurrent = pBLDC->CurrFilt.W_CurrFilt;
}

/* public functions ----------------------------------------------------------*/
void Motor_CurrentOffsetCalibrate( BDC_Info_t *pBDC, BLDC_Info_t *pBLDC )
{
	uint32_t sum_bdc = 0;
	uint32_t sum_bldc[3] = {0};

	/* 电机静止时做多次平均，得到各电流通道的零点偏置。 */
	BEEP_ON;
	for ( uint8_t i = 0; i < 200; i++ )
	{
		sum_bdc += gADC1CaptureBuffer[BDC_MotorCurrent];
		sum_bldc[0] += gADC3CaptureBuffer[BLDC_U_Current];
		sum_bldc[1] += gADC3CaptureBuffer[BLDC_V_Current];
		sum_bldc[2] += gADC3CaptureBuffer[BLDC_W_Current];
		HAL_Delay(2);
	}
	BEEP_OFF;

	pBDC->CurrZeroOffsetV = ((float)sum_bdc / 200.0f) * ADC_REF_VOLTAGE / ADC_MAX_COUNTS;
	pBDC->CurrFilt = 0.0f;
	pBDC->CurrentRealTime = 0.0f;

	pBLDC->CurrZeroOffsetV.U_PhaseSetV = ((float)sum_bldc[0] / 200.0f) * ADC_REF_VOLTAGE / ADC_MAX_COUNTS;
	pBLDC->CurrZeroOffsetV.V_PhaseSetV = ((float)sum_bldc[1] / 200.0f) * ADC_REF_VOLTAGE / ADC_MAX_COUNTS;
	pBLDC->CurrZeroOffsetV.W_PhaseSetV = ((float)sum_bldc[2] / 200.0f) * ADC_REF_VOLTAGE / ADC_MAX_COUNTS;
	pBLDC->CurrFilt.U_CurrFilt = 0.0f;
	pBLDC->CurrFilt.V_CurrFilt = 0.0f;
	pBLDC->CurrFilt.W_CurrFilt = 0.0f;
}

void ADC_Cyclic( void )
{
	/* 周期刷新BDC/BLDC的电压、电流和温度测量值。 */
	prvBDC_ADCCollects(&BDC_Info);
	prvBLDC_ADCCollects(&BLDC_Info);
	BLDC_Info.MotorTemperature = prvBLDC_TemperatureCal();
}
