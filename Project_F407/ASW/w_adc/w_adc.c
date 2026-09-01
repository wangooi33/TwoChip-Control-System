/* Includes ------------------------------------------------------------------*/
#include "w_adc.h"
#include <math.h>
#include "adc.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/
#define CURRENT_FILTER_ALPHA		0.5f
#define VBUS_FILTER_ALPHA			0.1f
#define TEMP_FILTER_ALPHA			0.05f

/* public functions ----------------------------------------------------------*/
void ADC_Enable(void)
{
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_ADCEx_InjectedStart(&hadc3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)gADC1CaptureBuffer,ADC1_CAPTURE_BUF_MAXSIZE);
}
/* 计算零点偏置 */
void Motor_CurrentOffsetCal(BLDC_Info_t *pBLDC)
{
	static uint16_t wCount = 0;
	static uint64_t sumU = 0;
	static uint64_t sumV = 0;
	static uint64_t sumW = 0;

	sumU += ADC3->JDR1;
	sumV += ADC2->JDR1;
	sumW += ADC1->JDR1;
	if (++wCount >= 500)
	{
		pBLDC->ZeroOffsetADC[0] = (uint16_t)(sumU / 500);
		pBLDC->ZeroOffsetADC[1] = (uint16_t)(sumV / 500);
		pBLDC->ZeroOffsetADC[2] = (uint16_t)(sumW / 500);
		BLDC_Info.ZeroOffsetFlag = 1;
	}
}
void BLDC_PhaseCurrentCal(void)
{
	static uint8_t filterInit = 0;
	int32_t adcU,adcV,adcW;
	float currentU,currentV,currentW;
	
	adcU = (int32_t)ADC3->JDR1 - (int32_t)BLDC_Info.ZeroOffsetADC[0];
	adcV = (int32_t)ADC2->JDR1 - (int32_t)BLDC_Info.ZeroOffsetADC[1];
	adcW = (int32_t)ADC1->JDR1 - (int32_t)BLDC_Info.ZeroOffsetADC[2];
	currentU = (float)adcU * 3.3f / 4095.0f / 8.0f / 0.02f;
	currentV = (float)adcV * 3.3f / 4095.0f / 8.0f / 0.02f;
	currentW = (float)adcW * 3.3f / 4095.0f / 8.0f / 0.02f;

	if (filterInit == 0)
	{
		BLDC_Info.PhaseCurrent[0] = currentU;
		BLDC_Info.PhaseCurrent[1] = currentV;
		BLDC_Info.PhaseCurrent[2] = currentW;
		filterInit = 1;
	}
	else
	{
		/* 一阶低通滤波 */
		BLDC_Info.PhaseCurrent[0] += CURRENT_FILTER_ALPHA * (currentU - BLDC_Info.PhaseCurrent[0]);
		BLDC_Info.PhaseCurrent[1] += CURRENT_FILTER_ALPHA * (currentV - BLDC_Info.PhaseCurrent[1]);
		BLDC_Info.PhaseCurrent[2] += CURRENT_FILTER_ALPHA * (currentW - BLDC_Info.PhaseCurrent[2]);
	}
}
void BLDC_TemperatureCal(void)
{
	static uint8_t filterInit = 0;
	static float filterADC = 0;
	float adcValue;
	float Vntc;
	float Rt;

	adcValue = (float)gADC1CaptureBuffer[1];

	if (filterInit == 0)
	{
		filterADC = adcValue;
		filterInit = 1;
	}
	else
	{
		filterADC += TEMP_FILTER_ALPHA * (adcValue - filterADC);
	}

	Vntc = filterADC * 3.3f / 4095.0f;
	Rt = 4700.0f * (3.3f - Vntc) / Vntc;
	BLDC_Info.MotorTemperature = 1.0f / (1.0f / 298.15f + logf(Rt / 10000.0f) / 3950.0f) - 273.15f;
}
void BLDC_VBusCal(void)
{
	static uint8_t filterInit = 0;
	float currentVBUS;

	currentVBUS = (gADC1CaptureBuffer[0] * 3.3f / 4095.0f- 1.24f) * 37.0f;
	if (filterInit == 0)
	{
		BLDC_Info.Power = currentVBUS;
		filterInit = 1;
	}
	else
	{
		BLDC_Info.Power += VBUS_FILTER_ALPHA * (currentVBUS - BLDC_Info.Power);
	}
}

