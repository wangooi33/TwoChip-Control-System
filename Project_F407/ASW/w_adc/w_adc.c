/* Includes ------------------------------------------------------------------*/
#include "w_adc.h"
#include <math.h>

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
void Motor_CurrentOffsetCalibrate(BLDC_Info_t *pBLDC)
{
    uint32_t sumU = 0;
    uint32_t sumV = 0;
    uint32_t sumW = 0;

    for (uint8_t i = 0; i < 200; i++)
    {

    }

    pBLDC->ZeroOffset[0] = ADC_TO_VOLT(sumU);
    pBLDC->ZeroOffset[1] = ADC_TO_VOLT(sumV);
    pBLDC->ZeroOffset[2] = ADC_TO_VOLT(sumW);
}
void BLDC_PhaseCurrentCal(void)
{
	BLDC_Info.PhaseCurrent[0] = ((ADC_TO_VOLT(ADC3->JDR1) - 1.24f)) * 37.0f;
	BLDC_Info.PhaseCurrent[1] = ((ADC_TO_VOLT(ADC2->JDR1) - 1.24f)) * 37.0f;
	BLDC_Info.PhaseCurrent[2] = ((ADC_TO_VOLT(ADC1->JDR1) - 1.24f)) * 37.0f;
}
void BLDC_TemperatureCal(void)
{
	float Vntc = ADC_TO_VOLT(gADC1CaptureBuffer[1]);
	float Rt = 4700.0f * (3.3f - Vntc) / Vntc;
	BLDC_Info.MotorTemperature = 1.0f / (1.0f / 298.15f + logf(Rt / 10000.0f) / 3950.0f) - 273.15f;
}
void BLDC_VBusCal(void)
{
	BLDC_Info.Power = (ADC_TO_VOLT(gADC1CaptureBuffer[0])- 1.24f) * 37.0f;
}

