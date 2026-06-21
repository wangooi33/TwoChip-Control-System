/* Includes ------------------------------------------------------------------*/
#include "w_adc.h"
#include <math.h>

/* global variable -----------------------------------------------------------*/


/* functions declaration -----------------------------------------------------*/

/* functions implementation --------------------------------------------------*/
void Motoer_CurrentOffsetCalibrate( BDC_Info_t *pBDC, BLDC_Info_t *pBLDC )
{
	//电流零偏计算,作为系统初始化的最后一个函数,加入蜂鸣器提示
	uint32_t Sum[4] = {0};
	
	BEEP_ON;
	// 确保电机断电、无电流
	for ( uint8_t i = 0; i < 200; i++ )
	{
		Sum[0] += gADC1CaptureBuffer[BDC_MotorCurrent];
		Sum[1] += gADC3CaptureBuffer[BLDC_U_Current];
		Sum[2] += gADC3CaptureBuffer[BLDC_V_Current];
		Sum[3] += gADC3CaptureBuffer[BLDC_W_Current];
		HAL_Delay(2);
	}
	BEEP_OFF;
	pBDC->CurrZeroOffsetV = ((float)Sum[0] / 200.0f) * 3.3f / 4095.0f;
	pBDC->CurrFilt = 0.0f;

	pBLDC->CurrZeroOffsetV.U_PhaseSetV = ((float)Sum[1] / 200.0f) * 3.3f / 4095.0f;
	pBLDC->CurrFilt.U_CurrFilt = 0.0f;
	
	pBLDC->CurrZeroOffsetV.V_PhaseSetV = ((float)Sum[2] / 200.0f) * 3.3f / 4095.0f;
	pBLDC->CurrFilt.V_CurrFilt = 0.0f;
	
	pBLDC->CurrZeroOffsetV.W_PhaseSetV = ((float)Sum[3] / 200.0f) * 3.3f / 4095.0f;
	pBLDC->CurrFilt.W_CurrFilt = 0.0f;
}
void BDC_ADCCollects( BDC_Info_t *pBDC )
{
	float VBusADC = (float)gADC1CaptureBuffer[BDC_PowerVoltage] * 3.3f / 4095.0f;
	float CurrADC = (float)gADC1CaptureBuffer[BDC_MotorCurrent] * 3.3f / 4095.0f;
	float CurrRaw;

	pBDC->PowerVoltage = (VBusADC - 1.24f) * 37.0f;								//电压
	CurrRaw = (CurrADC - pBDC->CurrZeroOffsetV) / (8.0f * 0.02f) * 1000.0f;		//电流(带零偏)
	pBDC->CurrFilt = 0.9f * pBDC->CurrFilt + 0.1f * CurrRaw;					//一阶低通滤波


	if ( pBDC->CurrFilt < 0 )
	{
		pBDC->CurrFilt = 0;		//单向限幅
	}
	pBDC->CurrentRealTime = pBDC->CurrFilt;
}

static float prvBLDCTemperatureCal( void )
{
	float Temperature = 0;			// 测量温度
	float R_TemperatureSensor = 0;	// 测量电阻
	float Ka = 273.15f;				// 0℃时对应的温度(开尔文)
	float R25 = 10000.0f;			// 25℃电阻值
	float T25 = Ka + 25.0f;			// 25℃时对应的温度(开尔文)
	float B = 3950.0f;				//B-常数:B = ln(R25 / R_TemperatureSensor) / (1 / T – 1 / T25),其中 T = 25 + 273.15

	float V_TemperatureSensor = (float)gADC3CaptureBuffer[BLDC_MotorTemperature] / 4096.0f * 3.3f;
	R_TemperatureSensor = (3.3f - V_TemperatureSensor) / (V_TemperatureSensor / 4700.0f);

	Temperature = B * T25 / (B + log(R_TemperatureSensor / R25) * T25) - Ka;

	return Temperature;
}
void BLDC_ADCCollects( BLDC_Info_t *pBLDC )
{
	float *Zero = &pBLDC->CurrZeroOffsetV.U_PhaseSetV;
	float *Filt = &pBLDC->CurrFilt.U_CurrFilt;
	float *Current = &pBLDC->CurrentPhase.U_PhaseCurrent;

	for (uint8_t i = 0; i < 3; i++)
	{
		float VBus = (float)gADC3CaptureBuffer[BLDC_PowerVoltage] * 3.3f / 4095.0f;
		float Cur  = (float)gADC3CaptureBuffer[i] * 3.3f / 4095.0f;

		pBLDC->PowerVoltage = (VBus - 1.24f) * 37.0f;					//电压
		float Raw = (Cur - Zero[i]) / (8.0f * 0.02f) * 1000.0f;			//电流(带零偏)
		Filt[i] = 0.9f * Filt[i] + 0.1f * Raw;							//一阶低通滤波
		Current[i] = Filt[i];
	}
}
static float prvBLDC_GetPeakCurrent_mA( const BLDC_Info_t *pBLDC )
{
	float IU = fabsf(pBLDC->CurrentPhase.U_PhaseCurrent);
	float IV = fabsf(pBLDC->CurrentPhase.V_PhaseCurrent);
	float IW = fabsf(pBLDC->CurrentPhase.W_PhaseCurrent);

	float Peak = IU;
	if ( IV > Peak )
	{
		Peak = IV;
	}
	if ( IW > Peak )
	{
		Peak = IW;
	}

	return Peak;
}
void BLDC_CurrentLimit( BLDC_Info_t *pBLDC )
{
	float Ipeak = prvBLDC_GetPeakCurrent_mA(pBLDC);

	//硬保护:立即关断
	if (Ipeak >= BLDC_CURRENT_TRIP_mA)
	{
		pBLDC->MotorStalling = 1;
		pBLDC->Pulse = 0;
		BLDC_Disable();
		return;
	}

	//软限流:逐步削减PWM
	if ( Ipeak > BLDC_CURRENT_SOFT_LIMIT_mA )
	{
		float Delta = Ipeak - BLDC_CURRENT_SOFT_LIMIT_mA;
		uint16_t ReductionValue = (uint16_t)(Delta * BLDC_CURRENT_LIMIT_KP);

		if ( ReductionValue < 1U )
		{
			ReductionValue = 1U;
		}
		if ( pBLDC->Pulse > (BLDC_PWM_MIN_DUTY + ReductionValue) )
		{
			pBLDC->Pulse -= ReductionValue;
		}
		else
		{
			pBLDC->Pulse = BLDC_PWM_MIN_DUTY;
		}
	}
	else if ( Ipeak < BLDC_CURRENT_RELEASE_mA )
	{
		//处于安全区域
	}
}

void ADC_Cyclic( void )
{
	BDC_ADCCollects(&BDC_Info);
	BLDC_ADCCollects(&BLDC_Info);
	BLDC_CurrentLimit(&BLDC_Info);
	BLDC_Info.MotorTemperature = prvBLDCTemperatureCal();
}

