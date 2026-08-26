/* includes ------------------------------------------------------------------*/
#include "bldc_control.h"
#include "w_adc.h"

/* annotation ----------------------------------------------------------------*/

//死区时间 = 2 *(31 + 8) + 20 = 108

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;

/* public functions ----------------------------------------------------------*/
void BLDC_Enable(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	BLDC_SD_ENABLE();
}
void BLDC_Disable(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

	BLDC_SD_DISABLE();
}
float Valpha,Vbeta,Tcm1,Tcm2,Tcm3;

void BLDC_Run(void)
{
	static float theta;

	theta += 0.005f;
	if (theta > 2 * PI)
		theta -= 2 * PI;
	else if (theta < 0)
		theta += 2 * PI;

	RevPark(0,1,theta,&Valpha,&Vbeta);
	SVPWM(Valpha,Vbeta,24.0f, 20000,&Tcm1,&Tcm2,&Tcm3);

	TIM1->CCR1 = Tcm1;
	TIM1->CCR2 = Tcm2;
	TIM1->CCR3 = Tcm3;
}

