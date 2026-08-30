/* includes ------------------------------------------------------------------*/
#include "bldc_control.h"
#include "tim.h"
#include "w_adc.h"
#include "foc.h"
#include "pid.h"
#include "hall.h"

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;
PID_t d_pid;
PID_t q_pid;

float theta;
uint16_t cnt;

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
void BLDC_PidInit(void)
{
	PID_Init(&d_pid,2.0f,0.5f,0,5,0,0.0001f);
	PID_Init(&q_pid,3.0f,0.5f,0,5,0,0.0001f);
}
void BLDC_Run(void)
{
	/* 霍尔角度插值 */
	Hall_Info.angle += Hall_Info.angle_inc;
	if (Hall_Info.angle > 2 * PI)
	{
		Hall_Info.angle -= 2 * PI;
	}
	else if (Hall_Info.angle < 0)
	{
		Hall_Info.angle += 2 * PI;
	}

	if (cnt < 10000)
	{
		cnt++;
		/* 开环角度自增 */
		if (BLDC_Info.Direction == 1)
		{
			theta += 0.005f;
		}
		else
		{
			theta -= 0.005f; /* 逆时针 */
		}
		if (theta > 2 * PI)
		{
			theta -= 2 * PI;
		}
		else if (theta < 0)
		{
			theta += 2 * PI;
		}
		BLDC_Info.Theta = theta;
	}
	else
	{
		BLDC_Info.Theta = Hall_Info.angle;
	}

	BLDC_PhaseCurrentCal();
	
	Clark(BLDC_Info.PhaseCurrent[0],BLDC_Info.PhaseCurrent[1],&FOC_Info.Ialpha,&FOC_Info.Ibeta);
	Park(FOC_Info.Ialpha,FOC_Info.Ibeta,BLDC_Info.Theta,&FOC_Info.Id,&FOC_Info.Iq);

	/* 电流环 */
	FOC_Info.Vd = PID_Update(&d_pid,FOC_Info.Id_Ref,FOC_Info.Id);
	FOC_Info.Vq = PID_Update(&q_pid,FOC_Info.Iq_Ref,FOC_Info.Iq);

	RevPark(FOC_Info.Vd,FOC_Info.Vq,BLDC_Info.Theta,&FOC_Info.Valpha,&FOC_Info.Vbeta);
	SVPWM(FOC_Info.Valpha,FOC_Info.Vbeta,24.0f, (8400.0f * 2.0f),&FOC_Info.Tcm1,&FOC_Info.Tcm2,&FOC_Info.Tcm3);

	TIM1->CCR1 = FOC_Info.Tcm1;
	TIM1->CCR2 = FOC_Info.Tcm2;
	TIM1->CCR3 = FOC_Info.Tcm3;
}

