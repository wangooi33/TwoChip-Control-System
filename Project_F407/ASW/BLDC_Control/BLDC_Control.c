/* includes ------------------------------------------------------------------*/
#include "bldc_control.h"
#include <math.h>
#include "tim.h"
#include "w_adc.h"
#include "foc.h"
#include "pid.h"
#include "hall.h"

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;
PID_t d_pid;
PID_t q_pid;
PID_t speed_pid;

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

	BLDC_Info.Direction = 1;
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
	PID_Init(&d_pid,1.0f,0.5f,0,(24.0f * SQRT3 / 3.0f),0,0.0001f);
	PID_Init(&q_pid,1.0f,0.5f,0,(24.0f * SQRT3 / 3.0f),0,0.0001f);
	
	PID_Init(&speed_pid,0.03f,0.02f,0,4.0f,0.5f,0.001f);
	
	FOC_Info.Speed_Ref = 400.0f;
}
void BLDC_SpeedPID(void)
{
	static uint8_t speedInit = 0;
	static float rampRef = 0.0f;

	float speed_rpm = Hall_Info.Speed_Filter / BLDC_POLE_PAIRS * 60.0f / (2.0f * PI);

	float error;
	float out;
	
	BLDC_Info.RPM = speed_rpm;
	if (BLDC_Info.Direction == 1)
	{
		speed_rpm = speed_rpm;
	}
	else
	{
		speed_rpm = -speed_rpm;
	}
	if (speedInit == 0)
	{
		rampRef = speed_rpm;
		speed_pid.Integral = FOC_Info.Iq_Ref;
		speed_pid.PrevErr = 0.0f;
		speedInit = 1;
	}

	/* 约 20 RPM/s 的斜坡 */
	if (rampRef > FOC_Info.Speed_Ref)
	{
		rampRef -= 0.02f;
		if (rampRef < FOC_Info.Speed_Ref)
		{
			rampRef = FOC_Info.Speed_Ref;
		}
	}
	else if (rampRef < FOC_Info.Speed_Ref)
	{
		rampRef += 0.02f;
		if (rampRef > FOC_Info.Speed_Ref)
		{
			rampRef = FOC_Info.Speed_Ref;
		}
    }
	error = rampRef - speed_rpm;
	out = speed_pid.Kp * error + speed_pid.Integral;

	/* 只有未饱和时才积分，防止反向制动时积分越积越负 */
	if (out > -speed_pid.Limit && out < speed_pid.Limit)
	{
		speed_pid.Integral += speed_pid.Ki * error * speed_pid.Ts;
	}
	speed_pid.Integral = Clampf(speed_pid.Integral, -speed_pid.Limit, speed_pid.Limit);

	/* 限幅 */
	FOC_Info.Iq_Ref = Clampf(out,-speed_pid.Limit,speed_pid.Limit);
}

void BLDC_CurrentPID(void)
{
	float Udc,Uref_max,Uref,scale;

	FOC_Info.Vd = PID_Update(&d_pid,FOC_Info.Id_Ref,FOC_Info.Id);
	FOC_Info.Vq = PID_Update(&q_pid,FOC_Info.Iq_Ref,FOC_Info.Iq);
	
	/* 电压矢量限幅 */
	Udc = 24.0f;
	Uref_max = Udc * SQRT3 / 3.0f;
	Uref = sqrtf(FOC_Info.Vd * FOC_Info.Vd + FOC_Info.Vq * FOC_Info.Vq);
	if (Uref > Uref_max)
	{
		scale = Uref_max / Uref;
		FOC_Info.Vd *= scale;
		FOC_Info.Vq *= scale;
	}
}

void BLDC_Run(void)
{
	/* 霍尔角度插值 */
	if (BLDC_Info.Direction == 1)
	{
		Hall_Info.angle += Hall_Info.angle_inc;
	}
	else
	{
		Hall_Info.angle -= Hall_Info.angle_inc;
	}
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
		Hall_Info.ClosedLoop_Flag = 1;
		BLDC_Info.Theta = Hall_Info.angle;
	}

	/* 三相电流采集 */
	BLDC_PhaseCurrentCal();
	
	Clark(BLDC_Info.PhaseCurrent[0],BLDC_Info.PhaseCurrent[1],&FOC_Info.Ialpha,&FOC_Info.Ibeta);
	Park(FOC_Info.Ialpha,FOC_Info.Ibeta,BLDC_Info.Theta,&FOC_Info.Id,&FOC_Info.Iq);

	/* 电流环 */
	BLDC_CurrentPID();

	RevPark(FOC_Info.Vd,FOC_Info.Vq,BLDC_Info.Theta,&FOC_Info.Valpha,&FOC_Info.Vbeta);
	SVPWM(FOC_Info.Valpha,FOC_Info.Vbeta,24.0f,(8400.0f * 2.0f),&FOC_Info.Tcm1,&FOC_Info.Tcm2,&FOC_Info.Tcm3);

	TIM1->CCR1 = FOC_Info.Tcm1;
	TIM1->CCR2 = FOC_Info.Tcm2;
	TIM1->CCR3 = FOC_Info.Tcm3;
}

