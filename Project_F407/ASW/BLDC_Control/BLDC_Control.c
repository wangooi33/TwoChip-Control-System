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
	PID_Init(&d_pid,1.0f,0.5f,0,(24.0f * SQRT3 / 3.0f),0,0.0001f);
	PID_Init(&q_pid,1.0f,0.5f,0,(24.0f * SQRT3 / 3.0f),0,0.0001f);
	
	PID_Init(&speed_pid,0.03f,0.02f,0,4.0f,0.5f,0.001f);
	FOC_Info.Speed_Ref = 400.0f;
}

#if 0
void BLDC_SpeedPID(void)
{
	float error,out;
	static uint8_t speedInit = 0;
	static float rampRef = 0.0f;
	float speed_rpm = Hall_Info.Speed_RPM;
	
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
#endif

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
static void FOC_Run(void)
{
	BLDC_Info.Theta = Hall_Info.angle;
	
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
void BLDC_Run(void)
{
	uint8_t hall_state;
	
	/* 三相电流采集 */
	BLDC_PhaseCurrentCal();

	switch (BLDC_Info.MotorRunStage)
	{
		case Motor_Start_Idle:
			FOC_Info.Id_Ref = 0.0f;
			FOC_Info.Iq_Ref = 0.0f;
			BLDC_Info.MotorRunStage = Motor_Start_CheckHall;
			break;
			
		case Motor_Start_CheckHall:
			hall_state = Hall_ReadState();
			if (hall_state == 0 || hall_state == 7)
			{
				BLDC_Info.MotorRunStage = Motor_Stop;
			}
			else
			{
				Hall_Info.state = hall_state;
				BLDC_Info.MotorRunStage = Motor_Start_HallValid;
			}
			break;
			
		case Motor_Start_HallValid:
			/* hall中心角: 60°区间 + 30° */
			Hall_Info.hall_angle = hall_angle_table[Hall_Info.state] + PI / 6.0f;
			Hall_Info.hall_angle = Angle_Normalize(Hall_Info.hall_angle);
			/* 初始电角度 */
			Hall_Info.angle = Hall_Info.hall_angle;
			BLDC_Info.MotorRunStage = Motor_Start_Run;
			break;
			
		case Motor_Start_Run:
			FOC_Info.Id_Ref = 0.0f;
			FOC_Info.Iq_Ref = 1.0f;

			/* 启动阶段开环推进电角度 */
			Hall_Info.angle += 0.005f;
			if (Hall_Info.angle >= TWO_PI)
			{
				Hall_Info.angle -= TWO_PI;
			}
			
			if (Hall_Info.new_event == 1)
			{
				Hall_Info.new_event = 0;
				BLDC_Info.MotorRunStage = Motor_Start_Interpolation;
			}
			break;
			
		case Motor_Start_Interpolation:
			/* 获取两次Hall跳变边沿之间的时间间隔 */
			if (Hall_Info.hall_period > 0)
			{
				BLDC_Info.MotorRunStage = Motor_Run;
			}
			break;

		case Motor_Run:
			/* 霍尔角度插值 */
			Hall_Interpolate(0.0001f);
		
//		    Hall_Info.angle += 0.005f;
//			if (Hall_Info.angle >= TWO_PI)
//			{
//				Hall_Info.angle -= TWO_PI;
//			}
			break;
			
		case Motor_Stop:
			FOC_Info.Id_Ref = 0.0f;
			FOC_Info.Iq_Ref = 0.0f;
			BLDC_Disable();
			break;
		
		default:
			BLDC_Info.MotorRunStage = Motor_Start_Idle;
			break;
	}
	
	FOC_Run();
}

