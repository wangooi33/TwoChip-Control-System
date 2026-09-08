/* includes ------------------------------------------------------------------*/
#include "bldc_control.h"
#include <math.h>
#include "tim.h"
#include "w_adc.h"
#include "foc.h"
#include "pid.h"
#include "hall.h"

/* macro ---------------------------------------------------------------------*/
#define SPEED_RAMP_RPM_PER_MS		(0.05f)		/* 每1ms变化0.05RPM */
#define SPEED_PID_LIMIT				(3.0f)

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
	
	PID_Init(&speed_pid,0.01f,0.005f,0,SPEED_PID_LIMIT,0.0f,0.001f);
	BLDC_SetSpeedRef(400.0f);
}

void BLDC_SetSpeedRef(float speedRpm)
{
	FOC_Info.Speed_Ref = Clampf(speedRpm, -BLDC_MAX_SPEED_RPM, BLDC_MAX_SPEED_RPM);
}
float BLDC_GetSpeedRef(void)
{
	return FOC_Info.Speed_Ref;
}

void BLDC_SetPositionRef(float turns)
{
	FOC_Info.Position_Ref = turns;
}

float BLDC_GetCurrentTurns(void)
{
	return Hall_Info.position_turns;
}

uint8_t BLDC_PositionPID(void)
{
	static uint8_t active = 0;					/* 位置环是否在进行控制 */
	static float lastRef = 0.0f;				/* 上一次执行位置移动时使用的Position_Ref */
	static float startSteps = 0.0f;				/* 起点霍尔步数 */
	static float commandedElectricalDeg = 0.0f;	/* 已经命令推进了多少电角度 */
	static float stepElectricalDeg = BLDC_POSITION_START_ANGLE_STEP * (180.0f / PI);/* 位置环推进的电角度增量 */
	static float oneHallTurn = 1.0f / (6.0f * (float)BLDC_POLE_PAIRS);/* 一个霍尔扇区对应的机械圈数 */
	float targetTurns;			/* 目标机械圈数 */
	float targetSteps;			/* 目标机械圈数对应的目标霍尔步数 */
	float traveledSteps;		/* 从本次移动起点开始,实际已经走过了多少个霍尔步 */
	float direction;
	float speedRpm;
	
	/* 目标位置只取幅值,运动方向由原始给定值的符号决定 */
	targetTurns = fabsf(FOC_Info.Position_Ref);
	if (targetTurns < 0.0001f)
	{
		/* 无有效位置指令退出 */
		active = 0;
		lastRef = 0.0f;
		return 0;
	}

	/* 外部修改了位置环目标,需要等待当前目标执行完毕 */
	if (active == 0 || FOC_Info.Position_Ref != lastRef)
	{
		/* 首次进入或指令变化时,以当前霍尔步数作为本次行程起点 */
		active = 1;
		lastRef = FOC_Info.Position_Ref;
		startSteps = (float)Hall_Info.hall_step_count;
		commandedElectricalDeg = 0.0f;
	}

	/* 把目标圈数转化为霍尔步数:一对极对应6个霍尔沿,因此目标步数 = 圈数 * 极对数 * 6 */
	targetSteps = targetTurns * (6.0f * (float)BLDC_POLE_PAIRS);
	traveledSteps = fabsf((float)Hall_Info.hall_step_count - startSteps);
	if (traveledSteps >= targetSteps)
	{
		/* 已走完目标步数,停止输出并清除位置指令 */
		FOC_Info.Iq_Ref = 0.0f;
		FOC_Info.Position_Ref = 0.0f;
		lastRef = 0.0f;
		active = 0;
		return 0;
	}

	direction = (FOC_Info.Position_Ref > 0.0f) ? 1.0f : -1.0f;
	/* 将滤波后的电角速度折算为带符号机械转速 */
	speedRpm = Hall_Info.speed_filter * 60.0f / (2.0f * PI * (float)BLDC_POLE_PAIRS);
	BLDC_Info.RPM = speedRpm;
	if (fabsf(speedRpm) > BLDC_POSITION_MAX_RPM)
	{
		/* 超过定位限速时停止输出，避免位置环持续加速 */
		FOC_Info.Iq_Ref = 0.0f;
	}
	else
	{
		/* 未超速时用固定q轴电流推动电机 */
		FOC_Info.Iq_Ref = BLDC_POSITION_CURRENT;
	}

	/* 开环推进FOC的转子电角度 */
	Hall_Info.angle = Angle_Normalize(Hall_Info.angle + direction * BLDC_POSITION_START_ANGLE_STEP);
	commandedElectricalDeg += stepElectricalDeg;
	if (targetTurns < oneHallTurn && commandedElectricalDeg >= targetTurns * (float)BLDC_POLE_PAIRS * 360.0f)
	{
		/* 不足一个霍尔扇区的短行程,用累计推进电角度判断是否到位 */
		FOC_Info.Iq_Ref = 0.0f;
		FOC_Info.Position_Ref = 0.0f;
		lastRef = 0.0f;
		active = 0;
		return 0;
	}

	return 1;
}

void BLDC_SpeedPID(void)
{
	static uint8_t speedLoopInited = 0;
	static int8_t speedLoopDirection = 0;	/* 上一次速度环使用的方向 */
	static float rampRef = 0.0f;			/* 斜坡目标速度 */
	float speedRpmSigned;	/* 带符号的实际机械转速,单位RPM */
	float speedRpm;			/* 速度环内部使用的转速绝对值,单位RPM */
	float speedRefAbs;		/* Speed_Ref的绝对值,单位RPM */
	float error;			/* 速度误差:给定值 - 实际值 */
	float outUnlimited;		/* 限幅前的PI输出电流 */
	float out;				/* 限幅后的输出电流 */
	float outMin;			/* 输出电流下限 */
	float outMax;			/* 输出电流上限 */
	int8_t targetDirection;	/* Speed_Ref 当前要求的方向 */
	uint8_t integrate;		/* 当前周期是否允许更新积分项 */

	if (BLDC_Info.MotorRunStage != Motor_Run)
	{
		speedLoopInited = 0;
		return;
	}

	/* 霍尔滤波值是电角速度,先转换为机械RPM */
	speedRpmSigned = Hall_Info.speed_filter * 60.0f / (2.0f * PI * (float)BLDC_POLE_PAIRS);

	/* 速度环统一使用绝对值,方向只由Speed_Ref的符号决定。 */
	speedRpm = fabsf(speedRpmSigned);
	speedRefAbs = fabsf(FOC_Info.Speed_Ref);
	targetDirection = (FOC_Info.Speed_Ref < 0.0f) ? -1 : 1;

	/* 方向改变时,重新走开环启动 */
	if (speedLoopDirection != 0 && speedLoopDirection != targetDirection)
	{
		speedLoopDirection = targetDirection;
		speedLoopInited = 0;
		FOC_Info.Iq_Ref = 0.0f;
		BLDC_Info.MotorRunStage = Motor_Start_Idle;
		return;
	}
	speedLoopDirection = targetDirection;
	/* 首次进入运行态时,斜坡从当前实际转速开始,避免给定阶跃。 */
	if (speedLoopInited == 0)
	{
		speedLoopInited = 1;
		rampRef = speedRpm;
		speed_pid.Integral = (speedRpm < speedRefAbs) ? FOC_Info.Iq_Ref : 0.0f;
		speed_pid.PrevErr = 0.0f;
	}

	/* 更新外部观测转速 */
	BLDC_Info.RPM = speedRpmSigned;
	/* 速度斜坡增量 */
	if (rampRef < speedRefAbs)
	{
		rampRef += SPEED_RAMP_RPM_PER_MS;
		if (rampRef > speedRefAbs)
		{
			rampRef = speedRefAbs;
		}
	}
	else if (rampRef > speedRefAbs)
	{
		rampRef -= SPEED_RAMP_RPM_PER_MS;
		if (rampRef < speedRefAbs)
		{
			rampRef = speedRefAbs;
		}
	}

	/* 计算速度误差,并输出限幅前的比例 + 积分结果。 */
	error = rampRef - speedRpm;
	outUnlimited = speed_pid.Kp * error + speed_pid.Integral;
	outMin = 0.0f;
	outMax = SPEED_PID_LIMIT;
	out = Clampf(outUnlimited, outMin, outMax);
	/* 输出限幅 */
	integrate = ((outUnlimited > outMin) && (outUnlimited < outMax))
				|| ((error > 0.0f) && (outUnlimited <= outMin))
				|| ((error < 0.0f) && (outUnlimited >= outMax))
				|| ((error < 0.0f) && (outUnlimited <= outMin));
	if (integrate != 0)
	{
		/* 抗积分饱和 */
		speed_pid.Integral += speed_pid.Ki * error * speed_pid.Ts;
	}
	speed_pid.Integral = Clampf(speed_pid.Integral, outMin, outMax);
	speed_pid.PrevErr = error;

	/* 超速时输出为零,速度环不主动输出负电流减速电机 */
	FOC_Info.Iq_Ref = out;
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
	float startupDirection;
	
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
			/* 每次启动都从干净的霍尔状态开始,方向由Speed_Ref决定。 */
			Hall_Info.initialized = 0;
			Hall_Info.direction = 0;
			Hall_Info.pending_valid = 0;
			Hall_Info.pending_direction = 0;
			Hall_Info.pending_period = 0;
			Hall_Info.speed = 0.0f;
			Hall_Info.speed_filter = 0.0f;
			Hall_Info.speed_filter_init = 0;
			Hall_Info.hall_period = 0;
			Hall_Info.new_event = 0;
			/* hall中心角: 60°区间 + 30° */
			Hall_Info.hall_angle = hall_angle_table[Hall_Info.state] + PI / 6.0f;
			Hall_Info.hall_angle = Angle_Normalize(Hall_Info.hall_angle);
			/* 初始电角度 */
			Hall_Info.angle = Hall_Info.hall_angle;
			BLDC_Info.MotorRunStage = Motor_Start_Run;
			break;
			
		case Motor_Start_Run:
			FOC_Info.Id_Ref = 0.0f;
			startupDirection = (FOC_Info.Speed_Ref < 0.0f) ? -1.0f : 1.0f;
			FOC_Info.Iq_Ref = 1.0f;

			/* 启动阶段开环推进电角度 */
			Hall_Info.angle = Angle_Normalize(Hall_Info.angle + startupDirection * 0.005f);
			
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

