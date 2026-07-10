/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"
#include <math.h>

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;
Hall_Info_t Hall_Info =
{
	.HallFirstEdge = 1
};

BLDCMosCom_t gComFwd[8] =
{
	[0] = {PHASE_NONE,	PHASE_NONE},
	[1] = {PHASE_U,		PHASE_W},
	[2] = {PHASE_V,		PHASE_U},
	[3] = {PHASE_V,		PHASE_W},
	[4] = {PHASE_W,		PHASE_V},
	[5] = {PHASE_U,		PHASE_V},
	[6] = {PHASE_W,		PHASE_U},
	[7] = {PHASE_NONE,	PHASE_NONE},
};
BLDCMosCom_t gComRev[8] =
{
	[0] = {PHASE_NONE,	PHASE_NONE},
	[1] = {PHASE_W,		PHASE_U},
	[2] = {PHASE_U,		PHASE_V},
	[3] = {PHASE_W,		PHASE_V},
	[4] = {PHASE_V,		PHASE_W},
	[5] = {PHASE_V,		PHASE_U},
	[6] = {PHASE_U,		PHASE_W},
	[7] = {PHASE_NONE,	PHASE_NONE},
};
BLDCMosCom_t *pHallTable = NULL;

/* local helpers -------------------------------------------------------------*/
static void prvDisableAllMos( void )
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_RESET);
}

static float prvClampf( float value, float min, float max )
{
	if ( value < min )
	{
		return min;
	}
	if ( value > max )
	{
		return max;
	}
	return value;
}

static void prvBLDC_PIDSpeedInit( BLDC_PID_Pos_t *pPID )
{
	pPID->Kp = BLDC_SPEED_PID_KP;
	pPID->Ki = BLDC_SPEED_PID_KI;
	pPID->Kd = BLDC_SPEED_PID_KD;
	pPID->PreError = 0.0f;
	pPID->SumError = 0.0f;
	pPID->Output = 0.0f;
}

static void prvBLDC_PIDCurrentInit( BLDC_PID_Pos_t *pPID )
{
	pPID->Kp = BLDC_CURRENT_PID_KP;
	pPID->Ki = BLDC_CURRENT_PID_KI;
	pPID->Kd = BLDC_CURRENT_PID_KD;
	pPID->PreError = 0.0f;
	pPID->SumError = 0.0f;
	pPID->Output = 0.0f;
}

static void prvBLDC_PIDPositionInit( BLDC_PID_Pos_t *pPID )
{
	pPID->Kp = BLDC_POSITION_PID_KP;
	pPID->Ki = BLDC_POSITION_PID_KI;
	pPID->Kd = BLDC_POSITION_PID_KD;
	pPID->PreError = 0.0f;
	pPID->SumError = 0.0f;
	pPID->Output = 0.0f;
}

static void prvBLDC_PIDReset( BLDC_PID_Pos_t *pPID )
{
	pPID->PreError = 0.0f;
	pPID->SumError = 0.0f;
	pPID->Output = 0.0f;
}

/* 切换控制模式时清空各环路的积分和历史状态。 */
static void prvBLDC_ResetAllLoopState( BLDC_Info_t *pBLDC )
{
	prvBLDC_PIDReset(&pBLDC->PIDPos_SpeedLoop);
	prvBLDC_PIDReset(&pBLDC->PID_CurrentLoop);
	prvBLDC_PIDReset(&pBLDC->PIDPos_PositionLoop);
	pBLDC->ExpectedRPM_Ramp = 0.0f;
}

static void prvBLDC_RampTargetRPM( BLDC_Info_t *pBLDC )
{
	if ( pBLDC->ExpectedRPM_Ramp < pBLDC->ExpectedRPM )
	{
		pBLDC->ExpectedRPM_Ramp += BLDC_RPM_RAMP_STEP;
		if ( pBLDC->ExpectedRPM_Ramp > pBLDC->ExpectedRPM )
		{
			pBLDC->ExpectedRPM_Ramp = pBLDC->ExpectedRPM;
		}
	}
	else if ( pBLDC->ExpectedRPM_Ramp > pBLDC->ExpectedRPM )
	{
		pBLDC->ExpectedRPM_Ramp -= BLDC_RPM_RAMP_STEP;
		if ( pBLDC->ExpectedRPM_Ramp < pBLDC->ExpectedRPM )
		{
			pBLDC->ExpectedRPM_Ramp = pBLDC->ExpectedRPM;
		}
	}
}

/* 取三相电流绝对值中的最大值，作为电流环和保护的反馈量。 */
static float prvBLDC_GetCurrentMagnitude( const BLDC_Info_t *pBLDC )
{
	float iu = fabsf(pBLDC->CurrentPhase.U_PhaseCurrent);
	float iv = fabsf(pBLDC->CurrentPhase.V_PhaseCurrent);
	float iw = fabsf(pBLDC->CurrentPhase.W_PhaseCurrent);
	float peak = iu;

	if ( iv > peak )
	{
		peak = iv;
	}
	if ( iw > peak )
	{
		peak = iw;
	}

	return peak;
}

static float prvBLDC_SpeedPID_Calc( BLDC_Info_t *pBLDC, float expectation, float feedback )
{
	BLDC_PID_Pos_t *pPID = &pBLDC->PIDPos_SpeedLoop;
	float error = expectation - feedback;
	float p = pPID->Kp * error;
	float d = pPID->Kd * (error - pPID->PreError);
	float output;

	pPID->SumError += error;
	output = p + pPID->Ki * pPID->SumError + d;

	if ( output > BLDC_MAX_CUR_TARGET_mA )
	{
		output = BLDC_MAX_CUR_TARGET_mA;
		pPID->SumError -= error;
	}
	else if ( output < 0.0f )
	{
		output = 0.0f;
		pPID->SumError -= error;
	}

	pPID->Output = output;
	pPID->PreError = error;
	return output;
}

static float prvBLDC_CurrentPID_Calc( BLDC_Info_t *pBLDC, float expectation, float feedback )
{
	BLDC_PID_Pos_t *pPID = &pBLDC->PID_CurrentLoop;
	float error = expectation - feedback;
	float output;

	pPID->SumError += error;
	output = pPID->Kp * error + pPID->Ki * pPID->SumError + pPID->Kd * (error - pPID->PreError);

	if ( output > (float)BLDC_PWM_MAX_DUTY )
	{
		output = (float)BLDC_PWM_MAX_DUTY;
		pPID->SumError -= error;
	}
	else if ( output < 0.0f )
	{
		output = 0.0f;
		pPID->SumError -= error;
	}

	pPID->Output = output;
	pPID->PreError = error;
	return output;
}

static float prvBLDC_PositionPID_Calc( BLDC_Info_t *pBLDC, float expectation, float feedback )
{
	BLDC_PID_Pos_t *pPID = &pBLDC->PIDPos_PositionLoop;
	float error = expectation - feedback;
	float p = pPID->Kp * error;
	float d = pPID->Kd * (error - pPID->PreError);
	float temp_output = p + d + pPID->Ki * pPID->SumError;
	float output;

	if ( fabsf(error) <= BLDC_POSITION_DEADBAND_DEG )
	{
		pPID->PreError = error;
		pPID->SumError = 0.0f;
		pPID->Output = 0.0f;
		return 0.0f;
	}

	if ( !((temp_output >= BLDC_MAX_CUR_TARGET_mA && error > 0.0f) ||
			(temp_output <= -BLDC_MAX_CUR_TARGET_mA && error < 0.0f)) )
	{
		pPID->SumError += error;
	}

	output = p + pPID->Ki * pPID->SumError + d;
	output = prvClampf(output, -BLDC_MAX_CUR_TARGET_mA, BLDC_MAX_CUR_TARGET_mA);

	pPID->Output = output;
	pPID->PreError = error;
	return output;
}

static void prvBLDC_ApplyPulse( uint16_t duty )
{
	BLDC_Info.Pulse = duty;
	if ( BLDC_Info.Pulse > BLDC_PWM_MAX_DUTY )
	{
		BLDC_Info.Pulse = BLDC_PWM_MAX_DUTY;
	}
}

static void prvBLDC_UpdateActiveDuty( uint16_t duty )
{
	if ( BLDC_Info.ActivePwmPhase == PHASE_U )
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty);
	}
	else if ( BLDC_Info.ActivePwmPhase == PHASE_V )
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);
	}
	else if ( BLDC_Info.ActivePwmPhase == PHASE_W )
	{
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
	}
}

static uint32_t prvMedian3( uint32_t Data1, uint32_t Median, uint32_t Data3 )
{
	uint32_t TempValue;
	if ( Data1 > Median )
	{
		TempValue = Data1;
		Data1 = Median;
		Median = TempValue;
	}
	if ( Median > Data3 )
	{
		TempValue = Median;
		Median = Data3;
		Data3 = TempValue;
	}
	if ( Data1 > Median )
	{
		TempValue = Data1;
		Data1 = Median;
		Median = TempValue;
	}
	return Median;
}

static uint32_t prvHallPeriodFilter_Update( Hall_Info_t *pHall, uint32_t RawValue )
{
	HallSpeedFilter_t *Filter = &pHall->HallSpeedFilter;
	uint32_t Median = RawValue;

	if ( RawValue == 0U )
	{
		return 0U;
	}

	Filter->HallTickBuf[Filter->Index] = RawValue;
	Filter->Index = (Filter->Index + 1U) % 3U;
	if ( Filter->ValidCnt < 3U )
	{
		Filter->ValidCnt++;
	}

	if ( Filter->Inited == 0U )
	{
		Filter->LastFilter = RawValue;
		Filter->Inited = 1U;
		return RawValue;
	}

	if ( Filter->ValidCnt >= 3U )
	{
		uint8_t i0 = Filter->Index;
		uint8_t i1 = (Filter->Index + 1U) % 3U;
		uint8_t i2 = (Filter->Index + 2U) % 3U;
		Median = prvMedian3(Filter->HallTickBuf[i0], Filter->HallTickBuf[i1], Filter->HallTickBuf[i2]);
	}

	Filter->LastFilter = Filter->LastFilter + ((int32_t)Median - (int32_t)Filter->LastFilter) / 8;
	return Filter->LastFilter;
}

static int8_t prvBLDC_GetHallStepDelta( uint8_t previousHall, uint8_t currentHall )
{
	static const uint8_t hall_forward_seq[6] = {1U, 5U, 4U, 6U, 2U, 3U};
	int8_t previous_index = -1;
	int8_t current_index = -1;
	uint8_t i;

	for ( i = 0U; i < 6U; i++ )
	{
		if ( hall_forward_seq[i] == previousHall )
		{
			previous_index = (int8_t)i;
		}
		if ( hall_forward_seq[i] == currentHall )
		{
			current_index = (int8_t)i;
		}
	}

	if ( previous_index < 0 || current_index < 0 )
	{
		return 0;
	}

	if ( hall_forward_seq[(previous_index + 1) % 6] == currentHall )
	{
		return 1;
	}
	if ( hall_forward_seq[(previous_index + 5) % 6] == currentHall )
	{
		return -1;
	}

	return 0;
}

/* 消费最新一次Hall沿，更新转速估算，再刷新当前换相。 */
static void prvBLDC_HallCyclic( void )
{
	uint8_t hall;
	uint32_t raw_delta;
	uint32_t filt_delta;

	if ( Hall_Info.HallEdgeFlag == 0U )
	{
		return;
	}

	Hall_Info.HallEdgeFlag = 0U;
	hall = Hall_Info.HallStateShadow;
	BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

	raw_delta = Hall_Info.HallTickCnt;
	if ( raw_delta > 0U )
	{
		filt_delta = prvHallPeriodFilter_Update(&Hall_Info, raw_delta);
		if ( filt_delta > 0U )
		{
			BLDC_Info.RPM = 60.0f * (float)BLDC_HALL_TIMER_HZ / ((float)filt_delta * 6.0f * (float)BLDC_POLE_PAIRS);
		}
	}

	if ( hall >= 1U && hall <= 6U &&
		pHallTable[hall].PwmPhase != PHASE_NONE &&
		pHallTable[hall].LowPhase != PHASE_NONE )
	{
		BLDC_ChangeMOSstate(pHallTable[hall].PwmPhase, pHallTable[hall].LowPhase, BLDC_Info.Pulse);
	}
}

/* public functions ----------------------------------------------------------*/
void BLDC_PIDInit( BLDC_Info_t *pBLDC )
{
	prvBLDC_PIDSpeedInit(&pBLDC->PIDPos_SpeedLoop);
	prvBLDC_PIDCurrentInit(&pBLDC->PID_CurrentLoop);
	prvBLDC_PIDPositionInit(&pBLDC->PIDPos_PositionLoop);
}

void BLDC_ResetControlState( BLDC_Info_t *pBLDC )
{
	pBLDC->RPM = 0.0f;
	pBLDC->CurrentMagnitude = 0.0f;
	pBLDC->ExpectedCurrent = 0.0f;
	pBLDC->Pulse = 0U;
	pBLDC->MotorStalling = 0U;
	pBLDC->MotorRunning = 0U;
	pBLDC->ExpectedRPM_Ramp = 0.0f;
	pBLDC->PIDPos_SpeedLoop.PreError = 0.0f;
	pBLDC->PIDPos_SpeedLoop.SumError = 0.0f;
	pBLDC->PIDPos_SpeedLoop.Output = 0.0f;
	pBLDC->PID_CurrentLoop.PreError = 0.0f;
	pBLDC->PID_CurrentLoop.SumError = 0.0f;
	pBLDC->PID_CurrentLoop.Output = 0.0f;
	pBLDC->PIDPos_PositionLoop.PreError = 0.0f;
	pBLDC->PIDPos_PositionLoop.SumError = 0.0f;
	pBLDC->PIDPos_PositionLoop.Output = 0.0f;
	pBLDC->ActivePwmPhase = PHASE_NONE;
	pBLDC->ActiveLowPhase = PHASE_NONE;
}

void BLDC_PositionReset( BLDC_Info_t *pBLDC )
{
	pBLDC->CurrentAngleDeg = 0.0f;
	pBLDC->ExpectedAngleDeg = 0.0f;
	pBLDC->HallStepCount = 0;
	pBLDC->CtrlMode = BLDC_CTRL_SPEED;
	pBLDC->Direction = MOTOR_FWD;
	pBLDC->PositionCmdActive = 0U;
}

void BLDC_SetExpectedRPM( float expectedRPM )
{
	if ( BLDC_Info.CtrlMode != BLDC_CTRL_SPEED )
	{
		prvBLDC_ResetAllLoopState(&BLDC_Info);
	}
	BLDC_Info.ExpectedRPM = prvClampf(expectedRPM, 0.0f, BLDC_MAX_RPM_TARGET);
	BLDC_Info.CtrlMode = BLDC_CTRL_SPEED;
	BLDC_Info.PositionCmdActive = 0U;
}

float BLDC_GetExpectedRPM( void )
{
	return BLDC_Info.ExpectedRPM;
}

void BLDC_SetExpectedCurrent( float expectedCurrent )
{
	if ( BLDC_Info.CtrlMode != BLDC_CTRL_CURRENT )
	{
		prvBLDC_ResetAllLoopState(&BLDC_Info);
	}
	BLDC_Info.ExpectedCurrent = prvClampf(expectedCurrent, 0.0f, BLDC_MAX_CUR_TARGET_mA);
	BLDC_Info.CtrlMode = BLDC_CTRL_CURRENT;
	BLDC_Info.PositionCmdActive = 0U;
}

float BLDC_GetExpectedCurrent( void )
{
	return BLDC_Info.ExpectedCurrent;
}

void BLDC_SetExpectedAngle( float expectedAngleDeg )
{
	if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION )
	{
		prvBLDC_ResetAllLoopState(&BLDC_Info);
	}
	BLDC_Info.ExpectedAngleDeg = expectedAngleDeg;
	BLDC_Info.CtrlMode = BLDC_CTRL_POSITION;
	BLDC_Info.PositionCmdActive = 1U;
}

void BLDC_AddExpectedAngle( float deltaAngleDeg )
{
	float base_angle = BLDC_Info.ExpectedAngleDeg;

	/* 如果当前位置环未激活，则以当前反馈角度作为新的增量起点。 */
	if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION || BLDC_Info.PositionCmdActive == 0U )
	{
		base_angle = BLDC_Info.CurrentAngleDeg;
	}

	BLDC_SetExpectedAngle(base_angle + deltaAngleDeg);
}

float BLDC_GetExpectedAngle( void )
{
	return BLDC_Info.ExpectedAngleDeg;
}

float BLDC_GetCurrentAngle( void )
{
	return BLDC_Info.CurrentAngleDeg;
}

void BLDC_PositionTask( void )
{
	if ( BLDC_Info.PositionCmdActive == 0U )
	{
		return;
	}
}

void BLDC_Disable( void )
{
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	prvDisableAllMos();
	BLDC_SD_DISABLE();
}

void BLDC_Enable( void )
{
	BLDC_SD_ENABLE();
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

/* 重新启动功率级，但保留当前外环目标量不丢失。 */
void BLDC_Start( void )
{
	uint16_t start_pulse = BLDC_Info.Pulse;
	float expected_rpm = BLDC_Info.ExpectedRPM;
	float expected_current = BLDC_Info.ExpectedCurrent;
	float expected_angle = BLDC_Info.ExpectedAngleDeg;
	BLDC_CtrlMode_t ctrl_mode = BLDC_Info.CtrlMode;
	MotorDir_t direction = BLDC_Info.Direction;
	uint8_t position_cmd_active = BLDC_Info.PositionCmdActive;

	BLDC_Info.MotorStalling = 0U;
	BLDC_ResetControlState(&BLDC_Info);
	BLDC_Info.ExpectedRPM = expected_rpm;
	BLDC_Info.ExpectedCurrent = expected_current;
	BLDC_Info.ExpectedAngleDeg = expected_angle;
	BLDC_Info.CtrlMode = ctrl_mode;
	BLDC_Info.Direction = direction;
	BLDC_Info.PositionCmdActive = position_cmd_active;
	if ( start_pulse < BLDC_STARTUP_DUTY )
	{
		start_pulse = BLDC_STARTUP_DUTY;
	}
	BLDC_Info.Pulse = start_pulse;
	BLDC_Info.MotorRunning = 1U;
	BLDC_Enable();
	Hall_Start();
}

void BLDC_SetPulse( int32_t duty )
{
	if ( duty < 0 )
	{
		duty = 0;
	}
	if ( duty > (int32_t)BLDC_PWM_MAX_DUTY )
	{
		duty = (int32_t)BLDC_PWM_MAX_DUTY;
	}
	BLDC_Info.Pulse = (uint16_t)duty;
	if ( BLDC_Info.MotorRunning != 0U )
	{
		prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
	}
}

/* 位置环和速度环都先输出电流目标，再由电流环闭合到PWM占空比。 */
void BLDC_ControlTask( void )
{
	float position_cur_cmd = 0.0f;
	float speed_cur_cmd = 0.0f;
	float current_target = 0.0f;
	float angle_error = BLDC_Info.ExpectedAngleDeg - BLDC_Info.CurrentAngleDeg;
	float current_feedback = prvBLDC_GetCurrentMagnitude(&BLDC_Info);
	uint16_t duty_cmd;

	BLDC_Info.CurrentMagnitude = current_feedback;

	switch ( BLDC_Info.CtrlMode )
	{
		case BLDC_CTRL_POSITION:
			/* 位置环先判断方向，再输出带符号的电流需求。 */
			if ( BLDC_Info.PositionCmdActive == 0U )
			{
				return;
			}

			if ( fabsf(angle_error) <= BLDC_POSITION_DEADBAND_DEG )
			{
				BLDC_Info.ExpectedCurrent = 0.0f;
				BLDC_Info.ExpectedRPM = 0.0f;
				BLDC_Info.ExpectedRPM_Ramp = 0.0f;
				BLDC_Info.PositionCmdActive = 0U;
				BLDC_Stop();
				return;
			}

			if ( angle_error > 0.0f )
			{
				BLDC_SetDirection(MOTOR_FWD);
				position_cur_cmd = prvBLDC_PositionPID_Calc(&BLDC_Info,
															BLDC_Info.ExpectedAngleDeg,
															BLDC_Info.CurrentAngleDeg);
			}
			else
			{
				BLDC_SetDirection(MOTOR_REV);
				position_cur_cmd = -prvBLDC_PositionPID_Calc(&BLDC_Info,
															 BLDC_Info.ExpectedAngleDeg,
															 BLDC_Info.CurrentAngleDeg);
			}

			current_target = fabsf(position_cur_cmd);
			if ( current_target > 0.0f && current_target < BLDC_POSITION_MIN_CUR_mA )
			{
				current_target = BLDC_POSITION_MIN_CUR_mA;
			}
			break;

		case BLDC_CTRL_SPEED:
			/* 速度环在斜坡处理后的转速给定基础上输出电流需求。 */
			prvBLDC_RampTargetRPM(&BLDC_Info);
			speed_cur_cmd = prvBLDC_SpeedPID_Calc(&BLDC_Info,
												  BLDC_Info.ExpectedRPM_Ramp,
												  BLDC_Info.RPM);
			current_target = speed_cur_cmd;
			break;

		case BLDC_CTRL_CURRENT:
			/* 电流模式直接绕过外环。 */
			current_target = BLDC_Info.ExpectedCurrent;
			break;

		default:
			current_target = 0.0f;
			break;
	}

	current_target = prvClampf(current_target, 0.0f, BLDC_MAX_CUR_TARGET_mA);
	BLDC_Info.ExpectedCurrent = current_target;

	if ( current_target <= 0.0f )
	{
		/* 在速度/电流模式下，电流目标为0时可以直接释放功率级。 */
		if ( BLDC_Info.CtrlMode != BLDC_CTRL_POSITION )
		{
			BLDC_Stop();
		}
		return;
	}

	/* 电流内环把电流误差换算成PWM占空比指令。 */
	duty_cmd = (uint16_t)prvBLDC_CurrentPID_Calc(&BLDC_Info,
												 current_target,
												 current_feedback);
	if ( duty_cmd < BLDC_STARTUP_DUTY )
	{
		duty_cmd = BLDC_STARTUP_DUTY;
	}

	prvBLDC_ApplyPulse(duty_cmd);
	if ( BLDC_Info.MotorRunning == 0U )
	{
		BLDC_Start();
	}
	else
	{
		prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
	}
}

uint16_t BLDC_GetPulse( void )
{
	return BLDC_Info.Pulse;
}

void BLDC_SetDirection( MotorDir_t dir )
{
	BLDC_Info.Direction = dir;
}

void BLDC_Stop( void )
{
	BLDC_ResetControlState(&BLDC_Info);
	BLDC_Info.ExpectedRPM = 0.0f;
	BLDC_Info.ExpectedRPM_Ramp = 0.0f;
	BLDC_Info.PositionCmdActive = 0U;
	BLDC_Disable();
	Hall_Disable();
}

void BLDC_TripStop( void )
{
	BLDC_ResetControlState(&BLDC_Info);
	BLDC_Info.ExpectedRPM = 0.0f;
	BLDC_Info.ExpectedRPM_Ramp = 0.0f;
	BLDC_Info.PositionCmdActive = 0U;
	BLDC_Info.MotorStalling = 1U;
	BLDC_Disable();
	Hall_Disable();
}

void Hall_Start( void )
{
	uint8_t Hall;

	Hall_Info.HallFirstEdge   = 1U;
	Hall_Info.HallEdgeFlag	  = 0U;
	Hall_Info.HallTickCnt	  = 0U;
	Hall_Info.HallStateShadow = Hall_GetState();
	Hall_Info.HallLastEdgeMs  = SystemRunTime_1ms;
	Hall_Info.HallSpeedFilter.Index = 0U;
	Hall_Info.HallSpeedFilter.ValidCnt = 0U;
	Hall_Info.HallSpeedFilter.LastFilter = 0U;
	Hall_Info.HallSpeedFilter.Inited = 0U;
	Hall_Info.HallSpeedFilter.HallTickBuf[0] = 0U;
	Hall_Info.HallSpeedFilter.HallTickBuf[1] = 0U;
	Hall_Info.HallSpeedFilter.HallTickBuf[2] = 0U;

	HAL_TIMEx_HallSensor_Start_IT(&htim5);
	Hall = Hall_Info.HallStateShadow;
	BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

	if ( Hall >= 1U && Hall <= 6U )
	{
		BLDC_ChangeMOSstate(pHallTable[Hall].PwmPhase, pHallTable[Hall].LowPhase, BLDC_Info.Pulse);
	}
}

void Hall_enable( void )
{
	Hall_Start();
}

void Hall_Disable(void)
{
	__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
	__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
	HAL_TIMEx_HallSensor_Stop(&htim5);
}

uint8_t Hall_GetState( void )
{
	uint8_t State = 0U;

	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 0;
	}
	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 1;
	}
	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 2;
	}
	return State;
}

/* 每个有效Hall跳变都对应机械角度前进一步。 */
void BLDC_OnHallTransition( uint8_t previousHall, uint8_t currentHall )
{
	int8_t step_delta = prvBLDC_GetHallStepDelta(previousHall, currentHall);

	if ( step_delta == 0 )
	{
		if ( previousHall == currentHall || currentHall < 1U || currentHall > 6U )
		{
			return;
		}

		step_delta = (BLDC_Info.Direction == MOTOR_FWD) ? 1 : -1;
	}

	BLDC_Info.HallStepCount += step_delta;
	BLDC_Info.CurrentAngleDeg = (float)BLDC_Info.HallStepCount * BLDC_MECH_DEG_PER_SECTOR;
}

void BLDC_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty )
{
	prvDisableAllMos();

	switch (PwmPhase)
	{
		case PHASE_U:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Duty);
			break;
		case PHASE_V:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Duty);
			break;
		case PHASE_W:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Duty);
			break;
		default:
			return;
	}

	switch (LowPhase)
	{
		case PHASE_U:
			HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_SET);
			break;
		case PHASE_V:
			HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_SET);
			break;
		case PHASE_W:
			HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_SET);
			break;
		default:
			return;
	}

	BLDC_Info.ActivePwmPhase = PwmPhase;
	BLDC_Info.ActiveLowPhase = LowPhase;
	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);
}

void BLDC_HallTableSelect( MotorDir_t Dir )
{
	if ( Dir == MOTOR_FWD )
	{
		pHallTable = gComFwd;
	}
	else
	{
		pHallTable = gComRev;
	}
}

MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC )
{
	return pBLDC->Direction;
}

/* 软限流逐步减小占空比，硬过流则立即停机。 */
void BLDC_CurrentProtect( void )
{
	float peak = prvBLDC_GetCurrentMagnitude(&BLDC_Info);

	if ( peak >= BLDC_CURRENT_TRIP_mA )
	{
		BLDC_TripStop();
		return;
	}

	if ( peak > BLDC_CURRENT_SOFT_LIMIT_mA )
	{
		float delta = peak - BLDC_CURRENT_SOFT_LIMIT_mA;
		uint16_t reduction = (uint16_t)(delta * BLDC_CURRENT_LIMIT_KP);

		if ( reduction < 1U )
		{
			reduction = 1U;
		}
		if ( BLDC_Info.Pulse > (BLDC_PWM_MIN_DUTY + reduction) )
		{
			BLDC_Info.Pulse -= reduction;
		}
		else
		{
			BLDC_Info.Pulse = BLDC_PWM_MIN_DUTY;
		}
		prvBLDC_UpdateActiveDuty(BLDC_Info.Pulse);
	}
}

/* 先处理Hall反馈，再跑串级控制，最后做电流保护。 */
void BLDC_Cyclic( void )
{
	if ( BLDC_Info.MotorStalling != 0U )
	{
		BLDC_TripStop();
		return;
	}

	if ( BLDC_Info.MotorRunning != 0U )
	{
		prvBLDC_HallCyclic();
	}

	BLDC_ControlTask();
	if ( BLDC_Info.MotorRunning != 0U )
	{
		BLDC_CurrentProtect();
	}
}
