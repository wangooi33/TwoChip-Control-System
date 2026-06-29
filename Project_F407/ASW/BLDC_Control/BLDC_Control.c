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

static void prvBLDC_PIDPositionInit( BLDC_PID_Pos_t *pPID )
{
	pPID->Kp = BLDC_POSITION_PID_KP;
	pPID->Ki = BLDC_POSITION_PID_KI;
	pPID->Kd = BLDC_POSITION_PID_KD;
	pPID->PreError = 0.0f;
	pPID->SumError = 0.0f;
	pPID->Output = 0.0f;
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

static uint16_t prvBLDC_SpeedPID_Calc( BLDC_Info_t *pBLDC, float expectation, float feedback )
{
	BLDC_PID_Pos_t *pPID = &pBLDC->PIDPos_SpeedLoop;
	float error = expectation - feedback;
	float p = pPID->Kp * error;
	float d = pPID->Kd * (error - pPID->PreError);
	float output;

	pPID->SumError += error;
	output = p + pPID->Ki * pPID->SumError + d;

	if ( output > (float)BLDC_PWM_MAX_DUTY )
	{
		output = (float)BLDC_PWM_MAX_DUTY;
		pPID->SumError -= error;
	}
	else if ( output < (float)BLDC_PWM_MIN_DUTY )
	{
		output = (float)BLDC_PWM_MIN_DUTY;
		pPID->SumError -= error;
	}

	pPID->Output = output;
	pPID->PreError = error;
	return (uint16_t)output;
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

	if ( !((temp_output >= BLDC_POSITION_MAX_RPM && error > 0.0f) ||
			(temp_output <= -BLDC_POSITION_MAX_RPM && error < 0.0f)) )
	{
		pPID->SumError += error;
	}

	output = p + pPID->Ki * pPID->SumError + d;
	output = prvClampf(output, -BLDC_POSITION_MAX_RPM, BLDC_POSITION_MAX_RPM);

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
	prvBLDC_PIDPositionInit(&pBLDC->PIDPos_PositionLoop);
}

void BLDC_ResetControlState( BLDC_Info_t *pBLDC )
{
	pBLDC->RPM = 0.0f;
	pBLDC->Pulse = 0U;
	pBLDC->MotorStalling = 0U;
	pBLDC->MotorRunning = 0U;
	pBLDC->ExpectedRPM_Ramp = 0.0f;
	pBLDC->PIDPos_SpeedLoop.PreError = 0.0f;
	pBLDC->PIDPos_SpeedLoop.SumError = 0.0f;
	pBLDC->PIDPos_SpeedLoop.Output = 0.0f;
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
	BLDC_Info.ExpectedRPM = prvClampf(expectedRPM, 0.0f, BLDC_MAX_RPM_TARGET);
	BLDC_Info.CtrlMode = BLDC_CTRL_SPEED;
	BLDC_Info.PositionCmdActive = 0U;
}

float BLDC_GetExpectedRPM( void )
{
	return BLDC_Info.ExpectedRPM;
}

void BLDC_SetExpectedAngle( float expectedAngleDeg )
{
	BLDC_Info.ExpectedAngleDeg = expectedAngleDeg;
	BLDC_Info.CtrlMode = BLDC_CTRL_POSITION;
	BLDC_Info.PositionCmdActive = 1U;
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
	float angle_error;
	int32_t pulse_cmd;

	if ( BLDC_Info.PositionCmdActive == 0U )
	{
		return;
	}

	angle_error = BLDC_Info.ExpectedAngleDeg - BLDC_Info.CurrentAngleDeg;

	if ( fabsf(angle_error) <= BLDC_POSITION_DEADBAND_DEG )
	{
		BLDC_Stop();
		return;
	}

	if ( angle_error > 0.0f )
	{
		BLDC_SetDirection(MOTOR_FWD);
	}
	else
	{
		BLDC_SetDirection(MOTOR_REV);
	}

	pulse_cmd = (int32_t)(fabsf(angle_error) * BLDC_POSITION_PULSE_KP);
	if ( pulse_cmd < (int32_t)BLDC_POSITION_MIN_PULSE )
	{
		pulse_cmd = BLDC_POSITION_MIN_PULSE;
	}
	if ( pulse_cmd > (int32_t)BLDC_POSITION_MAX_PULSE )
	{
		pulse_cmd = BLDC_POSITION_MAX_PULSE;
	}

	BLDC_SetPulse(pulse_cmd);

	if ( BLDC_Info.MotorRunning == 0U )
	{
		BLDC_Start();
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

void BLDC_Start( void )
{
	uint16_t start_pulse = BLDC_Info.Pulse;

	BLDC_Info.MotorStalling = 0U;
	BLDC_ResetControlState(&BLDC_Info);
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

void BLDC_CurrentProtect( void )
{
	float peak = fabsf(BLDC_Info.CurrentPhase.U_PhaseCurrent);
	float iv = fabsf(BLDC_Info.CurrentPhase.V_PhaseCurrent);
	float iw = fabsf(BLDC_Info.CurrentPhase.W_PhaseCurrent);

	if ( iv > peak )
	{
		peak = iv;
	}
	if ( iw > peak )
	{
		peak = iw;
	}

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

void BLDC_Cyclic( void )
{
	if ( BLDC_Info.MotorStalling != 0U )
	{
		BLDC_TripStop();
		return;
	}

	if ( BLDC_Info.MotorRunning == 0U )
	{
		return;
	}

	prvBLDC_HallCyclic();

	BLDC_CurrentProtect();
}
