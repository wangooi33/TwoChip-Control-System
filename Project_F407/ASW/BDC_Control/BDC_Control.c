/* includes ------------------------------------------------------------------*/
#include "BDC_Control.h"

/* global variable -----------------------------------------------------------*/
BDC_Info_t BDC_Info;
int16_t EncoderLastCnt = 0;

/* forward declarations ------------------------------------------------------*/
void BDC_ResetControlState( BDC_Info_t *pBDC );
void BDC_EncoderReset( BDC_Info_t *pBDC );
void BDC_PIDInit( BDC_Info_t *pBDC );

/* local helpers -------------------------------------------------------------*/
static void BDC_PIDIncSpeedInit( PID_Inc_t *pPID )
{
	pPID->Kp = 5.5f;
	pPID->Ki = 3.2f;
	pPID->Kd = 0;
	pPID->PreError = 0;
	pPID->PrePreError = 0;
	pPID->Output = 0;
}

static void BDC_PIDPosSpeedInit( PID_Pos_t *pPID )
{
	pPID->Kp = 5.5f;
	pPID->Ki = 3.2f;
	pPID->Kd = 0.5f;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}

static void BDC_PIDPosPosInit( PID_Pos_t *pPID )
{
	pPID->Kp = 1.3f;
	pPID->Ki = 0.025f;
	pPID->Kd = 0.2f;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}

static void BDC_PIDCurInit( PID_Pos_t *pPID )
{
	pPID->Kp = 0.8f;
	pPID->Ki = 0.05f;
	pPID->Kd = 0;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}

static void BDC_RampTargetRPM( BDC_Info_t *pBDC )
{
	const float step = 2.0f;

	if ( pBDC->Expectation.ExpectedRPM_Ramp < pBDC->Expectation.ExpectedRPM )
	{
		pBDC->Expectation.ExpectedRPM_Ramp += step;
		if ( pBDC->Expectation.ExpectedRPM_Ramp > pBDC->Expectation.ExpectedRPM )
		{
			pBDC->Expectation.ExpectedRPM_Ramp = pBDC->Expectation.ExpectedRPM;
		}
	}
	else if ( pBDC->Expectation.ExpectedRPM_Ramp > pBDC->Expectation.ExpectedRPM )
	{
		pBDC->Expectation.ExpectedRPM_Ramp -= step;
		if ( pBDC->Expectation.ExpectedRPM_Ramp < pBDC->Expectation.ExpectedRPM )
		{
			pBDC->Expectation.ExpectedRPM_Ramp = pBDC->Expectation.ExpectedRPM;
		}
	}
}

/* public functions ----------------------------------------------------------*/
void BDC_Disable( void )
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	BDC_SD_DISABLE();
}

void BDC_Enable( void )
{
	BDC_SD_ENABLE();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void BDC_Start( BDC_Info_t *pBDC, float expectedRPM )
{
	BDC_Disable();
	BDC_ResetControlState(pBDC);
	BDC_EncoderReset(pBDC);
	BDC_PIDInit(pBDC);
	pBDC->Expectation.ExpectedRPM = expectedRPM;
	pBDC->Expectation.ExpectedRPM_Ramp = 0.0f;
	BDC_Enable();
}

void BDC_PIDInit( BDC_Info_t *pBDC )
{
	BDC_PIDIncSpeedInit(&pBDC->PIDInc_SpeedLoop);
	BDC_PIDPosSpeedInit(&pBDC->PIDPos_SpeedLoop);
	BDC_PIDPosPosInit(&pBDC->PID_PositionLoop);
	BDC_PIDCurInit(&pBDC->PID_CurrentLoop);
}

void BDC_EncoderReset( BDC_Info_t *pBDC )
{
	EncoderLastCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	pBDC->RPM = 0.0f;
}

void BDC_ResetControlState( BDC_Info_t *pBDC )
{
	pBDC->Expectation.ExpectedCur = 0.0f;
	pBDC->CurrentRealTime = 0.0f;
	pBDC->CurrFilt = 0.0f;

	pBDC->PIDPos_SpeedLoop.SumError = 0.0f;
	pBDC->PIDPos_SpeedLoop.PreError  = 0.0f;
	pBDC->PIDPos_SpeedLoop.Output    = 0.0f;

	pBDC->PID_CurrentLoop.SumError = 0.0f;
	pBDC->PID_CurrentLoop.PreError  = 0.0f;
	pBDC->PID_CurrentLoop.Output    = 0.0f;

	pBDC->PID_PositionLoop.SumError = 0.0f;
	pBDC->PID_PositionLoop.PreError  = 0.0f;
	pBDC->PID_PositionLoop.Output    = 0.0f;

	pBDC->RPM = 0.0f;
	pBDC->PulseCnt = 0.0f;
}

void BDC_EncoderCollects( BDC_Info_t *pBDC )
{
	int16_t now_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	int16_t delta = now_cnt - EncoderLastCnt;
	EncoderLastCnt = now_cnt;

	pBDC->PulseCnt += (float)(ENCODER_DIRSIGN * delta);
	pBDC->RPM = (float)(ENCODER_DIRSIGN * delta) * 60.0f / (BDC_PPR * 0.01f);
}

float BDC_SpeedPosPID_Cal( PID_Pos_t *pPID, float expectation, float current_val )
{
	float error_k = expectation - current_val;
	float p = pPID->Kp * error_k;
	float d = pPID->Kd * (error_k - pPID->PreError);
	float temp_output = p + d + pPID->Ki * pPID->SumError;
	float output;

	if ( !((temp_output >= BDC_MAX_CUR_TARGET && error_k > 0) ||
			(temp_output <= BDC_MIN_CUR_TARGET && error_k < 0)) )
	{
		pPID->SumError += error_k;
	}

	output = p + pPID->Ki * pPID->SumError + d;
	if ( output > BDC_MAX_CUR_TARGET )
	{
		output = BDC_MAX_CUR_TARGET;
	}
	else if ( output < BDC_MIN_CUR_TARGET )
	{
		output = BDC_MIN_CUR_TARGET;
	}

	pPID->Output = output;
	pPID->PreError = error_k;
	return output;
}

int16_t BDC_CurrentPI_Cal( PID_Pos_t *pID, float target, float feedback )
{
	float error = target - feedback;
	float output;

	pID->SumError += error;
	output = pID->Kp * error + pID->Ki * pID->SumError;

	if ( output > BDC_MAX_PWMDUTY )
	{
		output = BDC_MAX_PWMDUTY;
		pID->SumError -= error;
	}
	else if ( output < BDC_MIN_PWMDUTY )
	{
		output = BDC_MIN_PWMDUTY;
		pID->SumError -= error;
	}

	pID->Output = output;
	pID->PreError = error;
	return (int16_t)output;
}

void BDC_MotorCtrl( int16_t pulse )
{
	if ( pulse > -BDC_PWM_DEADZONE && pulse < BDC_PWM_DEADZONE )
	{
		pulse = 0;
	}

	if ( pulse >= 0 )
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -pulse);
	}
}

void BDC_Cyclic( BDC_Info_t *pBDC )
{
	static uint8_t speed_loop_cnt = 0;
	int16_t pulse;

	BDC_RampTargetRPM(pBDC);

	if ( ++speed_loop_cnt >= 2 )
	{
		speed_loop_cnt = 0;
		BDC_EncoderCollects(pBDC);
		pBDC->Expectation.ExpectedCur = BDC_SpeedPosPID_Cal(&pBDC->PIDPos_SpeedLoop,
															pBDC->Expectation.ExpectedRPM_Ramp,
															pBDC->RPM);
	}

	pulse = BDC_CurrentPI_Cal(&pBDC->PID_CurrentLoop,
							  pBDC->Expectation.ExpectedCur,
							  pBDC->CurrentRealTime);
	BDC_MotorCtrl(pulse);
}
