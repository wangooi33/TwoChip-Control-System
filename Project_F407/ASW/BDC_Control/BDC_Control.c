/* includes ------------------------------------------------------------------*/
#include "BDC_Control.h"
/* global variable -----------------------------------------------------------*/
BDC_Info_t BDC_Info;
int16_t EncoderLastCnt = 0;

/* functions declaration -----------------------------------------------------*/

/* functions implementation --------------------------------------------------*/
void BDC_Disable( void )
{
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
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
void BDC_PIDIncSpeedInit( PID_Inc_t *pPID )
{
	pPID->Kp = 5.5f;
	pPID->Ki = 3.2f;
	pPID->Kd = 0;
	pPID->PreError = 0;
	pPID->PrePreError = 0;
	pPID->Output = 0;
}
void BDC_PIDPosSpeedInit( PID_Pos_t *pPID )
{
	pPID->Kp = 5.5f;
	pPID->Ki = 3.2f;
	pPID->Kd = 0.5f;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}
void BDC_PIDPosPosInit( PID_Pos_t *pPID )
{
	pPID->Kp = 1.3f;
	pPID->Ki = 0.025f;
	pPID->Kd = 0.2f;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}
void BDC_PIDCurInit( PID_Pos_t *pPID )
{
	pPID->Kp = 0.8f;
	pPID->Ki = 0.05f;
	pPID->Kd = 0;
	pPID->PreError = 0;
	pPID->SumError = 0;
	pPID->Output = 0;
}

void BDC_EncoderCollects( BDC_Info_t *pBDC )
{
	//差分法
	int16_t NowCnt;
	int16_t Delta;

	NowCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	Delta = NowCnt - EncoderLastCnt;
	EncoderLastCnt = NowCnt;

	pBDC->PulseCnt += (float)(ENCODER_DIRSIGN * Delta);

	//rps = (delta / PPR) / 0.01    ->(10ms周期)
	//RPM = rps × 60
	pBDC->RPM = (float)(ENCODER_DIRSIGN * Delta) * 60.0f / (BDC_PPR * 0.01f);
}

float BDC_SpeedIncPID_Cal( PID_Inc_t *pPID, float Expectation, float CurrentVal )
{
	float Error_k;    //当前误差
	float Delta;
	float Output;

	Error_k = Expectation - CurrentVal;
	Delta = pPID->Kp * (Error_k - pPID->PreError) + pPID->Ki * Error_k;

	Output = pPID->Output + Delta;
	if ( Output > BDC_MAX_CUR_TARGET )
	{
	    Output = BDC_MAX_CUR_TARGET;
	}
	else if ( Output < BDC_MIN_CUR_TARGET )
	{
	    Output = BDC_MIN_CUR_TARGET;
	}

	pPID->Output = Output;
	pPID->PrePreError = pPID->PreError;
	pPID->PreError = Error_k;

	return Output;
}
float BDC_SpeedPosPID_Cal( PID_Pos_t *pPID, float Expectation, float CurrentVal )
{
	float Error_k = Expectation - CurrentVal;
	float Output;
	float P = pPID->Kp * Error_k;
	float D = pPID->Kd * (Error_k - pPID->PreError);

	float tempOutput = P + D + pPID->Ki * pPID->SumError;

	if (!((tempOutput >= BDC_MAX_CUR_TARGET && Error_k > 0) ||
			(tempOutput <= BDC_MIN_CUR_TARGET && Error_k < 0)))
	{
	    pPID->SumError += Error_k;
	}

	Output = P + pPID->Ki * pPID->SumError + D;

	if ( Output > BDC_MAX_CUR_TARGET )
	{
		Output = BDC_MAX_CUR_TARGET;
	}
	else if ( Output < BDC_MIN_CUR_TARGET )
	{
		Output = BDC_MIN_CUR_TARGET;
	}
	pPID->Output = Output;
	pPID->PreError = Error_k;

	return Output;
}
int16_t BDC_CurrentPI_Cal( PID_Pos_t *pID, float Target, float Feedback )
{
	float Error = Target - Feedback;
	float Output;

	pID->SumError += Error;

	Output = pID->Kp * Error + pID->Ki * pID->SumError;

	if (Output > BDC_MAX_PWMDUTY)
	{
		Output = BDC_MAX_PWMDUTY;
		pID->SumError -= Error;   // 抗积分饱和
	}
	else if (Output < BDC_MIN_PWMDUTY)
	{
		Output = BDC_MIN_PWMDUTY;
		pID->SumError -= Error;
	}

	pID->Output = Output;
	pID->PreError = Error;

	return (int16_t)Output;
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
void BDC_PIDInit( void )
{
	BDC_PIDIncSpeedInit(&BDC_Info.PIDInc_SpeedLoop);
	BDC_PIDPosSpeedInit(&BDC_Info.PIDPos_SpeedLoop);
	BDC_PIDPosPosInit(&BDC_Info.PID_PositionLoop);
	BDC_PIDCurInit(&BDC_Info.PID_CurrentLoop);
}
void BDC_EncoderReset( void )
{
	EncoderLastCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	BDC_Info.RPM = 0.0f;
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

static void BDC_RampTargetRPM( BDC_Info_t *pBDC )
{
	const float Step = 2.0f;			// 每5ms变化2RPM

	if ( pBDC->Expectation.ExpectedRPM_Ramp < pBDC->Expectation.ExpectedRPM )
	{
		pBDC->Expectation.ExpectedRPM_Ramp += Step;
		if ( pBDC->Expectation.ExpectedRPM_Ramp > pBDC->Expectation.ExpectedRPM )
		{
			pBDC->Expectation.ExpectedRPM_Ramp = pBDC->Expectation.ExpectedRPM;
		}
		
	}
	else if ( pBDC->Expectation.ExpectedRPM_Ramp > pBDC->Expectation.ExpectedRPM )
	{
		pBDC->Expectation.ExpectedRPM_Ramp -= Step;
		if ( pBDC->Expectation.ExpectedRPM_Ramp < pBDC->Expectation.ExpectedRPM )
		{
			pBDC->Expectation.ExpectedRPM_Ramp = pBDC->Expectation.ExpectedRPM;
		}
	}
}
void BDC_Cyclic(void)
{
	int16_t Pulse;
	static uint8_t SpeedLoopCnt = 0;

	BDC_RampTargetRPM(&BDC_Info);		//斜坡增量

	//速度环降频:10ms
	if ( ++SpeedLoopCnt >= 2 )
	{
		SpeedLoopCnt = 0;

		BDC_EncoderCollects(&BDC_Info);

		BDC_Info.Expectation.ExpectedCur = BDC_SpeedPosPID_Cal(&BDC_Info.PIDPos_SpeedLoop,
																BDC_Info.Expectation.ExpectedRPM_Ramp,
																BDC_Info.RPM);
	}

	//电流环
	Pulse = BDC_CurrentPI_Cal(&BDC_Info.PID_CurrentLoop,BDC_Info.Expectation.ExpectedCur,BDC_Info.CurrentRealTime);

	BDC_MotorCtrl(Pulse);
}

