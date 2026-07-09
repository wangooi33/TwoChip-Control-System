/* Includes ------------------------------------------------------------------*/
#include "ec11.h"
#include "BLDC_Control.h"

/* global variable -----------------------------------------------------------*/
int16_t EC11_EncoderLastCnt = 0;
float EC11_PulseCnt = 0.0f;
float EC11_TargetAngleDeg = 0.0f;

/* public functions ----------------------------------------------------------*/
void EC11_Init( void )
{
	EC11_EncoderLastCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	EC11_PulseCnt = 0.0f;
	EC11_TargetAngleDeg = 0.0f;
}

void EC11_Cyclic( void )
{
	int16_t NowCnt;
	int16_t Delta;

	NowCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	Delta = NowCnt - EC11_EncoderLastCnt;
	EC11_EncoderLastCnt = NowCnt;

	if ( Delta == 0 )
	{
		return;
	}

	EC11_PulseCnt += (float)(EC11_DIR_SIGN * Delta);
	EC11_TargetAngleDeg = EC11_PulseCnt * EC11_DEG_PER_COUNT;
	BLDC_SetExpectedAngle(EC11_TargetAngleDeg);
}
