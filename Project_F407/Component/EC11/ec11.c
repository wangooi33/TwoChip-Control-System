/* Includes ------------------------------------------------------------------*/
#include "ec11.h"
#include "BLDC_Control.h"

/* global variable -----------------------------------------------------------*/
int16_t EC11_EncoderLastCnt = 0;
float EC11_PulseCnt = 0.0f;
float EC11_TargetAngleDeg = 0.0f;

/* local variable ------------------------------------------------------------*/
static int16_t EC11_CountRemainder = 0;

/* public functions ----------------------------------------------------------*/
void EC11_Init( void )
{
	EC11_EncoderLastCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	EC11_CountRemainder = 0;
	EC11_PulseCnt = 0.0f;
	EC11_TargetAngleDeg = BLDC_GetCurrentAngle();
}

void EC11_Cyclic( void )
{
	int16_t NowCnt;
	int16_t Delta;
	int16_t SignedDelta;
	int16_t StepDelta = 0;

	NowCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	Delta = NowCnt - EC11_EncoderLastCnt;
	EC11_EncoderLastCnt = NowCnt;

	if ( Delta == 0 )
	{
		return;
	}

	SignedDelta = (int16_t)(EC11_DIR_SIGN * Delta);
	EC11_PulseCnt += (float)SignedDelta;
	EC11_CountRemainder += SignedDelta;

	/* 将编码器计数折算成整格位移，确保旋钮每转一格只下发一次18度目标。 */
	while ( EC11_CountRemainder >= EC11_COUNTER_X )
	{
		StepDelta++;
		EC11_CountRemainder -= EC11_COUNTER_X;
	}
	while ( EC11_CountRemainder <= -EC11_COUNTER_X )
	{
		StepDelta--;
		EC11_CountRemainder += EC11_COUNTER_X;
	}

	if ( StepDelta == 0 )
	{
		return;
	}

	/* 旋钮只负责叠加目标角，实际执行仍由BLDC的位置电流环完成。 */
	BLDC_AddExpectedAngle((float)StepDelta * EC11_DEG_PER_STEP);
	EC11_TargetAngleDeg = BLDC_GetExpectedAngle();
}
