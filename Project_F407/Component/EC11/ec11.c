/* Includes ------------------------------------------------------------------*/
#include "ec11.h"

/* private variable ----------------------------------------------------------*/
EC11_Info_t EC11_Info;

/* local helpers -------------------------------------------------------------*/
static uint8_t prvEC11_GetABState( void )
{
	uint8_t state = 0U;

	if ( HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) != GPIO_PIN_RESET )
	{
		state |= 0x01U;
	}
	if ( HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) != GPIO_PIN_RESET )
	{
		state |= 0x02U;
	}
	return state;
}

static void prvEC11_ApplyCountDelta( EC11_Info_t *pEC11, int16_t count_delta )
{
	if ( count_delta == 0 )
	{
		return;
	}

	pEC11->PendingCounts += (int16_t)(EC11_DIR_SIGN * count_delta);

	while ( pEC11->PendingCounts >= EC11_TIM4_COUNTS_PER_DETENT )
	{
		pEC11->PendingCounts -= EC11_TIM4_COUNTS_PER_DETENT;
		pEC11->StepCount++;
		pEC11->StepDeltaPending++;
		pEC11->TargetUpdated = 1U;
	}

	while ( pEC11->PendingCounts <= -EC11_TIM4_COUNTS_PER_DETENT )
	{
		pEC11->PendingCounts += EC11_TIM4_COUNTS_PER_DETENT;
		pEC11->StepCount--;
		pEC11->StepDeltaPending--;
		pEC11->TargetUpdated = 1U;
	}

	pEC11->AngleDeg = (float)pEC11->StepCount * EC11_STEP_DEG;
}

static void prvEC11_ClearPosition( EC11_Info_t *pEC11 )
{
	__HAL_TIM_SET_COUNTER(&htim4, 0U);
	pEC11->LastCounter = 0;
	pEC11->PendingCounts = 0;
	pEC11->StepCount = 0;
	pEC11->StepDeltaPending = 0;
	pEC11->QuadAccumulator = 0;
	pEC11->LastABState = prvEC11_GetABState();
	pEC11->AngleDeg = 0.0f;
}

/* functions implementation --------------------------------------------------*/
void EC11_Init( EC11_Info_t *pEC11 )
{
	if ( HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK )
	{
		Error_Handler();
	}

	pEC11->ResetPending = 0U;
	pEC11->ResetEventPending = 0U;
	pEC11->TargetUpdated = 0U;
	pEC11->LastButtonTickMs = 0U;
	prvEC11_ClearPosition(pEC11);
}

void EC11_Cyclic( EC11_Info_t *pEC11 )
{
	int16_t current_counter;
	int16_t delta;
	uint8_t current_ab_state;
	static const int8_t quad_table[16] =
	{
		0, -1,  1,  0,
		1,  0,  0, -1,
	   -1,  0,  0,  1,
		0,  1, -1,  0
	};

	if ( pEC11->ResetPending != 0U )
	{
		pEC11->ResetPending = 0U;
		prvEC11_ClearPosition(pEC11);
		pEC11->ResetEventPending = 1U;
		pEC11->TargetUpdated = 1U;
		return;
	}

	current_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
	delta = (int16_t)(current_counter - pEC11->LastCounter);
	pEC11->LastCounter = current_counter;

	if ( delta != 0 )
	{
		prvEC11_ApplyCountDelta(pEC11, delta);
	}
	else
	{
		current_ab_state = prvEC11_GetABState();
		if ( current_ab_state != pEC11->LastABState )
		{
			pEC11->QuadAccumulator += (int8_t)(EC11_DIR_SIGN * quad_table[(pEC11->LastABState << 2) | current_ab_state]);
			pEC11->LastABState = current_ab_state;

			while ( pEC11->QuadAccumulator >= EC11_TIM4_COUNTS_PER_DETENT )
			{
				pEC11->QuadAccumulator -= EC11_TIM4_COUNTS_PER_DETENT;
				prvEC11_ApplyCountDelta(pEC11, EC11_TIM4_COUNTS_PER_DETENT);
			}

			while ( pEC11->QuadAccumulator <= -EC11_TIM4_COUNTS_PER_DETENT )
			{
				pEC11->QuadAccumulator += EC11_TIM4_COUNTS_PER_DETENT;
				prvEC11_ApplyCountDelta(pEC11, -EC11_TIM4_COUNTS_PER_DETENT);
			}
		}
	}
}

void EC11_RequestReset( EC11_Info_t *pEC11 )
{
	uint32_t now = SystemRunTime_1ms;

	if ( (now - pEC11->LastButtonTickMs) < EC11_BUTTON_DEBOUNCE_MS )
	{
		return;
	}

	pEC11->LastButtonTickMs = now;
	pEC11->ResetPending = 1U;
}

uint8_t EC11_ConsumeUpdateFlag( EC11_Info_t *pEC11 )
{
	uint8_t updated = pEC11->TargetUpdated;

	pEC11->TargetUpdated = 0U;
	return updated;
}

int32_t EC11_ConsumeStepDelta( EC11_Info_t *pEC11 )
{
	int32_t step_delta = pEC11->StepDeltaPending;

	pEC11->StepDeltaPending = 0;
	return step_delta;
}

uint8_t EC11_ConsumeResetEvent( EC11_Info_t *pEC11 )
{
	uint8_t reset_event = pEC11->ResetEventPending;

	pEC11->ResetEventPending = 0U;
	return reset_event;
}

float EC11_GetAngleDeg( const EC11_Info_t *pEC11 )
{
	return pEC11->AngleDeg;
}
