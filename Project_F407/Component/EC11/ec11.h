#ifndef __EC11_H
#define __EC11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/
#define EC11_DETENTS_PER_REV           (20.0f)
#define EC11_TIM4_COUNTS_PER_DETENT    (4)
#define EC11_STEP_DEG                  (360.0f / EC11_DETENTS_PER_REV)
#define EC11_DIR_SIGN                  (1)
#define EC11_BUTTON_DEBOUNCE_MS        (50U)

/* enum ----------------------------------------------------------------------*/

/* types ---------------------------------------------------------------------*/
typedef struct
{
	int16_t LastCounter;
	int16_t PendingCounts;
	int32_t StepCount;
	int32_t StepDeltaPending;
	int8_t QuadAccumulator;
	uint8_t LastABState;
	float AngleDeg;
	volatile uint8_t ResetPending;
	uint8_t ResetEventPending;
	uint8_t TargetUpdated;
	uint32_t LastButtonTickMs;
} EC11_Info_t;

/* constants -----------------------------------------------------------------*/

/* global variable -----------------------------------------------------------*/
extern EC11_Info_t EC11_Info;

/* functions prototypes ------------------------------------------------------*/
void EC11_Init( EC11_Info_t *pEC11 );
void EC11_Cyclic( EC11_Info_t *pEC11 );
void EC11_RequestReset( EC11_Info_t *pEC11 );
uint8_t EC11_ConsumeUpdateFlag( EC11_Info_t *pEC11 );
int32_t EC11_ConsumeStepDelta( EC11_Info_t *pEC11 );
uint8_t EC11_ConsumeResetEvent( EC11_Info_t *pEC11 );
float EC11_GetAngleDeg( const EC11_Info_t *pEC11 );

#ifdef __cplusplus
}
#endif

#endif /* __EC11_H */


