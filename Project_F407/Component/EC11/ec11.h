#ifndef __EC11_H
#define __EC11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/
#define EC11_ENCODER_PPR			(20.0f)	/* EC11机械一圈输出20个正交周期。 */
#define EC11_COUNTER_X			 	(2)		/* TIM3当前为双边计数，一格对应2个计数。 */
#define EC11_DIR_SIGN				(1)		/* 旋钮方向修正，必要时改成-1。 */
#define EC11_DEG_PER_STEP			(360.0f / EC11_ENCODER_PPR)	/* 旋钮每一格对应18度。 */
#define EC11_DEG_PER_COUNT			(EC11_DEG_PER_STEP / (float)EC11_COUNTER_X)

/* global variable -----------------------------------------------------------*/
extern int16_t EC11_EncoderLastCnt;
extern float EC11_PulseCnt;
extern float EC11_TargetAngleDeg;


/* functions prototypes ------------------------------------------------------*/
void EC11_Init( void );
void EC11_Cyclic( void );


#ifdef __cplusplus
}
#endif

#endif /* __EC11_H */


