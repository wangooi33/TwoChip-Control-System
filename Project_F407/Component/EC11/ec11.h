#ifndef __EC11_H
#define __EC11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/
#define EC11_ENCODER_PPR            (20.0f)     /* EC11 机械一圈输出 20 个正交周期 */
#define EC11_COUNTER_X              (2)         /* TIM3 双沿计数，一格对应 2 个计数 */
#define EC11_DIR_SIGN               (1)         /* 旋钮方向修正，必要时改为 -1 */
#define EC11_DEG_PER_STEP           (360.0f / EC11_ENCODER_PPR)   /* 旋钮每一格对应 18° */
#define EC11_DEG_PER_COUNT          (EC11_DEG_PER_STEP / (float)EC11_COUNTER_X)

/* 防抖与限幅 */
#define EC11_MAX_DELTA_PER_CYCLE    (16)        /* 单周期最大计数变化，超过则视为噪声 */
#define EC11_CYCLE_PERIOD_MS        (5)         /* EC11_Cyclic 调用周期 [ms] */

/* global variable -----------------------------------------------------------*/
extern int16_t EC11_EncoderLastCnt;
extern float EC11_PulseCnt;
extern float EC11_TargetAngleDeg;
extern float EC11_AbsoluteAngleDeg;
extern float EC11_SpeedDegPerSec;

/* functions prototypes ------------------------------------------------------*/
void EC11_Init( void );
void EC11_Cyclic( void );

#ifdef __cplusplus
}
#endif

#endif /* __EC11_H */
