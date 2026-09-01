#ifndef __HALL_H
#define __HALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/
#define HALL_TIMEOUT_MS             (300U)                /* Hall 超时窗口，用于堵转检测 */
#define HALL_SECTORS_PER_REV        (6U * BLDC_POLE_PAIRS) /* 机械一圈对应 Hall 扇区数 */
#define HALL_DEG_PER_SECTOR         (360.0f / (float)HALL_SECTORS_PER_REV) /* 每个 Hall 步进对应机械角度 */

/* types ---------------------------------------------------------------------*/

/* 霍尔采集状态 */
typedef struct
{
	uint8_t State;
	uint32_t TimerCnt;
	float angle;
	float angle_inc;
	float Speed;
	float Speed_Filter;			/* 霍尔速度滤波值 */
	uint8_t ClosedLoop_Flag;	/* 闭环标志位 */

	uint32_t HallLastEdgeMs;       /* 最近一次 Hall 沿时刻 [ms] */
	uint32_t HallSectorStartMs;    /* 进入当前 Hall 扇区的时刻 [ms] */
	uint32_t HallSectorPeriodMs;   /* 上一个 Hall 扇区用时 [ms] */
	uint8_t HallFirstEdge;
} Hall_Info_t;

/* global variable -----------------------------------------------------------*/
extern Hall_Info_t Hall_Info;

/* functions prototypes ------------------------------------------------------*/
void Hall_Enable(void);


#ifdef __cplusplus
}
#endif

#endif /* __HALL_H */
