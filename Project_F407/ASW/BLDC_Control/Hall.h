#ifndef __HALL_H
#define __HALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/

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
} Hall_Info_t;

/* global variable -----------------------------------------------------------*/
extern Hall_Info_t Hall_Info;

/* functions prototypes ------------------------------------------------------*/
void Hall_Enable(void);


#ifdef __cplusplus
}
#endif

#endif /* __HALL_H */
