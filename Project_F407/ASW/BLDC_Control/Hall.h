#ifndef __HALL_H
#define __HALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"

/* macro ---------------------------------------------------------------------*/

/* Hall 测速和机械角度换算参数 */
#define HALL_TIMER_HZ               (84000000UL / 84UL)   /* TIM5 Hall 捕获计时频率 1MHz */
#define HALL_TIMEOUT_MS             (300U)                /* Hall 超时窗口，用于堵转检测 */
#define HALL_SECTORS_PER_REV        (6U * BLDC_POLE_PAIRS) /* 机械一圈对应 Hall 扇区数 */
#define HALL_DEG_PER_SECTOR         (360.0f / (float)HALL_SECTORS_PER_REV) /* 每个 Hall 步进对应机械角度 */

/* types ---------------------------------------------------------------------*/

/* 霍尔采集状态 */
typedef struct
{
    volatile uint32_t HallTickCnt;          /* 相邻 Hall 沿间隔 [1us 计数] */
    volatile uint8_t  HallEdgeFlag;         /* 有新 Hall 沿待处理 */
    volatile uint8_t  HallStateShadow;      /* 最近一次 Hall 状态 */
    volatile uint32_t HallLastEdgeMs;       /* 最近一次 Hall 沿时刻 [ms] */
    volatile uint32_t HallSectorStartMs;    /* 进入当前 Hall 扇区的时刻 [ms] */
    volatile uint32_t HallSectorPeriodMs;   /* 上一个 Hall 扇区用时 [ms] */
    uint8_t  HallFirstEdge;

} Hall_Info_t;

/* global variable -----------------------------------------------------------*/
extern Hall_Info_t Hall_Info;

/* functions prototypes ------------------------------------------------------*/
void    Hall_Start( void );                  /* 复位并启动 TIM5 Hall 捕获 */
void    Hall_Disable( void );                /* 停止 TIM5 Hall 捕获 */
uint8_t Hall_GetState( void );               /* 读取当前三路霍尔状态 1~6 */
void    Hall_OnTransition( uint8_t previousHall, uint8_t currentHall ); /* Hall 跳变: 更新角度/扇区 */

#ifdef __cplusplus
}
#endif

#endif /* __HALL_H */
