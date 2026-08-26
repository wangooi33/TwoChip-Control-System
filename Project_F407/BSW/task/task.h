#ifndef __TASK_H
#define __TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/
#define TASK_PERIOD_1MS       1
#define TASK_PERIOD_2MS       2
#define TASK_PERIOD_5MS       5
#define TASK_PERIOD_10MS      10

#define TASK_PERIOD_20MS      20
#define TASK_PERIOD_50MS      50
#define TASK_PERIOD_100MS     100
#define TASK_PERIOD_500MS     500
#define TASK_PERIOD_1000MS    1000

/* enum ----------------------------------------------------------------------*/

/* types ---------------------------------------------------------------------*/

/* global variable -----------------------------------------------------------*/
extern volatile uint32_t SystemRunTime_1ms;
/* functions prototypes ------------------------------------------------------*/
void TaskSchedule(void);


#ifdef __cplusplus
}
#endif

#endif /* __TASK_H */

