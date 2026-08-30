#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/

/* 电机硬件参数 */
#define BLDC_POLE_PAIRS				(2U)			/* 电机极对数 */
#define BLDC_L						(0.00112f)		/* 线电感 */
#define BLDC_R						(0.42f)			/* 线电阻 */

#define Wc							(1000 * PI)		/* ωc */

/* shutdown */
#define BLDC_SD_ENABLE()			HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET)
#define BLDC_SD_DISABLE() 			HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/

/* types ---------------------------------------------------------------------*/
typedef struct
{
	uint16_t ZeroOffsetADC[3];	/* 三相零电流时的电压偏置(ADC原始值) */
	uint8_t ZeroOffsetFlag;		/* 是否完成偏置计算 */
	float Power;				/* 母线电压 */
	float PhaseCurrent[3];		/* 三相电流, 采样/滤波后 */
	float MotorTemperature;		/* 电机温度 */
	uint8_t Direction;			/* 1:顺时针正转 */
	float Theta;
	
	float RPM;
	
    float CurrentAngleDeg;       /* 当前机械角度 [°], Hall 累计 */
    int32_t HallStepCount;       /* Hall 扇区步进计数, 位置累计 */
    uint8_t MotorRunning;        /* 电机运行标志: 1=运行 */
    uint8_t MotorStalling;       /* 堵转/故障标志: 1=故障停机 */
} BLDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;
extern float theta;
/* functions prototypes ------------------------------------------------------*/
void BLDC_Enable(void);
void BLDC_Disable(void);
void BLDC_PidInit(void);
void BLDC_Run(void);


#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
