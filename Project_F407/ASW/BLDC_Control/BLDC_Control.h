#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "foc.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/

/* 定时器参数 */
#define PWM_PERIOD							(8399U)
#define TIM_CLK_HZ							(168000000.0f)	/* 中心对齐计数频率   */
#define BLDC_POLE_PAIRS						(2U)			/* 电机极对数 */

/* shutdown */
#define BLDC_SD_ENABLE()                HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET)
#define BLDC_SD_DISABLE()               HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/
typedef enum
{
    MOTOR_REV = 0,
    MOTOR_FWD,
} MotorDir_t;

/* types ---------------------------------------------------------------------*/
typedef struct
{
    float U_CurrFilt;
    float V_CurrFilt;
    float W_CurrFilt;
} PhaseCurrFilt_t;

typedef struct
{
	float ZeroOffset[3];		/* 三相零电流时的电压偏置(ADC原始值) */
	float Power;				/* 母线电压 */
	float PhaseCurrent[3];		/* 三相电流, 采样/滤波后 */
	float MotorTemperature;		/* 电机温度 */
	
    PhaseCurrFilt_t CurrFilt;    /* 三相电流一阶低通滤波值 [mA] */
    float RPM;
    float CurrentAngleDeg;       /* 当前机械角度 [°], Hall 累计 */
    MotorDir_t Direction;
    int32_t HallStepCount;       /* Hall 扇区步进计数, 位置累计 */
    uint8_t MotorRunning;        /* 电机运行标志: 1=运行 */
    uint8_t MotorStalling;       /* 堵转/故障标志: 1=故障停机 */
} BLDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;
extern float Valpha,Vbeta,Tcm1,Tcm2,Tcm3;
/* functions prototypes ------------------------------------------------------*/
void BLDC_Enable(void);
void BLDC_Disable(void);
void BLDC_Run(void);


#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
