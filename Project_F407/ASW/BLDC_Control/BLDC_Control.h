#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/
#define BLDC_MAX_SPEED_RPM			(2000.0f)

/* 电机硬件参数 */
#define BLDC_POLE_PAIRS				(2U)			/* 电机极对数 */
#define BLDC_L						(0.00112f)		/* 线电感 */
#define BLDC_R						(0.42f)			/* 线电阻 */

#define Wc							(1000 * PI)		/* ωc */

/* 位置环参数 */
#define BLDC_POSITION_CURRENT			(1.2f)
#define BLDC_POSITION_MAX_RPM			(150.0f)
#define BLDC_POSITION_DEADBAND_TURNS	(7.0f / 360.0f)
#define BLDC_POSITION_START_ANGLE_STEP	(0.05f)

/* shutdown */
#define BLDC_SD_ENABLE()			HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET)
#define BLDC_SD_DISABLE() 			HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/
typedef enum
{
	Motor_Start_Idle,
	Motor_Start_CheckHall,
	Motor_Start_HallValid,
	Motor_Start_Run,
	Motor_Start_Interpolation,
	Motor_Run,
	Motor_Stop
}MotorRunStage_t;

/* types ---------------------------------------------------------------------*/
typedef struct
{
	uint16_t ZeroOffsetADC[3];		/* 三相零电流时的电压偏置(ADC原始值) */
	uint8_t ZeroOffsetFlag;			/* 是否完成偏置计算 */
	float Power;					/* 母线电压 */
	float PhaseCurrent[3];			/* 三相电流, 采样/滤波后 */
	float MotorTemperature;			/* 电机温度 */

	float Theta;					/* FOC电角度 */
	volatile MotorRunStage_t MotorRunStage;
	float RPM;
} BLDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;

/* functions prototypes ------------------------------------------------------*/
void BLDC_Enable(void);
void BLDC_Disable(void);
void BLDC_PidInit(void);
void BLDC_Run(void);
void BLDC_SpeedPID(void);
/* 有符号RPM: 正值为正转,负值为反转 */
void BLDC_SetSpeedRef(float speedRpm);
float BLDC_GetSpeedRef(void);
void BLDC_SetPositionRef(float turns);
float BLDC_GetCurrentTurns(void);
uint8_t BLDC_PositionPID(void);


#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
