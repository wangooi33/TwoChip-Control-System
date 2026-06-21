#ifndef __BDC_CONTROL_H
#define __BDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "adc.h"
/* macro ---------------------------------------------------------------------*/
#define BDC_MAX_CUR_TARGET		(10000.0f)								//10A
#define BDC_MIN_CUR_TARGET		(-10000.0f)
#define BDC_MAX_PWMDUTY     	(5600 * 0.9)
#define BDC_MIN_PWMDUTY     	-(5600 * 0.9)
#define BDC_PWM_DEADZONE  		(20)									//PWM死区
#define ENCODER_Lines			(32)									//编码器线数
#define ENCODER_DIRSIGN   		(-1)									//编码器方向
#define BDC_SPEEDRATIO			(30)									//电机减速比
#define BDC_PPR					(BDC_SPEEDRATIO * ENCODER_Lines * 4)	//Pulse/r 每转脉冲数

#define BDC_SD_ENABLE()  		HAL_GPIO_WritePin(BDC_SD_GPIO_Port, BDC_SD_Pin, GPIO_PIN_SET)
#define BDC_SD_DISABLE()  		HAL_GPIO_WritePin(BDC_SD_GPIO_Port, BDC_SD_Pin, GPIO_PIN_RESET)
/* enum ----------------------------------------------------------------------*/
typedef enum
{
	BDC_PowerVoltage,		//Channel_8:电源电压
	BDC_MotorCurrent,		//Channel_9:电机电流
}ADC1_ChannelIndex_t;

/* types ---------------------------------------------------------------------*/
typedef struct
{
	float Kp, Ki, Kd;
	float PreError;
	float PrePreError;
	float Output;
}PID_Inc_t;

typedef struct
{
	float Kp, Ki, Kd;
	float PreError;
	float SumError;
	float Output;
}PID_Pos_t;

typedef struct
{
	float ExpectedRPM;
	float ExpectedRPM_Ramp;				//斜坡速度增量
	float ExpectedPos;
	float ExpectedCur;
}Expectations_t;

typedef struct BDC
{
	float PowerVoltage;					//电源电压
	float CurrentRealTime;				//当前电流
	float CurrZeroOffsetV;				//电流零偏 (单位：V)
	float CurrFilt;						//电流滤波值
	float PulseCnt;						//脉冲计数
	float RPM;
	
	Expectations_t Expectation;
	PID_Inc_t PIDInc_SpeedLoop;			//增量式速度环:转速
	PID_Pos_t PIDPos_SpeedLoop;			//位置式速度环
	PID_Pos_t PID_CurrentLoop;			//电流环
	PID_Pos_t PID_PositionLoop;			//位置环:脉冲
}BDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BDC_Info_t BDC_Info;
extern int16_t EncoderLastCnt;

/* functions prototypes ------------------------------------------------------*/
void BDC_Disable( void );
void BDC_Enable( void );
void BDC_PIDInit( void );
void BDC_EncoderReset( void );
void BDC_ResetControlState( BDC_Info_t *pBDC );

void BDC_Cyclic( void );


#endif /*__BDC_CONTROL_H */

