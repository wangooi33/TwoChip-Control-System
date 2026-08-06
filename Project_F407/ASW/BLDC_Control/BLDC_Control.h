#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"


/* 电机参数 */
#define BLDC_POLE_PAIRS                 (2U)        /* 电机极对数 */

/* 驱动器使能控制 */
#define BLDC_SD_ENABLE()                HAL_GPIO_WritePin(BLDC_SD_GPIO_Port, BLDC_SD_Pin, GPIO_PIN_SET)
#define BLDC_SD_DISABLE()               HAL_GPIO_WritePin(BLDC_SD_GPIO_Port, BLDC_SD_Pin, GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/
typedef enum
{
    MOTOR_REV = 0,
    MOTOR_FWD,
} MotorDir_t;

typedef enum
{
    BLDC_U_Current,
    BLDC_V_Current,
    BLDC_W_Current,
    BLDC_PowerVoltage,
    BLDC_MotorTemperature,
} ADC3_ChannelIndex_t;

/* types ---------------------------------------------------------------------*/
typedef struct
{
    float U_PhaseCurrent;
    float V_PhaseCurrent;
    float W_PhaseCurrent;
} CurrentPhase_t;

typedef struct
{
    float U_PhaseSetV;
    float V_PhaseSetV;
    float W_PhaseSetV;
} PhaseSetV_t;

typedef struct
{
    float U_CurrFilt;
    float V_CurrFilt;
    float W_CurrFilt;
} PhaseCurrFilt_t;

/* 全局电机状态 */
typedef struct
{
    float PowerVoltage;          /* 母线电压 [V], w_adc 分压反算 */
    CurrentPhase_t CurrentPhase; /* 三相电流 [mA], 采样/滤波后 */
    PhaseSetV_t CurrZeroOffsetV; /* 三相电流零点偏置 [V], 上电校准 */
    PhaseCurrFilt_t CurrFilt;    /* 三相电流一阶低通滤波值 [mA] */
    float MotorTemperature;      /* 电机温度 [°C], NTC 换算 */

    float RPM;                   /* 当前转速 [rpm], FOC 速度反馈同步 */
    float CurrentAngleDeg;       /* 当前机械角度 [°], Hall 累计 */
    MotorDir_t Direction;        /* 电机方向, Hall 跳变方向兜底 */
    int32_t HallStepCount;       /* Hall 扇区步进计数, 位置累计 */
    uint8_t MotorRunning;        /* 电机运行标志: 1=运行 */
    uint8_t MotorStalling;       /* 堵转/故障标志: 1=故障停机 */
} BLDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;

/* functions prototypes ------------------------------------------------------*/
void BLDC_Start( void );        /* 使能驱动并启动 FOC */
void BLDC_Stop( void );         /* 停止 FOC 并关闭驱动 */
void BLDC_TripStop( void );     /* 故障停机 */
void BLDC_Disable( void );      /* 关闭功率级 */

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
