#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "pid.h"

/* =============================================================================
 *  BLDC 电机控制核心
 *
 *  职责:
 *    位置环 / 速度环 / 电流环 PID, 目标设定, 启停, 保护。
 *
 *  霍尔传感器: 见 Hall.h
 *  六步换相:   见 SixStep.h
 *  矢量控制:   见 FOC.h
 * ==========================================================================*/

/* 电流保护阈值和电流目标限幅 */
#define BLDC_CURRENT_SOFT_LIMIT_mA      (5900.0f)   /* 超过该电流后开始逐步削减 PWM */
#define BLDC_CURRENT_TRIP_mA            (7500.0f)   /* 硬件级过流停机阈值 */
#define BLDC_MAX_CUR_TARGET_mA          (5000.0f)   /* 电流环允许的最大目标电流 */
#define BLDC_CURRENT_LIMIT_KP           (0.08f)     /* 软限流时 PWM 削减增益 */

/* 电流内环参数 */
#define BLDC_CURRENT_PID_KP             (0.80f)
#define BLDC_CURRENT_PID_KI             (0.05f)
#define BLDC_CURRENT_PID_KD             (0.00f)

/* PWM 驱动占空比限制 */
#define BLDC_PWM_MIN_DUTY               (50U)       /* 限流后保持的最小占空比 */
#define BLDC_PWM_MAX_DUTY               (5600U)     /* 功率级允许的最大占空比 */
#define BLDC_STARTUP_DUTY               (600U)      /* 电机起转时保证换相的最小占空比 */

/* 速度环参数 */
#define BLDC_MAX_RPM_TARGET             (6000.0f)   /* 速度给定上限 */
#define BLDC_RPM_RAMP_STEP              (20.0f)     /* 每次控制周期的速度斜坡步进 */
#define BLDC_SPEED_PID_KP               (2.20f)
#define BLDC_SPEED_PID_KI               (0.08f)
#define BLDC_SPEED_PID_KD               (0.00f)

/* 位置环参数，用于 EC11 角度跟随 */
#define BLDC_POSITION_PID_KP            (80.0f)
#define BLDC_POSITION_PID_KI            (0.00f)
#define BLDC_POSITION_PID_KD            (4.00f)
#define BLDC_POSITION_DEADBAND_DEG      (15.0f)     /* 到达目标角附近后的停止死区 */
#define BLDC_POSITION_MIN_CUR_mA        (900.0f)    /* 位置模式下维持转动的最小电流 */

/* 电机参数 */
#define BLDC_POLE_PAIRS                 (2U)        /* 电机极对数 */

/* 驱动器使能控制 */
#define BLDC_SD_ENABLE()                HAL_GPIO_WritePin(BLDC_SD_GPIO_Port, BLDC_SD_Pin, GPIO_PIN_SET)
#define BLDC_SD_DISABLE()               HAL_GPIO_WritePin(BLDC_SD_GPIO_Port, BLDC_SD_Pin, GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/
typedef enum
{
    PHASE_U = 0,
    PHASE_V,
    PHASE_W,
    PHASE_NONE
} Phase_t;

typedef enum
{
    MOTOR_REV = 0,
    MOTOR_FWD,
} MotorDir_t;

typedef enum
{
    BLDC_CTRL_SPEED = 0,
    BLDC_CTRL_FOC,
    BLDC_CTRL_POSITION,
    BLDC_CTRL_CURRENT,
} BLDC_CtrlMode_t;

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
    float PowerVoltage;
    CurrentPhase_t CurrentPhase;
    PhaseSetV_t CurrZeroOffsetV;
    PhaseCurrFilt_t CurrFilt;
    float MotorTemperature;

    float RPM;
    float ExpectedRPM;
    float ExpectedRPM_Ramp;
    float CurrentMagnitude;
    float ExpectedCurrent;
    float CurrentAngleDeg;
    float ExpectedAngleDeg;
    PID_t PIDPos_SpeedLoop;
    PID_t PID_CurrentLoop;
    PID_t PIDPos_PositionLoop;
    Phase_t ActivePwmPhase;
    Phase_t ActiveLowPhase;
    BLDC_CtrlMode_t CtrlMode;
    MotorDir_t Direction;
    int32_t HallStepCount;
    uint16_t Pulse;
    uint8_t PositionCmdActive;
    uint8_t MotorRunning;
    uint8_t MotorStalling;
} BLDC_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;

/* functions prototypes ------------------------------------------------------*/
void BLDC_Disable( void );
void BLDC_Enable( void );
void BLDC_Start( void );
void BLDC_Stop( void );
void BLDC_TripStop( void );
void BLDC_CurrentProtect( void );
void BLDC_PIDInit( BLDC_Info_t *pBLDC );
void BLDC_ResetControlState( BLDC_Info_t *pBLDC );
void BLDC_PositionReset( BLDC_Info_t *pBLDC );
void BLDC_SetExpectedRPM( float expectedRPM );
void BLDC_SetExpectedCurrent( float expectedCurrent );
float BLDC_GetExpectedCurrent( void );
void BLDC_SetExpectedAngle( float expectedAngleDeg );
void BLDC_AddExpectedAngle( float deltaAngleDeg );
float BLDC_GetExpectedAngle( void );
float BLDC_GetCurrentAngle( void );
void BLDC_SetPulse( int32_t duty );
void BLDC_SetDirection( MotorDir_t dir );
void BLDC_SetFOCMode( void );
MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC );
void BLDC_ControlTask( void );
void BLDC_Cyclic( void );

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
