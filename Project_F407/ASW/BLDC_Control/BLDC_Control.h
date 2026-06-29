#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* macro ---------------------------------------------------------------------*/
#define BLDC_CURRENT_SOFT_LIMIT_mA		(5900.0f)
#define BLDC_CURRENT_RELEASE_mA			(5400.0f)
#define BLDC_CURRENT_TRIP_mA			(7500.0f)
#define BLDC_CURRENT_LIMIT_KP			(0.08f)
#define BLDC_PWM_MIN_DUTY				(50U)
#define BLDC_PWM_MAX_DUTY				(5600U)
#define BLDC_STARTUP_DUTY				(600U)
#define BLDC_MAX_RPM_TARGET				(6000.0f)
#define BLDC_RPM_RAMP_STEP				(20.0f)
#define BLDC_SPEED_PID_KP				(0.80f)
#define BLDC_SPEED_PID_KI				(0.02f)
#define BLDC_SPEED_PID_KD				(0.00f)
#define BLDC_POSITION_PID_KP			(12.0f)
#define BLDC_POSITION_PID_KI			(0.00f)
#define BLDC_POSITION_PID_KD			(0.50f)
#define BLDC_POSITION_MAX_RPM			(1200.0f)
#define BLDC_POSITION_DEADBAND_DEG		(15.0f)
#define BLDC_POSITION_MIN_PULSE			(650U)
#define BLDC_POSITION_MAX_PULSE			(1800U)
#define BLDC_POSITION_PULSE_KP			(10.0f)

#define BLDC_POLE_PAIRS					(2U)
#define BLDC_HALL_TIMER_HZ				(84000000UL / 84UL)
#define BLDC_HALL_MIN_TICKS				(8U)
#define BLDC_HALL_TIMEOUT_MS			(300U)
#define BLDC_MECH_SECTORS_PER_REV		(6U * BLDC_POLE_PAIRS)
#define BLDC_MECH_DEG_PER_SECTOR		(360.0f / (float)BLDC_MECH_SECTORS_PER_REV)

#define BLDC_SD_ENABLE()				HAL_GPIO_WritePin(BLDC_SD_GPIO_Port,BLDC_SD_Pin,GPIO_PIN_SET)
#define BLDC_SD_DISABLE()				HAL_GPIO_WritePin(BLDC_SD_GPIO_Port,BLDC_SD_Pin,GPIO_PIN_RESET)

/* enum ----------------------------------------------------------------------*/
typedef enum
{
	PHASE_U = 0,
	PHASE_V,
	PHASE_W,
	PHASE_NONE
}Phase_t;

typedef enum
{
	MOTOR_REV = 0,
	MOTOR_FWD,
}MotorDir_t;

typedef enum
{
	BLDC_CTRL_SPEED = 0,
	BLDC_CTRL_POSITION,
}BLDC_CtrlMode_t;

typedef enum
{
	BLDC_U_Current,
	BLDC_V_Current,
	BLDC_W_Current,
	BLDC_PowerVoltage,
	BLDC_MotorTemperature,
}ADC3_ChannelIndex_t;

/* types ---------------------------------------------------------------------*/
typedef struct
{
	Phase_t PwmPhase;
	Phase_t LowPhase;
}BLDCMosCom_t;

typedef struct
{
	float U_PhaseCurrent;
	float V_PhaseCurrent;
	float W_PhaseCurrent;
}CurrentPhase_t;

typedef struct
{
	float U_PhaseSetV;
	float V_PhaseSetV;
	float W_PhaseSetV;
}PhaseSetV_t;

typedef struct
{
	float U_CurrFilt;
	float V_CurrFilt;
	float W_CurrFilt;
}PhaseCurrFilt_t;

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float PreError;
	float SumError;
	float Output;
}BLDC_PID_Pos_t;

typedef struct
{
	uint32_t HallTickBuf[3];
	uint8_t  Index;
	uint8_t  ValidCnt;
	uint32_t LastFilter;
	uint8_t  Inited;
}HallSpeedFilter_t;

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
	float CurrentAngleDeg;
	float ExpectedAngleDeg;
	BLDC_PID_Pos_t PIDPos_SpeedLoop;
	BLDC_PID_Pos_t PIDPos_PositionLoop;
	Phase_t ActivePwmPhase;
	Phase_t ActiveLowPhase;
	BLDC_CtrlMode_t CtrlMode;
	MotorDir_t Direction;
	int32_t HallStepCount;
	uint16_t Pulse;
	uint8_t PositionCmdActive;
	uint8_t MotorRunning;
	uint8_t MotorStalling;
}BLDC_Info_t;

typedef struct
{
	volatile uint32_t HallTickCnt;
	volatile uint8_t  HallEdgeFlag;
	volatile uint8_t  HallStateShadow;
	volatile uint32_t HallLastEdgeMs;
	uint8_t  HallFirstEdge;

	HallSpeedFilter_t HallSpeedFilter;
}Hall_Info_t;

/* global variable -----------------------------------------------------------*/
extern BLDC_Info_t BLDC_Info;
extern Hall_Info_t Hall_Info;
extern BLDCMosCom_t *pHallTable;

/* functions prototypes ------------------------------------------------------*/
void Hall_enable( void );
void Hall_Disable( void );
void Hall_Start( void );
uint8_t Hall_GetState( void );
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
float BLDC_GetExpectedRPM( void );
void BLDC_SetExpectedAngle( float expectedAngleDeg );
float BLDC_GetExpectedAngle( void );
float BLDC_GetCurrentAngle( void );
void BLDC_SetPulse( int32_t duty );
uint16_t BLDC_GetPulse( void );
void BLDC_SetDirection( MotorDir_t dir );
MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC );
void BLDC_HallTableSelect( MotorDir_t Dir );
void BLDC_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty );
void BLDC_OnHallTransition( uint8_t previousHall, uint8_t currentHall );
void BLDC_PositionTask( void );

void BLDC_Cyclic( void );

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
