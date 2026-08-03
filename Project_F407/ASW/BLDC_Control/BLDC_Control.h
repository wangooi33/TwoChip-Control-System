#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* =============================================================================
 *  BLDC 控制参数
 * ==========================================================================*/
/* 电流保护阈值和电流目标限幅 */
#define BLDC_CURRENT_SOFT_LIMIT_mA      (5900.0f)   /* 超过该电流后开始逐步削减 PWM */
#define BLDC_CURRENT_RELEASE_mA         (5400.0f)   /* 软限流释放阈值，预留滞回 */
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

/* Hall 测速和机械角度换算参数 */
#define BLDC_POLE_PAIRS                 (2U)        /* 电机极对数 */
#define BLDC_HALL_TIMER_HZ              (84000000UL / 84UL)   /* TIM5 Hall 捕获计时频率 1MHz */
#define BLDC_HALL_MIN_TICKS             (8U)        /* 预留的 Hall 最小有效周期 */
#define BLDC_HALL_TIMEOUT_MS            (300U)      /* Hall 超时窗口，用于堵转检测 */
#define BLDC_MECH_SECTORS_PER_REV       (6U * BLDC_POLE_PAIRS)  /* 机械一圈对应 Hall 扇区数 */
#define BLDC_MECH_DEG_PER_SECTOR        (360.0f / (float)BLDC_MECH_SECTORS_PER_REV)  /* 每个 Hall 步进对应机械角度 */

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
    Phase_t PwmPhase;
    Phase_t LowPhase;
} BLDCMosCom_t;

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

/* 位置式 PID，带积分限幅与退积分反馈抗饱和 */
typedef struct
{
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */
    float PreError;     /* 上一次误差 */
    float SumError;     /* 积分累积值 */
    float Output;       /* 当前输出 */
    float OutLimit;     /* 输出限幅 */
    float IntLimit;     /* 积分限幅 */
    float Kb;           /* 退积分反馈增益，一般为 1~5 */
    float PrevOut;      /* 上一次输出 */
    float Dt;           /* 控制周期 [s] */
} BLDC_PID_Pos_t;

typedef struct
{
    uint32_t HallTickBuf[3];
    uint8_t  Index;
    uint8_t  ValidCnt;
    uint32_t LastFilter;
    uint8_t  Inited;
} HallSpeedFilter_t;

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
    BLDC_PID_Pos_t PIDPos_SpeedLoop;
    BLDC_PID_Pos_t PID_CurrentLoop;
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
} BLDC_Info_t;

typedef struct
{
    volatile uint32_t HallTickCnt;          /* 相邻 Hall 沿间隔 [1us 计数] */
    volatile uint8_t  HallEdgeFlag;         /* 有新 Hall 沿待处理 */
    volatile uint8_t  HallStateShadow;      /* 最近一次 Hall 状态 */
    volatile uint32_t HallLastEdgeMs;       /* 最近一次 Hall 沿时刻 [ms] */
    volatile uint32_t HallSectorStartMs;    /* 进入当前 Hall 扇区的时刻 [ms] */
    volatile uint32_t HallSectorPeriodMs;   /* 上一个 Hall 扇区用时 [ms] */
    uint8_t  HallFirstEdge;

    HallSpeedFilter_t HallSpeedFilter;
} Hall_Info_t;

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
float BLDC_PID_Calc( BLDC_PID_Pos_t *pPID, float ref, float fdb );
void BLDC_ResetControlState( BLDC_Info_t *pBLDC );
void BLDC_PositionReset( BLDC_Info_t *pBLDC );
void BLDC_SetExpectedRPM( float expectedRPM );
float BLDC_GetExpectedRPM( void );
void BLDC_SetExpectedCurrent( float expectedCurrent );
float BLDC_GetExpectedCurrent( void );
void BLDC_SetExpectedAngle( float expectedAngleDeg );
void BLDC_AddExpectedAngle( float deltaAngleDeg );
float BLDC_GetExpectedAngle( void );
float BLDC_GetCurrentAngle( void );
void BLDC_SetPulse( int32_t duty );
uint16_t BLDC_GetPulse( void );
void BLDC_SetDirection( MotorDir_t dir );
MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC );
void BLDC_HallTableSelect( MotorDir_t Dir );
void BLDC_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty );
void BLDC_OnHallTransition( uint8_t previousHall, uint8_t currentHall );
void BLDC_ControlTask( void );
void BLDC_PositionTask( void );
void BLDC_Cyclic( void );

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
