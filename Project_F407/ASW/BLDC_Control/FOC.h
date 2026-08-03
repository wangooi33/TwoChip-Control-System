#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* =============================================================================
 *  FOC 配置参数
 *
 *  [TIM8 PWM]
 *    时钟 = 84 MHz, 预分频 = 1, 中心对齐计数频率 = 42 MHz
 *    ARR = 5599 -> PWM 周期 = 2 * 5599 / 42MHz ≈ 266.7 us
 *    死区 = 20 时钟数 / 168 MHz ≈ 119 ns
 *
 *  [控制率]
 *    电流环 1kHz, 速度环 10ms(100Hz), 位置环 100ms(10Hz)
 *    与附件流程一致: 电流环 -> 速度环 -> 位置环 级联
 *
 *  [Id/Iq 电流环]
 *    位置式 PI + 积分限幅 + 退积分反馈抗饱和
 *    d 轴(磁场方向)控制为 0, q 轴(切线方向)输出力矩
 *
 *  [Hall 位置观测]
 *    霍尔 60 度跳变, 扇区内按时间比例线性插值连续电角度
 * ==========================================================================*/

/* TIM8 PWM 参数 */
#define FOC_PWM_PERIOD              (5599U)
#define FOC_DEADTIME_CYCLES         (20U)
#define FOC_TIM_CLK_HZ              (42000000.0f)   /* 中心对齐计数频率 */

/* 控制周期 */
#define FOC_CTRL_RATE_HZ            (1000.0f)
#define FOC_CTRL_PERIOD_S           (0.001f)

/* 电流环 PI 参数 */
#define FOC_KP_CURRENT              (0.60f)
#define FOC_KI_CURRENT              (0.04f)

/* 速度环 PI 参数 (10ms 执行) */
#define FOC_KP_SPEED                (2.20f)
#define FOC_KI_SPEED                (0.08f)
#define FOC_SPEED_LOOP_MS           (10U)
#define FOC_SPEED_LIMIT_mA          (5000.0f)   /* 速度环输出 Iq 限幅 */

/* 位置环 PI 参数 (100ms 执行) */
#define FOC_KP_POSITION             (80.0f)
#define FOC_KI_POSITION             (0.00f)
#define FOC_KD_POSITION             (4.00f)
#define FOC_POSITION_LOOP_MS        (100U)
#define FOC_POSITION_LIMIT_RPM      (3000.0f)   /* 位置环输出转速限幅 */

/* 限幅 */
#define FOC_VOLTAGE_LIMIT           (0.95f)
#define FOC_CURRENT_LIMIT_mA        (5000.0f)

/* 数学常量 */
#define DEG_TO_RAD                  (0.01745329252f)
#define INV_SQRT3                   (0.57735026919f)

/* =============================================================================
 *  数据结构
 * ==========================================================================*/
/* 位置式 PID，带积分限幅与退积分反馈抗饱和 */
typedef struct
{
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */
    float Integral;     /* 积分累积 */
    float Output;       /* 当前输出 */
    float Limit;        /* 输出限幅 */
    float IntLimit;     /* 积分限幅 */
    float Kb;           /* 退积分反馈增益 */
    float PrevErr;      /* 上一次误差 */
    float PrevOut;      /* 上一次输出 */
    float Dt;           /* 执行周期 [s] */
} FOC_PID_t;

typedef FOC_PID_t FOC_PI_t;     /* 兼容旧代码别名 */

/* FOC 环路模式 */
typedef enum
{
    FOC_LOOP_CURRENT = 0,   /* 直接电流给定 */
    FOC_LOOP_SPEED,         /* 速度环 -> 电流环 */
    FOC_LOOP_POSITION,      /* 位置环 -> 速度环 -> 电流环 */
} FOC_LoopMode_t;

typedef struct
{
    /* ---- 角度估计: Hall 扇区线性插值 ---- */
    float ThetaElec_Rad;            /* 连续电角度 [rad], [0, 2*PI) */
    float OmegaElec_RadPs;          /* 电角速度 [rad/s] */
    float PositionDeg;              /* 机械位置反馈 [deg] */
    float SpeedRPM;                 /* 转速反馈 [rpm] */

    /* ---- 三相电流 ---- */
    float Iu_mA;
    float Iv_mA;
    float Iw_mA;

    /* ---- 变换中间量 ---- */
    float Ialpha_mA;
    float Ibeta_mA;
    float Id_mA;
    float Iq_mA;

    /* ---- 电流给定 ---- */
    float Id_Ref_mA;
    float Iq_Ref_mA;

    /* ---- 外环给定 ---- */
    float SpeedRef_RPM;
    float PositionRef_Deg;

    /* ---- 电压命令 ---- */
    float Vd;                       /* d 轴电压 (PI 原始输出) */
    float Vq;                       /* q 轴电压 (PI 原始输出) */
    float Valpha;                   /* α 轴电压 [V] */
    float Vbeta;                    /* β 轴电压 [V] */

    /* ---- 电流偏置 (校准零漂) ---- */
    float Iu_OffsetV;
    float Iv_OffsetV;
    float Iw_OffsetV;

    /* ---- PID 控制器 ---- */
    FOC_PID_t PID_Id;
    FOC_PID_t PID_Iq;
    FOC_PID_t PID_Speed;
    FOC_PID_t PID_Position;

    /* ---- 环路调度 ---- */
    uint32_t LoopCounter;           /* 1ms 计数 */
    FOC_LoopMode_t LoopMode;

    /* ---- 状态标志 ---- */
    uint8_t Calibrated;
    uint8_t Enabled;
} FOC_Info_t;

/* =============================================================================
 *  全局变量 & 函数声明
 * ==========================================================================*/
extern FOC_Info_t FOC_Info;

void  FOC_Init( void );
void  FOC_Enable( void );
void  FOC_Disable( void );
void  FOC_Update( void );

/* 电流给定模式 */
void  FOC_SetIqRef( float iq_mA );
void  FOC_SetIdRef( float id_mA );
float FOC_GetIqRef( void );
float FOC_GetIdRef( void );

/* 速度/位置模式 */
void  FOC_SetSpeedRef( float rpm );
float FOC_GetSpeedRPM( void );
void  FOC_SetPositionRef( float deg );
float FOC_GetPositionDeg( void );

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
