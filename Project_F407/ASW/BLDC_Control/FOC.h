#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* TIM8 PWM 参数 */
#define FOC_PWM_PERIOD                  (5599U)
#define FOC_DEADTIME_CYCLES             (20U)

/* 控制周期 */
#define FOC_CTRL_RATE_HZ                (1000.0f)
#define FOC_CTRL_PERIOD_S               (0.001f)

/* Id/Iq 电流环 PI 参数 */
#define FOC_KP_CURRENT                  (0.60f)
#define FOC_KI_CURRENT                  (0.04f)

/* Hall-PLL 观测器参数 */
#define FOC_PLL_KP                      (100.0f)
#define FOC_PLL_KI                      (10.0f)

/* 限幅 */
#define FOC_VOLTAGE_LIMIT               (0.95f)
#define FOC_CURRENT_LIMIT_mA            (5000.0f)

/* 数学常量 */
#define DEG_TO_RAD                      (0.01745329252f)
#define INV_SQRT3                       (0.57735026919f)


typedef struct
{
    float Kp;
    float Ki;
    float Integral;
    float Output;
    float Limit;
} FOC_PI_t;

typedef struct
{
    /* ---- 角度估计: Hall-PLL 观测器 ---- */
    float ThetaElec_Rad;            /* 连续电角度 [rad], [0, 2*PI) */
    float OmegaElec_RadPs;          /* 电角速度 [rad/s] */

    /* ---- 三相电流 ---- */
    float Iu_mA;
    float Iv_mA;
    float Iw_mA;

    /* ---- 变换中间量 ---- */
    float Ialpha_mA;                /* α 轴电流 (静止) */
    float Ibeta_mA;                 /* β 轴电流 (静止) */
    float Id_mA;                    /* d 轴电流 (旋转, 励磁) */
    float Iq_mA;                    /* q 轴电流 (旋转, 转矩) */

    /* ---- 电流给定 ---- */
    float Id_Ref_mA;                /* d 轴目标: Id=0 控制时恒为 0 */
    float Iq_Ref_mA;                /* q 轴目标: 正 = 正向转矩 */

    /* ---- 电压命令 ---- */
    float Vd;                       /* d 轴电压 (PI 原始输出, mA 量纲) */
    float Vq;                       /* q 轴电压 (PI 原始输出, mA 量纲) */
    float Valpha;                   /* α 轴电压 (归一化 [-1, 1]) */
    float Vbeta;                    /* β 轴电压 (归一化 [-1, 1]) */

    /* ---- 电流偏置 (校准零漂) ---- */
    float Iu_OffsetV;
    float Iv_OffsetV;
    float Iw_OffsetV;

    /* ---- PI 控制器 ---- */
    FOC_PI_t PID_Id;                /* d 轴电流环: 控制 Id → 0 */
    FOC_PI_t PID_Iq;                /* q 轴电流环: 控制 Iq → 转矩目标 */

    /* ---- 状态标志 ---- */
    uint8_t Calibrated;             /* 电流偏置校准完成 */
    uint8_t Enabled;                /* FOC 已使能, 正在运行 */
} FOC_Info_t;


extern FOC_Info_t FOC_Info;

void  FOC_Init( void );
void  FOC_SetIqRef( float iq_mA );
void  FOC_SetIdRef( float id_mA );
float FOC_GetIqRef( void );
float FOC_GetIdRef( void );
void  FOC_Enable( void );
void  FOC_Disable( void );
void  FOC_Update( void );

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
