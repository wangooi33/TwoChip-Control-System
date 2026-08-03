#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* =============================================================================
 *  可移植 PID 模块
 *
 *  位置式 PID + 积分限幅 + 退积分反馈抗饱和:
 *    error  = ref - fdb
 *    integral += Ki * error * dt      (限幅到 IntLimit)
 *    output = Kp*error + integral + Kd*(error - PrevErr)/dt
 *    output 限幅到 Limit
 *    输出饱和时把差值 * Kb 反馈回积分器, 自动退积分
 *
 *  该模块不依赖任何硬件, 可直接复用到 BLDC / FOC / BDC 等控制环。
 * ==========================================================================*/

/* 浮点数钳位工具 */
static inline float Clampf( float value, float min, float max )
{
    if ( value < min )
    {
        return min;
    }
    if ( value > max )
    {
        return max;
    }
    return value;
}

/* PID 控制对象 */
typedef struct
{
    float Kp;           /* 比例增益 */
    float Ki;           /* 积分增益 */
    float Kd;           /* 微分增益 */
    float Integral;     /* 积分累积值 */
    float Output;       /* 当前输出 */
    float Limit;        /* 输出限幅 */
    float IntLimit;     /* 积分限幅 */
    float Kb;           /* 退积分反馈增益, 一般 1~5 */
    float PrevErr;      /* 上一次误差 */
    float PrevOut;      /* 上一次输出 */
    float Dt;           /* 控制周期 [s] */
} PID_t;

/* 初始化 PID 参数并清零状态 */
void PID_Init( PID_t *pPID, float Kp, float Ki, float Kd,
               float limit, float intLimit, float Kb, float dt );

/* 复位积分和历史状态, 保留参数 */
void PID_Reset( PID_t *pPID );

/* 执行一次 PID 运算, 返回限幅后的输出 */
float PID_Update( PID_t *pPID, float ref, float fdb );

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
