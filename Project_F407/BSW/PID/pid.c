#include "pid.h"

void PID_Init( PID_t *pPID, float Kp, float Ki, float Kd,
               float limit, float intLimit, float Kb, float dt )
{
    pPID->Kp       = Kp;
    pPID->Ki       = Ki;
    pPID->Kd       = Kd;
    pPID->Integral = 0.0f;
    pPID->Output   = 0.0f;
    pPID->Limit    = limit;
    pPID->IntLimit = intLimit;
    pPID->Kb       = Kb;
    pPID->PrevErr  = 0.0f;
    pPID->PrevOut  = 0.0f;
    pPID->Dt       = dt;
}

void PID_Reset( PID_t *pPID )
{
    pPID->Integral = 0.0f;
    pPID->Output   = 0.0f;
    pPID->PrevErr  = 0.0f;
    pPID->PrevOut  = 0.0f;
}
/* ref(reference):目标值 / 给定值 , fdb(feedback):反馈值 / 实测值 */
float PID_Update( PID_t *pPID, float ref, float fdb )
{
    float error = ref - fdb;
    float dt = pPID->Dt;
    float deriv = 0.0f;

    /* 微分项 */
    if ( dt > 0.0f )
    {
        deriv = pPID->Kd * (error - pPID->PrevErr) / dt;
    }

    /* 积分累积 + 积分限幅 */
    pPID->Integral += pPID->Ki * error * dt;
    pPID->Integral  = Clampf(pPID->Integral, -pPID->IntLimit, pPID->IntLimit);

    /* 限幅前输出 */
    float outUnlim = pPID->Kp * error + pPID->Integral + deriv;

    /* 限幅后输出 */
    float outSat = Clampf(outUnlim, -pPID->Limit, pPID->Limit);

    /* 退积分反馈: 输出饱和时把多余积分退回去 */
    float diff = outSat - outUnlim;
    if ( pPID->Kb > 0.0f )
    {
        pPID->Integral += pPID->Kb * diff * dt;
    }

    pPID->PrevErr = error;
    pPID->PrevOut = outSat;
    pPID->Output  = outSat;
    return outSat;
}
