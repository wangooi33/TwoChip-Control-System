#include "pid.h"

void PID_Init(PID_t *pPID, float Kp, float Ki, float Kd,float limit, float Kb ,float Ts)
{
	pPID->Kp = Kp;
	pPID->Ki = Ki;
	pPID->Kd = Kd;
	pPID->Integral = 0.0f;
	pPID->Limit = limit;
	pPID->Kb = Kb;
	pPID->PrevErr = 0.0f;
	pPID->Ts = Ts;
}
/* ref(reference):目标值 / 给定值 , fdb(feedback):反馈值 / 实测值 */
float PID_Update(PID_t *pPID, float ref, float fdb)
{
	float OutUnlim,Out;
	float error = ref - fdb;

	pPID->Integral += pPID->Ki * error * pPID->Ts;				/* 离散积分项,Ts = 1 / 控制频率(PWM频率) */
	OutUnlim = pPID->Kp * error + pPID->Integral + pPID->Kd * (error - pPID->PrevErr) / pPID->Ts;
	Out = Clampf(OutUnlim,-pPID->Limit,pPID->Limit);
	pPID->Integral += pPID->Kb * (Out - OutUnlim) * pPID->Ts;	/* 抗积分饱和 */

	pPID->PrevErr = error;
	return Out;
}

