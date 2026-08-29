#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* types ---------------------------------------------------------------------*/
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Integral;		/* 积分累积值 */
	float Limit;		/* 输出限幅 */
	float Kb;			/* 退积分反馈增益 */
	float PrevErr;		/* 上一次误差 */
	float Ts;
} PID_t;

/* functions prototypes ------------------------------------------------------*/
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

void PID_Init(PID_t *pPID, float Kp, float Ki, float Kd,float limit, float Kb ,float Ts);
float PID_Update( PID_t *pPID, float ref, float fdb );

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
