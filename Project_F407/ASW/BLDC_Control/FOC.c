/* includes ------------------------------------------------------------------*/
#include "foc.h"
#include "pid.h"
#include <math.h>

/* global variable -----------------------------------------------------------*/
FOC_Info_t FOC_Info;

/* function implementation ---------------------------------------------------*/

/* Clark变换   (abc -> αβ) */
void Clark(float Ia, float Ib, float *pIalpha, float *pIbeta)
{
	*pIalpha = Ia;
	*pIbeta  = (Ia + 2.0f * Ib) * 0.577350269f;
}
/* Park变换 (αβ -> dq) */
void Park(float Ialpha, float Ibeta, float theta, float *pId, float *pIq)
{
	float c = cosf(theta);
	float s = sinf(theta);
	*pId =  Ialpha * c + Ibeta * s;
	*pIq = -Ialpha * s + Ibeta * c;
}
/* 逆Park变换 (dq -> αβ) */
void RevPark(float Vd, float Vq, float theta, float *pValpha, float *pVbeta)
{
	float c = cosf(theta);
	float s = sinf(theta);
	*pValpha = Vd * c - Vq * s;
	*pVbeta  = Vd * s + Vq * c;
}
void SVPWM(float Valpha, float Vbeta, float Udc, float Tperiod_count, float *Tcm1, float *Tcm2, float *Tcm3)
{
	/* Tperiod_count:定时器一次计数周期对应的计数值(需要区分边沿对齐和中心对齐) */

	uint8_t sector = 0;
	float Tx,Ty,T0,T,Ta,Tb,Tc;

	/* 扇区判断 */
	if (Vbeta > 0)
	{
		sector |= 1;
	}
	if (SQRT3_BY_2 * Valpha - 0.5f * Vbeta > 0)
	{
		sector |= 2;
	}
	if (-SQRT3_BY_2 * Valpha - 0.5f * Vbeta > 0)
	{
		sector |= 4;
	}

	/* 矢量作用时间 */
	switch (sector)
	{
		case 1:
			Tx = SQRT3 * Tperiod_count / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = SQRT3 * Tperiod_count / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		case 2:
			Tx = SQRT3 * Tperiod_count / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = -SQRT3 * Tperiod_count * Vbeta / Udc;
			break;
		case 3:
			Tx = -SQRT3 * Tperiod_count / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = SQRT3 * Tperiod_count * Vbeta / Udc;
			break;
		case 4:
			Tx = -SQRT3 * Tperiod_count * Vbeta / Udc;
			Ty = SQRT3 * Tperiod_count / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		case 5:
			Tx = SQRT3 * Tperiod_count * Vbeta / Udc;
			Ty = -SQRT3 * Tperiod_count / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		default:
			Tx = -SQRT3 * Tperiod_count / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = -SQRT3 * Tperiod_count / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
	}

	/* 过调制 */
	T = Tx + Ty;
	if (T > Tperiod_count)
	{
		Tx = Tx * Tperiod_count / T;
		Ty = Ty * Tperiod_count / T;
	}
	T0 = Tperiod_count - Tx -Ty;
	
	/* 扇区切换时间点 */
	Ta = T0 / 4.0f;
	Tb = Ta + Tx / 2.0f;
	Tc = Tb + Ty / 2.0f;
	switch (sector)
	{
		case 1:
			*Tcm1 = Tb;
			*Tcm2 = Ta;
			*Tcm3 = Tc;
		    break;
		case 2:
			*Tcm1 = Ta;
			*Tcm2 = Tc;
			*Tcm3 = Tb;
			break;
		case 3:
			*Tcm1 = Ta;
			*Tcm2 = Tb;
			*Tcm3 = Tc;
			break;
		case 4:
			*Tcm1 = Tc;
			*Tcm2 = Tb;
			*Tcm3 = Ta;
			break;
		case 5:
			*Tcm1 = Tc;
			*Tcm2 = Ta;
			*Tcm3 = Tb;
			break;
		case 6:
			*Tcm1 = Tb;
			*Tcm2 = Tc;
			*Tcm3 = Ta;
			break;
	}
}

void FOC_Init(FOC_Info_t *pFOC)
{
	pFOC->Id_Ref = 0;
	pFOC->Iq_Ref = 1.0f;
}

