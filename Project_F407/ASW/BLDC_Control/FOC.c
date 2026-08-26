/* includes ------------------------------------------------------------------*/
#include "foc.h"
#include "hall.h"
#include "w_adc.h"

/* global variable -----------------------------------------------------------*/

/* function implementation ---------------------------------------------------*/

/* Park变换 (αβ -> dq) */
void Park(float alpha, float beta, float theta, float *pId, float *pIq)
{
	float c = cosf(theta);
	float s = sinf(theta);
	*pId =  alpha * c + beta * s;
	*pIq = -alpha * s + beta * c;
}
/* 逆Park变换 (dq -> αβ) */
void RevPark(float vd, float vq, float theta, float *pValpha, float *pVbeta)
{
	float c = cosf(theta);
	float s = sinf(theta);
	*pValpha = vd * c - vq * s;
	*pVbeta  = vd * s + vq * c;
}
void SVPWM(float Valpha, float Vbeta, float Udc, float Tpwm, float *Tcm1, float *Tcm2, float *Tcm3)
{
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
			Tx = SQRT3 * Tpwm / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = SQRT3 * Tpwm / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		case 2:
			Tx = SQRT3 * Tpwm / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = -SQRT3 * Tpwm * Vbeta / Udc;
			break;
		case 3:
			Tx = -SQRT3 * Tpwm / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = SQRT3 * Tpwm * Vbeta / Udc;
			break;
		case 4:
			Tx = -SQRT3 * Tpwm * Vbeta / Udc;
			Ty = SQRT3 * Tpwm / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		case 5:
			Tx = SQRT3 * Tpwm * Vbeta / Udc;
			Ty = -SQRT3 * Tpwm / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
		default:
			Tx = -SQRT3 * Tpwm / Udc * (SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			Ty = -SQRT3 * Tpwm / Udc * (-SQRT3_BY_2 * Valpha + 0.5f * Vbeta);
			break;
	}

	/* 过调制 */
	T = Tx + Ty;
	if (T > Tpwm)
	{
		Tx = Tx * Tpwm / T;
		Ty = Ty * Tpwm / T;
	}
	T0 = Tpwm - Tx -Ty;
	
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


