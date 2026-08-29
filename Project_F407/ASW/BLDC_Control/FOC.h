#ifndef __FOC_H
#define __FOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/

/* 通用数学常量 */
#define DEG_TO_RAD					(0.01745329252f)	/* = (π / 180),角度-弧度 */
#define SQRT3						1.732050807f
#define SQRT3_BY_2					0.866025403f
#define PI							3.14159265358979323846f

/* enum ----------------------------------------------------------------------*/

/* types ---------------------------------------------------------------------*/
typedef struct
{
	/* ---- 变换中间量 ---- */
	float Ialpha;
	float Ibeta;
	float Id;
	float Iq;
	float Valpha;
	float Vbeta;
	float Vd;
	float Vq;

	/* to motor */
	float Tcm1;
	float Tcm2;
	float Tcm3;

	/* ---- 期待值 ---- */
	float Id_Ref;				/* d轴电流目标 (励磁) */
	float Iq_Ref;				/* q轴电流目标 (力矩) */
	float SpeedRef;				/* 速度环目标 (RPM) */
	float PositionRef;			/* 位置环目标 (机械角度) */
} FOC_Info_t;

/* global variable -----------------------------------------------------------*/
extern FOC_Info_t FOC_Info;

/* functions prototypes ------------------------------------------------------*/
void Clark(float Ia, float Ib, float *pIalpha, float *pIbeta);
void Park(float Ialpha, float Ibeta, float theta, float *pId, float *pIq);
void RevPark(float Vd, float Vq, float theta, float *pValpha, float *pVbeta);
void SVPWM(float Valpha, float Vbeta, float Udc, float Tpwm, float *Tcm1, float *Tcm2, float *Tcm3);
void FOC_Init(FOC_Info_t *pFOC);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
