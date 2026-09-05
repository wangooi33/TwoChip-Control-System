#ifndef __HALL_H
#define __HALL_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes ------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/
#define HALL_STEP_ANGLE			(3.14159265358979323846f / 3.0f)
#define TWO_PI					(2.0f * 3.14159265358979323846f)

/* types ---------------------------------------------------------------------*/
typedef struct
{
	uint8_t state;
	uint8_t last_state;
	float angle;			/* Park使用的连续电角度 */
	float hall_angle;		/* Hall当前状态对应的离散位置 */
	float speed;			/* 电角速度 rad/s */
	uint32_t hall_period;
	int8_t direction;
	uint8_t initialized;

	uint8_t new_event;
} Hall_Info_t;

/* global variable -----------------------------------------------------------*/
extern Hall_Info_t Hall_Info;
extern const uint8_t hall_sequence[6];
extern const float hall_angle_table[8];

/* functions prototypes ------------------------------------------------------*/
void Hall_Enable(void);
uint8_t Hall_ReadState(void);
void Hall_UpdateEdge(uint8_t hall_state, uint32_t now_time);
void Hall_Interpolate(float Ts);

float Angle_Normalize(float angle);
int Hall_GetIndex(uint8_t state);
int8_t Hall_GetDirection(uint8_t old_state, uint8_t new_state);

#ifdef __cplusplus
}
#endif

#endif /* __HALL_H */
