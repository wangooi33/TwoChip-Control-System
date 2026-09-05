/* includes ------------------------------------------------------------------*/
#include "hall.h"
#include "tim.h"
#include "foc.h"

/* global variable -----------------------------------------------------------*/
Hall_Info_t Hall_Info;
const uint8_t hall_sequence[6] = {0x06,0x04,0x05,0x01,0x03,0x02};
const float hall_angle_table[8] = 
{
	0.0f,
	3.0f * HALL_STEP_ANGLE,
	5.0f * HALL_STEP_ANGLE,
	4.0f * HALL_STEP_ANGLE,
	1.0f * HALL_STEP_ANGLE,
	2.0f * HALL_STEP_ANGLE,
	0.0f,
	0.0f
};

/* public functions ----------------------------------------------------------*/
float Angle_Normalize(float angle)
{
	while (angle >= TWO_PI)
	{
		angle -= TWO_PI;
	}
	while (angle < 0.0f)
	{
		angle += TWO_PI;
	}
	return angle;
}
int Hall_GetIndex(uint8_t state)
{
	for (int i = 0; i < 6; i++)
	{
		if (hall_sequence[i] == state)
		{
			return i;
		}
	}
	return -1;
}
int8_t Hall_GetDirection(uint8_t old_state, uint8_t new_state)
{
	int old_index = Hall_GetIndex(old_state);
	int new_index = Hall_GetIndex(new_state);

	if (old_index < 0 || new_index < 0)
	{
		return 0;
	}
	/* 正转 */
	if (((old_index + 1) % 6) == new_index)
	{
		return 1;
	}
	/* 反转 */
	if (((old_index + 5) % 6) == new_index)
	{
		return -1;
	}

	return 0;
}


void Hall_Enable(void)
{
	//__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_TRIGGER);
	HAL_TIMEx_HallSensor_Start_IT(&htim3);
}
uint8_t Hall_ReadState(void)
{
	uint8_t hall_state = 0;

	hall_state  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	hall_state |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) << 1;
	hall_state |= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) << 2;

	return hall_state;
}
void Hall_UpdateEdge(uint8_t hall_state, uint32_t hall_period)
{
	int8_t dir;

	if (hall_state == 0x0 || hall_state == 0x7)
	{
		return;
	}

	if (!Hall_Info.initialized)
	{
		Hall_Info.state = hall_state;
		Hall_Info.last_state = hall_state;
		Hall_Info.hall_angle =Angle_Normalize(hall_angle_table[hall_state] + PI / 6.0f);
		Hall_Info.angle = Hall_Info.hall_angle;
		Hall_Info.initialized = 1;
		return;
	}
	if (hall_state == Hall_Info.state)
	{
		return;
	}
	dir = Hall_GetDirection(Hall_Info.state,hall_state);
	if (dir == 0)
	{
		return;
	}
	if (hall_period == 0 || hall_period > 65535)
	{
		return;
	}
	Hall_Info.last_state = Hall_Info.state;
	Hall_Info.state = hall_state;
	Hall_Info.direction = dir;
	Hall_Info.hall_period = hall_period;

	/* 计时器频率 = 1MHz */
	Hall_Info.speed = (float)dir * HALL_STEP_ANGLE * 1000000.0f / (float)hall_period;
	Hall_Info.hall_angle =  Angle_Normalize(hall_angle_table[hall_state] + PI / 6.0f);

	Hall_Info.new_event = 1;
}
void Hall_Interpolate(float Ts)
{
	if (!Hall_Info.initialized)
	{
		return;
	}

	/* θ(k+1) = θ(k) + ωe × Ts */
	Hall_Info.angle += Hall_Info.speed * Ts;
	Hall_Info.angle = Angle_Normalize(Hall_Info.angle);
}

