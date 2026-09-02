/* includes ------------------------------------------------------------------*/
#include "hall.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
Hall_Info_t Hall_Info;

/* public functions ----------------------------------------------------------*/
void Hall_Enable(void)
{
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_TRIGGER);
	HAL_TIMEx_HallSensor_Start_IT(&htim3);
}

