#include "delay.h"
#include "tim.h"

void Delay_us(uint32_t us)
{
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim7);
	while ((__HAL_TIM_GET_COUNTER(&htim7) - start) < us);
}
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}
