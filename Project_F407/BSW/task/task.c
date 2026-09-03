/* Includes ------------------------------------------------------------------*/
#include "task.h"
#include "w_adc.h"
#include "key.h"
#include "hall.h"

/* global variable -----------------------------------------------------------*/
volatile uint32_t SystemRunTime_1ms = 0;

/* public functions ----------------------------------------------------------*/
uint32_t GetTick_1ms(void)
{
  return SystemRunTime_1ms;
}
void Task_1ms()
{
	if (Hall_Info.ClosedLoop_Flag == 1)
	{
		//BLDC_SpeedPID();
	}
}
void Task_2ms()
{

}
void Task_5ms()
{

}
void Task_10ms()
{

}
void Task_20ms()
{

}
void Task_50ms()
{
	KeyTask_Cyclic();
}
void Task_100ms()
{
	
}
void Task_500ms()
{
	BLDC_VBusCal();
}
void Task_1000ms()
{
	BLDC_TemperatureCal();
	LED4_TOGGLE;
}
void TaskSchedule()
{
    uint32_t now = GetTick_1ms();

    static uint32_t t1 = 0;
    static uint32_t t2 = 0;
    static uint32_t t5 = 0;
    static uint32_t t10 = 0;
    static uint32_t t20 = 0;
    static uint32_t t50 = 0;
    static uint32_t t100 = 0;
    static uint32_t t500 = 0;
    static uint32_t t1000 = 0;

    if ((int32_t)(now - t1) >= TASK_PERIOD_1MS)
    {
        t1 += TASK_PERIOD_1MS;
        Task_1ms();
    }

    if ((int32_t)(now - t2) >= TASK_PERIOD_2MS)
    {
        t2 += TASK_PERIOD_2MS;
        Task_2ms();
    }

    if ((int32_t)(now - t5) >= TASK_PERIOD_5MS)
    {
        t5 += TASK_PERIOD_5MS;
        Task_5ms();
    }

    if ((int32_t)(now - t10) >= TASK_PERIOD_10MS)
    {
        t10 += TASK_PERIOD_10MS;
        Task_10ms();
    }

    if ((int32_t)(now - t20) >= TASK_PERIOD_20MS)
    {
        t20 += TASK_PERIOD_20MS;
        Task_20ms();
    }

    if ((int32_t)(now - t50) >= TASK_PERIOD_50MS)
    {
        t50 += TASK_PERIOD_50MS;
        Task_50ms();
    }

    if ((int32_t)(now - t100) >= TASK_PERIOD_100MS)
    {
        t100 += TASK_PERIOD_100MS;
        Task_100ms();
    }

    if ((int32_t)(now - t500) >= TASK_PERIOD_500MS)
    {
        t500 += TASK_PERIOD_500MS;
        Task_500ms();
    }

    if ((int32_t)(now - t1000) >= TASK_PERIOD_1000MS)
    {
        t1000 += TASK_PERIOD_1000MS;
        Task_1000ms();
    }
}


