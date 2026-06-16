#include "task_manage.h"
#include "iwdg.h"

/* global variable ----------------------------------------------------------*/
wTaskHandle_st wTaskHandle;

/* function implementation --------------------------------------------------*/
static void TaskCreateError(uint8_t taskid)
{
	uint8_t Errid;
	if (taskid == 0)
	{
		for (;;);
	}
	else
	{
		Errid = taskid;
	}
}
static void AllTaskCreat(void *pvParameters)
{
	uint8_t xReturn = pdFAIL;
	
	xReturn = xTaskCreate(Task1_RainbowRGB, "Task1_RGB", 64, NULL, 0, &wTaskHandle.Task1_RGBHandle);
	if (xReturn != pdPASS)
	{
		TaskCreateError(1);
	}

//	//需要二次确认
//	xReturn = xTaskCreate(Task2_IAP, "Task2_IAP", 128, NULL, 3, &wTaskHandle.Task2_IAPHandle);
//	if (xReturn != pdPASS)
//	{
//		TaskCreateError(2);
//	}

	xReturn = xTaskCreate(Task3_InfraredScan, "Task3_IRCtrl", 512, NULL, 3, &wTaskHandle.Task3_IRCtrlHandle);
	if (xReturn != pdPASS)
	{
		TaskCreateError(3);
	}
	
	vTaskDelete(wTaskHandle.Task0_RootHandle);
}
static void AllQueueCreat(void *pvParameters)
{
	//Usart1RxQueue = xQueueCreate(1, sizeof(UsartRxMsg_t));

}

void RootTaskCreate(void *pvParameters)
{
	uint8_t xReturn = pdFAIL;
	
	xReturn = xTaskCreate(AllTaskCreat, "RootTask", 512, NULL, 0, &wTaskHandle.Task0_RootHandle);
	if (pdPASS != xReturn)
	{
		TaskCreateError(0);
	}
	AllQueueCreat(NULL);
}

void vApplicationIdleHook(void)
{
	/* 空闲任务钩子 */
    /* 低功耗、统计、喂狗都可以放这 */
#if IWDG_ENABLE
	HAL_IWDG_Refresh(&hiwdg);
#endif
}
void vApplicationStackOverflowHook(TaskHandle_t xTask,char *pcTaskName)
{
	/* 栈溢出钩子 */
	(void)xTask;
	(void)pcTaskName;

	taskDISABLE_INTERRUPTS();
	for (;;);
}
void vApplicationTickHook(void)
{
	/* 当内核时钟跳动一次时，在内核上下文里，顺手执行一下钩子函数 */
}
void vApplicationMallocFailedHook(void)
{
	/* 内存分配失败钩子 */
	taskDISABLE_INTERRUPTS();
	for (;;);
}

