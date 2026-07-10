#include "task_manage.h"
#include "iwdg.h"

/* global variable ----------------------------------------------------------*/
wTaskHandle_st wTaskHandle;
static TimerHandle_t IWDGTimerHandle;

/* function implementation --------------------------------------------------*/
static void IWDG_TimerCallback(TimerHandle_t xTimer)
{
	(void)xTimer;
	HAL_IWDG_Refresh(&hiwdg);
}

/**
 * @note:Root task Function
 *
 */
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
	BaseType_t xReturn = pdFAIL;
	
	xReturn = xTaskCreate(Task1_RainbowRGB, "Task1_RGB", 64, NULL, 1, &wTaskHandle.Task1_RGBHandle);
	if (xReturn != pdPASS)
	{
		TaskCreateError(1);
	}

//	xReturn = xTaskCreate(Task2_IAP, "Task2_IAP", 128, NULL, 3, &wTaskHandle.Task2_IAPHandle);
//	if (xReturn != pdPASS)
//	{
//		TaskCreateError(2);
//	}

	xReturn = xTaskCreate(Task3_InfraredScan, "Task3_IRCtrl", 512, NULL, 2, &wTaskHandle.Task3_IRCtrlHandle);
	if (xReturn != pdPASS)
	{
		TaskCreateError(3);
	}

	CMotor_Init();
	xReturn = xTaskCreate(Task6_ComMotorHandshake, "Task6_CMotor", 256, NULL, 2, &wTaskHandle.Task6_CMotorHandshake);
	if (xReturn != pdPASS)
	{
		TaskCreateError(6);
	}

#if IWDG_ENABLE
	IWDGTimerHandle = xTimerCreate("TmrIWDG",
								   pdMS_TO_TICKS(500),
								   pdTRUE,
								   NULL,
								   IWDG_TimerCallback);
	if (IWDGTimerHandle == NULL)
	{
		TaskCreateError(5);
	}
	xReturn = xTimerStart(IWDGTimerHandle, 0);
	if (xReturn != pdPASS)
	{
		TaskCreateError(5);
	}
#endif
	
	vTaskDelete(wTaskHandle.Task0_RootHandle);
}
void RootTaskCreate(void *pvParameters)
{
	BaseType_t xReturn = pdFAIL;
	
	xReturn = xTaskCreate(AllTaskCreat, "RootTask", 512, NULL, 0, &wTaskHandle.Task0_RootHandle);
	if (pdPASS != xReturn)
	{
		TaskCreateError(0);
	}
}


/**
 * @note:Hook Function
 *
 */
void vApplicationIdleHook(void)
{
	/* 空闲任务钩子 */
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
	/* 内核时钟跳动钩子函数 */
}
void vApplicationMallocFailedHook(void)
{
	/* 内存分配失败钩子 */
	taskDISABLE_INTERRUPTS();
	for (;;);
}

