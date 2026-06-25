#ifndef __TASK_MANAGE_H
#define __TASK_MANAGE_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "delay.h"
#include "at24Cxx.h"
#include "nm25Qxx.h"
#include "IAP.h"
#include "ov7725.h"
#include "lcd.h"
#include "lf0038.h"
#include "rgb_led.h"
#include "tjc.h"
#include "motor.h"
#include "w_can.h"

/* macro --------------------------------------------------------------------*/

/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/
typedef struct
{
	TaskHandle_t Task0_RootHandle;
	TaskHandle_t Task1_RGBHandle;
	TaskHandle_t Task2_IAPHandle;
	TaskHandle_t Task3_IRCtrlHandle;
	TaskHandle_t Task4_CameraHandle;
	TaskHandle_t Task5_IWDGHandle;
	TaskHandle_t Task6_CMotorHandshake;
	TaskHandle_t Task7_CMotorProcess;
}wTaskHandle_st;

/* global variable ----------------------------------------------------------*/
extern wTaskHandle_st wTaskHandle;

/* functions prototypes -----------------------------------------------------*/
void RootTaskCreate(void *pvParameters);


#endif  /* __TASK_MANAGE_H */
