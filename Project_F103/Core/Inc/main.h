/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "list.h"
#include "queue.h"
#include "timers.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define IWDG_ENABLE 1
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOE
#define OV7725_D0_Pin GPIO_PIN_0
#define OV7725_D0_GPIO_Port GPIOC
#define OV7725_D1_Pin GPIO_PIN_1
#define OV7725_D1_GPIO_Port GPIOC
#define OV7725_D2_Pin GPIO_PIN_2
#define OV7725_D2_GPIO_Port GPIOC
#define OV7725_D3_Pin GPIO_PIN_3
#define OV7725_D3_GPIO_Port GPIOC
#define KEY_UP_Pin GPIO_PIN_0
#define KEY_UP_GPIO_Port GPIOA
#define OV7725_D4_Pin GPIO_PIN_4
#define OV7725_D4_GPIO_Port GPIOC
#define OV7725_D5_Pin GPIO_PIN_5
#define OV7725_D5_GPIO_Port GPIOC
#define NORFLASH_CS_Pin GPIO_PIN_12
#define NORFLASH_CS_GPIO_Port GPIOB
#define OV7725_D6_Pin GPIO_PIN_6
#define OV7725_D6_GPIO_Port GPIOC
#define OV7725_D7_Pin GPIO_PIN_7
#define OV7725_D7_GPIO_Port GPIOC
#define OV7725_VSYNC_Pin GPIO_PIN_8
#define OV7725_VSYNC_GPIO_Port GPIOA
#define OV7725_VSYNC_EXTI_IRQn EXTI9_5_IRQn
#define SCCB_SCL_Pin GPIO_PIN_3
#define SCCB_SCL_GPIO_Port GPIOD
#define OV7725_WRST_Pin GPIO_PIN_6
#define OV7725_WRST_GPIO_Port GPIOD
#define SCCB_SDA_Pin GPIO_PIN_13
#define SCCB_SDA_GPIO_Port GPIOG
#define OV7725_RRST_Pin GPIO_PIN_14
#define OV7725_RRST_GPIO_Port GPIOG
#define OV7725_OE_Pin GPIO_PIN_15
#define OV7725_OE_GPIO_Port GPIOG
#define OV7725_WEN_Pin GPIO_PIN_3
#define OV7725_WEN_GPIO_Port GPIOB
#define OV7725_RCLK_Pin GPIO_PIN_4
#define OV7725_RCLK_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOB
#define IIC_SCL_Pin GPIO_PIN_6
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_7
#define IIC_SDA_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED0_ON HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)
#define LED0_OFF HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)
#define LED0_TOGGLE HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)

#define LED1_ON HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)
#define BEEP_TOGGLE HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
