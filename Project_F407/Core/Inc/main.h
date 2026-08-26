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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "timers.h"
#include "w_adc.h"
#include "bldc_control.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern const char SoftWareID[];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOE
#define IU_Pin GPIO_PIN_3
#define IU_GPIO_Port GPIOA
#define IV_Pin GPIO_PIN_4
#define IV_GPIO_Port GPIOA
#define IW_Pin GPIO_PIN_6
#define IW_GPIO_Port GPIOA
#define VBUS_Pin GPIO_PIN_0
#define VBUS_GPIO_Port GPIOB
#define TEMP_Pin GPIO_PIN_1
#define TEMP_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOA
#define SD_Pin GPIO_PIN_12
#define SD_GPIO_Port GPIOG
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED1_ON HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED1_TOGGLE HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)

#define LED2_ON HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define LED2_TOGGLE HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)

#define LED3_ON HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)
#define LED3_OFF HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED3_TOGGLE HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#define LED4_ON HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED4_OFF HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED4_TOGGLE HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin)

#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)
#define BEEP_TOGGLE HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
