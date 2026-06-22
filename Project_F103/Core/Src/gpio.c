/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "task_manage.h"
#include "ov7725.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TOUCH_SPI_MO_Pin|TOUCH_SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TOUCH_SPI_CLK_Pin|NORFLASH_CS_Pin|OV7725_WEN_Pin|OV7725_RCLK_Pin
                          |LED0_Pin|IIC_SCL_Pin|IIC_SDA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SCCB_SCL_Pin|OV7725_WRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SCCB_SDA_Pin|OV7725_RRST_Pin|OV7725_OE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY1_Pin KEY0_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_SPI_MO_Pin TOUCH_SPI_CS_Pin */
  GPIO_InitStruct.Pin = TOUCH_SPI_MO_Pin|TOUCH_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_PEN_Pin */
  GPIO_InitStruct.Pin = TOUCH_PEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_PEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OV7725_D0_Pin OV7725_D1_Pin OV7725_D2_Pin OV7725_D3_Pin
                           OV7725_D4_Pin OV7725_D5_Pin OV7725_D6_Pin OV7725_D7_Pin */
  GPIO_InitStruct.Pin = OV7725_D0_Pin|OV7725_D1_Pin|OV7725_D2_Pin|OV7725_D3_Pin
                          |OV7725_D4_Pin|OV7725_D5_Pin|OV7725_D6_Pin|OV7725_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_UP_Pin */
  GPIO_InitStruct.Pin = KEY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_SPI_CLK_Pin NORFLASH_CS_Pin OV7725_WEN_Pin OV7725_RCLK_Pin */
  GPIO_InitStruct.Pin = TOUCH_SPI_CLK_Pin|NORFLASH_CS_Pin|OV7725_WEN_Pin|OV7725_RCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_SPI_MI_Pin */
  GPIO_InitStruct.Pin = TOUCH_SPI_MI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_SPI_MI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV7725_VSYNC_Pin */
  GPIO_InitStruct.Pin = OV7725_VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OV7725_VSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCCB_SCL_Pin */
  GPIO_InitStruct.Pin = SCCB_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCCB_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OV7725_WRST_Pin */
  GPIO_InitStruct.Pin = OV7725_WRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OV7725_WRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCCB_SDA_Pin */
  GPIO_InitStruct.Pin = SCCB_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SCCB_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OV7725_RRST_Pin OV7725_OE_Pin */
  GPIO_InitStruct.Pin = OV7725_RRST_Pin|OV7725_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED0_Pin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IIC_SCL_Pin IIC_SDA_Pin */
  GPIO_InitStruct.Pin = IIC_SCL_Pin|IIC_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
	if (GPIO_Pin == OV7725_VSYNC_Pin)
	{
		if (OV7725_Info.FrameHandleSt == Frame_Done)
		{
			OV7725_Info.FrameHandleSt = Frame_Pend;
			vTaskNotifyGiveFromISR(wTaskHandle.Task4_CameraHandle,&pxHigherPriorityTaskWoken);
		}
		else
		{
			OV7725_WEN(0);
		}
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
    
}

/* USER CODE END 2 */
