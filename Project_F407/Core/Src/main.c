/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "beep.h"
#include "BDC_Control.h"
#include "BLDC_Control.h"
#include "ec11.h"
#include "timers.h"
#include "w_adc.h"
#include "communication.h"
#include "FOC.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const char SoftWareID[] = "S018";

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TASK_PERIOD_1MS       1
#define TASK_PERIOD_2MS       2
#define TASK_PERIOD_5MS       5
#define TASK_PERIOD_10MS      10

#define TASK_PERIOD_20MS      20
#define TASK_PERIOD_50MS      50
#define TASK_PERIOD_100MS     100
#define TASK_PERIOD_500MS     500
#define TASK_PERIOD_1000MS    1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint32_t SystemRunTime_1ms = 0;

uint32_t GetTick_1ms(void)
{
  return SystemRunTime_1ms;
}

void Task_1ms()
{
	TimersManagerTask();
	FOC_Update();
}
void Task_2ms()
{

}
void Task_5ms()
{
	ADC_Cyclic();
	BDC_Cyclic(&BDC_Info);

	EC11_Cyclic();
	BLDC_Cyclic();
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
	LED1_TOGGLE;
}
void Task_500ms()
{
	LED1_TOGGLE;
}
void Task_1000ms()
{
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  EC11_Init();
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)gADC1CaptureBuffer,ADC1_CAPTURE_BUF_MAXSIZE);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t *)gADC3CaptureBuffer,ADC3_CAPTURE_BUF_MAXSIZE);
  BDC_Disable();
  BDC_PIDInit(&BDC_Info);
  BLDC_Disable();
  BLDC_PIDInit(&BLDC_Info);
  BLDC_ResetControlState(&BLDC_Info);
  BLDC_PositionReset(&BLDC_Info);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_ConfigCommutationEvent(&htim8,TIM_TS_ITR3,TIM_COMMUTATION_SOFTWARE);
  Motor_CurrentOffsetCalibrate(&BDC_Info,&BLDC_Info);
  Com103_Init();
  FOC_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    TaskSchedule();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

