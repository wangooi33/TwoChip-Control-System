/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"
#include "FOC.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;

/* local helpers -------------------------------------------------------------*/
static void DisableAllMos( void )
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
}

/* public functions ----------------------------------------------------------*/
void BLDC_Disable( void )
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    DisableAllMos();
    BLDC_SD_DISABLE();
}

/* 启动 FOC: 使能驱动 + 启动矢量控制 */
void BLDC_Start( void )
{
    BLDC_Info.MotorStalling = 0U;
    BLDC_Info.MotorRunning = 1U;
    BLDC_SD_ENABLE();
    FOC_Enable();       /* Hall + TIM8 中心对齐 + PWM */
}

/* 正常停止 */
void BLDC_Stop( void )
{
    BLDC_Info.MotorRunning = 0U;
    FOC_Disable();
    BLDC_Disable();
}

/* 故障停机 */
void BLDC_TripStop( void )
{
    BLDC_Info.MotorStalling = 1U;
    BLDC_Stop();
}