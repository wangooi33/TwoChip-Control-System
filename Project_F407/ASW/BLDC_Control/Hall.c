/* includes ------------------------------------------------------------------*/
#include "Hall.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
Hall_Info_t Hall_Info =
{
    .HallFirstEdge = 1
};

/* local helpers -------------------------------------------------------------*/

/* 判断 Hall 状态跳变方向: 正转 +1, 反转 -1 */
static int8_t Hall_GetStepDelta( uint8_t previousHall, uint8_t currentHall )
{
    static const uint8_t hallForwardSeq[6] = {1U, 5U, 4U, 6U, 2U, 3U};
    int8_t previousIndex = -1;
    int8_t currentIndex = -1;
    uint8_t i;

    for ( i = 0U; i < 6U; i++ )
    {
        if ( hallForwardSeq[i] == previousHall )
        {
            previousIndex = (int8_t)i;
        }
        if ( hallForwardSeq[i] == currentHall )
        {
            currentIndex = (int8_t)i;
        }
    }

    if ( previousIndex < 0 || currentIndex < 0 )
    {
        return 0;
    }

    if ( hallForwardSeq[(previousIndex + 1) % 6] == currentHall )
    {
        return 1;
    }
    if ( hallForwardSeq[(previousIndex + 5) % 6] == currentHall )
    {
        return -1;
    }

    return 0;
}

/* public functions ----------------------------------------------------------*/

/* 启动 Hall 采集: 复位状态并打开 TIM5 捕获中断 */
void Hall_Start( void )
{
    Hall_Info.HallFirstEdge = 1U;
    Hall_Info.HallEdgeFlag = 0U;
    Hall_Info.HallTickCnt = 0U;
    Hall_Info.HallStateShadow = Hall_GetState();
    Hall_Info.HallLastEdgeMs = SystemRunTime_1ms;
    Hall_Info.HallSectorStartMs = SystemRunTime_1ms;
    Hall_Info.HallSectorPeriodMs = 0U;

    HAL_TIMEx_HallSensor_Start_IT(&htim5);
}

void Hall_Disable( void )
{
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIMEx_HallSensor_Stop(&htim5);
}

/* 读取三路 Hall 状态, 编码为 1~6 */
uint8_t Hall_GetState( void )
{
    uint8_t state = 0U;

    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 0;
    }
    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 1;
    }
    if ( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) != GPIO_PIN_RESET )
    {
        state |= 0x01U << 2;
    }
    return state;
}

/* 每个有效 Hall 跳变对应机械角度前进一步, 同时记录扇区时间用于插值 */
void Hall_OnTransition( uint8_t previousHall, uint8_t currentHall )
{
    int8_t stepDelta = Hall_GetStepDelta(previousHall, currentHall);
    uint32_t nowMs = SystemRunTime_1ms;

    if ( stepDelta == 0 )
    {
        if ( previousHall == currentHall || currentHall < 1U || currentHall > 6U )
        {
            return;
        }
        stepDelta = (BLDC_Info.Direction == MOTOR_FWD) ? 1 : -1;
    }

    /* 记录当前扇区用时, 并更新扇区起点时刻 */
    Hall_Info.HallSectorPeriodMs = nowMs - Hall_Info.HallSectorStartMs;
    Hall_Info.HallSectorStartMs = nowMs;

    BLDC_Info.HallStepCount += stepDelta;
    BLDC_Info.CurrentAngleDeg = (float)BLDC_Info.HallStepCount * HALL_DEG_PER_SECTOR;
}

