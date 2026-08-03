/* =============================================================================
 *  SixStep.c — BLDC 六步换相
 *
 *  本模块是"方波驱动"专属:
 *    1. 根据当前 Hall 状态从换向表选取 PWM 相和低边相
 *    2. 通过 TIM8 CCR + 低边 GPIO 驱动 MOSFET
 *
 *  FOC 模式不调用本模块, 由 FOC_Update 直接写 TIM8 CCR。
 * ==========================================================================*/

/* includes ------------------------------------------------------------------*/
#include "SixStep.h"
#include "tim.h"

/* global variable -----------------------------------------------------------*/
BLDCMosCom_t gComFwd[8] =
{
    [0] = {PHASE_NONE, PHASE_NONE},
    [1] = {PHASE_U, PHASE_W},
    [2] = {PHASE_V, PHASE_U},
    [3] = {PHASE_V, PHASE_W},
    [4] = {PHASE_W, PHASE_V},
    [5] = {PHASE_U, PHASE_V},
    [6] = {PHASE_W, PHASE_U},
    [7] = {PHASE_NONE, PHASE_NONE},
};
BLDCMosCom_t gComRev[8] =
{
    [0] = {PHASE_NONE, PHASE_NONE},
    [1] = {PHASE_W, PHASE_U},
    [2] = {PHASE_U, PHASE_V},
    [3] = {PHASE_W, PHASE_V},
    [4] = {PHASE_V, PHASE_W},
    [5] = {PHASE_V, PHASE_U},
    [6] = {PHASE_U, PHASE_W},
    [7] = {PHASE_NONE, PHASE_NONE},
};
BLDCMosCom_t *pHallTable = NULL;

/* public functions ----------------------------------------------------------*/

/* 关闭全部功率管, 避免上下桥直通 */
void SixStep_DisableAllMos( void )
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_RESET);
}

/* 选择正转/反转换向表 */
void SixStep_HallTableSelect( MotorDir_t Dir )
{
    if ( Dir == MOTOR_FWD )
    {
        pHallTable = gComFwd;
    }
    else
    {
        pHallTable = gComRev;
    }
}

/* 切换 MOSFET 状态 */
void SixStep_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty )
{
    SixStep_DisableAllMos();

    switch ( PwmPhase )
    {
        case PHASE_U:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Duty);
            break;
        case PHASE_V:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Duty);
            break;
        case PHASE_W:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Duty);
            break;
        default:
            return;
    }

    switch ( LowPhase )
    {
        case PHASE_U:
            HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_SET);
            break;
        case PHASE_V:
            HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_SET);
            break;
        case PHASE_W:
            HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_SET);
            break;
        default:
            return;
    }

    BLDC_Info.ActivePwmPhase = PwmPhase;
    BLDC_Info.ActiveLowPhase = LowPhase;
    HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);
}

/* 更新当前 PWM 相的占空比 */
void SixStep_UpdateActiveDuty( uint16_t duty )
{
    if ( BLDC_Info.ActivePwmPhase == PHASE_U )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, duty);
    }
    else if ( BLDC_Info.ActivePwmPhase == PHASE_V )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);
    }
    else if ( BLDC_Info.ActivePwmPhase == PHASE_W )
    {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, duty);
    }
}

/* 消费最新一次 Hall 沿: 更新转速估算, 再刷新当前换相 */
void SixStep_HallCyclic( void )
{
    uint8_t hall;

    if ( Hall_UpdateSpeed() == 0U )
    {
        return;
    }

    hall = Hall_Info.HallStateShadow;
    SixStep_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

    if ( hall >= 1U && hall <= 6U &&
         pHallTable[hall].PwmPhase != PHASE_NONE &&
         pHallTable[hall].LowPhase != PHASE_NONE )
    {
        SixStep_ChangeMOSstate(pHallTable[hall].PwmPhase,
                               pHallTable[hall].LowPhase,
                               BLDC_Info.Pulse);
    }
}

/* 启动六步驱动: 选表 + 启动 Hall 捕获 + 按当前 Hall 摆 MOS */
void SixStep_Start( void )
{
    uint8_t hall;

    SixStep_HallTableSelect(BLDC_GetDirection(&BLDC_Info));
    Hall_Start();

    hall = Hall_GetState();
    if ( hall >= 1U && hall <= 6U )
    {
        SixStep_ChangeMOSstate(pHallTable[hall].PwmPhase,
                               pHallTable[hall].LowPhase,
                               BLDC_Info.Pulse);
    }
}

/* 停止六步驱动 */
void SixStep_Stop( void )
{
    Hall_Disable();
    BLDC_Info.ActivePwmPhase = PHASE_NONE;
    BLDC_Info.ActiveLowPhase = PHASE_NONE;
}
