/* =============================================================================
 *  EC11 旋钮编码器
 *
 *  EC11 内部集成机械编码器，旋转时输出 A/B 两相正交脉冲，
 *  TIM3 编码器模式自动判向并计数。
 *
 *  本模块负责:
 *    1. 周期性读取 TIM3 计数值差值
 *    2. 异常跳变防抖与限幅
 *    3. 将计数折算为机械角度步进
 *    4. 维护绝对目标角度，并下发到 BLDC 位置环
 * ==========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "ec11.h"
#include "BLDC_Control.h"

/* global variable -----------------------------------------------------------*/
int16_t EC11_EncoderLastCnt = 0;
float EC11_PulseCnt = 0.0f;
float EC11_TargetAngleDeg = 0.0f;
float EC11_AbsoluteAngleDeg = 0.0f;
float EC11_SpeedDegPerSec = 0.0f;

/* local variable ------------------------------------------------------------*/
static int16_t EC11_CountRemainder = 0;

/* public functions ----------------------------------------------------------*/

/* 初始化: 以当前电机角度作为绝对角度基准 */
void EC11_Init( void )
{
    EC11_EncoderLastCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    EC11_CountRemainder = 0;
    EC11_PulseCnt = 0.0f;
    EC11_SpeedDegPerSec = 0.0f;
    EC11_AbsoluteAngleDeg = BLDC_GetCurrentAngle();
    EC11_TargetAngleDeg = EC11_AbsoluteAngleDeg;
}

/* 周期任务: 读取编码器差值, 更新目标角度 */
void EC11_Cyclic( void )
{
    int16_t nowCnt;
    int16_t delta;
    int16_t signedDelta;
    int16_t stepDelta = 0;

    nowCnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    delta = nowCnt - EC11_EncoderLastCnt;
    EC11_EncoderLastCnt = nowCnt;

    if ( delta == 0 )
    {
        EC11_SpeedDegPerSec = 0.0f;
        return;
    }

    /* 异常跳变限幅: 单周期变化过大视为毛刺/误触, 忽略本次并重新对齐 */
    if ( delta > EC11_MAX_DELTA_PER_CYCLE || delta < -EC11_MAX_DELTA_PER_CYCLE )
    {
        EC11_EncoderLastCnt = nowCnt;
        EC11_CountRemainder = 0;
        EC11_SpeedDegPerSec = 0.0f;
        return;
    }

    signedDelta = (int16_t)(EC11_DIR_SIGN * delta);
    EC11_PulseCnt += (float)signedDelta;
    EC11_CountRemainder += signedDelta;

    /* 将编码器计数折算成整格步进, 保证一格只下发一次 18° 目标 */
    while ( EC11_CountRemainder >= EC11_COUNTER_X )
    {
        stepDelta++;
        EC11_CountRemainder -= EC11_COUNTER_X;
    }
    while ( EC11_CountRemainder <= -EC11_COUNTER_X )
    {
        stepDelta--;
        EC11_CountRemainder += EC11_COUNTER_X;
    }

    /* 旋转速度估算: 度/秒 */
    EC11_SpeedDegPerSec = (float)signedDelta * EC11_DEG_PER_COUNT
                        * (1000.0f / (float)EC11_CYCLE_PERIOD_MS);

    if ( stepDelta == 0 )
    {
        return;
    }

    /* 维护绝对目标角度, 并激活 BLDC 位置环 */
    EC11_AbsoluteAngleDeg += (float)stepDelta * EC11_DEG_PER_STEP;
    BLDC_SetExpectedAngle(EC11_AbsoluteAngleDeg);
    EC11_TargetAngleDeg = BLDC_GetExpectedAngle();
}
