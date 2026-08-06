/* Includes ------------------------------------------------------------------*/
#include "key.h"
#include "BLDC_Control.h"
#include "FOC.h"

/* private variable ----------------------------------------------------------*/
uint8_t KeyCnt[KEY_NUM] = {0};
uint8_t KeyState[KEY_NUM] = {0};
static float KeySpeedTarget = 0.0f;

static GPIO_TypeDef* KEY_PORT[KEY_NUM] =
{
    KEY1_GPIO_Port,
    KEY2_GPIO_Port,
    KEY3_GPIO_Port,
    KEY4_GPIO_Port,
    KEY5_GPIO_Port
};

static uint16_t KEY_PIN[KEY_NUM] =
{
    KEY1_Pin,
    KEY2_Pin,
    KEY3_Pin,
    KEY4_Pin,
    KEY5_Pin
};

/* local helpers -------------------------------------------------------------*/
static KeyEvent_t KeyScan( void )
{
    for ( int i = 0; i < KEY_NUM; i++ )
    {
        if ( HAL_GPIO_ReadPin(KEY_PORT[i], KEY_PIN[i]) == GPIO_PIN_SET )
        {
            if ( KeyCnt[i] < DEBOUNCE_CNT )
            {
                KeyCnt[i]++;
            }
            if ( KeyCnt[i] >= DEBOUNCE_CNT && KeyState[i] == 0 )
            {
                KeyState[i] = 1;
                return (KeyEvent_t)(KEY1_PRESS + i);
            }
        }
        else
        {
            KeyCnt[i] = 0;
            KeyState[i] = 0;
        }
    }
    return KEY_NONE;
}

/* 设置 FOC 速度目标并确保电机已启动 */
static void KeySetSpeed( float rpm )
{
    KeySpeedTarget = rpm;
    FOC_SetSpeedRef(KeySpeedTarget);
    if ( BLDC_Info.MotorRunning == 0U )
    {
        BLDC_Start();
    }
}

/* public functions ----------------------------------------------------------*/
void KeyTask_Cyclic( void )
{
    switch ( KeyScan() )
    {
        case KEY1_PRESS:
            KeySetSpeed(1000.0f);
            break;

        case KEY2_PRESS:
            BLDC_Stop();
            break;

        case KEY3_PRESS:
            KeySetSpeed(KeySpeedTarget + 200.0f);
            break;

        case KEY4_PRESS:
            KeySetSpeed(KeySpeedTarget - 200.0f);
            break;

        case KEY5_PRESS:
            KeySetSpeed(-KeySpeedTarget);
            break;

        default:
            break;
    }
}
