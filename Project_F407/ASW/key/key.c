/* Includes ------------------------------------------------------------------*/
#include "key.h"
#include "BLDC_Control.h"

/* private variable ----------------------------------------------------------*/
uint8_t KeyCnt[KEY_NUM] = {0};
uint8_t KeyState[KEY_NUM] = {0};

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

/* public functions ----------------------------------------------------------*/
void KeyTask_Cyclic( void )
{
	switch ( KeyScan() )
	{
		case KEY1_PRESS:
			BLDC_SetDirection(MOTOR_FWD);
			BLDC_SetExpectedCurrent(1200.0f);
			break;

		case KEY2_PRESS:
			BLDC_Stop();
			break;

		case KEY3_PRESS:
			BLDC_SetExpectedCurrent(BLDC_GetExpectedCurrent() + 200.0f);
			break;

		case KEY4_PRESS:
			if ( BLDC_GetExpectedCurrent() > 400.0f )
			{
				BLDC_SetExpectedCurrent(BLDC_GetExpectedCurrent() - 200.0f);
			}
			else
			{
				BLDC_SetExpectedCurrent(200.0f);
			}
			break;

		case KEY5_PRESS:
			BLDC_SetDirection((BLDC_GetDirection(&BLDC_Info) == MOTOR_FWD) ? MOTOR_REV : MOTOR_FWD);
			break;

		default:
			break;
	}
}
