/* Includes ------------------------------------------------------------------*/
#include "key.h"
#include "foc.h"

/* variable ------------------------------------------------------------------*/
uint8_t KeyCnt[5] = {0};
uint8_t KeyState[5] = {0};
static GPIO_TypeDef* KEY_PORT[5] =
{
	KEY1_GPIO_Port,KEY2_GPIO_Port,KEY3_GPIO_Port,KEY4_GPIO_Port,KEY5_GPIO_Port
};
static uint16_t KEY_PIN[5] =
{
	KEY1_Pin,KEY2_Pin,KEY3_Pin,KEY4_Pin,KEY5_Pin
};

/* local helpers -------------------------------------------------------------*/
static KeyEvent_t KeyScan( void )
{
	for (uint8_t i = 0; i < 5; i++)
	{
		if (HAL_GPIO_ReadPin(KEY_PORT[i], KEY_PIN[i]) == GPIO_PIN_SET)
		{
			if (KeyCnt[i] < 2)
			{
				KeyCnt[i]++;
			}
			if ( KeyCnt[i] >= 2 && KeyState[i] == 0 )
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
	switch (KeyScan())
	{
		case KEY1_PRESS:
			FOC_Info.Id_Ref += 0.5f;
			break;

		case KEY2_PRESS:
			FOC_Info.Iq_Ref += 0.5f;
			break;

		case KEY3_PRESS:
			break;

		case KEY4_PRESS:
			break;

		case KEY5_PRESS:
			BLDC_Disable();
			break;

		default:
			break;
	}
}

