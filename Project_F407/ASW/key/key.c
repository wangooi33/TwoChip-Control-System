/* Includes ------------------------------------------------------------------*/
#include "key.h"
#include "BDC_Control.h"
#include "BLDC_Control.h"
/* private variable ----------------------------------------------------------*/
uint8_t KeyCnt[KEY_NUM] = {0};
uint8_t KeyState[KEY_NUM] = {0}; // 0:松开 1:按下
// GPIO映射
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
/* functions implementation --------------------------------------------------*/
static KeyEvent_t KeyScan(void)
{
	for (int i = 0; i < KEY_NUM; i++)
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

void KeyTask_Cyclic(void)
{
	KeyEvent_t key = KeyScan();

	switch (key)
	{
		case KEY1_PRESS:	// 启动
			#if 0
			BDC_Disable();
			BDC_ResetControlState(&BDC_Info);
			BDC_EncoderReset();
			BDC_PIDInit();

			BDC_Info.Expectation.ExpectedRPM = 60.0f;
			BDC_Info.Expectation.ExpectedRPM_Ramp = 0.0f;
			
			BDC_Enable();
			#endif
			BLDC_Info.Direction = MOTOR_FWD;
			BLDC_Info.Pulse = 600;

			BLDC_Enable();
			Hall_enable();
		break;

		case KEY2_PRESS:	// 停止
			//BDC_Info.Expectation.ExpectedRPM = 0.0f;
			BLDC_Disable();
			Hall_Disable();
			break;

		case KEY3_PRESS:	// 加速
			//BDC_Info.Expectation.ExpectedRPM += 5.0f;
			BLDC_Info.Pulse += 200;
			break;

		case KEY4_PRESS:	// 减速
			//BDC_Info.Expectation.ExpectedRPM -= 5.0f;
			BLDC_Info.Pulse -= 200;
			break;

		case KEY5_PRESS:	// 反转
			//BDC_Info.Expectation.ExpectedRPM = -BDC_Info.Expectation.ExpectedRPM;
			if ( BLDC_Info.Direction == MOTOR_FWD )
			{
				BLDC_Info.Direction = MOTOR_REV;
			}
			else if ( BLDC_Info.Direction == MOTOR_REV )
			{
				BLDC_Info.Direction = MOTOR_FWD;
			}
			break;

		default:
			break;
	}
}


