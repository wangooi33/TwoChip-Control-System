#include "LF0038.h"
#include "task_manage.h"
#include "ov7725.h"
#include "lcd_cfg.h"
#include "tjc.h"

/* global variable ----------------------------------------------------------*/
volatile InfraredCtrl_Info_t InfraredCtrl_Info = {0};

/* function implementation --------------------------------------------------*/
uint8_t InfraredCtrl_isBit0(uint16_t us)
{
	return (us > NEC_BIT0_MIN_US) && (us < NEC_BIT0_MAX_US);
}
uint8_t InfraredCtrl_isBit1(uint16_t us)
{
	return (us > NEC_BIT1_MIN_US) && (us < NEC_BIT1_MAX_US);
}
uint8_t InfraredCtrl_isLead(uint16_t us)
{
	return (us > NEC_LEAD_MIN_US) && (us < NEC_LEAD_MAX_US);
}
uint8_t InfraredCtrl_isRepeat(uint16_t us)
{
	return (us > NEC_REPEAT_MIN_US) && (us < NEC_REPEAT_MAX_US);
}

void InfraredCtrl_Reset(void)
{
	InfraredCtrl_Info.Frame = 0;
	InfraredCtrl_Info.BitCnt = 0;
	InfraredCtrl_Info.State = InfraredCtrl_Idle;
}
void InfraredCtrl_Framing(uint8_t bit)
{
	/**
	 * 将红外解码过程中收到的每一个bit组成一个32位帧 
	 * 8位地址码 + 8位地址反码 + 8位命令码 + 8位命令反码
	 * 控制端以 低位在前,高位在后 的方式发送
	 */
	InfraredCtrl_Info.Frame >>= 1;
	if (bit == 1)
	{
		InfraredCtrl_Info.Frame |= 0x80000000UL;
	}

	InfraredCtrl_Info.BitCnt++;
	if (InfraredCtrl_Info.BitCnt >= 32)
	{
		/* 一帧完成 */
		InfraredCtrl_Info.FrameOk = 1;
		InfraredCtrl_Info.State = InfraredCtrl_Idle;
	}
}
uint8_t InfraredCtrl_FrameValid(uint32_t Frame)
{
	uint8_t Addr;
	uint8_t Addr_Inv;
	uint8_t Cmd;
	uint8_t Cmd_Inv;

	Addr = Frame & 0xFF;
	Addr_Inv = (Frame >> 8) & 0xFF;
	Cmd = (Frame >> 16) & 0xFF;
	Cmd_Inv  = (Frame >> 24) & 0xFF;

	if (Addr != (uint8_t)(~Addr_Inv) || Cmd != (uint8_t)(~Cmd_Inv) || Addr != LF0038_ID)
	{
		return 0;
	}
	return 1;
}
uint8_t InfraredCtrl_Scan(void)
{
	uint8_t Cmd;

	if (InfraredCtrl_Info.FrameOk == 0)
	{
		return 0;
	}

	InfraredCtrl_Info.FrameOk = 0;
	if (InfraredCtrl_FrameValid(InfraredCtrl_Info.Frame) == 0)
	{
		return 0;
	}
	Cmd = (InfraredCtrl_Info.Frame >> 16) & 0xFF;
	return Cmd;
}
void Task3_InfraredScan(void *pvParameters)
{
	uint32_t cmd = 0;
	uint8_t FirstRun = 1;
	for (;;)
	{
		if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &cmd, portMAX_DELAY) == pdPASS)
		{
			switch (cmd)
			{
				case InfraredCtrl_Cmd_ZDYZ:
					if (OV7725_Info.isEnable == 0)
					{
						OV7725_Info.isEnable = 1;
						if (FirstRun == 1)
						{
							xTaskCreate(Task4_Camera, "Task4_Camera", 4096, NULL, 2, &wTaskHandle.Task4_CameraHandle);
							FirstRun = 0;
						}
						else
						{
							LCD_DisplayON();
							OV7725_EnableOutput();
							vTaskResume(wTaskHandle.Task4_CameraHandle);
						}
					}
					else
					{
						OV7725_Info.isEnable = 0;
						LCD_DisplayOFF();
						OV7725_DisableOutput();
						vTaskSuspend(wTaskHandle.Task4_CameraHandle);
					}
					break;

				case InfraredCtrl_Cmd_Left:
					if (TJC_Info.CurrentPage <= 0)
					{
						TJC_ChangePage(0);
					}
					else
					{
						TJC_ChangePage(--TJC_Info.CurrentPage);
					}
					break;
				case InfraredCtrl_Cmd_Right:
					if (TJC_Info.CurrentPage < TJC_MAX_PAGENUM)
					{
						TJC_ChangePage(++TJC_Info.CurrentPage);
					}
					else
					{
						TJC_ChangePage(TJC_MAX_PAGENUM);
					}
					break;
				case InfraredCtrl_Cmd_Play:
					if (TJC_Info.Videoisplay)
					{
						TJC_ChangeVideoState(0,2);
						TJC_Info.Videoisplay = 0;
					}
					else
					{
						TJC_ChangeVideoState(0,1);
						TJC_Info.Videoisplay = 1;
					}
					break;
					
				default:
					break;
			}
		}		
	}
	
}

