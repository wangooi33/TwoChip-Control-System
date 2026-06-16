#include "LCD_cfg.h"
#include "delay.h"
#include "tim.h"

/* global variable ----------------------------------------------------------*/
LCD_Info_t LCD_Info;
	
/* function implementation --------------------------------------------------*/
void LCD_WriteRAM(volatile uint16_t Data)
{
	LCD->LCD_RAM = Data;
}
void LCD_WriteREGNo(volatile uint16_t RegNo)
{
	LCD->LCD_REG = RegNo;
}
void LCD_WriteREG(uint16_t RegNo, uint16_t Data)
{
	LCD->LCD_REG = RegNo;
	LCD->LCD_RAM = Data;
}
static void prvDelay(uint32_t i)
{
	while(i--);
}
void LCD_DisplayON(void)
{
	LCD_WriteREGNo(LCD_Command_DisplayON);
}
void LCD_DisplayOFF(void)
{
	LCD_WriteREGNo(LCD_Command_DisplayOFF);
}

uint16_t LCD_ReadData(void)
{
	volatile uint16_t Data;
	prvDelay(2);
	Data = LCD->LCD_RAM;
	return Data;
}
static void LCD_ReadID4(void)
{
	LCD_WriteREGNo(LCD_Command_ReadID4);
	LCD_Info.ID = LCD_ReadData();	/* dummy read */
	LCD_Info.ID = LCD_ReadData();  	/* 读到0X00 */
	LCD_Info.ID = LCD_ReadData();  	/* 读取0X93 */
	LCD_Info.ID <<= 8;
	LCD_Info.ID |= LCD_ReadData(); 	/* 读取0X41 */
}

static void ILI9341_LG2_8_Initial(void) 
{ 
	/* 省略硬件复位 */

	LCD_WriteREGNo(0xCB); 
	LCD_WriteRAM(0x39); 
	LCD_WriteRAM(0x2C); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x34); 
	LCD_WriteRAM(0x02); 

	LCD_WriteREGNo(0xCF); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0XC1); 
	LCD_WriteRAM(0X30); 

	LCD_WriteREGNo(0xE8);
	LCD_WriteRAM(0x85); 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x78); 

	LCD_WriteREGNo(0xEA);
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x00); 

	LCD_WriteREGNo(0xED);
	LCD_WriteRAM(0x64); 
	LCD_WriteRAM(0x03); 
	LCD_WriteRAM(0X12); 
	LCD_WriteRAM(0X81);
	
	LCD_WriteREGNo(0xF7);
	LCD_WriteRAM(0x20); 

	LCD_WriteREGNo(0xC0);//Power control 
	LCD_WriteRAM(0x1b); //VRH[5:0] 

	LCD_WriteREGNo(0xC1);//Power control 
	LCD_WriteRAM(0x10); //SAP[2:0];BT[3:0] 

	LCD_WriteREGNo(0xC5);//VCM control 
	LCD_WriteRAM(0x2d); 
	LCD_WriteRAM(0x33); 

//	LCD_WriteREGNo(0xC7); //VCM control2 
//	LCD_WriteRAM(0xCf);

	LCD_WriteREGNo(0x36);// Memory Access Control 
	LCD_WriteRAM(0x48); 

	LCD_WriteREGNo(0xB1);
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x1d); 

	LCD_WriteREGNo(0xB6);// Display Function Control 
	LCD_WriteRAM(0x0A); 
	LCD_WriteRAM(0x02); 

	LCD_WriteREGNo(0xF2);// 3Gamma Function Disable 
	LCD_WriteRAM(0x00); 

	LCD_WriteREGNo(0x26);//Gamma curve selected 
	LCD_WriteRAM(0x01); 

	LCD_WriteREGNo(0xE0);//Set Gamma 
	LCD_WriteRAM(0x0F); 
	LCD_WriteRAM(0x3a); 
	LCD_WriteRAM(0x36); 
	LCD_WriteRAM(0x0b); 
	LCD_WriteRAM(0x0d); 
	LCD_WriteRAM(0x06); 
	LCD_WriteRAM(0x4c); 
	LCD_WriteRAM(0x91); 
	LCD_WriteRAM(0x31); 
	LCD_WriteRAM(0x08); 
	LCD_WriteRAM(0x10); 
	LCD_WriteRAM(0x04); 
	LCD_WriteRAM(0x11); 
	LCD_WriteRAM(0x0c); 
	LCD_WriteRAM(0x00); 

	LCD_WriteREGNo(0XE1); //Set Gamma 
	LCD_WriteRAM(0x00); 
	LCD_WriteRAM(0x06); 
	LCD_WriteRAM(0x0a); 
	LCD_WriteRAM(0x05); 
	LCD_WriteRAM(0x12); 
	LCD_WriteRAM(0x09); 
	LCD_WriteRAM(0x2c); 
	LCD_WriteRAM(0x92); 
	LCD_WriteRAM(0x3f); 
	LCD_WriteRAM(0x08); 
	LCD_WriteRAM(0x0e); 
	LCD_WriteRAM(0x0b); 
	LCD_WriteRAM(0x2e); 
	LCD_WriteRAM(0x33); 
	LCD_WriteRAM(0x0F); 

	LCD_WriteREGNo(0x11); //Exit Sleep 
	Delay_ms(120); 
	LCD_WriteREGNo(0x29); //Display on 
} 
static void LCD_SetScanDir(LCD_ScanDir_t ScanDir)
{
	uint16_t CtrlData = 0;
	LCD_Info.ScanDir = ScanDir;
	
	switch (ScanDir)
	{
		case LeftToRight_TopToBottom:
			CtrlData |= (0 << 7) | (0 << 6) | (0 << 5);
			break;
		case LeftToRight_BottomToTop:
			CtrlData |= (0 << 7) | (1 << 6) | (0 << 5);
			break;
		case RightToLeft_TopToBottom:
			CtrlData |= (1 << 7) | (0 << 6) | (0 << 5);
			break;
		case RightToLeft_BottomToTop:
			CtrlData |= (1 << 7) | (1 << 6) | (0 << 5);
			break;
		case TopToBottom_LeftToRight:
			CtrlData |= (0 << 7) | (0 << 6) | (1 << 5);
			break;
		case TopToBottom_RightToLeft:
			CtrlData |= (0 << 7) | (1 << 6) | (1 << 5);
			break;
		case BottomToTop_LeftToRight:
			CtrlData |= (1 << 7) | (0 << 6) | (1 << 5);
			break;
		case BottomToTop_RightToLeft:
			CtrlData |= (1 << 7) | (1 << 6) | (1 << 5);
			break;

		default:
			break;
	}
	CtrlData |= (1 << 3);
	LCD_WriteREG(LCD_Command_MemoryAccessControl,CtrlData);
}
void LCD_SetDisplayDir(uint8_t Dir, LCD_ScanDir_t ScanDir)
{
	LCD_Info.Direction = Dir;

	switch (Dir)
	{
		case 0:
			LCD_Info.Width = 240;
			LCD_Info.Height = 320;
			break;

		case 1:
			LCD_Info.Width = 320;
			LCD_Info.Height = 240;
			break;

		default:
			break;
	}
	LCD_SetScanDir(ScanDir);
}
void LCD_SetWindow(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height)
{
	uint16_t SXtoEx = Width - StartX -1;
	uint16_t SYtoEY = Height - StartY -1;
	
	LCD_WriteREGNo(LCD_Command_ColumnAddressSet);
	LCD_WriteRAM(StartX >> 8);
	LCD_WriteRAM(StartX & 0xFF);
	LCD_WriteRAM(SXtoEx >> 8);
	LCD_WriteRAM(SXtoEx & 0xFF);

	LCD_WriteREGNo(LCD_Command_PageAddressSet);
	LCD_WriteRAM(StartY >> 8);
	LCD_WriteRAM(StartY & 0xFF);
	LCD_WriteRAM(SYtoEY >> 8);
	LCD_WriteRAM(SYtoEY & 0xFF);
}
void LCD_SetCursor(uint16_t X, uint16_t Y)
{
	LCD_WriteREGNo(LCD_Command_ColumnAddressSet);
	LCD_WriteRAM(X >> 8);
	LCD_WriteRAM(X & 0xFF);

	LCD_WriteREGNo(LCD_Command_PageAddressSet);
	LCD_WriteRAM(Y >> 8);
	LCD_WriteRAM(Y & 0xFF);
}
void LCD_ClearScreen(uint16_t Color)
{
	uint32_t Cnt = LCD_Info.Width * LCD_Info.Height;

	LCD_WriteREGNo(LCD_Command_MemoryWrite);
	for(uint32_t i = 0; i < Cnt; i++)
	{
		LCD->LCD_RAM = Color;
	}
}
void LCD_SetBL(uint16_t duty)
{
	uint16_t BL = duty % 1000;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, BL);
}
void LCD_Init(void)
{
	LCD_ReadID4();
	ILI9341_LG2_8_Initial();
	LCD_SetDisplayDir(0,LeftToRight_TopToBottom);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	LCD_SetBL(800);
	
	LCD_ClearScreen(65535);
}

