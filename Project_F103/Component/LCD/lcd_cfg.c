#include "lcd_cfg.h"
#include "delay.h"
#include "tim.h"

/* macro --------------------------------------------------------------------*/
//原始ADC值采样次数
#define TOUCH_SAMPLES_Cnt			5
//每边丢弃的个数(最大和最小各丢弃1个)
#define TOUCH_FILTER_Cnt			1
//差值
#define TOUCH_FILTER_DELTA			50

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
	LCD_Info.ID = LCD_ReadData();  	/* 读到0X85 */
	LCD_Info.ID = LCD_ReadData();  	/* 读取0X85 */
	LCD_Info.ID <<= 8;
	LCD_Info.ID |= LCD_ReadData(); 	/* 读取0X52 */
	if (LCD_Info.ID == 0x8552)
	{
		LCD_Info.ID = 0x7789;		/* ST7789 */
	}
}
static void ST7789_RegInit(void)
{
	LCD_WriteREGNo(0x11);
	Delay_ms(120); 

	LCD_WriteREGNo(0x36);
	LCD_WriteRAM(0x00);

	LCD_WriteREGNo(0x3A);
	LCD_WriteRAM(0X05);

	LCD_WriteREGNo(0xB2);
	LCD_WriteRAM(0x0C);
	LCD_WriteRAM(0x0C);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x33);
	LCD_WriteRAM(0x33);

	LCD_WriteREGNo(0xB7);
	LCD_WriteRAM(0x35);

	LCD_WriteREGNo(0xBB); /* vcom */
	LCD_WriteRAM(0x32);  /* 30 */

	LCD_WriteREGNo(0xC0);
	LCD_WriteRAM(0x0C);

	LCD_WriteREGNo(0xC2);
	LCD_WriteRAM(0x01);

	LCD_WriteREGNo(0xC3); /* vrh */
	LCD_WriteRAM(0x10);  /* 17 0D */

	LCD_WriteREGNo(0xC4); /* vdv */
	LCD_WriteRAM(0x20);  /* 20 */

	LCD_WriteREGNo(0xC6);
	LCD_WriteRAM(0x0f);

	LCD_WriteREGNo(0xD0);
	LCD_WriteRAM(0xA4); 
	LCD_WriteRAM(0xA1); 

	LCD_WriteREGNo(0xE0); /* Set Gamma  */
	LCD_WriteRAM(0xd0);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x02);
	LCD_WriteRAM(0x07);
	LCD_WriteRAM(0x0a);
	LCD_WriteRAM(0x28);
	LCD_WriteRAM(0x32);
	LCD_WriteRAM(0X44);
	LCD_WriteRAM(0x42);
	LCD_WriteRAM(0x06);
	LCD_WriteRAM(0x0e);
	LCD_WriteRAM(0x12);
	LCD_WriteRAM(0x14);
	LCD_WriteRAM(0x17);

	LCD_WriteREGNo(0XE1);  /* Set Gamma */
	LCD_WriteRAM(0xd0);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x02);
	LCD_WriteRAM(0x07);
	LCD_WriteRAM(0x0a);
	LCD_WriteRAM(0x28);
	LCD_WriteRAM(0x31);
	LCD_WriteRAM(0x54);
	LCD_WriteRAM(0x47);
	LCD_WriteRAM(0x0e);
	LCD_WriteRAM(0x1c);
	LCD_WriteRAM(0x17);
	LCD_WriteRAM(0x1b); 
	LCD_WriteRAM(0x1e);

	LCD_WriteREGNo(0x2A);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0xef);

	LCD_WriteREGNo(0x2B);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x00);
	LCD_WriteRAM(0x01);
	LCD_WriteRAM(0x3f);

	//LCD_WriteREGNo(0x29); /* display on */
}

void LCD_SetScanDir(LCD_ScanDir_t ScanDir)
{
	uint16_t CtrlData = 0;
	LCD_Info.ScanDir = ScanDir;
	
	switch (ScanDir)
	{
		case LeftToRight_TopToBottom:
			CtrlData |= (0 << 7) | (0 << 6) | (0 << 5);
			break;
		case LeftToRight_BottomToTop:
			CtrlData |= (1 << 7) | (0 << 6) | (0 << 5);
			break;
		case RightToLeft_TopToBottom:
			CtrlData |= (0 << 7) | (1 << 6) | (0 << 5);
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
	CtrlData |= 0x08;	/* 设置RGB位 */
	LCD_WriteREG(LCD_Command_MemoryAccessControl,CtrlData);
}
void LCD_SetDisplayDir(uint8_t Dir)
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
	
	LCD_SetCursor(0x0000, 0x0000);
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
	
	ST7789_RegInit();
	LCD_SetDisplayDir(0);
	LCD_SetScanDir(LeftToRight_TopToBottom);
	LCD_SetWindow(0,0,LCD_Info.Width,LCD_Info.Height);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	LCD_SetBL(800);
	
	LCD_ClearScreen(10000);
}
void LCD_TouchWrite(uint8_t Byte)
{
	uint8_t SendData = 0;

	for (int8_t i = 7; i >= 0; i--)
	{
		SendData = ((Byte >> i) & 1);
		TOUCH_SPI_MO(SendData);
		TOUCH_SPI_CLK(0);
		Delay_us(1);
		TOUCH_SPI_CLK(1);
	}
}

uint16_t LCD_TouchRead(uint8_t Cmd)
{
	uint16_t Data = 0;

	TOUCH_SPI_CS(0);
	TOUCH_SPI_MO(0);
	TOUCH_SPI_CLK(0);
	Delay_us(1);

	LCD_TouchWrite(Cmd);
	Delay_us(4);
	
	TOUCH_SPI_CLK(0);
	Delay_us(1);
	TOUCH_SPI_CLK(1);
	Delay_us(1);

	for (int8_t i = 15; i >= 0; i++)
	{
		TOUCH_SPI_CLK(0);
		Delay_us(1);
		TOUCH_SPI_CLK(1);
		Data |= (TOUCH_SPI_READ_MI() << i);
	}
	Data >>= 4;

	TOUCH_SPI_CS(1);
	return Data;
}
static uint16_t Filter_PostAverage(uint8_t Cmd)
{
	/* 去极值平均滤波:取5次,去掉最大最小值,取剩余3个数的平均值 */
	uint16_t OriginValue[TOUCH_SAMPLES_Cnt] = {0};
	uint16_t temp;
	uint8_t RealCnt;
	uint16_t Sum;

	for (uint8_t i = 0; i < TOUCH_SAMPLES_Cnt; i++)
	{
		OriginValue[i] = LCD_TouchRead(Cmd);
	}

	/* 升序冒泡排序,筛选极值 */
	for (uint8_t i = 0; i < TOUCH_SAMPLES_Cnt - 1; i++)
	{
		for (uint8_t j = TOUCH_SAMPLES_Cnt - 1; j > i; j--)
		{
			if (OriginValue[j - 1] > OriginValue[j])
			{
				temp = OriginValue[j - 1];
				OriginValue[j - 1] = OriginValue[j];
				OriginValue[j] = temp;
			}
		}
	}

	/* 计算平均值 */
	RealCnt = TOUCH_SAMPLES_Cnt - (TOUCH_FILTER_Cnt << 1);
	for (uint8_t k = TOUCH_FILTER_Cnt; k < TOUCH_FILTER_Cnt + RealCnt; k++)
	{
		Sum += OriginValue[k];
	}

	return Sum / RealCnt;
}
static uint16_t Filter_Delta(uint8_t Cmd)
{
	/* 差值校验滤波 */
	uint16_t Data1,Data2;
	uint16_t Delta;

	do
	{
		Data1 = Filter_PostAverage(Cmd);
		Data2 = Filter_PostAverage(Cmd);
		if (Data1 > Data2)
		{
			Delta = Data1 - Data2;
		}
		else
		{
			Delta = Data2 - Data1;
		}
	}while (Delta > TOUCH_FILTER_DELTA);

	return (Data1 + Data2) >> 1; 
}
uint8_t LCD_GetTouchCoord(uint16_t *X, uint16_t *Y)
{
	if (GPIO_PIN_RESET == TOUCH_READ_PEN())
	{
		*X = Filter_Delta(TOUCH_Command_GetX);
		*Y = Filter_Delta(TOUCH_Command_GetY);
		return 1;
	}
	else
	{
		return 0;
	}
}
