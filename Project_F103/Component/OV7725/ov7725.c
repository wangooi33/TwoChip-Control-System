#include "ov7725.h"
#include "lcd.h"

/* macro --------------------------------------------------------------------*/
/* 制造商ID */
#define OV7725_MID			0x7FA2
/* 产品ID */
#define OV7725_PID			0x7721

/* global variable ----------------------------------------------------------*/
OV7725_Info_t OV7725_Info;

/* private function ---------------------------------------------------------*/
static uint8_t OV7725_GetByteFormFIFO(void);

static inline void OV7725_FIFO_Delay(void)
{
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

static void OV7725_FIFO_ResetReadPtr(void)
{
	OV7725_RCLK(0);
	OV7725_RRST(0);
	OV7725_FIFO_Delay();
	OV7725_RCLK(1);
	OV7725_FIFO_Delay();
	OV7725_RCLK(0);
	OV7725_FIFO_Delay();
	OV7725_RCLK(1);
	OV7725_FIFO_Delay();
	OV7725_RCLK(0);
	OV7725_FIFO_Delay();
	OV7725_RRST(1);
	OV7725_FIFO_Delay();
}

static uint8_t OV7725_FIFO_ReadByte(void)
{
	uint8_t Byte;

	OV7725_RCLK(0);
	OV7725_FIFO_Delay();
	OV7725_RCLK(1);
	OV7725_FIFO_Delay();
	Byte = OV7725_GetByteFormFIFO();
	OV7725_RCLK(0);
	OV7725_FIFO_Delay();

	return Byte;
}

static void OV7725_LCD_PrepareFrame(void)
{
	LCD_DisplayON();
	LCD_SetDisplayDir(0);
	LCD_SetScanDir(LeftToRight_TopToBottom);
	LCD_SetWindow(0, 0, LCD_Info.Width, LCD_Info.Height);
	LCD_WriteREGNo(LCD_Command_MemoryWrite);
}

/* function implementation --------------------------------------------------*/
void OV7725_SetOutputWindow(OV7725_OutputMode_t OutputMode, uint16_t Width, uint16_t Height)
{
	uint16_t OffsetX;
	uint16_t OffsetY;

	uint8_t OldHFrameStart;
	uint8_t NewHFrameStart;
	uint8_t OldVFrameStart;
	uint8_t NewVFrameStart;

	uint8_t OldHREF;
	uint8_t NewHREF;
	uint8_t NewEXHCH;
        
    switch (OutputMode)
    {
		case OV7725_OUTPUT_MODE_VGA:
			if (Width > OV7725_VGA_WIDTH_MAX || Height > OV7725_VGA_HEIGHT_MAX)
			{
				return;
			}
			/* 居中:
			水平方向左右空白总宽度的一半(像素单位)
			垂直方向上下空白总高度的一半 */
			OffsetX = (OV7725_VGA_WIDTH_MAX - Width) >> 1;
			OffsetY = (OV7725_VGA_HEIGHT_MAX - Height) >> 1;
			OV7725_WriteReg(OV7725_REG_COM7,0x06);
			OV7725_WriteReg(OV7725_REG_HSTART,0x23);
			OV7725_WriteReg(OV7725_REG_HSIZE,0xA0);
			OV7725_WriteReg(OV7725_REG_VSTRT,0x07);
			OV7725_WriteReg(OV7725_REG_VSIZE,0xF0);
			OV7725_WriteReg(OV7725_REG_HOutSize,0xA0);
			OV7725_WriteReg(OV7725_REG_VOutSize,0xF0);
			OV7725_WriteReg(OV7725_REG_HREF,0x00);
			break;
		case OV7725_OUTPUT_MODE_QVGA:
			if (Width > OV7725_QVGA_WIDTH_MAX || Height > OV7725_QVGA_HEIGHT_MAX)
			{
				return;
			}
			OffsetX = (OV7725_VGA_WIDTH_MAX - Width) >> 1;
			OffsetY = (OV7725_VGA_HEIGHT_MAX - Height) >> 1;
			OV7725_WriteReg(OV7725_REG_COM7,0x46);
			OV7725_WriteReg(OV7725_REG_HSTART,0x3F);
			OV7725_WriteReg(OV7725_REG_HSIZE,0x50);
			OV7725_WriteReg(OV7725_REG_VSTRT,0x03);
			OV7725_WriteReg(OV7725_REG_VSIZE,0x78);
			OV7725_WriteReg(OV7725_REG_HOutSize,0x50);
			OV7725_WriteReg(OV7725_REG_VOutSize,0x78);
			OV7725_WriteReg(OV7725_REG_HREF,0x00);
			break;
		default:
			break;
	}
    
	OldHFrameStart = OV7725_ReadReg(OV7725_REG_HSTART);
	NewHFrameStart = OldHFrameStart + (OffsetX >> 2);
	OV7725_WriteReg(OV7725_REG_HSTART, NewHFrameStart);
	OV7725_WriteReg(OV7725_REG_HSIZE,Width >> 2);
	OV7725_Info.OutputWidth = OV7725_ReadReg(OV7725_REG_HSIZE) << 2;
	
	OldVFrameStart = OV7725_ReadReg(OV7725_REG_VSTRT);
	NewVFrameStart = OldVFrameStart + (OffsetY >> 1);
	OV7725_WriteReg(OV7725_REG_VSTRT, NewVFrameStart);
	OV7725_WriteReg(OV7725_REG_VSIZE, Height >> 1);
	OV7725_Info.OutputHeight = OV7725_ReadReg(OV7725_REG_VSIZE) << 1;
	
	OldHREF = OV7725_ReadReg(OV7725_REG_HREF);
	NewHREF = ((OffsetY & 0x01) << 6) | ((OffsetX & 0x03) << 4) | ((Height & 0x01) << 2) | (Width & 0x03) | OldHREF;
	OV7725_WriteReg(OV7725_REG_HREF, NewHREF);
	
	OV7725_WriteReg(OV7725_REG_HOutSize, Width >> 2);
	OV7725_WriteReg(OV7725_REG_VOutSize, Height >> 1);
	
	NewEXHCH = (OldHREF | (Width & 0x03) | ((Height & 0x01) << 2));
	OV7725_WriteReg(OV7725_REG_EXHCH, NewEXHCH);

}
static uint8_t OV7725_GetByteFormFIFO(void)
{
	uint8_t Byte = 0;
#if 0
	Byte |= (((OV7725_D0_GPIO_Port->IDR & OV7725_D0_Pin) == 0) ? (0) : (1)) << 0;
	Byte |= (((OV7725_D1_GPIO_Port->IDR & OV7725_D1_Pin) == 0) ? (0) : (1)) << 1;
	Byte |= (((OV7725_D2_GPIO_Port->IDR & OV7725_D2_Pin) == 0) ? (0) : (1)) << 2;
	Byte |= (((OV7725_D3_GPIO_Port->IDR & OV7725_D3_Pin) == 0) ? (0) : (1)) << 3;
	Byte |= (((OV7725_D4_GPIO_Port->IDR & OV7725_D4_Pin) == 0) ? (0) : (1)) << 4;
	Byte |= (((OV7725_D5_GPIO_Port->IDR & OV7725_D5_Pin) == 0) ? (0) : (1)) << 5;
	Byte |= (((OV7725_D6_GPIO_Port->IDR & OV7725_D6_Pin) == 0) ? (0) : (1)) << 6;
	Byte |= (((OV7725_D7_GPIO_Port->IDR & OV7725_D7_Pin) == 0) ? (0) : (1)) << 7;
#else
	Byte = GPIOC->IDR & 0x00FF;
#endif
	return Byte;
}
void OV7725_Init(void)
{
	/* HW_Init */
	OV7725_WRST(1);
	OV7725_RRST(1);
	OV7725_OE(1);
	OV7725_RCLK(0);
	OV7725_WEN(1);
	OV7725_Info.isEnable = 0;
	OV7725_Info.FrameCount = 0;
	OV7725_Info.FrameHandleSt = Frame_WaitFrameStart;

	/* SCCB_Init */
	SCCB_Init();
	OV7725_SCCBRegReset();
	/* Get Device ID */
	OV7725_Info.MID = OV7725_ReadReg(OV7725_REG_MIDH) << 8;
	OV7725_Info.MID |= OV7725_ReadReg(OV7725_REG_MIDL);
	OV7725_Info.PID = OV7725_ReadReg(OV7725_REG_PID) << 8;
	OV7725_Info.PID |= OV7725_ReadReg(OV7725_REG_VER);
	if (OV7725_Info.MID != OV7725_MID || OV7725_Info.PID != OV7725_PID)
	{
		LED1_ON;
		return;
	}

	/* Ov7725_Reg Init */
	for (uint16_t RegIndex = 0; RegIndex < (sizeof(OV7725_RegInitData) / sizeof(OV7725_RegInitData[0])); RegIndex++)
	{
		OV7725_WriteReg(OV7725_RegInitData[RegIndex][0],OV7725_RegInitData[RegIndex][1]);
		switch (OV7725_RegInitData[RegIndex][0])
		{
			case OV7725_REG_HSIZE:
				OV7725_Info.OutputWidth = OV7725_RegInitData[RegIndex][1] << 2;
				break;
			case OV7725_REG_VSIZE:
				OV7725_Info.OutputHeight = OV7725_RegInitData[RegIndex][1] << 1;
				break;
			default:
				break;
		}
	}
}

void OV7725_EnableOutput(void)
{
	OV7725_WEN(1);
	OV7725_Info.FrameHandleSt = Frame_WaitFrameStart;
	OV7725_OE(0);
}
void OV7725_DisableOutput(void)
{
	OV7725_WEN(0);
	OV7725_Info.FrameHandleSt = Frame_WaitFrameStart;
	OV7725_OE(1);
}
void OV7725_StartCapture(void)
{
	OV7725_WEN(1);
	OV7725_Info.FrameHandleSt = Frame_Capturing;
}
void OV7725_StopCapture(void)
{
	OV7725_WEN(0);
	OV7725_FIFO_Delay();
	OV7725_WRST(0);
	OV7725_FIFO_Delay();
	OV7725_WRST(1);
	OV7725_Info.FrameHandleSt = Frame_Ready;
}
uint8_t OV7725_GetFrame(volatile uint16_t *pBuf, OV7725_GetFrameType_t Type)
{
	uint16_t Data;

	if (OV7725_Info.FrameHandleSt != Frame_Ready)
	{
		return 0;
	}
	/* 读复位 */
	OV7725_FIFO_ResetReadPtr();
	for (uint16_t HeightIndex = 0; HeightIndex < OV7725_Info.OutputHeight; HeightIndex++)
	{
		for (uint16_t WidthIndex = 0; WidthIndex < OV7725_Info.OutputWidth; WidthIndex++)
		{
			/* 读数据 */
			Data = ((uint16_t)OV7725_FIFO_ReadByte() << 8);
			Data |= OV7725_FIFO_ReadByte();
			*pBuf = Data;
			switch (Type)
			{
				case OV7725_GET_FRAME_TYPE_NOINC:
					break;
				case OV7725_GET_FRAME_TYPE_AUTO_INC:
					pBuf++;
					break;
				default:
					break;
			}
		}
	}
	OV7725_Info.FrameHandleSt = Frame_WaitFrameStart;
	OV7725_Info.FrameCount++;

	return 1;
}
void OV7725_SetMode(void)
{
	OV7725_SetOutputWindow(OV7725_OUTPUT_MODE_QVGA,OV7725_QVGA_WIDTH_MAX, OV7725_QVGA_HEIGHT_MAX);
	OV7725_SetLightMode(OV7725_LIGHT_MODE_AUTO);
	OV7725_SetColorSaturation(OV7725_COLOR_SATURATION_4);
	OV7725_SetBrightness(OV7725_BRIGHTNESS_4);
	OV7725_SetContrast(OV7725_CONTRAST_4);
	OV7725_SetSpecialEffects(OV7725_SPECIAL_EFFECT_NORMAL);
}
void Task4_Camera(void *pvParameters)
{
	OV7725_SetMode();
	OV7725_EnableOutput();
	OV7725_LCD_PrepareFrame();
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		OV7725_LCD_PrepareFrame();
		OV7725_GetFrame(&(LCD->LCD_RAM),OV7725_GET_FRAME_TYPE_NOINC);
	}
}


