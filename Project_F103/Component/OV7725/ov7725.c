#include "ov7725.h"
#include "delay.h"
#include "lcd.h"

/* macro --------------------------------------------------------------------*/
/* 制造商ID */
#define OV7725_MID          0x7FA2
/* 产品ID */
#define OV7725_PID          0x7721
/* 通讯地址 */
#define OV7725_SCCB_ADDR    0x21

/* global variable ----------------------------------------------------------*/
OV7725_Info_t OV7725_Info;

/* function implementation --------------------------------------------------*/
static void OV7725_WriteReg(uint8_t Reg, uint8_t Data)
{
    SCCB_3PhaseWrite(OV7725_SCCB_ADDR,Reg,Data);
}
static uint8_t OV7725_ReadReg(uint8_t Reg)
{
	uint8_t Data;
	
    SCCB_2PhaseWrite(OV7725_SCCB_ADDR,Reg);
    SCCB_2PhaseRead(OV7725_SCCB_ADDR, &Data);

	return Data;
}
static void OV7725_SCCBRegReset(void)
{
    OV7725_WriteReg(OV7725_REG_COM7,0x80);
    Delay_ms(2);
}
static void OV7725_HWInit(void)
{
    OV7725_WRST(1);
    OV7725_RRST(1);
    OV7725_OE(1);
    OV7725_RCLK(1);
    OV7725_WEN(1);
}
static void OV7725_RegInit(void)
{
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
static uint16_t OV7725_GetMID(void)
{
    uint16_t MID;
    MID = OV7725_ReadReg(OV7725_REG_MIDH) << 8;
    MID |= OV7725_ReadReg(OV7725_REG_MIDL);
    return MID;
}
static uint16_t OV7725_GetPID(void)
{
    uint16_t PID;
    PID = OV7725_ReadReg(OV7725_REG_PID) << 8;
    PID |= OV7725_ReadReg(OV7725_REG_VER);
    return PID;
}

uint8_t OV7725_GetByteFormFIFO(void)
{
    uint8_t Byte;

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
    OV7725_HWInit();
    SCCB_Init();
    OV7725_SCCBRegReset();
    if (OV7725_MID != OV7725_GetMID() || OV7725_PID != OV7725_GetPID())
    {
        return;
    }
    OV7725_RegInit();
}
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
void OV7725_SetLightMode(OV7725_Light_Mode_t Light_Mode)
{
    switch (Light_Mode)
    {
        case OV7725_LIGHT_MODE_AUTO:
            OV7725_WriteReg(OV7725_REG_COM8,0xFF);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            OV7725_WriteReg(OV7725_REG_ADVFL,0x00);
            OV7725_WriteReg(OV7725_REG_ADVFH,0x00);
            break;
        case OV7725_LIGHT_MODE_SUNNY:
            OV7725_WriteReg(OV7725_REG_COM8,0xFD);
            OV7725_WriteReg(OV7725_REG_BLUE,0x5A);
            OV7725_WriteReg(OV7725_REG_RED,0x5C);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            OV7725_WriteReg(OV7725_REG_ADVFL,0x00);
            OV7725_WriteReg(OV7725_REG_ADVFH,0x00);
            break;
        case OV7725_LIGHT_MODE_CLOUDY:
            OV7725_WriteReg(OV7725_REG_COM8,0xFD);
            OV7725_WriteReg(OV7725_REG_BLUE,0x58);
            OV7725_WriteReg(OV7725_REG_RED,0x60);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            OV7725_WriteReg(OV7725_REG_ADVFL,0x00);
            OV7725_WriteReg(OV7725_REG_ADVFH,0x00);
            break;
        case OV7725_LIGHT_MODE_OFFICE:
            OV7725_WriteReg(OV7725_REG_COM8,0xFD);
            OV7725_WriteReg(OV7725_REG_BLUE,0x84);
            OV7725_WriteReg(OV7725_REG_RED,0x4C);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            OV7725_WriteReg(OV7725_REG_ADVFL,0x00);
            OV7725_WriteReg(OV7725_REG_ADVFH,0x00);
            break;
        case OV7725_LIGHT_MODE_HOME:
            OV7725_WriteReg(OV7725_REG_COM8,0xFD);
            OV7725_WriteReg(OV7725_REG_BLUE,0x96);
            OV7725_WriteReg(OV7725_REG_RED,0x40);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            OV7725_WriteReg(OV7725_REG_ADVFL,0x00);
            OV7725_WriteReg(OV7725_REG_ADVFH,0x00);
            break;
        case OV7725_LIGHT_MODE_NIGHT:
            OV7725_WriteReg(OV7725_REG_COM8,0xFF);
            OV7725_WriteReg(OV7725_REG_COM5,0x65);
            break;
        default:
            break;
    }
}
void OV7725_SetColorSaturation(OV7725_Color_Saturation_t ColorSaturation)
{
    switch (ColorSaturation)
    {
        case OV7725_COLOR_SATURATION_0:
            OV7725_WriteReg(OV7725_REG_USAT,0x80);
            OV7725_WriteReg(OV7725_REG_VSAT,0x80);
            break;
        case OV7725_COLOR_SATURATION_1:
            OV7725_WriteReg(OV7725_REG_USAT,0x70);
            OV7725_WriteReg(OV7725_REG_VSAT,0x70);
            break;
        case OV7725_COLOR_SATURATION_2:
            OV7725_WriteReg(OV7725_REG_USAT,0x60);
            OV7725_WriteReg(OV7725_REG_VSAT,0x60);
            break;
        case OV7725_COLOR_SATURATION_3:
            OV7725_WriteReg(OV7725_REG_USAT,0x50);
            OV7725_WriteReg(OV7725_REG_VSAT,0x50);
            break;
        case OV7725_COLOR_SATURATION_4:
            OV7725_WriteReg(OV7725_REG_USAT,0x40);
            OV7725_WriteReg(OV7725_REG_VSAT,0x40);
            break;
        case OV7725_COLOR_SATURATION_5:
            OV7725_WriteReg(OV7725_REG_USAT,0x30);
            OV7725_WriteReg(OV7725_REG_VSAT,0x30);
            break;
        case OV7725_COLOR_SATURATION_6:
            OV7725_WriteReg(OV7725_REG_USAT,0x20);
            OV7725_WriteReg(OV7725_REG_VSAT,0x20);
            break;
        case OV7725_COLOR_SATURATION_7:
            OV7725_WriteReg(OV7725_REG_USAT,0x10);
            OV7725_WriteReg(OV7725_REG_VSAT,0x10);
            break;
        case OV7725_COLOR_SATURATION_8:
            OV7725_WriteReg(OV7725_REG_USAT,0x00);
            OV7725_WriteReg(OV7725_REG_VSAT,0x00);
            break;
        default:
            break;
    }
}
void OV7725_SetBrightness(OV7725_Brightness_t Brightness)
{
    switch (Brightness)
    {
        case OV7725_BRIGHTNESS_0:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x48);
            OV7725_WriteReg(OV7725_REG_SIGN,0x06);
            break;
        case OV7725_BRIGHTNESS_1:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x38);
            OV7725_WriteReg(OV7725_REG_SIGN,0x06);
            break;
        case OV7725_BRIGHTNESS_2:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x28);
            OV7725_WriteReg(OV7725_REG_SIGN,0x06);
            break;
        case OV7725_BRIGHTNESS_3:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x18);
            OV7725_WriteReg(OV7725_REG_SIGN,0x06);
            break;
        case OV7725_BRIGHTNESS_4:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x08);
            OV7725_WriteReg(OV7725_REG_SIGN,0x06);
            break;
        case OV7725_BRIGHTNESS_5:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x08);
            OV7725_WriteReg(OV7725_REG_SIGN,0x0E);
            break;
        case OV7725_BRIGHTNESS_6:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x18);
            OV7725_WriteReg(OV7725_REG_SIGN,0x0E);
            break;
        case OV7725_BRIGHTNESS_7:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x28);
            OV7725_WriteReg(OV7725_REG_SIGN,0x0E);
            break;
        case OV7725_BRIGHTNESS_8:
            OV7725_WriteReg(OV7725_REG_BRIGT,0x38);
            OV7725_WriteReg(OV7725_REG_SIGN,0x0E);
            break;
        default:
            break;
    }
}
void OV7725_SetContrast(OV7725_Contrast_t Contrast)
{
    switch (Contrast)
    {
        case OV7725_CONTRAST_0:
            OV7725_WriteReg(OV7725_REG_CNST,0x30);
            break;
        case OV7725_CONTRAST_1:
            OV7725_WriteReg(OV7725_REG_CNST,0x2C);
            break;
        case OV7725_CONTRAST_2:
            OV7725_WriteReg(OV7725_REG_CNST,0x28);
            break;
        case OV7725_CONTRAST_3:
            OV7725_WriteReg(OV7725_REG_CNST,0x24);
            break;
        case OV7725_CONTRAST_4:
            OV7725_WriteReg(OV7725_REG_CNST,0x20);
            break;
        case OV7725_CONTRAST_5:
            OV7725_WriteReg(OV7725_REG_CNST,0x1C);
            break;
        case OV7725_CONTRAST_6:
            OV7725_WriteReg(OV7725_REG_CNST,0x18);
            break;
        case OV7725_CONTRAST_7:
            OV7725_WriteReg(OV7725_REG_CNST,0x14);
            break;
        case OV7725_CONTRAST_8:
            OV7725_WriteReg(OV7725_REG_CNST,0x10);
            break;
        default:
            break;
    }
}
void OV7725_SetSpecialEffects(OV7725_Special_Effect_t SpecialEffects)
{
    switch (SpecialEffects)
    {
        case OV7725_SPECIAL_EFFECT_NORMAL:
            OV7725_WriteReg(OV7725_REG_SDE,0x06);
            OV7725_WriteReg(OV7725_REG_UFiX,0x80);
            OV7725_WriteReg(OV7725_REG_VFiX,0x80);
            break;
        case OV7725_SPECIAL_EFFECT_BW:
            OV7725_WriteReg(OV7725_REG_SDE,0x26);
            OV7725_WriteReg(OV7725_REG_UFiX,0x80);
            OV7725_WriteReg(OV7725_REG_VFiX,0x80);
            break;
        case OV7725_SPECIAL_EFFECT_BLUISH:
            OV7725_WriteReg(OV7725_REG_SDE,0x1E);
            OV7725_WriteReg(OV7725_REG_UFiX,0xA0);
            OV7725_WriteReg(OV7725_REG_VFiX,0x40);
            break;
        case OV7725_SPECIAL_EFFECT_SEPIA:
            OV7725_WriteReg(OV7725_REG_SDE,0x1E);
            OV7725_WriteReg(OV7725_REG_UFiX,0x40);
            OV7725_WriteReg(OV7725_REG_VFiX,0xA0);
            break;
        case OV7725_SPECIAL_EFFECT_REDISH:
            OV7725_WriteReg(OV7725_REG_SDE,0x1E);
            OV7725_WriteReg(OV7725_REG_UFiX,0x80);
            OV7725_WriteReg(OV7725_REG_VFiX,0xC0);
            break;
        case OV7725_SPECIAL_EFFECT_GREENISH:
            OV7725_WriteReg(OV7725_REG_SDE,0x1E);
            OV7725_WriteReg(OV7725_REG_UFiX,0x60);
            OV7725_WriteReg(OV7725_REG_VFiX,0x60);
            break;
        case OV7725_SPECIAL_EFFECT_NEGATIVE:
            OV7725_WriteReg(OV7725_REG_SDE,0x46);
            break;
        default:
            break;
    }
}
void OV7725_EnableOutput(void)
{
    //从FIFO中读出数据,传送给主控芯片
    OV7725_OE(0);
}
void OV7725_DisableOutput(void)
{
    OV7725_OE(1);
}
uint8_t OV7725_GetFrame(volatile uint16_t *pBuf, OV7725_GetFrameType_t Type)
{
    uint16_t WidthIndex;
    uint16_t HeightIndex;
    uint16_t Data;
    
    if (OV7725_Info.FrameHandleSt == Frame_Done)
    {
        return 0;
    }
    
    OV7725_RRST(0);
    OV7725_RCLK(0);
    OV7725_RCLK(1);
    OV7725_RCLK(0);
    OV7725_RRST(1);
    OV7725_RCLK(1);
    for (HeightIndex=0; HeightIndex < OV7725_Info.OutputHeight; HeightIndex++)
    {
        for (WidthIndex=0; WidthIndex < OV7725_Info.OutputWidth; WidthIndex++)
        {
            OV7725_RCLK(0);
            Data = (OV7725_GetByteFormFIFO() << 8);
            OV7725_RCLK(1);
            OV7725_RCLK(0);
            Data |= OV7725_GetByteFormFIFO();
            OV7725_RCLK(1);
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
    
    OV7725_Info.FrameHandleSt = Frame_Done;
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
	LCD_DisplayON();
	LCD_SetDisplayDir(0);
	LCD_SetScanDir(LeftToRight_TopToBottom);
	LCD_SetWindow(0,0,LCD_Info.Width,LCD_Info.Height);
	LCD_WriteREGNo(LCD_Command_MemoryWrite);
	for(;;)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		OV7725_WRST(0);
		OV7725_WEN(1);
		OV7725_WRST(1);
		OV7725_GetFrame(&(LCD->LCD_RAM),OV7725_GET_FRAME_TYPE_NOINC);
	}
}


