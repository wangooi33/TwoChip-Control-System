#include "ov7725_Reg.h"

/* macro --------------------------------------------------------------------*/
/* 通讯地址:Reg_0x0B */
#define OV7725_SCCB_ADDR	0x21

/* global variable ----------------------------------------------------------*/
const uint8_t OV7725_RegInitData[OV7725_REGInitBUF_MAXSIZE][2] = 
{
	{OV7725_REG_CLKRC,     0x00},
	{OV7725_REG_COM7,      0x46},
	{OV7725_REG_HSTART,    0x3F},
	{OV7725_REG_HSIZE,     0x50},
	{OV7725_REG_VSTRT,     0x03},
	{OV7725_REG_VSIZE,     0x78},
	{OV7725_REG_HREF,      0x00},
	{OV7725_REG_HOutSize,  0x50},
	{OV7725_REG_VOutSize,  0x78},
	{OV7725_REG_TGT_B,     0x7F},
	{OV7725_REG_FixGain,   0x09},
	{OV7725_REG_AWB_Ctrl0, 0xE0},
	{OV7725_REG_DSP_Ctrl1, 0xFF},
	{OV7725_REG_DSP_Ctrl2, 0x06},
	{OV7725_REG_DSP_Ctrl3, 0x00},
	{OV7725_REG_DSP_Ctrl4, 0x00},
	{OV7725_REG_COM8,      0xF0},
	{OV7725_REG_COM4,      0xC1},
	{OV7725_REG_COM6,      0xC5},
	{OV7725_REG_COM10,     0x02},
	{OV7725_REG_COM9,      0x11},
	{OV7725_REG_BDBase,    0x7F},
	{OV7725_REG_BDMStep,   0x03},
	{OV7725_REG_AEW,       0x40},
	{OV7725_REG_AEB,       0x30},
	{OV7725_REG_VPT,       0xA1},
	{OV7725_REG_EXHCL,     0x9E},
	{OV7725_REG_AWBCtrl3,  0xAA},
	{OV7725_REG_COM8,      0xFF},
	{OV7725_REG_EDGE1,     0x08},
	{OV7725_REG_DNSOff,    0x01},
	{OV7725_REG_EDGE2,     0x03},
	{OV7725_REG_EDGE3,     0x00},
	{OV7725_REG_MTX1,      0xB0},
	{OV7725_REG_MTX2,      0x9D},
	{OV7725_REG_MTX3,      0x13},
	{OV7725_REG_MTX4,      0x16},
	{OV7725_REG_MTX5,      0x7B},
	{OV7725_REG_MTX6,      0x91},
	{OV7725_REG_MTX_Ctrl,  0x1E},
	{OV7725_REG_BRIGT,     0x08},
	{OV7725_REG_CNST,      0x20},
	{OV7725_REG_UVDJ0,     0x81},
	{OV7725_REG_SDE,       0x06},
	{OV7725_REG_USAT,      0x65},
	{OV7725_REG_VSAT,      0x65},
	{OV7725_REG_HUECOS,    0x80},
	{OV7725_REG_HUESIN,    0x80},
	{OV7725_REG_GAM1,      0x0C},
	{OV7725_REG_GAM2,      0x16},
	{OV7725_REG_GAM3,      0x2A},
	{OV7725_REG_GAM4,      0x4E},
	{OV7725_REG_GAM5,      0x61},
	{OV7725_REG_GAM6,      0x6F},
	{OV7725_REG_GAM7,      0x7B},
	{OV7725_REG_GAM8,      0x86},
	{OV7725_REG_GAM9,      0x8E},
	{OV7725_REG_GAM10,     0x97},
	{OV7725_REG_GAM11,     0xA4},
	{OV7725_REG_GAM12,     0xAF},
	{OV7725_REG_GAM13,     0xC5},
	{OV7725_REG_GAM14,     0xD7},
	{OV7725_REG_GAM15,     0xE8},
	{OV7725_REG_SLOP,      0x20},
	{OV7725_REG_COM3,      0x00},
	{OV7725_REG_COM5,      0xF5},
	{OV7725_REG_COM13,     0x00},
	{OV7725_REG_COM15,     0xD0},
};
/* function implementation --------------------------------------------------*/
void OV7725_WriteReg(uint8_t Reg, uint8_t Data)
{
	SCCB_3PhaseWrite(OV7725_SCCB_ADDR,Reg,Data);
}
uint8_t OV7725_ReadReg(uint8_t Reg)
{
	uint8_t Data;
	SCCB_2PhaseWrite(OV7725_SCCB_ADDR,Reg);
	Data = SCCB_2PhaseRead(OV7725_SCCB_ADDR);
	return Data;
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
void OV7725_SCCBRegReset(void)
{
	OV7725_WriteReg(OV7725_REG_COM7,0x80);
	Delay_ms(5);
}

