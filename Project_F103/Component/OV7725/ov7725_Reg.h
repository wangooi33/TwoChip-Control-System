#ifndef __OV7725_REG_H
#define __OV7725_REG_H
/* includes -----------------------------------------------------------------*/
#include "main.h"

/* macro --------------------------------------------------------------------*/
#define OV7725_REG_GAIN                 0x00
#define OV7725_REG_BLUE                 0x01
#define OV7725_REG_RED                  0x02
#define OV7725_REG_GREEN                0x03

#define OV7725_REG_BAVG                 0x05
#define OV7725_REG_GAVG                 0x06
#define OV7725_REG_RAVG                 0x07
#define OV7725_REG_AECH                 0x08
#define OV7725_REG_COM2                 0x09
#define OV7725_REG_PID                  0x0A
#define OV7725_REG_VER                  0x0B
#define OV7725_REG_COM3                 0x0C
#define OV7725_REG_COM4                 0x0D
#define OV7725_REG_COM5                 0x0E
#define OV7725_REG_COM6                 0x0F
#define OV7725_REG_AEC                  0x10
#define OV7725_REG_CLKRC                0x11
#define OV7725_REG_COM7                 0x12
#define OV7725_REG_COM8                 0x13
#define OV7725_REG_COM9                 0x14
#define OV7725_REG_COM10                0x15
#define OV7725_REG_REG16                0x16
#define OV7725_REG_HSTART               0x17
#define OV7725_REG_HSIZE                0x18
#define OV7725_REG_VSTRT                0x19
#define OV7725_REG_VSIZE                0x1A
#define OV7725_REG_PSHFT                0x1B
#define OV7725_REG_MIDH                 0x1C
#define OV7725_REG_MIDL                 0x1D

#define OV7725_REG_LAEC                 0x1F
#define OV7725_REG_COM11                0x20

#define OV7725_REG_BDBase               0x22
#define OV7725_REG_BDMStep              0x23
#define OV7725_REG_AEW                  0x24
#define OV7725_REG_AEB                  0x25
#define OV7725_REG_VPT                  0x26

#define OV7725_REG_REG28                0x28
#define OV7725_REG_HOutSize             0x29
#define OV7725_REG_EXHCH                0x2A
#define OV7725_REG_EXHCL                0x2B
#define OV7725_REG_VOutSize             0x2C
#define OV7725_REG_ADVFL                0x2D
#define OV7725_REG_ADVFH                0x2E
#define OV7725_REG_YAVE                 0x2F
#define OV7725_REG_LumHTh               0x30
#define OV7725_REG_LumLTh               0x31
#define OV7725_REG_HREF                 0x32
#define OV7725_REG_DM_LNL               0x33
#define OV7725_REG_DM_LNH               0x34
#define OV7725_REG_ADoff_B              0x35
#define OV7725_REG_ADoff_R              0x36
#define OV7725_REG_ADoff_Gb             0x37
#define OV7725_REG_ADoff_Gr             0x38
#define OV7725_REG_Off_B                0x39
#define OV7725_REG_Off_G                0x3A
#define OV7725_REG_Off_Gb               0x3B
#define OV7725_REG_Off_Gr               0x3C
#define OV7725_REG_COM12                0x3D
#define OV7725_REG_COM13                0x3E
#define OV7725_REG_COM14                0x3F
#define OV7725_REG_COM15                0x40
#define OV7725_REG_COM16                0x41
#define OV7725_REG_TGT_B                0x42
#define OV7725_REG_TGT_R                0x43
#define OV7725_REG_TGT_Gb               0x44
#define OV7725_REG_TGT_Gr               0x45
#define OV7725_REG_LC_CTR               0x46
#define OV7725_REG_LC_XC                0x47
#define OV7725_REG_LC_YC                0x48
#define OV7725_REG_LC_COEF              0x49
#define OV7725_REG_LC_RADI              0x4A
#define OV7725_REG_LC_COEFB             0x4B
#define OV7725_REG_LC_COEFR             0x4C
#define OV7725_REG_FixGain              0x4D
#define OV7725_REG_AREF0                0x4E
#define OV7725_REG_AREF1                0x4F
#define OV7725_REG_AREF2                0x50
#define OV7725_REG_AREF3                0x51
#define OV7725_REG_AREF4                0x52
#define OV7725_REG_AREF5                0x53
#define OV7725_REG_AREF6                0x54
#define OV7725_REG_AREF7                0x55

#define OV7725_REG_UFiX                 0x60
#define OV7725_REG_VFiX                 0x61
#define OV7725_REG_AWBb_blk             0x62
#define OV7725_REG_AWB_Ctrl0            0x63
#define OV7725_REG_DSP_Ctrl1            0x64
#define OV7725_REG_DSP_Ctrl2            0x65
#define OV7725_REG_DSP_Ctrl3            0x66
#define OV7725_REG_DSP_Ctrl4            0x67
#define OV7725_REG_AWB_bias             0x68
#define OV7725_REG_AWBCtrl1             0x69
#define OV7725_REG_AWBCtrl2             0x6A
#define OV7725_REG_AWBCtrl3             0x6B
#define OV7725_REG_AWBCtrl4             0x6C
#define OV7725_REG_AWBCtrl5             0x6D
#define OV7725_REG_AWBCtrl6             0x6E
#define OV7725_REG_AWBCtrl7             0x6F
#define OV7725_REG_AWBCtrl8             0x70
#define OV7725_REG_AWBCtrl9             0x71
#define OV7725_REG_AWBCtrl10            0x72
#define OV7725_REG_AWBCtrl11            0x73
#define OV7725_REG_AWBCtrl12            0x74
#define OV7725_REG_AWBCtrl13            0x75
#define OV7725_REG_AWBCtrl14            0x76
#define OV7725_REG_AWBCtrl15            0x77
#define OV7725_REG_AWBCtrl16            0x78
#define OV7725_REG_AWBCtrl17            0x79
#define OV7725_REG_AWBCtrl18            0x7A
#define OV7725_REG_AWBCtrl19            0x7B
#define OV7725_REG_AWBCtrl20            0x7C
#define OV7725_REG_AWBCtrl21            0x7D
#define OV7725_REG_GAM1                 0x7E
#define OV7725_REG_GAM2                 0x7F
#define OV7725_REG_GAM3                 0x80
#define OV7725_REG_GAM4                 0x81
#define OV7725_REG_GAM5                 0x82
#define OV7725_REG_GAM6                 0x83
#define OV7725_REG_GAM7                 0x84
#define OV7725_REG_GAM8                 0x85
#define OV7725_REG_GAM9                 0x86
#define OV7725_REG_GAM10                0x87
#define OV7725_REG_GAM11                0x88
#define OV7725_REG_GAM12                0x89
#define OV7725_REG_GAM13                0x8A
#define OV7725_REG_GAM14                0x8B
#define OV7725_REG_GAM15                0x8C
#define OV7725_REG_SLOP                 0x8D
#define OV7725_REG_DNSTh                0x8E
#define OV7725_REG_EDGE0                0x8F
#define OV7725_REG_EDGE1                0x90
#define OV7725_REG_DNSOff               0x91
#define OV7725_REG_EDGE2                0x92
#define OV7725_REG_EDGE3                0x93
#define OV7725_REG_MTX1                 0x94
#define OV7725_REG_MTX2                 0x95
#define OV7725_REG_MTX3                 0x96
#define OV7725_REG_MTX4                 0x97
#define OV7725_REG_MTX5                 0x98
#define OV7725_REG_MTX6                 0x99
#define OV7725_REG_MTX_Ctrl             0x9A
#define OV7725_REG_BRIGT                0x9B
#define OV7725_REG_CNST                 0x9C

#define OV7725_REG_UVDJ0                0x9E
#define OV7725_REG_UVDJ1                0x9F
#define OV7725_REG_SCAL0                0xA0
#define OV7725_REG_SCAL1                0xA1
#define OV7725_REG_SCAL2                0xA2
#define OV7725_REG_FIFOdlyM             0xA3
#define OV7725_REG_FIFOdlyA             0xA4

#define OV7725_REG_SDE                  0xA6
#define OV7725_REG_USAT                 0xA7
#define OV7725_REG_VSAT                 0xA8
#define OV7725_REG_HUECOS               0xA9
#define OV7725_REG_HUESIN               0xAA
#define OV7725_REG_SIGN                 0xAB
#define OV7725_REG_DSPAuto              0xAC


#define OV7725_REGInitBUF_MAXSIZE		65

/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/

/* global variable ----------------------------------------------------------*/
extern const uint8_t OV7725_RegInitData[OV7725_REGInitBUF_MAXSIZE][2];

/* enum ---------------------------------------------------------------------*/

/* functions prototypes -----------------------------------------------------*/

#endif /* __OV7725_REG_H */
