#ifndef __OV7725_H
#define __OV7725_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "sccb.h"
#include "OV7725_Reg.h"

/* macro --------------------------------------------------------------------*/

#define OV7725_WRST(x)            do{ x ?                                                                          \
                                      HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_SET) :    \
                                      HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_RESET);   \
                                  }while(0)
                                      
#define OV7725_RRST(x)            do{ x ?                                                                          \
                                      HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_SET) :    \
                                      HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_RESET);   \
                                  }while(0)
                                      
#define OV7725_OE(x)              do{ x ?                                                                          \
                                      HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_SET) :        \
                                      HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_RESET);       \
                                  }while(0)
                                      
#define OV7725_RCLK(x)           do{ x ?                                                             \
                                      (OV7725_RCLK_GPIO_Port->BSRR = (uint32_t)OV7725_RCLK_Pin) :    \
                                      (OV7725_RCLK_GPIO_Port->BRR = (uint32_t)OV7725_RCLK_Pin);      \
                                  }while(0)
                                      
#define OV7725_WEN(x)             do{ x ?                                                                          \
                                      HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_SET) :      \
                                      HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_RESET);     \
                                  }while(0)

//分辨率
#define OV7725_VGA_WIDTH_MAX      640
#define OV7725_VGA_HEIGHT_MAX     480
#define OV7725_QVGA_WIDTH_MAX     320
#define OV7725_QVGA_HEIGHT_MAX    240

/* enum ---------------------------------------------------------------------*/
/* 灯光模式 */
typedef enum
{
    OV7725_LIGHT_MODE_AUTO = 0x00,         /* Auto */
    OV7725_LIGHT_MODE_SUNNY,               /* Sunny */
    OV7725_LIGHT_MODE_CLOUDY,              /* Cloudy */
    OV7725_LIGHT_MODE_OFFICE,              /* Office */
    OV7725_LIGHT_MODE_HOME,                /* Home */
    OV7725_LIGHT_MODE_NIGHT,               /* Night */
} OV7725_Light_Mode_t;
    
/* 色彩饱和度 */
typedef enum
{
    OV7725_COLOR_SATURATION_0 = 0x00,      /* +4 */
    OV7725_COLOR_SATURATION_1,             /* +3 */
    OV7725_COLOR_SATURATION_2,             /* +2 */
    OV7725_COLOR_SATURATION_3,             /* +1 */
    OV7725_COLOR_SATURATION_4,             /* 0 */
    OV7725_COLOR_SATURATION_5,             /* -1 */
    OV7725_COLOR_SATURATION_6,             /* -2 */
    OV7725_COLOR_SATURATION_7,             /* -3 */
    OV7725_COLOR_SATURATION_8,             /* -4 */
} OV7725_Color_Saturation_t;
    
/* 亮度 */ 
typedef enum
{
    OV7725_BRIGHTNESS_0 = 0x00,            /* +4 */
    OV7725_BRIGHTNESS_1,                   /* +3 */
    OV7725_BRIGHTNESS_2,                   /* +2 */
    OV7725_BRIGHTNESS_3,                   /* +1 */
    OV7725_BRIGHTNESS_4,                   /* 0 */
    OV7725_BRIGHTNESS_5,                   /* -1 */
    OV7725_BRIGHTNESS_6,                   /* -2 */
    OV7725_BRIGHTNESS_7,                   /* -3 */
    OV7725_BRIGHTNESS_8,                   /* -4 */
} OV7725_Brightness_t;
    
/* 对比度 */
typedef enum
{
    OV7725_CONTRAST_0 = 0x00,           /* +4 */
    OV7725_CONTRAST_1,                  /* +3 */
    OV7725_CONTRAST_2,                  /* +2 */
    OV7725_CONTRAST_3,                  /* +1 */
    OV7725_CONTRAST_4,                  /* 0 */
    OV7725_CONTRAST_5,                  /* -1 */
    OV7725_CONTRAST_6,                  /* -2 */
    OV7725_CONTRAST_7,                  /* -3 */
    OV7725_CONTRAST_8,                  /* -4 */
} OV7725_Contrast_t;

/* 特殊效果 */
typedef enum
{
    OV7725_SPECIAL_EFFECT_NORMAL = 0x00,    /* Normal */
    OV7725_SPECIAL_EFFECT_BW,               /* B&W */
    OV7725_SPECIAL_EFFECT_BLUISH,           /* Bluish */
    OV7725_SPECIAL_EFFECT_SEPIA,            /* Sepia */
    OV7725_SPECIAL_EFFECT_REDISH,           /* Redish */
    OV7725_SPECIAL_EFFECT_GREENISH,         /* Greenish */
    OV7725_SPECIAL_EFFECT_NEGATIVE,         /* Negative */
} OV7725_Special_Effect_t;

/* 输出模式 */
typedef enum
{
    OV7725_OUTPUT_MODE_VGA = 0x00,         /* VGA */
    OV7725_OUTPUT_MODE_QVGA,               /* QVGA */
} OV7725_OutputMode_t;

/*  */
typedef enum
{
    OV7725_GET_FRAME_TYPE_NOINC = 0x00,    /* 目的地址不自增 */
    OV7725_GET_FRAME_TYPE_AUTO_INC,        /* 目的地址自增 */
} OV7725_GetFrameType_t;

typedef enum
{
    Frame_Pend,
    Frame_Done,
} OV7725_HandleState_t;

/* types --------------------------------------------------------------------*/
typedef struct
{
	uint8_t isEnable;
    uint16_t OutputWidth;
    uint16_t OutputHeight;
    uint16_t FrameCount;
    OV7725_HandleState_t FrameHandleSt;
}OV7725_Info_t;

/* global variable ----------------------------------------------------------*/
extern OV7725_Info_t OV7725_Info;

/* functions prototypes -----------------------------------------------------*/
void OV7725_Init(void);
void OV7725_EnableOutput(void);
void OV7725_DisableOutput(void);
uint8_t OV7725_GetFrame(volatile uint16_t *pBuf, OV7725_GetFrameType_t Type);
void OV7725_SetMode(void);

void OV7725_SetOutputWindow(OV7725_OutputMode_t OutputMode, uint16_t Width, uint16_t Height);
void OV7725_SetLightMode(OV7725_Light_Mode_t Light_Mode);
void OV7725_SetColorSaturation(OV7725_Color_Saturation_t ColorSaturation);
void OV7725_SetBrightness(OV7725_Brightness_t Brightness);
void OV7725_SetContrast(OV7725_Contrast_t Contrast);
void OV7725_SetSpecialEffects(OV7725_Special_Effect_t SpecialEffects);

void Task4_Camera(void *pvParameters);

#endif /* __OV7725_H */
