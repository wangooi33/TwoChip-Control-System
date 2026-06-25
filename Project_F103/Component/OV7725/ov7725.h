#ifndef __OV7725_H
#define __OV7725_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "sccb.h"
#include "OV7725_Reg.h"

/* macro --------------------------------------------------------------------*/

#define OV7725_WRST(x)			do{ x ?																			\
									HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_WRST_Pin, GPIO_PIN_SET) :	\
									HAL_GPIO_WritePin(OV7725_WRST_GPIO_Port, OV7725_WRST_Pin, GPIO_PIN_RESET);	\
								}while(0)
                                      
#define OV7725_RRST(x)			do{ x ?																			\
									HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_SET) :	\
									HAL_GPIO_WritePin(OV7725_RRST_GPIO_Port, OV7725_RRST_Pin, GPIO_PIN_RESET);	\
								}while(0)
                                      
#define OV7725_OE(x)			do{ x ?																			\
									HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_SET) :		\
									HAL_GPIO_WritePin(OV7725_OE_GPIO_Port, OV7725_OE_Pin, GPIO_PIN_RESET);		\
								}while(0)
                                      
#define OV7725_RCLK(x)			do{ x ?																			\
									(OV7725_RCLK_GPIO_Port->BSRR = (uint32_t)OV7725_RCLK_Pin) :					\
									(OV7725_RCLK_GPIO_Port->BRR = (uint32_t)OV7725_RCLK_Pin);					\
								}while(0)
                                      
#define OV7725_WEN(x)			do{ x ?																			\
									HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_SET) :		\
									HAL_GPIO_WritePin(OV7725_WEN_GPIO_Port, OV7725_WEN_Pin, GPIO_PIN_RESET);	\
								}while(0)

/* enum ---------------------------------------------------------------------*/
/*  */
typedef enum
{
	OV7725_GET_FRAME_TYPE_NOINC = 0x00,    /* 目的地址不自增 */
	OV7725_GET_FRAME_TYPE_AUTO_INC,        /* 目的地址自增 */
} OV7725_GetFrameType_t;

typedef enum
{
	Frame_Ready,
	Frame_WaitFIFOReady,
} OV7725_HandleState_t;

/* types --------------------------------------------------------------------*/
typedef struct
{
	uint16_t MID;
	uint16_t PID;
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
void OV7725_SetOutputWindow(OV7725_OutputMode_t OutputMode, uint16_t Width, uint16_t Height);
void OV7725_SetMode(void);
void Task4_Camera(void *pvParameters);

#endif /* __OV7725_H */
