#ifndef __TJC_H
#define __TJC_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "message_buffer.h"

/* macro --------------------------------------------------------------------*/
#define TJC_MAX_PAGENUM		3
#define TJC_MOTORCMD_LENGTH	6
#define TJC_MSGBUF_SIZE		96
/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/
typedef struct
{
	uint8_t CurrentPage;
	uint8_t Videoisplay;
}TJC_Info_t;
/* global variable ----------------------------------------------------------*/
extern TJC_Info_t TJC_Info;
extern MessageBufferHandle_t TJCMotorMsgBuffer;

/* functions prototypes -----------------------------------------------------*/
void TJC_Init(void);
void TJC_ChangePage(uint8_t Page);
void TJC_ChangeVideoState(uint8_t id, uint8_t state);
void TJC_RxProcessFromISR(uint8_t *pBuf, uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken);

#endif /* __TJC_H */

