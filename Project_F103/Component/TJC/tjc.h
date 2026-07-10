#ifndef __TJC_H
#define __TJC_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "message_buffer.h"

/* macro --------------------------------------------------------------------*/
#define TJC_MAX_PAGENUM		3
#define TJC_RXMSG_MAXSIZE	U2BUF_MAXSIZE
#define TJC_MSGBUF_SIZE		320
#define TJC_TXBUF_SIZE		64
/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/
typedef struct
{
	uint8_t CurrentPage;
	uint8_t Videoisplay;
}TJC_Info_t;
/* global variable ----------------------------------------------------------*/
extern TJC_Info_t TJC_Info;
extern MessageBufferHandle_t TJCRxMsgBuffer;

/* functions prototypes -----------------------------------------------------*/
void TJC_Init(void);
void TJC_ChangePage(uint8_t Page);
void TJC_ChangeVideoState(uint8_t id, uint8_t state);
void TJC_RxProcessFromISR(uint8_t *pBuf, uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken);
void TJC_ProcessMotorCmdTask(void *pvParameters);
void TJC_ReportMotorValue(uint16_t Funid, uint32_t Value);

#endif /* __TJC_H */

