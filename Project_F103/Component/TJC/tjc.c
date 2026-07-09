#include "tjc.h"

/* global variable ----------------------------------------------------------*/
TJC_Info_t TJC_Info;
MessageBufferHandle_t TJCMotorMsgBuffer;

/* function implementation --------------------------------------------------*/
void TJC_Init(void)
{
	TJC_Info.CurrentPage = 1;
	TJC_Info.Videoisplay = 0;
	TJCMotorMsgBuffer = xMessageBufferCreate(TJC_MSGBUF_SIZE);
}

void TJC_ChangePage(uint8_t Page)
{
	/* page 2:70 61 67 65 20 32 ff ff ff */
	uint8_t TxBuffer[256] = {0};
	uint8_t index = 0;

	TxBuffer[index++] = 0x70;
	TxBuffer[index++] = 0x61;
	TxBuffer[index++] = 0x67;
	TxBuffer[index++] = 0x65;
	TxBuffer[index++] = 0x20;
	TxBuffer[index++] = Page + '0';
	TxBuffer[index++] = 0xFF;
	TxBuffer[index++] = 0xFF;
	TxBuffer[index++] = 0xFF;
	HAL_UART_Transmit(&huart2, TxBuffer, index, index*2);
}

void TJC_RxProcessFromISR(uint8_t *pBuf, uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken)
{
	uint16_t offset = 0;

	if ((pBuf == NULL) || (pxHigherPriorityTaskWoken == NULL))
	{
		return;
	}

	if (TJCMotorMsgBuffer == NULL)
	{
		return;
	}

	while ((offset + TJC_MOTORCMD_LENGTH) <= Size)
	{
		xMessageBufferSendFromISR(TJCMotorMsgBuffer,
								  &pBuf[offset],
								  TJC_MOTORCMD_LENGTH,
								  pxHigherPriorityTaskWoken);
		offset += TJC_MOTORCMD_LENGTH;
	}
}
void TJC_ChangeVideoState(uint8_t id, uint8_t state)
{
	/* v0.en=2:76 30 2E 65 6E 3D 32 ff ff ff */
	uint8_t TxBuffer[256] = {0};
	uint8_t index = 0;

	TxBuffer[index++] = 0x76;
	TxBuffer[index++] = id + '0';
	TxBuffer[index++] = 0x2E;
	TxBuffer[index++] = 0x65;
	TxBuffer[index++] = 0x6E;
	TxBuffer[index++] = 0x3D;
	TxBuffer[index++] = state + '0';
	TxBuffer[index++] = 0xFF;
	TxBuffer[index++] = 0xFF;
	TxBuffer[index++] = 0xFF;
	HAL_UART_Transmit(&huart2, TxBuffer, index, index*2);
}

