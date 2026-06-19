#include "tjc.h"

/* global variable ----------------------------------------------------------*/
TJC_Info_t TJC_Info;

/* function implementation --------------------------------------------------*/
void TJC_Init(void)
{
	TJC_Info.CurrentPage = 1;
	TJC_Info.Videoisplay = 0;
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

