#include "tjc.h"
#include "motor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* global variable ----------------------------------------------------------*/
TJC_Info_t TJC_Info;
MessageBufferHandle_t TJCRxMsgBuffer;
static SemaphoreHandle_t TJCUartMutex;

/* private types ------------------------------------------------------------*/
typedef struct
{
	const char *SetKey;
	const char *GetKey;
	ReadMotorType_t MotorType;
	CMotorFunid_t WriteFunid;
	CMotorFunid_t ReadFunid;
}TJCMotorMap_t;

/* private macro ------------------------------------------------------------*/
#define TJC_CMD_QUEUE_WAIT_MS	20
#define TJC_REPORT_SUFFIX		"\xFF\xFF\xFF"

/* private variable ---------------------------------------------------------*/
static const TJCMotorMap_t TJCMotorMap[] =
{
	{"bdc_set_rpm.txt=\"",  "bdc_get_rpm.val=",  BDC,  CMid_WriteBDC_RPM,  CMid_ReadBDC_RPM},
	{"bdc_set_pos.txt=\"",  "bdc_get_pos.val=",  BDC,  CMid_WriteBDC_Pos,  CMid_ReadBDC_Pos},
	{"bdc_set_cur.txt=\"",  "bdc_get_cur.val=",  BDC,  CMid_WriteBDC_Cur,  CMid_ReadBDC_Cur},
	{"bldc_set_rpm.txt=\"", "bldc_get_rpm.val=", BLDC, CMid_WriteBLDC_RPM, CMid_ReadBLDC_RPM},
	{"bldc_set_pos.txt=\"", "bldc_get_pos.val=", BLDC, CMid_WriteBLDC_Pos, CMid_ReadBLDC_Pos},
	{"bldc_set_cur.txt=\"", "bldc_get_cur.val=", BLDC, CMid_WriteBLDC_Cur, CMid_ReadBLDC_Cur},
};

/* private function declaration ---------------------------------------------*/
static const TJCMotorMap_t *TJC_FindMapBySetCmd(const char *pMsg);
static const TJCMotorMap_t *TJC_FindMapByReadFunid(uint16_t Funid);
static BaseType_t TJC_ParseMotorCmd(const uint8_t *pBuf, uint16_t Size, MotorCmd_t *pCmd, ReadMotorType_t *pMotorType);
static void TJC_U32ToDataBytes(uint32_t Value, uint8_t *pData);
static void TJC_SendBytes(const uint8_t *pBuf, uint16_t Len);

/* function implementation --------------------------------------------------*/
void TJC_Init(void)
{
	TJC_Info.CurrentPage = 1;
	TJC_Info.Videoisplay = 0;
	TJCRxMsgBuffer = xMessageBufferCreate(TJC_MSGBUF_SIZE);
	TJCUartMutex = xSemaphoreCreateMutex();
	if (TJCRxMsgBuffer == NULL || TJCUartMutex == NULL)
	{
		Error_Handler();
	}
}

void TJC_ChangePage(uint8_t Page)
{
	uint8_t TxBuffer[16] = {0};
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
	TJC_SendBytes(TxBuffer, index);
}

void TJC_ChangeVideoState(uint8_t id, uint8_t state)
{
	uint8_t TxBuffer[16] = {0};
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
	TJC_SendBytes(TxBuffer, index);
}

void TJC_RxProcessFromISR(uint8_t *pBuf, uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken)
{
	if ((pBuf == NULL) || (pxHigherPriorityTaskWoken == NULL) || (Size == 0U))
	{
		return;
	}

	if (TJCRxMsgBuffer == NULL)
	{
		return;
	}

	xMessageBufferSendFromISR(TJCRxMsgBuffer, pBuf, Size, pxHigherPriorityTaskWoken);
}

void TJC_ProcessMotorCmdTask(void *pvParameters)
{
	uint8_t RxBuf[TJC_RXMSG_MAXSIZE];
	size_t RxLen;
	MotorCmd_t MotorCmd;
	ReadMotorType_t MotorType;

	(void)pvParameters;

	for (;;)
	{
		RxLen = xMessageBufferReceive(TJCRxMsgBuffer,
									  RxBuf,
									  sizeof(RxBuf),
									  portMAX_DELAY);
		if ((RxLen > 0U) && (TJC_ParseMotorCmd(RxBuf, (uint16_t)RxLen, &MotorCmd, &MotorType) == pdPASS))
		{
			Motor_Info.ReadMotorType = MotorType;
			if (xQueueSend(CTaskQueue, &MotorCmd, pdMS_TO_TICKS(TJC_CMD_QUEUE_WAIT_MS)) != pdPASS)
			{
				/* drop command if motor write queue is full */
			}
		}
	}
}

void TJC_ReportMotorValue(uint16_t Funid, uint32_t Value)
{
	char TxBuffer[TJC_TXBUF_SIZE];
	const TJCMotorMap_t *pMap = TJC_FindMapByReadFunid(Funid);
	int len;
	const char *pKey;

	if (Funid == CMid_ReadBDC_PowerVoltage)
	{
		pKey = "bdc_get_power_voltage.val=";
	}
	else if (pMap != NULL)
	{
		pKey = pMap->GetKey;
	}
	else
	{
		return;
	}

	len = snprintf(TxBuffer,
				   sizeof(TxBuffer),
				   "%s%lu%s",
				   pKey,
				   (unsigned long)Value,
				   TJC_REPORT_SUFFIX);
	if ((len > 0) && (len < (int)sizeof(TxBuffer)))
	{
		TJC_SendBytes((uint8_t *)TxBuffer, (uint16_t)len);
	}
}

static const TJCMotorMap_t *TJC_FindMapBySetCmd(const char *pMsg)
{
	for (uint8_t i = 0; i < (sizeof(TJCMotorMap) / sizeof(TJCMotorMap[0])); i++)
	{
		if (strncmp(pMsg, TJCMotorMap[i].SetKey, strlen(TJCMotorMap[i].SetKey)) == 0)
		{
			return &TJCMotorMap[i];
		}
	}

	return NULL;
}

static const TJCMotorMap_t *TJC_FindMapByReadFunid(uint16_t Funid)
{
	for (uint8_t i = 0; i < (sizeof(TJCMotorMap) / sizeof(TJCMotorMap[0])); i++)
	{
		if (TJCMotorMap[i].ReadFunid == Funid)
		{
			return &TJCMotorMap[i];
		}
	}

	return NULL;
}

static BaseType_t TJC_ParseMotorCmd(const uint8_t *pBuf, uint16_t Size, MotorCmd_t *pCmd, ReadMotorType_t *pMotorType)
{
	char MsgBuf[TJC_RXMSG_MAXSIZE + 1];
	char *pValueStart;
	char *pValueEnd;
	uint32_t value;
	const TJCMotorMap_t *pMap;

	if ((pBuf == NULL) || (pCmd == NULL) || (pMotorType == NULL) || (Size == 0U) || (Size > TJC_RXMSG_MAXSIZE))
	{
		return pdFAIL;
	}

	memcpy(MsgBuf, pBuf, Size);
	MsgBuf[Size] = '\0';

	while ((Size >= 1U) && ((MsgBuf[Size - 1U] == '\r') || (MsgBuf[Size - 1U] == '\n') || (MsgBuf[Size - 1U] == (char)0xFF)))
	{
		MsgBuf[Size - 1U] = '\0';
		Size--;
	}

	pMap = TJC_FindMapBySetCmd(MsgBuf);
	if (pMap == NULL)
	{
		return pdFAIL;
	}

	pValueStart = MsgBuf + strlen(pMap->SetKey);
	pValueEnd = strchr(pValueStart, '"');
	if (pValueEnd == NULL)
	{
		return pdFAIL;
	}
	*pValueEnd = '\0';

	value = (uint32_t)strtoul(pValueStart, NULL, 10);
	pCmd->Funid = pMap->WriteFunid;
	TJC_U32ToDataBytes(value, pCmd->Data);
	*pMotorType = pMap->MotorType;

	return pdPASS;
}

static void TJC_U32ToDataBytes(uint32_t Value, uint8_t *pData)
{
	pData[0] = (uint8_t)(Value >> 24);
	pData[1] = (uint8_t)(Value >> 16);
	pData[2] = (uint8_t)(Value >> 8);
	pData[3] = (uint8_t)Value;
}

static void TJC_SendBytes(const uint8_t *pBuf, uint16_t Len)
{
	if (TJCUartMutex != NULL)
	{
		xSemaphoreTake(TJCUartMutex, portMAX_DELAY);
	}

	HAL_UART_Transmit(&huart2, (uint8_t *)pBuf, Len, (uint16_t)(Len * 2U));

	if (TJCUartMutex != NULL)
	{
		xSemaphoreGive(TJCUartMutex);
	}
}
