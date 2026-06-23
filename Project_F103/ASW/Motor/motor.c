#include "motor.h"
#include "usart.h"
#include "stream_buffer.h"
/* macro --------------------------------------------------------------------*/
#define CMOTOR_DATA_STARTINDEX		6

/* global variable ----------------------------------------------------------*/
Motor_Info_t Motor_Info;
StreamBufferHandle_t CMotorStreamBuffer;
uint8_t MotorTxBuf[128];

/* function implementation --------------------------------------------------*/
void ComMotorTxProcess(uint8_t *pBuf, uint16_t DataSize, CMotorFunid_t Funid)
{
	uint8_t index = 0;
	uint16_t crc16;
	/* 帧头 */
	MotorTxBuf[index++] = 0x3A;
	MotorTxBuf[index++] = 0x3A;
	/* 功能码 */
	MotorTxBuf[index++] = Funid >> 8;
	MotorTxBuf[index++] = Funid & 0xFF;
	/* 数据段长度 */
	MotorTxBuf[index++] = DataSize >> 8;
	MotorTxBuf[index++] = DataSize & 0xFF;
	/* 数据段 */
	for (uint8_t i = 0; i < DataSize; i++)
	{
		MotorTxBuf[index++] = pBuf[i];
	}
	/* 校验位 */
	crc16 = CheckCRC16(MotorTxBuf,index);
	MotorTxBuf[index++] = crc16 >> 8;
	MotorTxBuf[index++] = crc16 & 0xFF;
	/* 帧尾 */
	MotorTxBuf[index++] = 0x4C;
	MotorTxBuf[index++] = 0x5E;
	HAL_UART_Transmit_DMA(&huart3,MotorTxBuf,index);
}
uint8_t CMotorCmpCrc16(uint8_t *pBuf, uint16_t Size)
{
	uint16_t crc16 = CheckCRC16(pBuf,Size - 4);
	uint16_t crc16Origin = (pBuf[Size - 4] << 8) | pBuf[Size - 3];
	if (crc16 == crc16Origin)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void Task6_ComMotorHandshake(void *pvParameters)
{
	uint8_t pBuf[16];
	uint16_t RxSize;
	for (;;)
	{
		switch (Motor_Info.CMotorState)
		{
			case CMotor_Init:
				CMotorStreamBuffer = xStreamBufferCreate(128,10);
				Motor_Info.CMotorState = CMotor_Handshake;
				break;
				
			case CMotor_Handshake:
				ComMotorTxProcess(NULL,0,CMotorFunid_Handshake);
				RxSize = xStreamBufferReceive(CMotorStreamBuffer,pBuf,16,500);
				if (RxSize < 14)
				{
					Motor_Info.CMotorState = CMotor_HandshakeFail;
				}
				else if (!CMotorCmpCrc16(pBuf,RxSize))
				{
					Motor_Info.CMotorState = CMotor_HandshakeFail;
				}
				else if (pBuf[2] != 0x81 || pBuf[3] != 0x80)
				{
					Motor_Info.CMotorState = CMotor_HandshakeFail;
				}
				else
				{
					Motor_Info.MotorSoftWareID[0] = pBuf[CMOTOR_DATA_STARTINDEX];
					Motor_Info.MotorSoftWareID[1] = pBuf[CMOTOR_DATA_STARTINDEX + 1];
					Motor_Info.MotorSoftWareID[2] = pBuf[CMOTOR_DATA_STARTINDEX + 2];
					Motor_Info.MotorSoftWareID[3] = pBuf[CMOTOR_DATA_STARTINDEX + 3];
					Motor_Info.CMotorState = CMotor_HandshakeSucceed;
				}
				break;

			case CMotor_HandshakeSucceed:
				vTaskDelete(wTaskHandle.Task6_CMotorHandshake);
				//xTaskCreate(Task7_ComMotorProcess,"Task7_ComMotorProcess",512,NULL,3,&wTaskHandle.Task7_CMotorProcess);
				break;

			case CMotor_HandshakeFail:
				vTaskDelete(wTaskHandle.Task6_CMotorHandshake);
				break;

			default:
				break;
		}
	}
	
}
void Task7_ComMotorProcess(void *pvParameters)
{
	uint8_t pBuf[128];
	uint16_t RxSize;
	for (;;)
	{
		switch (Motor_Info.CMotorState)
		{
			case CMotor_ToBDC:

				break;
			
			
			default:
				break;
		}
	}
	
}

