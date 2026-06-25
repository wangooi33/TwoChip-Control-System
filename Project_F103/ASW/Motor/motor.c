#include "motor.h"
#include "usart.h"

/* macro --------------------------------------------------------------------*/
#define CMOTOR_FRAME_LENGTH			12
#define CMOTOR_DATA_STARTINDEX		6
#define CMOTOR_CRC_STARTINDEX		8

/* global variable ----------------------------------------------------------*/
Motor_Info_t Motor_Info;
QueueHandle_t CMotorQueue;
QueueHandle_t CTaskQueue;
SemaphoreHandle_t UsartMutex;

CMotorDataTable_t BDC_Data[4] = 
{
	{CMid_ReadBDC_RPM, 0},
	{CMid_ReadBDC_Pos, 0},
	{CMid_ReadBDC_Cur, 0},
	{CMid_ReadBDC_PowerVoltage, 0}
};
CMotorDataTable_t BLDC_Data[3] = 
{
	{CMid_ReadBLDC_RPM, 0},
	{CMid_ReadBLDC_Pos, 0},
	{CMid_ReadBLDC_Cur, 0}
};

/* function declaration -----------------------------------------------------*/
void Task7_ComMotorReadProcess(void *pvParameters);
void Task8_ComMotorWriteProcess(void *pvParameters);

/* function implementation --------------------------------------------------*/
void CMotor_Init(void)
{
	CMotorQueue = xQueueCreate(5,CMOTOR_FRAME_LENGTH);
	UsartMutex = xSemaphoreCreateMutex();
	Motor_Info.CMotorState = CMotor_Handshake;
	Motor_Info.ReadMotorType = BDC;
}
void ComMotorTxProcess(uint8_t *pBuf, CMotorFunid_t Funid)
{
	uint8_t MotorTxBuf[16];
	uint8_t index = 0;
	uint16_t crc16;
	
	/* 帧头 */
	MotorTxBuf[index++] = 0x3A;
	MotorTxBuf[index++] = 0x3A;
	
	/* 功能码 */
	MotorTxBuf[index++] = Funid >> 8;
	MotorTxBuf[index++] = Funid & 0xFF;
	
	/* 数据段 */
	if (pBuf != NULL)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			MotorTxBuf[index++] = pBuf[i];
		}
	}
	else
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			MotorTxBuf[index++] = 0x00;
		}
	}
	
	/* 校验位 */
	crc16 = CheckCRC16(MotorTxBuf,index);
	MotorTxBuf[index++] = crc16 >> 8;
	MotorTxBuf[index++] = crc16 & 0xFF;
	
	/* 帧尾 */
	MotorTxBuf[index++] = 0x4C;
	MotorTxBuf[index++] = 0x5E;
	HAL_UART_Transmit(&huart3,MotorTxBuf,index,50);
}
uint8_t CMotorCheckFrame(uint8_t *pBuf)
{
	uint16_t crc16 = CheckCRC16(pBuf,CMOTOR_CRC_STARTINDEX);
	uint16_t crc16Origin = (pBuf[CMOTOR_CRC_STARTINDEX] << 8) | pBuf[CMOTOR_CRC_STARTINDEX + 1];
	
	if (crc16 == crc16Origin && pBuf[0] == 0x4A && pBuf[1] == 0x4A)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
bool ComMotorTransfer(uint16_t Funid, uint8_t *TxData, uint8_t *RxBuf)
{
	xSemaphoreTake(UsartMutex, portMAX_DELAY);
	for(uint8_t retry = 0; retry < 3; retry++)
	{
		ComMotorTxProcess(TxData, Funid);

		if(xQueueReceive(CMotorQueue, RxBuf, 200) == pdTRUE)
		{
			if(CMotorCheckFrame(RxBuf))
			{
				xSemaphoreGive(UsartMutex);
				return true;
			}
		}
	}
	xSemaphoreGive(UsartMutex);
	
	return false;
}

void Task6_ComMotorHandshake(void *pvParameters)
{
	uint8_t pBuf[12];
	for (;;)
	{
		switch (Motor_Info.CMotorState)
		{
			case CMotor_Handshake:
				if (ComMotorTransfer(CMid_Handshake,NULL,pBuf))
				{
					Motor_Info.MotorSoftWareID[0] = pBuf[CMOTOR_DATA_STARTINDEX];
					Motor_Info.MotorSoftWareID[1] = pBuf[CMOTOR_DATA_STARTINDEX + 1];
					Motor_Info.MotorSoftWareID[2] = pBuf[CMOTOR_DATA_STARTINDEX + 2];
					Motor_Info.MotorSoftWareID[3] = pBuf[CMOTOR_DATA_STARTINDEX + 3];
					Motor_Info.CMotorState = CMotor_HandshakeSucceed;
				}
				else
				{
					Motor_Info.CMotorState = CMotor_HandshakeFail;
				}
				break;

			case CMotor_HandshakeSucceed:
				Motor_Info.CMotorState = CMotor_Communicating;
				CTaskQueue = xQueueCreate(3,sizeof(MotorCmd_t));
				xTaskCreate(Task7_ComMotorReadProcess,"Task7_ComMotorRead",256,NULL,1,&wTaskHandle.Task7_CMotorRead);
				xTaskCreate(Task8_ComMotorWriteProcess,"Task8_ComMotorWrite",128,NULL,3,&wTaskHandle.Task8_CMotorWrite);
				vTaskDelete(NULL);
				break;

			case CMotor_HandshakeFail:
				vTaskDelete(NULL);
				break;

			default:
				break;
		}
	}
}
void Task7_ComMotorReadProcess(void *pvParameters)
{
	uint8_t pBuf[12];
	uint16_t RxFunid;
	for (;;)
	{
		switch (Motor_Info.ReadMotorType)
		{
			case BDC:
				for (uint8_t i = 0; i < sizeof(BDC_Data) / sizeof(BDC_Data[0]); i++)
				{
					if (ComMotorTransfer(BDC_Data[i].Funid,NULL,pBuf))
					{
						RxFunid = ((uint16_t)pBuf[2] << 8) | pBuf[3];
						if (RxFunid == BDC_Data[i].Funid)
						{
							BDC_Data[i].Data = (pBuf[CMOTOR_DATA_STARTINDEX] << 24)
											   | (pBuf[CMOTOR_DATA_STARTINDEX + 1] << 16)
											   | (pBuf[CMOTOR_DATA_STARTINDEX + 2] << 8)
											   |  pBuf[CMOTOR_DATA_STARTINDEX + 3];
						}
					}
				}
				break;

			case BLDC:
				for (uint8_t i = 0; i < sizeof(BLDC_Data) / sizeof(BLDC_Data[0]); i++)
				{
					if (ComMotorTransfer(BLDC_Data[i].Funid,NULL,pBuf))
					{
						RxFunid = ((uint16_t)pBuf[2] << 8) | pBuf[3];
						if (RxFunid == BLDC_Data[i].Funid)
						{
							BLDC_Data[i].Data = (pBuf[CMOTOR_DATA_STARTINDEX] << 24)
											    | (pBuf[CMOTOR_DATA_STARTINDEX + 1] << 16)
											    | (pBuf[CMOTOR_DATA_STARTINDEX + 2] << 8)
											    |  pBuf[CMOTOR_DATA_STARTINDEX + 3];
						}
					}
				}
				break;

			default:
				break;
		}
		vTaskDelay(10);
	}
	
}
void Task8_ComMotorWriteProcess(void *pvParameters)
{
	MotorCmd_t MotorCmd;
	uint8_t pBuf[12];
	for (;;)
	{
		if (xQueueReceive(CTaskQueue,&MotorCmd,portMAX_DELAY) == pdTRUE)
		{
			ComMotorTransfer(MotorCmd.Funid,(uint8_t *)&MotorCmd.Data,pBuf);
		}
	}
}

