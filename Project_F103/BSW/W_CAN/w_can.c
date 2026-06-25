#include "w_can.h"

/* global variable ----------------------------------------------------------*/
CAN_FilterTypeDef FilterConfig = {0};
CAN_TxHeaderTypeDef TxHeader = {0};
CAN_RxHeaderTypeDef RxHeader = {0};

uint8_t CanRxBuf[8];

/* function implementation --------------------------------------------------*/
void WCAN_Init(void)
{
	/* 过滤器编号 */
	FilterConfig.FilterBank = 0;
	/* 过滤器模式:标识符屏蔽(模糊匹配) */
	FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	/* 过滤器位数:32位 */
	FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	/* 只接收ID为1的数据 */
	/* 期望ID */
	FilterConfig.FilterIdHigh = 0x0020;
	FilterConfig.FilterIdLow = 0x0000;
	/* 掩码 */
	FilterConfig.FilterMaskIdHigh = 0xFFE0;
	FilterConfig.FilterMaskIdLow = 0x0000;

	/* 接收器 */
	FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
	FilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &FilterConfig);

	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan);
}
void WCAN_Send(uint16_t id, uint8_t *data, uint8_t len)
{
	/* 邮箱编号 */
	uint32_t mailbox = 0;
	TxHeader.StdId = id;
	/* 标准帧 */
	TxHeader.IDE = CAN_ID_STD;
	/* 数据帧 */
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = len;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mailbox) != HAL_OK)
	{
		return;
	}
	/* 等待发送邮箱空闲 */
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanRxBuf);
		if (RxHeader.StdId == 0x001)
		{
			
		}
		
	}
}
