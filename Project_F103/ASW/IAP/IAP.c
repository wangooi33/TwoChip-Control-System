#include <stdio.h>
#include "IAP.h"
#include "task_manage.h"

IAP_Info_t IAP_Info;

void IAP_ChangeUpdateFlag(uint8_t isEnable)
{
    uint32_t Val = (isEnable) ? APP_UPDATE_FLAG : 0;
    AT24Cxx_Write(0,(uint8_t *)&Val,4);
}
void Task2_IAP(void *pvParameters)
{
	for (;;)
	{
		//等待任务通知,阻塞无限等待
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		//收到通知
		IAP_ChangeUpdateFlag(1);
		AT24Cxx_Read(0, IAP_Info.UpdateFlag, 4);
		//复位之前需要进行数据保存
		NVIC_SystemReset();
	}
}

void IAP_RxProcess(uint8_t *pData, uint16_t Size)
{
    if (pData[0] == 0x3A && pData[1] == 0x3A)
    {
        switch (pData[2])
        {
            case 0xF1:
                xTaskNotifyGive(wTaskHandle.Task2_IAPHandle);
                break;
        }
    }
}
