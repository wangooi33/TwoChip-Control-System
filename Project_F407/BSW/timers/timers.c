/* Includes ------------------------------------------------------------------*/
#include "timers.h"
#include "main.h"
/* global variable -----------------------------------------------------------*/
QueueHandle_t TimersManagerRxQueue;
TimerMinHeap_t TimerActiveHeap = {0};
TimerHandle_t *TimerRegistry[TIMER_MAX_REGISTRY] = {0};

/* functions declaration -----------------------------------------------------*/

/* functions implementation --------------------------------------------------*/

void AsmRaiseBasepri( void )
{
	__asm volatile("cpsid i");
}
void AsmSetBasepri( void )
{
	__asm volatile("cpsie i");
}
/*-----------------------------------------------------------*/

void QueueInitialize( QueueHandle_t *Queue, const char *pQueueName, uint8_t QueueID, uint8_t ItemSize )
{
	if ( Queue != NULL  && ItemSize > 0 && ItemSize <= QUEUE_MAX_DATALENGTH  )
	{
		Queue->QueueManager.pQueueName = pQueueName;
		Queue->QueueManager.QueueID = QueueID;
		Queue->QueueManager.QueueType = queueTYPE_QUEUE;
		Queue->QueueManager.pHead = Queue->QueueStorageArea;
		Queue->QueueManager.pTail = Queue->QueueStorageArea + (QUEUE_MAX_DATALENGTH - 1) * ItemSize;
		Queue->QueueManager.pWrite = Queue->QueueManager.pHead;
		Queue->QueueManager.pRead = Queue->QueueManager.pHead;
		Queue->QueueManager.MaxVolume = QUEUE_MAX_DATALENGTH;
		Queue->QueueManager.ItemSize = ItemSize;
		Queue->QueueManager.NumOfResources = 0;
	}
}
static void prvCopyDataToQueue( QueueGeneric_t * const pQueue, const void *pItemToQueue, const uint8_t Position )
{
	if ( Position == queueSEND_TOBACK )
	{
		//先拷贝数据,在移动指针.此时,这个指针指向下次的写地址,这个动作是在为下次操作准备
		memcpy(pQueue->pWrite,pItemToQueue,pQueue->ItemSize);
		pQueue->pWrite += pQueue->ItemSize;
		if ( pQueue->pWrite > pQueue->pTail )
		{
			//数据并不会写入不属于队列的内存,因为上面已经做出限制if ( pQueue->NumOfResources < pQueue->MaxLength || CopyPosition == queueSEND_OVERWRITE )
			pQueue->pWrite = pQueue->pHead;
		}
		pQueue->NumOfResources++;
	}
	else 
	{
		if ( Position == queueSEND_TOFRONT )
		{
			//把 pRead 往前挪一个位置,然后把写入新数据
			pQueue->pRead -= pQueue->ItemSize;
			if ( pQueue->pRead < pQueue->pHead )
			{
				pQueue->pRead = pQueue->pTail;
			}
			memcpy(pQueue->pRead, pItemToQueue, pQueue->ItemSize);
			pQueue->NumOfResources++;
		}
		else if ( Position == queueSEND_OVERWRITE )
		{
			memcpy(pQueue->pHead, pItemToQueue, pQueue->ItemSize);
			pQueue->pWrite = pQueue->pHead + pQueue->ItemSize;
			if ( pQueue->pWrite > pQueue->pTail )
			{
				pQueue->pWrite = pQueue->pHead;
			}
			pQueue->pRead = pQueue->pHead;
			pQueue->NumOfResources = 1;
		}
	}

}
uint8_t QueueGenericSend( QueueHandle_t *Queue, const void * const pItemToQueue, const uint8_t CopyPosition )
{
	//一次只生产一个资源
	uint8_t xReturn;
	if ( Queue == NULL || pItemToQueue == NULL )
	{
		xReturn = pdFALSE;
		return xReturn;
	}
	QueueGeneric_t *pQueue = &(Queue->QueueManager);
	AsmRaiseBasepri();
	{
		if ( (pQueue->NumOfResources) < (pQueue->MaxVolume) || CopyPosition == queueSEND_OVERWRITE )
		{
			prvCopyDataToQueue(pQueue,pItemToQueue,CopyPosition);
			xReturn = pdTRUE;
		}
		else
		{
			//队列已满
			xReturn = pdFALSE;
		}
	}
	AsmSetBasepri();
	return xReturn;
}
uint8_t QueueSendToBack( QueueHandle_t *Queue, const void * const pItemToQueue )
{
	return QueueGenericSend(Queue, pItemToQueue, queueSEND_TOBACK);
}
uint8_t QueueSendToFront( QueueHandle_t *Queue, const void * const pItemToQueue )
{
	return QueueGenericSend(Queue, pItemToQueue, queueSEND_TOFRONT);
}
uint8_t QueueOverWrite( QueueHandle_t *Queue, const void * const pItemToQueue )
{
	return QueueGenericSend(Queue, pItemToQueue, queueSEND_OVERWRITE);
}

static void prvCopyDataFromQueue( QueueGeneric_t * const pQueue, void * const pBuffer )
{
	if( pQueue->ItemSize != 0 && pBuffer != NULL )
	{
		memcpy(pBuffer,pQueue->pRead,pQueue->ItemSize);
		memset(pQueue->pRead,0,pQueue->ItemSize);
		pQueue->pRead += pQueue->ItemSize;
		if ( pQueue->pRead > pQueue->pTail )
		{
			pQueue->pRead = pQueue->pHead;
		}
	}
}
uint8_t QueueReceive( QueueHandle_t *Queue, void * const pBuffer )
{
	uint8_t xReturn;
	QueueGeneric_t *pQueue = &(Queue->QueueManager);
	if ( Queue == NULL && pBuffer == NULL )
	{
		xReturn = pdFALSE;
		return xReturn;
	}
	AsmRaiseBasepri();
	{
		if ( pQueue->NumOfResources > 0 )
		{
			prvCopyDataFromQueue(pQueue,pBuffer);
			pQueue->NumOfResources--;
			xReturn = pdTRUE;
		}
		else
		{
			xReturn = pdFALSE;
		}
	}
	AsmSetBasepri();

	return xReturn;
}

uint8_t QueueIsEmpty( QueueHandle_t *Queue )
{
	if ( Queue == NULL ) 
	{
		return pdTRUE;
	}
	return (Queue->QueueManager.NumOfResources == 0) ? pdTRUE : pdFALSE;
}
uint8_t QueueIsFull( QueueHandle_t *Queue )
{
	if ( Queue == NULL )
	{
		return pdTRUE;
	}
	return (Queue->QueueManager.NumOfResources >= Queue->QueueManager.MaxVolume) ? pdTRUE : pdFALSE;
}

/*-----------------------------------------------------------*/

void BinarySemaphoreInitialize( SemaphoreHandle_t *BinarySemaphore, const char *pQueueName, uint32_t QueueID )
{
	if ( BinarySemaphore != NULL )
	{
		BinarySemaphore->SemaphoreManager.pQueueName = pQueueName;
		BinarySemaphore->SemaphoreManager.QueueID = QueueID;
		BinarySemaphore->SemaphoreManager.QueueType = queueTYPE_BINARY_SEMAPHORE;
		BinarySemaphore->SemaphoreManager.pHead = NULL;
		BinarySemaphore->SemaphoreManager.pTail = NULL;
		BinarySemaphore->SemaphoreManager.pWrite = NULL;
		BinarySemaphore->SemaphoreManager.pRead = NULL;
		BinarySemaphore->SemaphoreManager.MaxVolume = 1;
		BinarySemaphore->SemaphoreManager.ItemSize = 0;
		BinarySemaphore->SemaphoreManager.NumOfResources = 0;
	}
}
void CountingSemaphoreInitialize( SemaphoreHandle_t *CountingSemaphore, const char *pQueueName, uint32_t QueueID, uint32_t MaxCount, uint32_t InitialCount )
{
	if ( CountingSemaphore != NULL )
	{
		CountingSemaphore->SemaphoreManager.pQueueName = pQueueName;
		CountingSemaphore->SemaphoreManager.QueueID = QueueID;
		CountingSemaphore->SemaphoreManager.QueueType = queueTYPE_COUNTING_SEMAPHORE;
		CountingSemaphore->SemaphoreManager.pHead = NULL;
		CountingSemaphore->SemaphoreManager.pTail = NULL;
		CountingSemaphore->SemaphoreManager.pWrite = NULL;
		CountingSemaphore->SemaphoreManager.pRead = NULL;
		CountingSemaphore->SemaphoreManager.MaxVolume = MaxCount;
		CountingSemaphore->SemaphoreManager.ItemSize = 0;
		CountingSemaphore->SemaphoreManager.NumOfResources = (InitialCount > MaxCount) ? MaxCount : InitialCount;
	}
}
static uint8_t prvSemaphoreGenericSend( QueueGeneric_t * const pSemaphore )
{
	if ( pSemaphore->NumOfResources >= pSemaphore->MaxVolume )
	{
		return pdFALSE;
	}
	pSemaphore->NumOfResources++;
	return pdTRUE;
}
uint8_t SemaphoreProduce( SemaphoreHandle_t *Semaphore )
{
	uint8_t xReturn = pdFALSE;
	if ( Semaphore == NULL )
	{
		return pdFALSE;
	}
	AsmRaiseBasepri();
	{
		xReturn = prvSemaphoreGenericSend(&(Semaphore->SemaphoreManager));
	}
	AsmSetBasepri();

	return xReturn;
}

static uint8_t prvSemaphoreGenericReceive( QueueGeneric_t * const pSemaphore )
{
	if ( pSemaphore->NumOfResources == 0 )
	{
		return pdFALSE;
	}

	pSemaphore->NumOfResources--;
	return pdTRUE;
}
uint8_t SemaphoreConsume( SemaphoreHandle_t *Semaphore )
{
	uint8_t xReturn = pdFALSE;

	if ( Semaphore == NULL )
	{
		return pdFALSE;
	}
	AsmRaiseBasepri();
	{
		xReturn = prvSemaphoreGenericReceive(&(Semaphore->SemaphoreManager));
	}
	AsmSetBasepri();

	return xReturn;
}
uint32_t SemaphoreGetCount( SemaphoreHandle_t *Semaphore )
{
	if ( Semaphore == NULL )
	{
		return 0;
	}
	return Semaphore->SemaphoreManager.NumOfResources;
}

/*-----------------------------------------------------------*/

static void prvMinHeapInsertNode( TimerMinHeap_t *MinHeap, TimerHandle_t *pTimer ) 
{
	if ( MinHeap->NumOfResources >= TIMER_MAX_NUM || pTimer == NULL )
	{
		return;
	}

	uint8_t LastNode = MinHeap->NumOfResources++;		//堆中最后一个结点
	MinHeap->Node[LastNode] = pTimer;
	while ( LastNode > 0 ) 
	{
		uint8_t Parent = (LastNode - 1) / 2;
		if ( (MinHeap->Node[Parent]->ExpireTime) <= (MinHeap->Node[LastNode]->ExpireTime) )
		{
			//双亲结点的值 < 子结点
			break;
		}
		// 交换
		TimerHandle_t *tmp = MinHeap->Node[Parent];
		MinHeap->Node[Parent] = MinHeap->Node[LastNode];
		MinHeap->Node[LastNode] = tmp;
		LastNode = Parent;
	}
}
static TimerHandle_t* prvMinHeapRemoveRootNode( TimerMinHeap_t *MinHeap ) 
{
	if ( MinHeap == NULL || MinHeap->NumOfResources == 0 )
	{
		return NULL;
	}

	TimerHandle_t *Root = MinHeap->Node[0];
	uint8_t last = MinHeap->NumOfResources - 1;

	if ( last > 0 )
	{
		MinHeap->Node[0] = MinHeap->Node[last];
	}

	MinHeap->Node[last] = NULL;
	MinHeap->NumOfResources--;

	uint8_t Current = 0;
	while ( MinHeap->NumOfResources > 0 )
	{
		uint8_t left = 2 * Current + 1;
		uint8_t right = 2 * Current + 2;
		uint8_t SmallestNode = Current;

		if ( left < MinHeap->NumOfResources && MinHeap->Node[left]->ExpireTime < MinHeap->Node[SmallestNode]->ExpireTime )
		{
			SmallestNode = left;
		}

		if ( right < MinHeap->NumOfResources && MinHeap->Node[right]->ExpireTime < MinHeap->Node[SmallestNode]->ExpireTime )
		{
			SmallestNode = right;
		}

		if ( SmallestNode == Current )
		{
			break;
		}

		TimerHandle_t *tmp = MinHeap->Node[Current];
		MinHeap->Node[Current] = MinHeap->Node[SmallestNode];
		MinHeap->Node[SmallestNode] = tmp;

		Current = SmallestNode;
	}

	return Root;
}
static TimerHandle_t* prvMinHeapRemoveNodeByID( TimerMinHeap_t *MinHeap, uint8_t TimerID )
{
	if ( MinHeap == NULL || MinHeap->NumOfResources == 0 )
	{
		return NULL;
	}

	uint8_t index;
	for ( index = 0; index < MinHeap->NumOfResources; ++index )
	{
		if ( MinHeap->Node[index]->TimerID == TimerID )
		{
			break;
		}
	}

	if ( index == MinHeap->NumOfResources )
	{
		return NULL;
	}

	uint8_t last = MinHeap->NumOfResources - 1;
	TimerHandle_t *RemoveTimer = MinHeap->Node[index];

	if ( index != last )
	{
		MinHeap->Node[index] = MinHeap->Node[last];
	}

	MinHeap->Node[last] = NULL;
	MinHeap->NumOfResources--;

	if ( index >= MinHeap->NumOfResources )
	{
		return RemoveTimer;				// 删除的是最后一个元素，不需要调整
	}

	uint8_t Parent = (index - 1) / 2;

	if ( index > 0 && MinHeap->Node[index]->ExpireTime < MinHeap->Node[Parent]->ExpireTime )
	{
		while ( index > 0 )
		{
			Parent = (index - 1) / 2;
			if ( MinHeap->Node[index]->ExpireTime >= MinHeap->Node[Parent]->ExpireTime )
			{
				break;
			}

			TimerHandle_t *tmp = MinHeap->Node[index];
			MinHeap->Node[index] = MinHeap->Node[Parent];
			MinHeap->Node[Parent] = tmp;

			index = Parent;
		}
	}
	else
	{
		while ( 1 )
		{
			uint8_t left = 2 * index + 1;
			uint8_t right = 2 * index + 2;
			uint8_t smallest = index;

			if ( left < MinHeap->NumOfResources && MinHeap->Node[left]->ExpireTime < MinHeap->Node[smallest]->ExpireTime )
			{
				smallest = left;
			}

			if ( right < MinHeap->NumOfResources && MinHeap->Node[right]->ExpireTime < MinHeap->Node[smallest]->ExpireTime )
			{
				smallest = right;
			}

			if ( smallest == index )
			{
				break;
			}

			TimerHandle_t *tmp = MinHeap->Node[index];
			MinHeap->Node[index] = MinHeap->Node[smallest];
			MinHeap->Node[smallest] = tmp;

			index = smallest;
		}
	}

	return RemoveTimer;
}

/*-----------------------------------------------------------*/

static void RegisterTimer( TimerHandle_t *Timer ) 
{
	if ( Timer == NULL )
	{
		return;
	}
	// 直接以TimerID为索引注册
	TimerRegistry[Timer->TimerID] = Timer;
}
static TimerHandle_t* GetTimerByID( uint8_t TimerID )
{
	if ( TimerID < TIMER_MAX_REGISTRY ) 
	{
		return TimerRegistry[TimerID];
	}
	return NULL;
}
void TimerInitialise( TimerHandle_t *pNewTimer, 
							const char *pTimerName, 
							uint8_t TimerID, 
							uint32_t TimerPeriodInTicks, 
							uint8_t isAutoReload, 
							TimerCallBackFunction_t pCallBackFunction )
{
	if ( pNewTimer != NULL && pCallBackFunction != NULL && TimerPeriodInTicks != 0 )
	{
		pNewTimer->pTimerName = pTimerName;
		pNewTimer->TimerID = TimerID;
		pNewTimer->TimerPeriodInTicks = TimerPeriodInTicks;
		pNewTimer->isAutoReload = isAutoReload;
		pNewTimer->pCallbackFunction = pCallBackFunction;
		RegisterTimer(pNewTimer);
		if ( TimersManagerRxQueue.QueueManager.pHead == NULL )
		{
			QueueInitialize(&TimersManagerRxQueue, timerRXQUEUE_NAME, timerRXQUEUE_ID, 6);
		}
	}
}
uint8_t TimerGenericCommand( TimerHandle_t *Timer, const uint8_t CommandID, uint32_t PeriodInTicks )
{
	if ( Timer == NULL )
	{
		return pdFALSE;
	}
	//定时器命令固定格式: 1Byte定时器ID + 1Byte命令码 + 4Byte数据段
	uint8_t CtrlData[6];
	CtrlData[0] = Timer->TimerID;
	CtrlData[1] = CommandID;
	CtrlData[2] = (PeriodInTicks >> 0) & 0xFF;
	CtrlData[3] = (PeriodInTicks >> 8) & 0xFF;
	CtrlData[4] = (PeriodInTicks >> 16) & 0xFF;
	CtrlData[5] = (PeriodInTicks >> 24) & 0xFF;

	return QueueSendToBack(&TimersManagerRxQueue,CtrlData);
}
static void prvProcessReceivedCommands( void )
{
	uint8_t RxData[6];
	TimerHandle_t *pTimer;

	while ( QueueReceive(&TimersManagerRxQueue,RxData) != pdFALSE )
	{
		pTimer = GetTimerByID(RxData[0]);
		if(pTimer == NULL) 
		{
			continue;
		}
		uint32_t PeriodInTicks = (RxData[5] << 24) | (RxData[4] << 16) | (RxData[3] << 8) | RxData[2];

		AsmRaiseBasepri();
		{
			prvMinHeapRemoveNodeByID(&TimerActiveHeap, pTimer->TimerID);
		}
		AsmSetBasepri();

		switch (RxData[1])
		{
			case timerCOMMAND_START:
			case timerCOMMAND_RESET:
				pTimer->ExpireTime = SystemRunTime_1ms + pTimer->TimerPeriodInTicks;
				prvMinHeapInsertNode(&TimerActiveHeap, pTimer);
				break;

			case timerCOMMAND_STOP:
				/* 计时器已从活动列表中移除,这里无需做任何处理 */
				break;

			case timerCOMMAND_CHANGE_PERIOD:
				pTimer->TimerPeriodInTicks = PeriodInTicks;
				pTimer->ExpireTime = SystemRunTime_1ms + PeriodInTicks;
				prvMinHeapInsertNode(&TimerActiveHeap, pTimer);
				break;

			case timerCOMMAND_DELETE:
				break;

			case timerCOMMAND_EXECUTECALLBACK:
				pTimer->pCallbackFunction(pTimer);
				break;

			default:
				break;
		}
	}
}
void TimersManagerTask( void )
{
	//处理队列消息
	prvProcessReceivedCommands();
	
	//扫描堆顶定时器
	uint32_t NowTime = SystemRunTime_1ms;
	AsmRaiseBasepri();
	{
		while (TimerActiveHeap.NumOfResources > 0) 
		{
			TimerHandle_t *Timer = TimerActiveHeap.Node[0];
			if ( Timer->ExpireTime > NowTime )
			{
				break;
			}
			prvMinHeapRemoveRootNode(&TimerActiveHeap);
			AsmSetBasepri();
			{
				Timer->pCallbackFunction(Timer);
			}
			
			AsmRaiseBasepri();
			
			if ( Timer->isAutoReload == pdTRUE ) 
			{
				Timer->ExpireTime = NowTime + Timer->TimerPeriodInTicks;
				prvMinHeapInsertNode(&TimerActiveHeap,Timer);
			}
		}
	}
	AsmSetBasepri();
}
void MyTestTimerCallback( void *Timer )
{
	TimerHandle_t *pTimer = (TimerHandle_t *)Timer;
	uint32_t TimerID = pTimer->TimerID;

	if( TimerID == 1 ) 
	{
		//定时器1到期
	} 
	if( TimerID == 2 ) 
	{
		//定时器2到期
	}
	if( TimerID == 3 ) 
	{
		//定时器3到期
	}
}

