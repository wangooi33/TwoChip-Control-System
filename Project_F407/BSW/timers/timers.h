#ifndef _TIMERS_H
#define _TIMERS_H
/* include -------------------------------------------------------------------*/
#include <string.h>
#include <stdint.h>
#include "usart.h"
/* macro ---------------------------------------------------------------------*/
#define pdFALSE							(0)
#define pdTRUE							(1)
#define QUEUE_MAX_DATALENGTH			(128)
#define TIMER_MAX_NUM					(16)
#define TIMER_MAX_REGISTRY				(256)

//入队到尾部(FIFO:先进先出)
#define queueSEND_TOBACK				(0xAA)
//入队到头部,类似“高优先级消息插队”
#define queueSEND_TOFRONT				(0xAB)
//只允许用于长度为 1 的队列,队列满时:覆盖已有元素
#define queueSEND_OVERWRITE				(0xAC)

#define queueTYPE_QUEUE					(0xA0)
#define queueTYPE_BINARY_SEMAPHORE		(0xA1)
#define queueTYPE_COUNTING_SEMAPHORE	(0xA2)

#define timerRXQUEUE_ID					(0x80)
#define timerRXQUEUE_NAME				("TimersRxQueue")
#define timerCOMMAND_START				(0x01)
#define timerCOMMAND_RESET				(0x02)
#define timerCOMMAND_STOP				(0x03)
#define timerCOMMAND_CHANGE_PERIOD		(0x04)
#define timerCOMMAND_DELETE				(0x05)
#define timerCOMMAND_EXECUTECALLBACK	(0x0A)

/* types ---------------------------------------------------------------------*/
typedef void (*TimerCallBackFunction_t)( void * Timer );

typedef struct Queue
{
	const char *pQueueName;
	uint8_t QueueID;
	uint8_t QueueType;
	uint8_t *pHead;						//首元素地址
	uint8_t *pTail;						//尾元素地址
	uint8_t *pWrite;
	uint8_t *pRead;
	uint32_t MaxVolume;
	uint8_t ItemSize;					//单个资源所占字节数
	volatile uint32_t NumOfResources;	//资源数量,即队列中元素的数量
}QueueGeneric_t;

typedef struct 
{
	QueueGeneric_t QueueManager;
	uint8_t QueueStorageArea[QUEUE_MAX_DATALENGTH];     //字节流
}QueueHandle_t;

typedef struct
{
	QueueGeneric_t SemaphoreManager;
}SemaphoreHandle_t;

typedef struct
{
	const char *pTimerName;
	uint8_t TimerID;
	uint32_t TimerPeriodInTicks;
	uint32_t ExpireTime;						//到期时间
	uint8_t isAutoReload;
	TimerCallBackFunction_t pCallbackFunction;	//回调函数执行:表示定时器到期
}TimerHandle_t;


typedef struct 
{
	//小根堆
	uint8_t NumOfResources;
	TimerHandle_t *Node[TIMER_MAX_NUM];
}TimerMinHeap_t;

/* constants -----------------------------------------------------------------*/


/* global variable -----------------------------------------------------------*/


/* functions prototypes ------------------------------------------------------*/
void QueueInitialize( QueueHandle_t *Queue, const char *pQueueName, uint8_t QueueID, uint8_t ItemSize );
void BinarySemaphoreInitialize( SemaphoreHandle_t *BinarySemaphore, const char *pQueueName, uint32_t QueueID );
void CountingSemaphoreInitialize( SemaphoreHandle_t *CountingSemaphore, const char *pQueueName, uint32_t QueueID, uint32_t MaxCount, uint32_t InitialCount );
uint8_t QueueSendToBack( QueueHandle_t *Queue, const void * const pItemToQueue );
uint8_t QueueSendToFront( QueueHandle_t *Queue, const void * const pItemToQueue );
uint8_t QueueOverWrite( QueueHandle_t *Queue, const void * const pItemToQueue );
uint8_t QueueReceive( QueueHandle_t *Queue, void * const pBuffer );
uint8_t SemaphoreProduce( SemaphoreHandle_t *Semaphore );
uint8_t SemaphoreConsume( SemaphoreHandle_t *Semaphore );
void TimerInitialise( TimerHandle_t *pNewTimer, const char *pTimerName,  uint8_t TimerID, uint32_t TimerPeriodInTicks, uint8_t isAutoReload, TimerCallBackFunction_t pCallBackFunction );
uint8_t TimerGenericCommand( TimerHandle_t *Timer, const uint8_t CommandID, uint32_t PeriodInTicks );
void TimersManagerTask( void );

#endif /* _TIMERS_H */

