#ifndef _IAP_H
#define _IAP_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "AT24Cxx.h"
#include "NM25Qxx.h"
#include "check.h"
#include <stdbool.h>
#include <string.h>

/* macro --------------------------------------------------------------------*/
#define APP_START_ADDRESS               0x08008000u
#define APP_END_ADDRESS                 0x0807FFFFu

#define APP_UPDATE_FLAG                 0x5AA55AA5u
/* enum ---------------------------------------------------------------------*/
typedef enum
{
    IAPRunSt_Idle,
    IAPRunSt_Change,
    IAPRunSt_JumpToBoot,
} IAPRunSt_t;
    
/* types --------------------------------------------------------------------*/
typedef struct
{
    IAPRunSt_t IAPRunSt;
    uint8_t UpdateFlag[4];
} IAP_Info_t;

/* functions prototypes -----------------------------------------------------*/
void Task2_IAP(void *pvParameters);
void IAP_RxProcess(uint8_t *pData, uint16_t Size);

#endif
