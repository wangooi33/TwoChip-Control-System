#ifndef __W_CAN_H
#define __W_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "can.h"

/* macro --------------------------------------------------------------------*/

/* global variable ----------------------------------------------------------*/
extern uint8_t CanRxBuf[8];

/* functions prototypes -----------------------------------------------------*/
void WCAN_Init(void);
void WCAN_Send(uint16_t id, uint8_t *data, uint8_t len);


#ifdef __cplusplus
}
#endif

#endif /* __W_CAN_H */

