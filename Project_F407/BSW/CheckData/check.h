#ifndef _CHECK_H_
#define _CHECK_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Exported global  variable*/
uint8_t CheckBcc(uint8_t *pdata,uint16_t len);
uint16_t CheckCRC16(uint8_t *ptr, uint32_t len);
uint8_t CheckSum(uint8_t *pdata,uint16_t len);
uint16_t CheckSum16(uint8_t *pdata,uint16_t len);
uint16_t CheckCRC16_ModBus(uint8_t *ptr, uint32_t len);

#endif /* _CHECK_H_ */
