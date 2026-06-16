#ifndef __AT24CXX_H
#define __AT24CXX_H

/* includes -----------------------------------------------------------------*/
#include "iic.h"

/* macro --------------------------------------------------------------------*/

#define AT24C01     127
#define AT24C02     255
#define AT24C04     511
#define AT24C08     1023
#define AT24C16     2047
#define AT24C32     4095
#define AT24C64     8191
#define AT24C128    16383
#define AT24C256    32767

#define EE_TYPE     AT24C02


/* functions prototypes -----------------------------------------------------*/
void AT24Cxx_Init(void);
void AT24Cxx_Read(uint16_t Addr, uint8_t *pBuf, uint16_t Datalen);
void AT24Cxx_Write(uint16_t Addr, uint8_t *pBuf, uint16_t Datalen);
uint8_t AT24Cxx_Check(void);


#endif /* __AT24CXX_H */

