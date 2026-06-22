#ifndef __IIC_H
#define __IIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* macro --------------------------------------------------------------------*/

#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET) :  \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); \
                          }while(0)

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET) :  \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET); \
                          }while(0)

#define IIC_READ_SDA()     HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin)


/* functions prototypes -----------------------------------------------------*/
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_WaitAck(void); 
void IIC_SendByte(uint8_t data);
uint8_t IIC_ReadByte(void);


#ifdef __cplusplus
}
#endif

#endif /* __IIC_H */

