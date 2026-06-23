#ifndef __SCCB_H
#define __SCCB_H
/* includes -----------------------------------------------------------------*/
#include "main.h"

/* macro --------------------------------------------------------------------*/
#define SCCB_WRITE				0x00
#define SCCB_READ				0x01

#define SCCB_SCL(x)				do{ (x) ? \
									HAL_GPIO_WritePin(SCCB_SCL_GPIO_Port, SCCB_SCL_Pin, GPIO_PIN_SET) :  \
									HAL_GPIO_WritePin(SCCB_SCL_GPIO_Port, SCCB_SCL_Pin, GPIO_PIN_RESET); \
								}while(0)

#define SCCB_SDA(x)				do{ (x) ? \
									HAL_GPIO_WritePin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin, GPIO_PIN_SET) :  \
									HAL_GPIO_WritePin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin, GPIO_PIN_RESET); \
								}while(0)

#define SCCB_READ_SDA()			HAL_GPIO_ReadPin(SCCB_SDA_GPIO_Port, SCCB_SDA_Pin) 

/* enum ---------------------------------------------------------------------*/

    
/* types --------------------------------------------------------------------*/

/* functions prototypes -----------------------------------------------------*/
void SCCB_Init(void);
void SCCB_Delay(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_NAck(void);
void SCCB_NoCare(void);
void SCCB_SendByte(uint8_t Byte);
uint8_t SCCB_ReceiveByte(void);

void SCCB_3PhaseWrite(uint8_t ID_Addr, uint8_t Sub_Addr, uint8_t Data);
void SCCB_2PhaseWrite(uint8_t ID_Addr, uint8_t Sub_Addr);
uint8_t SCCB_2PhaseRead(uint8_t ID_Addr);

#endif /* __SCCB_H */
