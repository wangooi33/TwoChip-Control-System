#ifndef __NM25Qxx_H
#define __NM25Qxx_H

/* includes -----------------------------------------------------------------*/
#include "spi.h"

/* macro --------------------------------------------------------------------*/

#define NORFLASH_CS(x)      		do{ x ? \
		                                  HAL_GPIO_WritePin(NORFLASH_CS_GPIO_Port, NORFLASH_CS_Pin, GPIO_PIN_SET) : \
		                                  HAL_GPIO_WritePin(NORFLASH_CS_GPIO_Port, NORFLASH_CS_Pin, GPIO_PIN_RESET); \
		                            }while(0)

#define W25Q80      0XEF13          /* W25Q80   芯片ID */
#define W25Q16      0XEF14          /* W25Q16   芯片ID */
#define W25Q32      0XEF15          /* W25Q32   芯片ID */
#define W25Q64      0XEF16          /* W25Q64   芯片ID */
#define W25Q128     0XEF17          /* W25Q128  芯片ID */
#define W25Q256     0XEF18          /* W25Q256  芯片ID */
#define BY25Q64     0X6816          /* BY25Q64  芯片ID */
#define BY25Q128    0X6817          /* BY25Q128 芯片ID */
#define NM25Q64     0X5216          /* NM25Q64  芯片ID */
#define NM25Q128    0X5217          /* NM25Q128 芯片ID */


#define FLASH_WriteEnable           0x06 
#define FLASH_WriteDisable          0x04 
#define FLASH_ReadStatusReg1        0x05 
#define FLASH_ReadStatusReg2        0x35 
#define FLASH_ReadStatusReg3        0x15 
#define FLASH_WriteStatusReg1       0x01 
#define FLASH_WriteStatusReg2       0x31 
#define FLASH_WriteStatusReg3       0x11 
#define FLASH_ReadData              0x03 
#define FLASH_FastReadData          0x0B 
#define FLASH_FastReadDual          0x3B 
#define FLASH_FastReadQuad          0xEB  
#define FLASH_PageProgram           0x02 
#define FLASH_PageProgramQuad       0x32 
#define FLASH_BlockErase            0xD8 
#define FLASH_SectorErase           0x20 
#define FLASH_ChipErase             0xC7 
#define FLASH_PowerDown             0xB9 
#define FLASH_ReleasePowerDown      0xAB 
#define FLASH_DeviceID              0xAB 
#define FLASH_ManufactDeviceID      0x90 
#define FLASH_JedecDeviceID         0x9F 
#define FLASH_Enable4ByteAddr       0xB7
#define FLASH_Exit4ByteAddr         0xE9
#define FLASH_SetReadParam          0xC0 
#define FLASH_EnterQPIMode          0x38
#define FLASH_ExitQPIMode           0xFF

/* global variable -----------------------------------------------------------*/
extern uint16_t FlashType;


/* functions prototypes -----------------------------------------------------*/
void Flash_Init(void);
void Flash_Read(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen);
void Flash_EraseChip(void);
void Flash_EraseSector(uint32_t SectorNum);
void Flash_Write(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen);



#endif /* __NM25Qxx_H */


