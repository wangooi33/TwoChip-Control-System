#include "NM25Qxx.h"
#include <string.h>

uint16_t FlashType = NM25Q128; 
uint8_t gFlashBuf[4096];

static uint8_t prvSwapByte(uint8_t TxData);
static void prvFlashWaitBusy(void);
static void prvFlashWriteEnable(void);
static void prvFlashSendAddress(uint32_t Address);
static uint8_t prvFlashReadStateReg(uint8_t RegNum);
static void prvFlashWriteStateReg(uint8_t RegNum, uint8_t Val);
static uint16_t prvFlashReadID(void);
static void prvFlashWritePage(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen);
static void prvFlashWriteNoCheck(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen);


void Flash_Init(void)
{
    uint8_t Addrlen;

    NORFLASH_CS(1);

    FlashType = prvFlashReadID();
    
    if (FlashType == W25Q256)				/* SPI FLASH为W25Q256, 必须使能4字节地址模式 */
    {
        Addrlen = prvFlashReadStateReg(3);	/* 读取状态寄存器3，判断地址模式 */
        if ((Addrlen & 0X01) == 0)			/* 如果不是4字节地址模式,则进入4字节地址模式 */
        {
            prvFlashWriteEnable();
            Addrlen |= 1 << 1;				/* ADP=1, 上电4位地址模式 */
            prvFlashWriteStateReg(3, Addrlen);
            
            NORFLASH_CS(0);
            prvSwapByte(FLASH_Enable4ByteAddr);
            NORFLASH_CS(1);
        }
    }
}
void Flash_Read(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen)
{
    uint16_t iterator;

    NORFLASH_CS(0);
    prvSwapByte(FLASH_ReadData);
    prvFlashSendAddress(Addr);
    
    for(iterator = 0; iterator < Datalen; iterator++)
    {
        pBuf[iterator] = prvSwapByte(0XFF);
    }
    NORFLASH_CS(1);
}
void Flash_EraseChip(void)
{
    prvFlashWriteEnable();
    prvFlashWaitBusy();
    NORFLASH_CS(0);
    prvSwapByte(FLASH_ChipErase);
    NORFLASH_CS(1);
    prvFlashWaitBusy();
}
void Flash_EraseSector(uint32_t SectorNum)
{
    SectorNum *= 4096;
    prvFlashWriteEnable();
    prvFlashWaitBusy();

    NORFLASH_CS(0);
    prvSwapByte(FLASH_SectorErase);
    prvFlashSendAddress(SectorNum);
    NORFLASH_CS(1);
    prvFlashWaitBusy();
}
void Flash_Write(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen)
{
    uint32_t SectorPos;
    uint16_t SectorOff;
    uint16_t SectorRemain;
    uint16_t iterator;
    uint8_t *pFlashBuf;

    pFlashBuf = gFlashBuf;
    SectorPos = Addr / 4096;			/* 扇区地址 */
    SectorOff = Addr % 4096;			/* 在扇区内的偏移 */
    SectorRemain = 4096 - SectorOff;	/* 扇区剩余空间大小 */

    if (Datalen <= SectorRemain)
    {
        SectorRemain = Datalen;
    }

    while (1)
    {
        Flash_Read(pFlashBuf, SectorPos * 4096, 4096);
        for (iterator = 0; iterator < SectorRemain; iterator++)
        {
            if (pFlashBuf[SectorOff + iterator] != 0XFF)
            {
                break;
            }
        }

        if (iterator < SectorRemain)
        {
            Flash_EraseSector(SectorPos);

            for (iterator = 0; iterator < SectorRemain; iterator++)
            {
                pFlashBuf[iterator + SectorOff] = pBuf[iterator];
            }

            prvFlashWriteNoCheck(pFlashBuf, SectorPos * 4096, 4096);
        }
        else
        {
            prvFlashWriteNoCheck(pBuf,Addr,SectorRemain);
        }

        if (Datalen == SectorRemain)
        {
            break;
        }
        else
        {
            SectorPos++;
            SectorOff = 0;

            pBuf += SectorRemain;
            Addr += SectorRemain;
            Datalen -= SectorRemain;

            if (Datalen > 4096)
            {
                SectorRemain = 4096;
            }
            else
            {
                SectorRemain = Datalen;
            }
        }
    }
}


static uint8_t prvSwapByte(uint8_t TxData)
{
	uint8_t RxData;
	HAL_SPI_TransmitReceive(&hspi2, &TxData,&RxData,1,1000);
	return RxData;
}
static void prvFlashWaitBusy(void)
{
	while ((prvFlashReadStateReg(1) & 0x01) == 0x01);
}
static void prvFlashWriteEnable(void)
{
	NORFLASH_CS(0);
	prvSwapByte(FLASH_WriteEnable);
	NORFLASH_CS(1);
}
static void prvFlashSendAddress(uint32_t Address)
{
    if (FlashType == W25Q256)
    {
        prvSwapByte((uint8_t)((Address)>>24));
    } 
    prvSwapByte((uint8_t)((Address)>>16));
    prvSwapByte((uint8_t)((Address)>>8));
    prvSwapByte((uint8_t)Address);
}
/**
 * @brief       读取25QXX的状态寄存器，25QXX一共有3个状态寄存器
 *   @note      状态寄存器1：
 *              BIT7  6   5   4   3   2   1   0
 *              SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *              SPR:默认0,状态寄存器保护位,配合WP使用
 *              TB,BP2,BP1,BP0:FLASH区域写保护设置
 *              WEL:写使能锁定
 *              BUSY:忙标记位(1,忙;0,空闲)
 *              默认:0x00
 *
 *              状态寄存器2：
 *              BIT7  6   5   4   3   2   1   0
 *              SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
 *
 *              状态寄存器3：
 *              BIT7      6    5    4   3   2   1   0
 *              HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
 *
 * @param       regno: 状态寄存器号，范围:1~3
 * @retval      状态寄存器值
 */
static uint8_t prvFlashReadStateReg(uint8_t RegNum)
{
    uint8_t RegVal = 0, Command = 0;

    switch (RegNum)
    {
        case 1:
            Command = FLASH_ReadStatusReg1;
            break;
        case 2:
            Command = FLASH_ReadStatusReg2;
            break;
        case 3:
            Command = FLASH_ReadStatusReg3;
            break;
        default:
            Command = FLASH_ReadStatusReg1;
            break;
    }

    NORFLASH_CS(0);
    prvSwapByte(Command);
    RegVal = prvSwapByte(0Xff);  /* 读取一个字节 */
    NORFLASH_CS(1);
    
    return RegVal;
}
static void prvFlashWriteStateReg(uint8_t RegNum, uint8_t Val)
{
    uint8_t Command = 0;
    switch (RegNum)
    {
        case 1:
            Command = FLASH_WriteStatusReg1;
            break;
        case 2:
            Command = FLASH_WriteStatusReg2;
            break;
        case 3:
            Command = FLASH_WriteStatusReg3;
            break;
        default:
            Command = FLASH_WriteStatusReg1;
            break;
    }

    NORFLASH_CS(0);
    prvSwapByte(Command);
    prvSwapByte(Val);
    NORFLASH_CS(1);
}
static uint16_t prvFlashReadID(void)
{
    uint16_t DID;

    NORFLASH_CS(0);
    prvSwapByte(FLASH_ManufactDeviceID);
    prvSwapByte(0);
    prvSwapByte(0);
    prvSwapByte(0);
    DID = prvSwapByte(0xFF) << 8;
    DID |= prvSwapByte(0xFF);
    NORFLASH_CS(1);

    return DID;
}
static void prvFlashWritePage(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen)
{
    uint16_t iterator;

	prvFlashWriteEnable();

    NORFLASH_CS(0);
    prvSwapByte(FLASH_PageProgram);
    prvFlashSendAddress(Addr);

    for(iterator = 0; iterator < Datalen; iterator++)
    {
        prvSwapByte(pBuf[iterator]);
    }
    
    NORFLASH_CS(1);
	prvFlashWaitBusy();
}
static void prvFlashWriteNoCheck(uint8_t *pBuf, uint32_t Addr, uint16_t Datalen)
{
    uint16_t PageRemain;
    PageRemain = 256 - Addr % 256;  /* 单页剩余的字节数 */

    if (Datalen <= PageRemain)
    {
        PageRemain = Datalen;
    }

    while (1)
    {
        prvFlashWritePage(pBuf,Addr,PageRemain);

        if (Datalen == PageRemain)
        {
            break;
        }
        else
        {
            pBuf += PageRemain;
            Addr += PageRemain;
            Datalen -= PageRemain;

            if (Datalen> 256)
            {
                PageRemain = 256;
            }
            else
            {
                PageRemain = Datalen;
            }
        }
    }
}


