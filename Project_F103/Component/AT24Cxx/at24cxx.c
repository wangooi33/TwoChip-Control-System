#include "AT24Cxx.h"
#include "delay.h"

void AT24Cxx_Init(void)
{
    IIC_Init();
}
uint8_t AT24Cxx_ReadByte(uint16_t Addr)
{
    uint8_t xReturn = 0;
    IIC_Start();
  
    if (EE_TYPE > AT24C16)
    {
        //写操作: 0xA0 | 0
        IIC_SendByte(0XA0);
        IIC_WaitAck();
        IIC_SendByte(Addr >> 8);
    }
    else 
    {
        /* 从一个11位的地址中,提取出高3位,并放入设备地址字节的bit3、bit2、bit1位置 */
        IIC_SendByte(0XA0 + ((Addr >> 8) << 1));
    }

    IIC_WaitAck();
    IIC_SendByte(Addr % 256);
    IIC_WaitAck();

    //读操作: 0xA0 | 1
    IIC_Start();
    IIC_SendByte(0XA1);
    IIC_WaitAck();
    xReturn = IIC_ReadByte(0);
    IIC_Stop();

    return xReturn;
}

void AT24Cxx_WriteByte(uint16_t Addr, uint8_t Data)
{
    IIC_Start();

    if (EE_TYPE > AT24C16)
    {
        IIC_SendByte(0XA0);
        IIC_WaitAck();
        IIC_SendByte(Addr >> 8);
    }
    else 
    {
        IIC_SendByte(0XA0 + ((Addr >> 8) << 1));
    }
    
    IIC_WaitAck();
    IIC_SendByte(Addr % 256);
    IIC_WaitAck();
    
    IIC_SendByte(Data);
    IIC_WaitAck();
    IIC_Stop();
    Delay_ms(10);
}

void AT24Cxx_Read(uint16_t Addr, uint8_t *pBuf, uint16_t Datalen)
{
    while (Datalen--)
    {
        *pBuf++ = AT24Cxx_ReadByte(Addr++);
    }
}
void AT24Cxx_Write(uint16_t Addr, uint8_t *pBuf, uint16_t Datalen)
{
    while (Datalen--)
    {
        AT24Cxx_WriteByte(Addr, *pBuf);
        Addr++;
        pBuf++;
    }
}
/**
 * @brief    :检查AT24CXX是否正常
 * @note     :检测原理: 在器件的末地址写如0X55, 然后再读取, 如果读取值为0X55
 *            则表示检测正常. 否则,则表示检测失败.
 * @retval   :检测结果
 *              1: 检测成功
 *              0: 检测失败
 */
uint8_t AT24Cxx_Check(void)
{
    uint8_t xReturn;
    uint16_t Addr = EE_TYPE;
    xReturn = AT24Cxx_ReadByte(Addr);

    if (xReturn == 0X55)
    {
        return 1;
    }
    else
    {
        AT24Cxx_WriteByte(Addr,0X55);
        xReturn = AT24Cxx_ReadByte(255);
        if (xReturn == 0X55)
        {
            return 1;
        }
    }

    return 0;
}
