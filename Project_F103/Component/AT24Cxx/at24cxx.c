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
	}
	else 
	{
		/* 地址超8位 */
		IIC_SendByte(0XA0 + ((Addr >> 8) << 1));
	}

	IIC_WaitAck();
	IIC_SendByte((uint8_t)Addr);
	IIC_WaitAck();

	//读操作: 0xA0 | 1
	IIC_Start();
	IIC_SendByte(0XA1);
	IIC_WaitAck();
	xReturn = IIC_ReadByte();
	IIC_NAck();
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

