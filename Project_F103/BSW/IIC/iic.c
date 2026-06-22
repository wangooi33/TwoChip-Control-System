#include "iic.h"

void IIC_Init(void)
{
	IIC_SDA(1);
	IIC_SCL(1);
}
static void IIC_Delay(void)
{
	volatile uint16_t i;
	for (i = 0; i < 30; i++);
}
void IIC_Start(void)
{
	IIC_SDA(1);
	IIC_SCL(1);
	IIC_Delay();
	IIC_SDA(0);
	IIC_Delay();
	IIC_SCL(0);
	IIC_Delay();
}
void IIC_Stop(void)
{
	IIC_SDA(0);
	IIC_SCL(1);
	IIC_Delay();
	IIC_SDA(1);
	IIC_Delay();
}
uint8_t IIC_WaitAck(void)
{
	uint8_t waittime = 0;
	
	IIC_SDA(1);
	IIC_SCL(0);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	while (IIC_READ_SDA())
	{
		if (waittime++ > 250)
		{
			IIC_Stop();
			return 0;
		}
	}
	IIC_SCL(0);
	IIC_Delay();
	return 1;
}
void IIC_Ack(void)
{
	IIC_SCL(0);
	IIC_SDA(0);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	IIC_SCL(0);

}
void IIC_NAck(void)
{
	IIC_SCL(0);
	IIC_SDA(1);
	IIC_Delay();
	IIC_SCL(1);
	IIC_Delay();
	IIC_SCL(0);
}
void IIC_SendByte(uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		IIC_SCL(0);
		IIC_SDA((data & 0x80) >> 7);
		IIC_Delay();
		IIC_SCL(1);
		IIC_Delay();
		IIC_SCL(0);
		data <<= 1;
	}
	IIC_SDA(1);
	IIC_Delay();
}
uint8_t IIC_ReadByte(void)
{
	uint8_t Data = 0;
	for (int8_t i = 7; i >= 0; i--)
	{
		IIC_SCL(1);
		IIC_Delay();
		Data |= IIC_READ_SDA() << i;
		IIC_SCL(0);
		IIC_Delay();
	}
	return Data;
}


