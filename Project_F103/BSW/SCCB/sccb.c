#include "sccb.h"
#include "delay.h"

void SCCB_Init(void)
{
	SCCB_SCL(1);
	SCCB_SDA(1);
}
void SCCB_Delay(void)
{
	Delay_us(10);
}
void SCCB_Start(void)
{
	SCCB_SDA(1);
	SCCB_SCL(1);
	SCCB_Delay();
	SCCB_SDA(0);
	SCCB_Delay();
	SCCB_SCL(0);
}
void SCCB_Stop(void)
{
	SCCB_SDA(0);
	SCCB_SCL(1);
	SCCB_Delay();
	SCCB_SDA(1);
	SCCB_Delay();
}
void SCCB_SendByte(uint8_t Byte)
{
	for (int8_t DataIndex = 7; DataIndex >= 0; DataIndex--)
	{
		SCCB_SCL(0);
		SCCB_SDA((Byte >> DataIndex) & 0x01);
		SCCB_Delay();
		SCCB_SCL(1);
		SCCB_Delay();
		SCCB_SCL(0);
	}
	//Don't care
	SCCB_SDA(1);
	SCCB_SCL(0);
	SCCB_Delay();
	SCCB_SCL(1);
	SCCB_Delay();
	SCCB_SCL(0);
	SCCB_Delay();
}
uint8_t SCCB_ReceiveByte(void)
{
	uint8_t Data = 0;
	SCCB_SDA(1);
	for (int8_t DataIndex = 7; DataIndex >= 0; DataIndex--)
	{
		SCCB_SCL(1);
		Data |= SCCB_READ_SDA() << DataIndex;
		SCCB_Delay();
		SCCB_SCL(0);
		SCCB_Delay();
	}
	//NACK
	SCCB_SCL(0);
	SCCB_SDA(1);
	SCCB_Delay();
	SCCB_SCL(1);
	SCCB_Delay();
	SCCB_SCL(0);
	
	return Data;
}
void SCCB_3PhaseWrite(uint8_t ID_Addr, uint8_t Sub_Addr, uint8_t Data)
{
	SCCB_Start();
	SCCB_SendByte((ID_Addr << 1) | SCCB_WRITE);
	SCCB_SendByte(Sub_Addr);
	SCCB_SendByte(Data);
	SCCB_Stop();
}
void SCCB_2PhaseWrite(uint8_t ID_Addr, uint8_t Sub_Addr)
{
	SCCB_Start();
	SCCB_SendByte((ID_Addr << 1) | SCCB_WRITE);
	SCCB_SendByte(Sub_Addr);
	SCCB_Stop();
}
uint8_t SCCB_2PhaseRead(uint8_t ID_Addr)
{
	uint8_t Data;
	SCCB_Start();
	SCCB_SendByte((ID_Addr << 1) | SCCB_READ);
	Data = SCCB_ReceiveByte();
	SCCB_Stop();
	return Data;
}
