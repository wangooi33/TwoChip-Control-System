#include "LCD.h"

uint16_t LCD_ReadPointColor(uint16_t X, uint16_t Y)
{
	uint16_t Data[3] = {0};
	uint16_t R,G,B;
	
	LCD_SetCursor(X,Y);
	LCD_WriteREGNo(LCD_Command_Memoryread);
	Data[0] = LCD_ReadData();		/* dummy read,假读 */
	Data[1] = LCD_ReadData();
	Data[2] = LCD_ReadData();
	/* RGB565 */
	R = (Data[1] >> 11) & 0x1F;       /* Data[1] 高 5 位 */
	G = (Data[1] >> 5)  & 0x3F;       /* Data[1] 低 8 位中的高 6 位 */
	B = (Data[2] >> 11) & 0x1F;       /* Data[2] 高 5 位 */
	
	return (R << 11) | (G << 5) | B;
}
void LCD_DrawPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
	LCD_SetCursor(X,Y);
	LCD_WriteREGNo(LCD_Command_MemoryWrite);
	LCD->LCD_RAM = Color;
}
void LCD_DrawLine(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{
	int16_t DeltaX,DeltaY;
	int8_t iteratorX,iteratorY;
	uint16_t Cnt;
	uint16_t X,Y;
	DeltaX = Width - StartX;
	DeltaY = Height - StartY;
	if (DeltaX > DeltaY)
	{
		if (DeltaY < 0)
		{
			iteratorY = -1;
			iteratorX = -(DeltaX / DeltaY);
			Cnt = -DeltaY;
		}
		else if (DeltaY > 0)
		{
			iteratorY = 1;
			iteratorX = (DeltaX / DeltaY);
			Cnt = DeltaY;
		}
	}
	else
	{
		if (DeltaX < 0)
		{
			iteratorX = -1;
			iteratorY = -(DeltaY / DeltaX);
			Cnt = -DeltaX;
		}
		else if (DeltaX > 0)
		{
			iteratorX = 1;
			iteratorY = (DeltaY / DeltaX);
			Cnt = DeltaX;
		}
	}

	for (uint16_t i = 0; i < Cnt; i++)
	{
		LCD_DrawPoint(X,Y,Color);
		X += iteratorX;
		Y += iteratorY;
	}
}
void LCD_DrawRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{
	LCD_DrawLine(StartX,StartY,0,Height,Color);
	LCD_DrawLine(StartX,StartY,Width,0,Color);
	LCD_DrawLine(StartX + Width,StartY,0,Height,Color);
	LCD_DrawLine(StartX,StartY + Height,Width,0,Color);
}
void LCD_DrawRound(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{

}
void LCD_FillRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{
	uint16_t SXtoEx = Width - StartX -1;

	for (uint16_t i = StartY; i < Height; i++)
	{
		LCD_SetCursor(StartX,i);
		LCD_WriteREGNo(LCD_Command_MemoryWrite);
		for (uint16_t j = 0; j < SXtoEx; j++)
		{
			LCD->LCD_RAM = Color;
		}
	}
}
void LCD_FillRound(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{

}
void LCD_ShowChar(uint16_t StartX, uint16_t StartY, char chr, uint8_t Size, uint8_t isCover, uint16_t Color)
{


}

