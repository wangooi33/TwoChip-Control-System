#include "lcd.h"
#include "lcd_font.h"

/* macro --------------------------------------------------------------------*/
#define CHAR_DEFAULT_FILLCOLOR		65535

/* global variable ----------------------------------------------------------*/

/* function implementation --------------------------------------------------*/
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
	LCD_WriteRAM(Color);
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
void LCD_FillRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color)
{
	uint16_t SXtoEx = Width - StartX -1;

	for (uint16_t i = StartY; i < Height; i++)
	{
		LCD_SetCursor(StartX,i);
		LCD_WriteREGNo(LCD_Command_MemoryWrite);
		for (uint16_t j = 0; j < SXtoEx; j++)
		{
			LCD_WriteRAM(Color);
		}
	}
}
void LCD_DrawRound(uint16_t StartX, uint16_t StartY, uint8_t r, uint16_t Color)
{
	/* Bresenham 中点画圆算法 */
	int DeltaX, DeltaY;
	int di;
	
	DeltaX = 0;			/* x轴步进 */
	DeltaY = r;			/* y轴步进 */
	di = 3 - (r << 1);	/* 决策参数 */
	while (DeltaX <= DeltaY)
	{
		LCD_DrawPoint(StartX + DeltaX, StartY - DeltaY, Color);
		LCD_DrawPoint(StartX + DeltaY, StartY - DeltaX, Color);
		LCD_DrawPoint(StartX + DeltaY, StartY + DeltaX, Color);
		LCD_DrawPoint(StartX + DeltaX, StartY + DeltaY, Color);
		LCD_DrawPoint(StartX - DeltaX, StartY + DeltaY, Color);
		LCD_DrawPoint(StartX - DeltaY, StartY + DeltaX, Color);
		LCD_DrawPoint(StartX - DeltaX, StartY - DeltaY, Color);
		LCD_DrawPoint(StartX - DeltaY, StartY - DeltaX, Color);
		DeltaX++;

		if (di < 0)
		{
			di += 4 * DeltaX + 6;
		}
		else
		{
			di += 10 + 4 * (DeltaX - DeltaY);
			DeltaY--;
		}
	}
}
void LCD_FillRound(uint16_t StartX, uint16_t StartY, uint8_t r, uint16_t Color)
{

}
void LCD_ShowChar(uint16_t StartX, uint16_t StartY, uint8_t chr, uint8_t Size, uint16_t Color)
{
	uint8_t temp;
	uint16_t iteratorY = StartY;
	uint8_t *pfont = 0;
	uint8_t ByteSize = (Size / 8 + ((Size % 8) ? 1 : 0)) * (Size / 2);	/* 字节大小 */

	chr = chr - ' ';

	switch (Size)
	{
		case 12:
			pfont = (uint8_t *)asc2_1206[chr];
			break;
		case 16:
			pfont = (uint8_t *)asc2_1608[chr];
			break;
		case 24:
			pfont = (uint8_t *)asc2_2412[chr];
			break;
		case 32:
			pfont = (uint8_t *)asc2_3216[chr];
			break;
		default:
			pfont = (uint8_t *)asc2_1608[chr];
			break;
	}

	for (uint8_t i = 0; i < ByteSize; i++)
	{
		temp = pfont[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			if (temp & 0x80)
			{
				//有效点
				LCD_DrawPoint(StartX,iteratorY,Color);
			}
			else
			{
				//默认填充白色
				LCD_DrawPoint(StartX,iteratorY,CHAR_DEFAULT_FILLCOLOR);
			}
			temp <<= 1;
			iteratorY++;
			if (iteratorY >= LCD_Info.Height)
			{
				return;
			}
			if ((iteratorY - StartY) == Size)
			{
				iteratorY = StartY;
				StartX++;
				if (StartX >= LCD_Info.Width)
				{
					return;
				}
				break;
			}
		}
	}
	
}
void LCD_ShowString(uint16_t StartX, uint16_t StartY, uint16_t BorderX, uint16_t BorderY, uint8_t *pChr, uint8_t Size, uint16_t Color)
{
	uint8_t iteratorX = StartX;
	BorderX += iteratorX;
	BorderY += StartY;

	while ((*pChr <= '~') && (*pChr >= ' '))
	{
	    if (iteratorX >= BorderX)
	    {
			//超边界自动换行
			iteratorX = StartX;
			StartY += Size;
	    }
	    if (StartY >= BorderY)
		{
			break;
		}

	    LCD_ShowChar(StartX,StartY,*pChr,Size,Color);
	    iteratorX += Size / 2;		/* ASCII字符宽高比1:2,每个字符的占位宽度固定为Size/2 */
	    pChr++;
	}
}

