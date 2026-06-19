#ifndef __LCD_H
#define __LCD_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "lcd_cfg.h"

/* macro --------------------------------------------------------------------*/

/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/

/* global variable ----------------------------------------------------------*/

/* functions prototypes -----------------------------------------------------*/
uint16_t LCD_ReadPointColor(uint16_t X, uint16_t Y);
void LCD_DrawPoint(uint16_t X, uint16_t Y, uint16_t Color);
void LCD_DrawLine(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color);
void LCD_DrawRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color);
void LCD_FillRectangle(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height, uint16_t Color);
void LCD_DrawRound(uint16_t StartX, uint16_t StartY, uint8_t r, uint16_t Color);
void LCD_FillRound(uint16_t StartX, uint16_t StartY, uint8_t r, uint16_t Color);
void LCD_ShowChar(uint16_t StartX, uint16_t StartY, uint8_t chr, uint8_t Size, uint16_t Color);
void LCD_ShowString(uint16_t StartX, uint16_t StartY, uint16_t BorderX, uint16_t BorderY, uint8_t *pChr, uint8_t Size, uint16_t Color);


#endif /* __LCD_H */
