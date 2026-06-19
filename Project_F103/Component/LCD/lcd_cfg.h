#ifndef __LCD_CFG_H
#define __LCD_CFG_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "fsmc.h"

/* enum ---------------------------------------------------------------------*/
typedef enum
{
	/* 行优先 */
	LeftToRight_TopToBottom,		/* 左到右,上到下 */
	LeftToRight_BottomToTop,		/* 左到右,下到上 */
	RightToLeft_TopToBottom,		/* 右到左,上到下 */
	RightToLeft_BottomToTop,		/* 右到左,下到上 */

	/* 列优先 */
	TopToBottom_LeftToRight,		/* 上到下,左到右 */
	TopToBottom_RightToLeft,		/* 上到下,右到左 */
	BottomToTop_LeftToRight,		/* 下到上,左到右 */
	BottomToTop_RightToLeft,		/* 下到上,右到左 */
}LCD_ScanDir_t;

/* types --------------------------------------------------------------------*/
typedef struct
{
    volatile uint16_t LCD_REG;
    volatile uint16_t LCD_RAM;
}LCD_TypeDef;

typedef struct
{
	uint16_t ID;
	uint16_t Width;
	uint16_t Height;
	uint8_t Direction;		/* 0:竖屏, 1:横屏 */
	LCD_ScanDir_t ScanDir;
}LCD_Info_t;

/* macro --------------------------------------------------------------------*/

/* LCD_BASE的详细解算方法:
 * 我们一般使用FSMC的块1(BANK1)来驱动TFTLCD液晶屏(MCU屏), 块1地址范围总大小为256MB,均分成4块:
 * 存储块1(FSMC_NE1)地址范围: 0X6000 0000 ~ 0X63FF FFFF
 * 存储块2(FSMC_NE2)地址范围: 0X6400 0000 ~ 0X67FF FFFF
 * 存储块3(FSMC_NE3)地址范围: 0X6800 0000 ~ 0X6BFF FFFF
 * 存储块4(FSMC_NE4)地址范围: 0X6C00 0000 ~ 0X6FFF FFFF
 *
 * 我们需要根据硬件连接方式选择合适的片选(连接LCD_CS)和地址线(连接LCD_RS)
 * 战舰F103开发板使用FSMC_NE4连接LCD_CS, FSMC_A10连接LCD_RS ,16位数据线,计算方法如下:
 * 首先FSMC_NE4的基地址为: 0X6C00 0000;     NEx的基址为(x=1/2/3/4): 0X6000 0000 + (0X400 0000 * (x - 1))
 * FSMC_A10对应地址值: 2^10 * 2 = 0X800;    FSMC_Ay对应的地址为(y = 0 ~ 25): 2^y * 2
 *
 * LCD->LCD_REG,对应LCD_RS = 0(LCD寄存器); LCD->LCD_RAM,对应LCD_RS = 1(LCD数据)
 * 则 LCD->LCD_RAM的地址为:  0X6C00 0000 + 2^10 * 2 = 0X6C00 0800
 *    LCD->LCD_REG的地址可以为 LCD->LCD_RAM之外的任意地址.
 * 由于我们使用结构体管理LCD_REG 和 LCD_RAM(REG在前,RAM在后,均为16位数据宽度)
 * 因此 结构体的基地址(LCD_BASE) = LCD_RAM - 2 = 0X6C00 0800 -2
 *
 * 更加通用的计算公式为((片选脚FSMC_NEx)x=1/2/3/4, (RS接地址线FSMC_Ay)y=0~25):
 *          LCD_BASE = (0X6000 0000 + (0X400 0000 * (x - 1))) | (2^y * 2 -2)
 *          等效于(使用移位操作)
 *          LCD_BASE = (0X6000 0000 + (0X400 0000 * (x - 1))) | ((1 << y) * 2 -2)
 */
#define LCD_FSMC_NEX         4              /* 使用FSMC_NE4接LCD_CS,取值范围只能是: 1~4 */
#define LCD_FSMC_AX          10             /* 使用FSMC_A10接LCD_RS,取值范围是: 0 ~ 25 */
#define LCD_BASE        (uint32_t)((0X60000000 + (0X4000000 * (LCD_FSMC_NEX - 1))) | (((1 << LCD_FSMC_AX) * 2) -2))
#define LCD             ((LCD_TypeDef *) LCD_BASE)



#define LCD_Command_ReadID4					0x04
#define LCD_Command_MemoryWrite				0x2C
#define LCD_Command_Memoryread				0x2E
#define LCD_Command_ColumnAddressSet		0x2A
#define LCD_Command_PageAddressSet			0x2B
#define LCD_Command_MemoryAccessControl		0x36
#define LCD_Command_DisplayON				0x29
#define LCD_Command_DisplayOFF				0x28


/* 触摸屏 ------ */
#define TOUCH_SPI_MO(x)                  do{ x ?                                                                                                         \
                                             HAL_GPIO_WritePin(TOUCH_SPI_MO_GPIO_Port, TOUCH_SPI_MO_Pin, GPIO_PIN_SET) :   \
                                             HAL_GPIO_WritePin(TOUCH_SPI_MO_GPIO_Port, TOUCH_SPI_MO_Pin, GPIO_PIN_RESET);  \
                                         }while(0)
#define TOUCH_SPI_CS(x)                  do{ x ?                                                                                                         \
                                             HAL_GPIO_WritePin(TOUCH_SPI_CS_GPIO_Port, TOUCH_SPI_CS_Pin, GPIO_PIN_SET) :  \
                                             HAL_GPIO_WritePin(TOUCH_SPI_CS_GPIO_Port, TOUCH_SPI_CS_Pin, GPIO_PIN_RESET); \
                                         }while(0)
#define TOUCH_SPI_CLK(x)                 do{ x ?                                                                                                         \
                                             HAL_GPIO_WritePin(TOUCH_SPI_CLK_GPIO_Port, TOUCH_SPI_CLK_Pin, GPIO_PIN_SET) :  \
                                             HAL_GPIO_WritePin(TOUCH_SPI_CLK_GPIO_Port, TOUCH_SPI_CLK_Pin, GPIO_PIN_RESET); \
                                         }while(0)
#define TOUCH_SPI_READ_MI()              HAL_GPIO_ReadPin(TOUCH_SPI_MI_GPIO_Port, TOUCH_SPI_MI_Pin)

/* 触摸屏幕时,输出低电平 */
#define TOUCH_READ_PEN()             	 HAL_GPIO_ReadPin(TOUCH_PEN_GPIO_Port, TOUCH_PEN_Pin)

/* 控制字 */
#define TOUCH_Command_GetX				0xD0
#define TOUCH_Command_GetY				0x90

/* global variable ----------------------------------------------------------*/
extern LCD_Info_t LCD_Info;

/* functions prototypes -----------------------------------------------------*/
void LCD_WriteRAM(volatile uint16_t Data);
void LCD_WriteREGNo(volatile uint16_t RegNo);
void LCD_WriteREG(uint16_t RegNo, uint16_t Data);
void LCD_DisplayON(void);
void LCD_DisplayOFF(void);
uint16_t LCD_ReadData(void);
void LCD_SetScanDir(LCD_ScanDir_t ScanDir);
void LCD_SetDisplayDir(uint8_t Dir);
void LCD_SetWindow(uint16_t StartX, uint16_t StartY, uint16_t Width, uint16_t Height);
void LCD_SetCursor(uint16_t X, uint16_t Y);
void LCD_ClearScreen(uint16_t Color);
void LCD_SetBL(uint16_t duty);
void LCD_Init(void);
uint8_t LCD_GetTouchCoord(uint16_t *X, uint16_t *Y);


#endif /* __LCD_CFG_H */
