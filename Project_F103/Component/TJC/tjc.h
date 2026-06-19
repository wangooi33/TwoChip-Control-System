#ifndef __TJC_H
#define __TJC_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

/* macro --------------------------------------------------------------------*/
#define TJC_MAX_PAGENUM		3
/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/
typedef struct
{
	uint8_t CurrentPage;
	uint8_t Videoisplay;
}TJC_Info_t;
/* global variable ----------------------------------------------------------*/
extern TJC_Info_t TJC_Info;

/* functions prototypes -----------------------------------------------------*/
void TJC_Init(void);
void TJC_ChangePage(uint8_t Page);
void TJC_ChangeVideoState(uint8_t id, uint8_t state);

#endif /* __TJC_H */

