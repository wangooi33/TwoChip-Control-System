#ifndef __RGB_LED_H
#define __RGB_LED_H
/* includes -----------------------------------------------------------------*/
#include "main.h"

/* macro --------------------------------------------------------------------*/
#define RGB_RED      (RGB_t){255,0,0}
#define RGB_GREEN    (RGB_t){0,255,0}
#define RGB_BLUE     (RGB_t){0,0,255}
#define RGB_WHITE    (RGB_t){255,255,255}
#define RGB_YELLOW   (RGB_t){255,255,0}
#define RGB_PURPLE   (RGB_t){255,0,255}
#define RGB_CYAN     (RGB_t){0,255,255}

/* enum ---------------------------------------------------------------------*/

/* types --------------------------------------------------------------------*/
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}RGB_t;

typedef struct
{
    uint8_t idx;
    int8_t dir;
}RGB_Breath_t;

/* global variable ----------------------------------------------------------*/
extern RGB_Breath_t Breath1;

/* functions prototypes -----------------------------------------------------*/
void RGB_BreathInit(RGB_Breath_t *breath);
void RGB_BreathTask(RGB_Breath_t *breath, RGB_t color);
void RGB_RainbowTask(void);
void RGB_Init(void);

void Task1_RainbowRGB(void *pvParameters);


#endif  /* __RGB_LED_H */

