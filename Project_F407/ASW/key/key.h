#ifndef __KEY_H
#define __KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* macro ---------------------------------------------------------------------*/
#define KEY_NUM        5
#define DEBOUNCE_CNT   2
/* enum ----------------------------------------------------------------------*/
typedef enum
{
    KEY_NONE = 0,
    KEY1_PRESS,
    KEY2_PRESS,
    KEY3_PRESS,
    KEY4_PRESS,
    KEY5_PRESS
}KeyEvent_t;
/* types ---------------------------------------------------------------------*/

/* constants -----------------------------------------------------------------*/


/* global variable -----------------------------------------------------------*/
extern uint8_t KeyCnt[KEY_NUM];
extern uint8_t KeyState[KEY_NUM];

/* functions prototypes ------------------------------------------------------*/
void KeyTask_Cyclic(void);


#endif /* __KEY_H */

