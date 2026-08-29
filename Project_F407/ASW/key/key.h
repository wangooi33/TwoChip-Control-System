#ifndef _KEY_H
#define _KEY_H

/* include -------------------------------------------------------------------*/
#include "main.h"

/* enum ---------------------------------------------------------------------*/
typedef enum
{
	KEY_NONE = 0,
	KEY1_PRESS,
	KEY2_PRESS,
	KEY3_PRESS,
	KEY4_PRESS,
	KEY5_PRESS
}KeyEvent_t;

/* functions prototypes ------------------------------------------------------*/
void KeyTask_Cyclic(void);

#endif /* _KEY_H */
