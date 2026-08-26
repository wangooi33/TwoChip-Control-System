#ifndef _W_ADC_H
#define _W_ADC_H

/* include -------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"

/* macro ---------------------------------------------------------------------*/
#define ADC_TO_VOLT(raw) ((float)(raw) * 3.3f / 4096.0f)

/* functions prototypes ------------------------------------------------------*/
void ADC_Enable(void);



#endif /* _W_ADC_H */
