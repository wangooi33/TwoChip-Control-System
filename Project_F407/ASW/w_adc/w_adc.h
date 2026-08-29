#ifndef _W_ADC_H
#define _W_ADC_H

/* include -------------------------------------------------------------------*/
#include "main.h"

/* macro ---------------------------------------------------------------------*/
#define ADC_TO_VOLT(raw) ((float)(raw) * 3.3f / 4096.0f)

/* functions prototypes ------------------------------------------------------*/
void ADC_Enable(void);
void Motor_CurrentOffsetCal(BLDC_Info_t *pBLDC);
void BLDC_PhaseCurrentCal(void);
void BLDC_TemperatureCal(void);
void BLDC_VBusCal(void);


#endif /* _W_ADC_H */
