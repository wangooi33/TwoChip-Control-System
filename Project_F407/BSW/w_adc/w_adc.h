#ifndef _W_ADC_H
#define _W_ADC_H
/* include -------------------------------------------------------------------*/
#include "main.h"
#include "BDC_Control.h"
#include "BLDC_Control.h"

/* macro ---------------------------------------------------------------------*/


/* types ---------------------------------------------------------------------*/


/* constants -----------------------------------------------------------------*/


/* global variable -----------------------------------------------------------*/


/* functions prototypes ------------------------------------------------------*/
void Motoer_CurrentOffsetCalibrate( BDC_Info_t *pBDC, BLDC_Info_t *pBLDC );
void ADC_Cyclic( void );

#endif /* _W_ADC_H */

