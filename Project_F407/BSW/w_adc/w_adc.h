#ifndef _W_ADC_H
#define _W_ADC_H

/* include -------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "BLDC_Control.h"

/* macro ---------------------------------------------------------------------*/

/* --电压采集-- */
#define ADC_R_UPPER_K       (100.0f)    /* 上分压电阻 [kOhm] */
#define ADC_R_LOWER_K       (10.0f)     /* 下分压电阻 [kOhm] */
#define ADC_DIV_RATIO       ((ADC_R_UPPER_K + ADC_R_LOWER_K) / ADC_R_LOWER_K)
/* ADC 参数 */
#define ADC_REF_V           (3.3f)      /* ADC 参考电压 [V] */
#define ADC_BITS            (4096.0f)   /* 12-bit = 4096 级 */
#define ADC_V_PER_LSB       (ADC_REF_V / ADC_BITS)
/* 母线保护阈值 [V] */
#define ADC_VBUS_OV_THRESH  (30.0f)     /* 过压 */
#define ADC_VBUS_UV_THRESH  (8.0f)      /* 欠压 */

/* --电流采集-- */
#define ADC_R_SHUNT         (0.005f)    /* 采样电阻 [Ohm] */
#define ADC_AMP_GAIN        (20.0f)     /* 差分放大倍数 */
#define ADC_V_OFFSET        (1.65f)     /* 抬升电压 [V] */

/* --温度采集--*/
#define ADC_TEMP_R_FIXED    (4700.0f)   /* 固定电阻 [Ohm] */
#define ADC_TEMP_R0         (10000.0f)  /* NTC 25C 标称阻值 [Ohm] */
#define ADC_TEMP_BETA       (3950.0f)   /* NTC B 常数 */
#define ADC_TEMP_T2         (298.15f)   /* 25C 开尔文温度 */
#define ADC_TEMP_OT_THRESH  (85.0f)     /* 过温保护 [C] */

/* functions prototypes ------------------------------------------------------*/
float ADC_to_Current( uint16_t raw, float offsetV );
float ADC_to_BusVoltage( uint16_t raw );
float ADC_to_Rt( uint16_t raw );
float Rt_to_Temperature( float rt );
void ReadPhaseCurrents( void );
void ReadBusVoltage( void );
void ReadTemperature( void );
uint8_t CheckBusFault( void );
uint8_t CheckTempFault( void );
void Clarke( float iu, float iv, float *pAlpha, float *pBeta );

void Motor_CurrentOffsetCalibrate( BLDC_Info_t *pBLDC );
void ADC_Cyclic( void );

#endif /* _W_ADC_H */
