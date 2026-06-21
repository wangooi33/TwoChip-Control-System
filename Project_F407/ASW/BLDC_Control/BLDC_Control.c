/* includes ------------------------------------------------------------------*/
#include "BLDC_Control.h"

/* global variable -----------------------------------------------------------*/
BLDC_Info_t BLDC_Info;
Hall_Info_t Hall_Info =
{
	.HallFirstEdge = 1
};

BLDCMosCom_t gComFwd[8] =
{
	[0] = {PHASE_NONE,	PHASE_NONE},	//000:无效
	[1] = {PHASE_U,		PHASE_W},		//001:U+ W-
	[2] = {PHASE_V,		PHASE_U},		//010:V+ U-
	[3] = {PHASE_V,		PHASE_W},		//011:V+ W-
	[4] = {PHASE_W,		PHASE_V},		//100:W+ V-
	[5] = {PHASE_U,		PHASE_V},		//101:U+ V-
	[6] = {PHASE_W,		PHASE_U},		//110:W+ U-
	[7] = {PHASE_NONE,	PHASE_NONE}, 	//111:无效
};
BLDCMosCom_t gComRev[8] =
{
	[0] = {PHASE_NONE,	PHASE_NONE},
	[1] = {PHASE_W,		PHASE_U},		//001
	[2] = {PHASE_U,		PHASE_V},		//010
	[3] = {PHASE_W,		PHASE_V},		//011
	[4] = {PHASE_V,		PHASE_W},		//100
	[5] = {PHASE_V,		PHASE_U},		//101
	[6] = {PHASE_U,		PHASE_W},		//110
	[7] = {PHASE_NONE,	PHASE_NONE},
};
BLDCMosCom_t *pHallTable = NULL;

/* functions declaration -----------------------------------------------------*/
static void prvDisableAllMos( void );
uint8_t Hall_GetState( void );
MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC );

/* functions implementation --------------------------------------------------*/
static void prvDisableAllMos( void )
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_RESET);
}
void BLDC_Disable( void )
{
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
	prvDisableAllMos();
	BLDC_SD_DISABLE();
}
void BLDC_Enable( void )
{
	BLDC_SD_ENABLE();
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}
void Hall_enable( void )
{
	uint8_t Hall;

	Hall_Info.HallFirstEdge   = 1;
	Hall_Info.HallEdgeFlag	  = 0;
	Hall_Info.HallTickCnt	  = 0;
	Hall_Info.HallStateShadow = 0;
	Hall_Info.HallLastEdgeMs  = SystemRunTime_1ms;

	HAL_TIMEx_HallSensor_Start_IT(&htim5);
	Hall = Hall_GetState();
	BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

	if ( Hall >= 1 && Hall <= 6 )
	{
		BLDC_ChangeMOSstate(pHallTable[Hall].PwmPhase,pHallTable[Hall].LowPhase,BLDC_Info.Pulse);
	}
}
void Hall_Disable(void)
{
	__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_TRIGGER);
	__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
	HAL_TIMEx_HallSensor_Stop(&htim5);
}
uint8_t Hall_GetState( void )
{
	uint8_t State = 0;

	//U相
	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_10) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 0;
	}

	//V相
	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 1;
	}

	//W相
	if( HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_12) != GPIO_PIN_RESET )
	{
		State |= 0x01U << 2;
	}
	
	return State;
}
void BLDC_ChangeMOSstate( Phase_t PwmPhase, Phase_t LowPhase, uint16_t Duty )
{
	prvDisableAllMos();

	switch (PwmPhase)
	{
		case PHASE_U:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,Duty);
			break;
		case PHASE_V:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,Duty);
			break;
		case PHASE_W:
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,Duty);
			break;
		default:
			return;
	}
	switch (LowPhase)
	{
		case PHASE_U:
			HAL_GPIO_WritePin(BLDC_CH1N_GPIO_Port, BLDC_CH1N_Pin, GPIO_PIN_SET);
			break;
		case PHASE_V:
			HAL_GPIO_WritePin(BLDC_CH2N_GPIO_Port, BLDC_CH2N_Pin, GPIO_PIN_SET);
			break;
		case PHASE_W:
			HAL_GPIO_WritePin(BLDC_CH3N_GPIO_Port, BLDC_CH3N_Pin, GPIO_PIN_SET);
			break;
		default:
			return;
	}

	HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_COM);
}
void BLDC_HallTableSelect( MotorDir_t Dir )
{
	if ( Dir == MOTOR_FWD )
	{
		pHallTable = gComFwd;
	}
	else
	{
		pHallTable = gComRev;
	}
}

MotorDir_t BLDC_GetDirection( BLDC_Info_t *pBLDC )
{
	return pBLDC->Direction;
}

static uint32_t prvMedian3( uint32_t Data1, uint32_t Median, uint32_t Data3 )
{
	//三点中值法:从大到小排序
	uint32_t TempValue;
	if ( Data1 > Median )
	{
		TempValue = Data1;
		Data1 = Median;
		Median = TempValue;
	}
	if ( Median > Data3 )
	{
		TempValue = Median;
		Median = Data3;
		Data3 = TempValue;
	}
	if ( Data1 > Median )
	{
		TempValue = Data1;
		Data1 = Median;
		Median = TempValue;
	}
	return Median;
}
static uint32_t prvHallPeriodFilter_Update( Hall_Info_t *pHall, uint32_t RawValue )
{
	HallSpeedFilter_t *Filter = &pHall->HallSpeedFilter;
	uint32_t Median = RawValue;

	if ( RawValue == 0U )
	{
		return 0U;
	}

	//填充
	Filter->HallTickBuf[Filter->Index] = RawValue;
	Filter->Index = (Filter->Index + 1) % 3;
	if ( Filter->ValidCnt < 3 )
	{
		Filter->ValidCnt++;
	}
	//初始化
	if ( Filter->Inited == 0 )
	{
		Filter->LastFilter = RawValue;
		Filter->Inited = 1;
		return RawValue;
	}

	if ( Filter->ValidCnt >= 3 )
	{
		uint8_t i0 = Filter->Index;
		uint8_t i1 = (Filter->Index + 1) % 3;
		uint8_t i2 = (Filter->Index + 2) % 3;
		Median = prvMedian3(Filter->HallTickBuf[i0],Filter->HallTickBuf[i1],Filter->HallTickBuf[i2]);
	}

	//一阶 IIR: α = 1/8
	Filter->LastFilter= Filter->LastFilter + ((int32_t)Median - (int32_t)Filter->LastFilter) / 8;

	return Filter->LastFilter;
}

void BLDC_HallCyclic( Hall_Info_t *pHall )
{
	uint8_t Hall;
	uint32_t RawDelta;
	uint32_t FiltDelta;
	
	if ( pHall->HallEdgeFlag )
	{
		pHall->HallEdgeFlag = 0;
		Hall = pHall->HallStateShadow;

		BLDC_HallTableSelect(BLDC_GetDirection(&BLDC_Info));

		if ( Hall >= 1 && Hall <= 6 &&
			pHallTable[Hall].PwmPhase != PHASE_NONE && pHallTable[Hall].LowPhase != PHASE_NONE)
		{
			BLDC_ChangeMOSstate(pHallTable[Hall].PwmPhase,pHallTable[Hall].LowPhase,BLDC_Info.Pulse);
		}

		//BLDC_Info.RPM = 60.0f * BLDC_HALL_TIMER_HZ / ((float)Hall_Info.HallTickCnt * 6.0f * BLDC_POLE_PAIRS);

		RawDelta = pHall->HallTickCnt;

		if ( RawDelta > 0 )
		{
			FiltDelta = prvHallPeriodFilter_Update(&Hall_Info,RawDelta);

			if ( FiltDelta > 0 )
			{
				BLDC_Info.RPM = 60.0f * (float)BLDC_HALL_TIMER_HZ / ((float)FiltDelta * 6.0f * (float)BLDC_POLE_PAIRS);
			}
		}
	}
}
void BLDC_Cyclic( void )
{
	BLDC_HallCyclic(&Hall_Info);

}

