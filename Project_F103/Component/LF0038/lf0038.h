#ifndef __LF0038_H
#define __LF0038_H
/* includes -----------------------------------------------------------------*/
#include "main.h"

/* macro --------------------------------------------------------------------*/

/* NEC协议参数 */
/*逻辑0*/
#define NEC_BIT0_MIN_US				300
#define NEC_BIT0_MAX_US				800
/*逻辑1*/
#define NEC_BIT1_MIN_US				1400
#define NEC_BIT1_MAX_US				1800
/*同步码*/
#define NEC_LEAD_MIN_US				4200
#define NEC_LEAD_MAX_US				4700
/*连发码*/
#define NEC_REPEAT_MIN_US			2000
#define NEC_REPEAT_MAX_US			3000


/* 键值 */
#define InfraredCtrl_Cmd_Power		69
#define InfraredCtrl_Cmd_ZDYZ		71
#define InfraredCtrl_Cmd_Up			70
#define InfraredCtrl_Cmd_Down		21
#define InfraredCtrl_Cmd_Play		64
#define InfraredCtrl_Cmd_Right		67
#define InfraredCtrl_Cmd_Left		68
#define InfraredCtrl_Cmd_VolReduce	7
#define InfraredCtrl_Cmd_VolAdd		9
#define InfraredCtrl_Cmd_Del		74

#define InfraredCtrl_Cmd_1			22
#define InfraredCtrl_Cmd_2			25
#define InfraredCtrl_Cmd_3			13
#define InfraredCtrl_Cmd_4			12
#define InfraredCtrl_Cmd_5			24
#define InfraredCtrl_Cmd_6			94
#define InfraredCtrl_Cmd_7			8
#define InfraredCtrl_Cmd_8			28
#define InfraredCtrl_Cmd_9			90
#define InfraredCtrl_Cmd_0			66

/* 红外遥控识别码(ID) */
#define LF0038_ID					0x00

/* enum ---------------------------------------------------------------------*/
typedef enum
{
    InfraredCtrl_Idle = 0,
    InfraredCtrl_ReadData,
}InfraredCtrl_State_t;

/* types --------------------------------------------------------------------*/
typedef struct
{
    volatile uint32_t Frame;
    volatile uint8_t BitCnt;
    volatile uint8_t FrameOk;
    volatile uint16_t RepeatCnt;
	InfraredCtrl_State_t State;
}InfraredCtrl_Info_t;

/* global variable ----------------------------------------------------------*/
extern volatile InfraredCtrl_Info_t InfraredCtrl_Info;

/* functions prototypes -----------------------------------------------------*/
uint8_t InfraredCtrl_isBit0(uint16_t us);
uint8_t InfraredCtrl_isBit1(uint16_t us);
uint8_t InfraredCtrl_isLead(uint16_t us);
uint8_t InfraredCtrl_isRepeat(uint16_t us);
void InfraredCtrl_Reset(void);
void InfraredCtrl_Framing(uint8_t bit);
uint8_t InfraredCtrl_FrameValid(uint32_t Frame);
uint8_t InfraredCtrl_Scan(void);

void Task3_InfraredScan(void *pvParameters);


#endif  /* __LF0038_H */
