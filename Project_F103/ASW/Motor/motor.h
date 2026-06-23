#ifndef __MOTOR_H
#define __MOTOR_H
/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "task_manage.h"
#include "check.h"

/* macro --------------------------------------------------------------------*/

/* enum ---------------------------------------------------------------------*/
typedef enum
{
	CMotorFunid_Handshake = 0x0100,


}CMotorFunid_t;

typedef enum
{
	CMotor_Init,
	CMotor_Handshake,
	CMotor_HandshakeFail,
	CMotor_HandshakeSucceed,

	CMotor_ToBDC,
	CMotor_ToBLDC,
}CMotorState_t;
    
/* types --------------------------------------------------------------------*/
typedef struct
{
	uint8_t MotorSoftWareID[5];
	CMotorState_t CMotorState;
}Motor_Info_t;

/* functions prototypes -----------------------------------------------------*/


#endif  /* __MOTOR_H */
