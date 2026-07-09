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
	CMid_Handshake = 0x0100,

	CMid_ReadBDC_RPM = 0x1101,
	CMid_ReadBDC_Pos = 0x1102,
	CMid_ReadBDC_Cur = 0x1103,
	CMid_ReadBDC_PowerVoltage = 0x1104,

	CMid_WriteBDC_RPM = 0x1201,
	CMid_WriteBDC_Pos = 0x1202,
	CMid_WriteBDC_Cur = 0x1203,

	
	CMid_ReadBLDC_RPM = 0x2101,
	CMid_ReadBLDC_Pos = 0x2102,
	CMid_ReadBLDC_Cur = 0x2103,

	CMid_WriteBLDC_RPM = 0x2201,
	CMid_WriteBLDC_Pos = 0x2202,
	CMid_WriteBLDC_Cur = 0x2203,
}CMotorFunid_t;

typedef enum
{
	BDC,
	BLDC,
}ReadMotorType_t;

typedef enum
{
	CMotor_Handshake,
	CMotor_HandshakeFail,
	CMotor_HandshakeSucceed,
	CMotor_Communicating,
}CMotorState_t;
    
/* types --------------------------------------------------------------------*/
typedef struct
{
	CMotorFunid_t Funid;
	uint8_t Data[4];
}MotorCmd_t;

typedef struct
{
	uint8_t MotorSoftWareID[5];
	ReadMotorType_t ReadMotorType;
	CMotorState_t CMotorState;
}Motor_Info_t;

typedef struct
{
	uint16_t Funid;
	uint32_t Data;
}CMotorDataTable_t;

/* global variable ----------------------------------------------------------*/
extern Motor_Info_t Motor_Info;
extern QueueHandle_t CMotorQueue;

/* functions prototypes -----------------------------------------------------*/


#endif  /* __MOTOR_H */
