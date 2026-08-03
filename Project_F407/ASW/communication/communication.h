#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------*/
#include "main.h"
#include "check.h"

/* macro --------------------------------------------------------------------*/
#define C103_FRAME_LENGTH           12U
#define C103_FUNID_HIGH_INDEX       2U
#define C103_FUNID_LOW_INDEX        3U
/* Actual frame layout is head(2) + funid(2) + data(4) + crc(2) + tail(2). */
#define C103_DATA_STARTINDEX        4U
#define C103_CRC_STARTINDEX         8U

/* enum ---------------------------------------------------------------------*/
typedef enum
{
    CMid_Handshake = 0x0100,


    CMid_ReadBLDC_RPM = 0x2101,
    CMid_ReadBLDC_Pos = 0x2102,
    CMid_ReadBLDC_Cur = 0x2103,

    CMid_WriteBLDC_RPM = 0x2201,
    CMid_WriteBLDC_Pos = 0x2202,
    CMid_WriteBLDC_Cur = 0x2203,
} C103Funid_t;

/* functions prototypes -----------------------------------------------------*/
void Com103_Init( void );
void Com103_RxEventHandler( uint8_t *pBuf, uint16_t Size );
void Com103_TxProcess( const uint8_t *pBuf, C103Funid_t Funid );
uint8_t Com103_CheckRequestFrame( const uint8_t *pBuf, uint16_t Size );

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */
