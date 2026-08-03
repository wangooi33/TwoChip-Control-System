#include "communication.h"
#include "usart.h"
#include "BLDC_Control.h"
#include <math.h>
#include <string.h>

/* local constants ----------------------------------------------------------*/
#define C103_REQ_HEAD_0             0x3A
#define C103_REQ_HEAD_1             0x3A
#define C103_REQ_TAIL_0             0x4C
#define C103_REQ_TAIL_1             0x5E
#define C103_RSP_HEAD_0             0x4A
#define C103_RSP_HEAD_1             0x4A
#define C103_RSP_TAIL_0             0x4D
#define C103_RSP_TAIL_1             0x5E

/* local helpers ------------------------------------------------------------*/
static uint16_t prvCom103_GetFunid( const uint8_t *pBuf )
{
    return ((uint16_t)pBuf[C103_FUNID_HIGH_INDEX] << 8) | pBuf[C103_FUNID_LOW_INDEX];
}

static int32_t prvCom103_GetS32( const uint8_t *pBuf )
{
    uint32_t value = ((uint32_t)pBuf[C103_DATA_STARTINDEX] << 24)
                   | ((uint32_t)pBuf[C103_DATA_STARTINDEX + 1U] << 16)
                   | ((uint32_t)pBuf[C103_DATA_STARTINDEX + 2U] << 8)
                   | ((uint32_t)pBuf[C103_DATA_STARTINDEX + 3U]);

    return (int32_t)value;
}

static void prvCom103_PutS32( uint8_t *pBuf, int32_t value )
{
    uint32_t raw = (uint32_t)value;

    pBuf[0] = (uint8_t)(raw >> 24);
    pBuf[1] = (uint8_t)(raw >> 16);
    pBuf[2] = (uint8_t)(raw >> 8);
    pBuf[3] = (uint8_t)raw;
}

static int32_t prvCom103_RoundToS32( float value )
{
    if ( value >= 0.0f )
    {
        return (int32_t)(value + 0.5f);
    }

    return (int32_t)(value - 0.5f);
}

static int32_t prvCom103_GetBLDCCurrentPeak_mA( void )
{
    float iu = fabsf(BLDC_Info.CurrentPhase.U_PhaseCurrent);
    float iv = fabsf(BLDC_Info.CurrentPhase.V_PhaseCurrent);
    float iw = fabsf(BLDC_Info.CurrentPhase.W_PhaseCurrent);
    float peak = iu;

    if ( iv > peak )
    {
        peak = iv;
    }
    if ( iw > peak )
    {
        peak = iw;
    }

    return prvCom103_RoundToS32(peak);
}

static void prvCom103_FillHandshakeData( uint8_t *pBuf )
{
    memset(pBuf, 0, 4U);
    memcpy(pBuf, SoftWareID, 4U);
}

static int32_t prvCom103_ReadValue( C103Funid_t Funid )
{
    switch ( Funid )
    {

        case CMid_ReadBLDC_RPM:
            return prvCom103_RoundToS32(BLDC_Info.RPM);

        case CMid_ReadBLDC_Pos:
            return prvCom103_RoundToS32(BLDC_Info.CurrentAngleDeg);

        case CMid_ReadBLDC_Cur:
            return prvCom103_GetBLDCCurrentPeak_mA();

        default:
            return 0;
    }
}


static void prvCom103_HandleBLDCWrite( C103Funid_t Funid, int32_t Value )
{
    switch ( Funid )
    {
        case CMid_WriteBLDC_RPM:
            BLDC_SetExpectedRPM((float)Value);
            if ( Value <= 0 )
            {
                BLDC_Stop();
            }
            break;

        case CMid_WriteBLDC_Pos:
            BLDC_SetExpectedAngle((float)Value);
            break;

        case CMid_WriteBLDC_Cur:
            BLDC_SetExpectedCurrent((float)Value);
            if ( Value <= 0 )
            {
                BLDC_Stop();
            }
            break;

        default:
            break;
    }
}

static void prvCom103_HandleWriteCommand( C103Funid_t Funid, int32_t Value )
{
    switch ( Funid )
    {

        case CMid_WriteBLDC_RPM:
        case CMid_WriteBLDC_Pos:
        case CMid_WriteBLDC_Cur:
            prvCom103_HandleBLDCWrite(Funid, Value);
            break;

        default:
            break;
    }
}

static uint8_t prvCom103_IsSupportedFunid( C103Funid_t Funid )
{
    switch ( Funid )
    {
        case CMid_Handshake:

        case CMid_ReadBLDC_RPM:
        case CMid_ReadBLDC_Pos:
        case CMid_ReadBLDC_Cur:
        case CMid_WriteBLDC_RPM:
        case CMid_WriteBLDC_Pos:
        case CMid_WriteBLDC_Cur:
            return 1U;

        default:
            return 0U;
    }
}

/* public functions ---------------------------------------------------------*/
void Com103_Init( void )
{
    memset(gU1TxRxBuf, 0, sizeof(gU1TxRxBuf));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gU1TxRxBuf, U1_TXRX_BUFMAX);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void Com103_TxProcess( const uint8_t *pBuf, C103Funid_t Funid )
{
    uint8_t TxBuf[C103_FRAME_LENGTH];
    uint8_t index = 0U;
    uint16_t crc16;

    TxBuf[index++] = C103_RSP_HEAD_0;
    TxBuf[index++] = C103_RSP_HEAD_1;
    TxBuf[index++] = (uint8_t)((uint16_t)Funid >> 8);
    TxBuf[index++] = (uint8_t)((uint16_t)Funid & 0xFFU);

    if ( pBuf != NULL )
    {
        memcpy(&TxBuf[index], pBuf, 4U);
    }
    else
    {
        memset(&TxBuf[index], 0, 4U);
    }
    index += 4U;

    crc16 = CheckCRC16(TxBuf, index);
    TxBuf[index++] = (uint8_t)(crc16 >> 8);
    TxBuf[index++] = (uint8_t)(crc16 & 0xFFU);
    TxBuf[index++] = C103_RSP_TAIL_0;
    TxBuf[index++] = C103_RSP_TAIL_1;

    HAL_UART_Transmit(&huart1, TxBuf, index, 50);
}

uint8_t Com103_CheckRequestFrame( const uint8_t *pBuf, uint16_t Size )
{
    uint16_t crc16;
    uint16_t crc16_origin;

    if ( pBuf == NULL || Size != C103_FRAME_LENGTH )
    {
        return 0U;
    }

    if ( pBuf[0] != C103_REQ_HEAD_0 || pBuf[1] != C103_REQ_HEAD_1 )
    {
        return 0U;
    }

    if ( pBuf[C103_FRAME_LENGTH - 2U] != C103_REQ_TAIL_0 ||
         pBuf[C103_FRAME_LENGTH - 1U] != C103_REQ_TAIL_1 )
    {
        return 0U;
    }

    crc16 = CheckCRC16((uint8_t *)pBuf, C103_CRC_STARTINDEX);
    crc16_origin = ((uint16_t)pBuf[C103_CRC_STARTINDEX] << 8) | pBuf[C103_CRC_STARTINDEX + 1U];

    return (crc16 == crc16_origin) ? 1U : 0U;
}

void Com103_RxEventHandler( uint8_t *pBuf, uint16_t Size )
{
    uint8_t tx_data[4];
    C103Funid_t funid;
    int32_t data;

    if ( Com103_CheckRequestFrame(pBuf, Size) == 0U )
    {
        return;
    }

    funid = (C103Funid_t)prvCom103_GetFunid(pBuf);
    if ( prvCom103_IsSupportedFunid(funid) == 0U )
    {
        return;
    }

    data = prvCom103_GetS32(pBuf);

    switch ( funid )
    {
        case CMid_Handshake:
            prvCom103_FillHandshakeData(tx_data);
            Com103_TxProcess(tx_data, funid);
            break;

        case CMid_ReadBLDC_RPM:
        case CMid_ReadBLDC_Pos:
        case CMid_ReadBLDC_Cur:
            prvCom103_PutS32(tx_data, prvCom103_ReadValue(funid));
            Com103_TxProcess(tx_data, funid);
            break;

        case CMid_WriteBLDC_RPM:
        case CMid_WriteBLDC_Pos:
        case CMid_WriteBLDC_Cur:
            prvCom103_HandleWriteCommand(funid, data);
            prvCom103_PutS32(tx_data, data);
            Com103_TxProcess(tx_data, funid);
            break;

        default:
            break;
    }
}
