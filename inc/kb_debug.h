#ifndef KB_DEBUG_H_INCLUDED
#define KB_DEBUG_H_INCLUDED

#include "mamachdep.h"

//---------------------------------------------------------------------------------------------------------------------------------------------------


// #define KB_DEBUG_ENABLED


//---------------------------------------------------------------------------------------------------------------------------------------------------

#ifdef KB_DEBUG_ENABLED

// log buffer size
#define LOG_ENTRIES_MAX 14400


// log buffer entry structure
#pragma pack(1)
// 4 bytes
typedef struct
{
    UINT16 ushCnt;
    UINT8 flag;
    UINT8 data;
} YMU262_LOG_ENTRY;
#pragma pack()

// log entry type definitions
typedef enum
{
    LOGMA3_RESET = 0,

    LOGMA3_INITIALIZE,
    LOGMA3_INIT_REGISTERS,
    LOGMA3_VERIFY_REGISTERS,
    LOGMA3_POWER_MANAGEMENT,
    LOGMA3_DEVICE_CONTROL,

    LOGMA3_CLEAR_FIFO,
    LOGMA3_FIFO,

    LOGMA3_SOUND_INITIALIZE,

    LOGMA3_HP_VOLUME,
    LOGMA3_EQ_VOLUME,
    LOGMA3_SP_VOLUME,
    LOGMA3_SET_VOLUME,

    LOGMA3_CHECK_STATUS,

    LOGMA3_WAIT_VALID_DATA,
    LOGMA3_CHECK_DELAYED_FIFO_EMPTY,
    LOGMA3_WAIT_DELAYED_FIFO_EMPTY,
    LOGMA3_WAIT_IMMEDIATE_FIFO_EMPTY,

    LOGMA3_SEND_DELAYED_PACKET,
    LOGMA3_SEND_DIRECT_PACKET,

    LOGMA3_RECEIVE_DATA,
    LOGMA3_SEND_DIRECT_RAM_DATA,
    LOGMA3_SEND_DIRECT_RAM_VAL,

    LOGMA3_INT_HANDLER,
    LOGMA3_CONTROL_INTERRUPT,

    LOGMA3_SOFT_INT0,
    LOGMA3_SOFT_INT1,
    LOGMA3_SOFT_INT2,
    LOGMA3_SOFT_INT3,

    LOGMA3_TIMER0,
    LOGMA3_TIMER1,

    LOGMA3_STANDBY,
    LOGMA3_START,

    LOGMA3_START_SEQUENCER,
    LOGMA3_STOP_SEQUENCER,

    LOGMA3_SEEK_CONTROL,

    LOGMA3_STREAM_SETUP,
    LOGMA3_STREAM_UPDATE,
    LOGMA3_STREAM_HANDLER,
} MA3LogFlag_Type;

//---------------------------------------------------------------------------------------------------------------------------------------------------

void AddToBuffer(int isRead, int isData, UINT8 bVal);
void AddToBuffer_SpecialFlag(MA3LogFlag_Type bSpecialFlag);

void setTickFirst(UINT32 tickFirstParam);
void dumpToUsb(void);

#else // #ifdef KB_DEBUG_ENABLED

#define AddToBuffer(isRead, isData, bVal)
#define AddToBuffer_SpecialFlag(bSpecialFlag)

#define setTickFirst(tickFirstParam)
#define dumpToUsb()

#endif // #ifdef KB_DEBUG_ENABLED

#endif /* KB_DEBUG_H_INCLUDED */

