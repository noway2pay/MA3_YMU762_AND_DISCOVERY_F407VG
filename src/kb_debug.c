#include "kb_debug.h"
#include "usbd_cdc_if.h"

#define USB_OUT_PACKET_MAX 512
#define SINGLE_LOG_ENTRY 13

// -10 because of the "timestamp:" preamble
#define NR_OF_LOG_ENTRIES_IN_PACKET_MAX ((USB_OUT_PACKET_MAX - SINGLE_LOG_ENTRY - 1)/SINGLE_LOG_ENTRY)

// log buffer
YMU262_LOG_ENTRY logbuf[LOG_ENTRIES_MAX];
// log buffer's variables
volatile UINT32 ulPos = 0;
volatile UINT32 ulCount = 0;


int nLen = 0;
int delta = 0;
uint32_t lastPos = LOG_ENTRIES_MAX - 1;           // last position (now) - initialized to the last log entry (so index 0 is pushed with the first chunk)
uint32_t lastPosPrev = 0;                         // last position (previous)
uint32_t firstPos = 0;                            // first position in current chunk of unsent log data
uint32_t tickNow = 0;
uint32_t tickFirst = 0;
int thisChunk = 0;

char strTmp[USB_OUT_PACKET_MAX] = { 0, };
uint32_t CDC_requests_sent = 0;


void AddToBuffer(int isRead, int isData, UINT8 bVal )
{
    UINT8 flag = (isRead ? 1 : 0) | (isData ? 2 : 0);

    // if repeat (e.g. read status - with the same data returned)
    if(((logbuf[ulPos].flag & 0x7F) == flag) && (logbuf[ulPos].data == bVal))
    {
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag | 0x80;    // mark repeat
    }
    else
    {
        // if "leaving" repeat - advance position not to overwrite the "repeat" log entry
        if( (logbuf[ulPos].flag & 0x80) != 0 )
        {
            ulPos++;

            if(ulPos >= LOG_ENTRIES_MAX)
            {
                ulPos = 0;
            }
        }

        // store next entry
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag;
        logbuf[ulPos].data = bVal;
        ulPos++;

        if(ulPos >= LOG_ENTRIES_MAX)
        {
            ulPos = 0;
        }
    }

    // some counter for reference ;)
    ulCount++;
}

void AddToBuffer_SpecialFlag(MA3LogFlag_Type bSpecialFlag)
{

    // if "leaving" repeat - advance position not to overwrite the "repeat" log entry
    if( (logbuf[ulPos].flag & 0x80) != 0 )
    {
        ulPos++;

        if(ulPos >= LOG_ENTRIES_MAX)
        {
            ulPos = 0;
        }
    }

    // store special
    logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
    logbuf[ulPos].flag = 0xC0;
    logbuf[ulPos].data = (UINT8) bSpecialFlag;
    ulPos++;

    if(ulPos >= LOG_ENTRIES_MAX)
    {
        ulPos = 0;
    }

    ulCount++;
}

void setTickFirst(uint32_t tickFirstParam)
{
    tickFirst = tickFirstParam;
}

void dumpToUsb(void)
{
    // update "previous" last position and new last position
    lastPosPrev = lastPos;
    lastPos = ulPos;

    // first position was previously the last one (+1 as we want to send only new log entries)
    firstPos = lastPosPrev + 1;

    if(firstPos >= LOG_ENTRIES_MAX)
    {
        firstPos = 0;
    }

    if(lastPos >= firstPos)                                             // some new samples stored (no boundary crossed)
        delta = lastPos - firstPos;
    else
        delta = LOG_ENTRIES_MAX - firstPos + lastPos;                   // boundary crossed

    // if TX busy or no data to be sent - check again in 1 ms
    if( (CDC_CheckTransmitPossible() == USBD_BUSY) || (delta == 0) )
    {
        //delay_ms(1);
        return;
    }

    tickNow = HAL_GetTick();
    nLen = sprintf(strTmp, ":%08X\r", (UINT16) (tickNow - tickFirst));

    thisChunk = MIN(delta, NR_OF_LOG_ENTRIES_IN_PACKET_MAX);

    while(delta > 0)
    {
        for(int i=0; i< thisChunk; i++)
        {
            if( (logbuf[firstPos].flag & 0xC0) == 0xC0 )    // 0xC0 means a flag (data is then a special code)
            {
                nLen += sprintf(strTmp + nLen,"%04X%04X--%02X\r", (UINT16) firstPos, logbuf[firstPos].ushCnt, logbuf[firstPos].data);
            }
            else
            {
                nLen += sprintf(strTmp + nLen,"%04X%04X%02X%02X\r", (UINT16) firstPos, logbuf[firstPos].ushCnt, logbuf[firstPos].flag, logbuf[firstPos].data);
            }

            firstPos++;
            if(firstPos >= LOG_ENTRIES_MAX)
            {
                firstPos = 0;
            }
        }

        CDC_Transmit_FS((UINT8 *) strTmp,nLen);

        nLen = 0;

        while( CDC_CheckTransmitPossible() == USBD_BUSY )
        {
            //delay_ms(1);
        }

        CDC_requests_sent++;

        delta -= thisChunk;
    }


    lastPos = firstPos;
}
