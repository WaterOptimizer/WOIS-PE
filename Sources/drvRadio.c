/******************************************************************************
 *                       Copyright (c) 2008, Jabil Circuit
 *
 * This source code and any compilation or derivative thereof is the sole
 * property of Jabil Circuit and is provided pursuant to a Software License
 * Agreement.  This code is the proprietary information of Jabil Circuit and
 * is confidential in nature.  Its use and dissemination by any party other
 * than Jabil Circuit is strictly limited by the confidential information
 * provisions of the Software License Agreement referenced above.
 *
 ******************************************************************************
 *
 * Project      : WaterOptimizer Irrigation System (WOIS)
 * Organization : WaterOptimizer, LLC
 * Module       : drvRadio.c
 * Description  : This file implements the radio serial port driver.
 *
 *****************************************************************************/

/* MODULE drvRadio */

#include <stdlib.h>
#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "drv.h"
#include "drvCts.h"
#include "hwExpIn.h"
#include "hwRadioDtr.h"
#include "hwRadioReset.h"
#include "hwRadioRts.h"
#include "platform.h"

#include "drvRadio.h"


/*
 * The largest packet that can be sent from the radio module is the ZigBee
 * Explicit Rx Indicator:
 *   Max Bytes  Field Description
 *   ---------  ----------------------------------------------
 *        1     Framing character
 *        2     Frame length
 *        1     Application ID (frame command/indication code)
 *        8     Source MAC address
 *        2     Source network address
 *        1     Source endpoint
 *        1     Destination endpoint
 *        2     Cluster ID
 *        2     Profile ID
 *        1     Options
 *       72     RF data (application payload)
 *        1     Frame CRC
 */
#define DRV_RADIO_RX_MAX    (1 + 2 + ((1 + 8 + 2 + 1 + 1 + 2 + 2 + 1) + 72) + 1)

/* circular receive and transmit buffer sizes */
#define DRV_RADIO_RX_BUF_SIZE   512     /* receive queue size */
#define DRV_RADIO_TX_BUF_SIZE   256     /* transmit queue size */

#define MAX_WAIT 2000          /* time out from AT command */

/*
 * receive buffer size thresholds for asserting/deasserting RTS flow control:
 *   - Block transfer when buffer fills to within 8 bytes of being full.
 *     (This allows for two bytes in the micro's receiver and two bytes in the
 *     radio's transmitter, times two for safety margin.)
 *   - Unblock transfer when buffer drop to 50% full.  (arbitrary choice)
 */
#define DRV_RADIO_RX_THRESH_HI  ((DRV_RADIO_RX_BUF_SIZE - 1) - 8)  /* RTS off */
#define DRV_RADIO_RX_THRESH_LO  ((DRV_RADIO_RX_BUF_SIZE - 1) / 2)  /* RTS on */

#if DRV_RADIO_RX_THRESH_LO >= DRV_RADIO_RX_THRESH_HI
#error "RTS-on threshold is not below RTS-off threshold!"
#endif

#if DRV_RADIO_RX_THRESH_LO <= DRV_RADIO_RX_MAX
#error "RTS-on threshold is below maximum packet size - may cause deadlock!"
#endif


/* Circular buffer for radio receive */
uint8_t  drvRadioRxBuf[DRV_RADIO_RX_BUF_SIZE];
static uint16_t drvRadioRxInsert = 0;
static uint16_t drvRadioRxRemove = 0;

/* Circular buffer for radio transmit */
uint8_t  drvRadioTxBuf[DRV_RADIO_TX_BUF_SIZE];
static uint16_t drvRadioTxInsert = 0;
static uint16_t drvRadioTxRemove = 0;

/* TX/RX state flags */
static bool_t drvRadioTxActive = FALSE;
static bool_t drvRadioRtsState;

/* Statistics */
uint32_t drvRadioStatRxBytes        = 0;
uint32_t drvRadioStatRxFrames       = 0;
uint32_t drvRadioStatRxBufOverflow  = 0;
uint32_t drvRadioStatRxBadChecksums = 0;
uint32_t drvRadioStatTxBytes        = 0;
uint32_t drvRadioStatTxFrames       = 0;
uint32_t drvRadioStatTxBufOverflow  = 0;


static inline void drvRadioRts(bool_t state);

static bool_t  drvRadioReadFrameSync(void);

static void    drvRadioRxEnqueue(uint8_t byte);
static bool_t  drvRadioRxDequeue(uint8_t *pByte);
bool_t  drvRadioRxPeek(uint8_t *pByte, uint16_t offset);
static int     drvRadioRxBufCount(void);

void    drvRadioTxEnqueue(uint8_t byte);
static bool_t  drvRadioTxDequeue(uint8_t *pByte);
static int     drvRadioTxBufSpace(void);
void    drvRadioTxStart(void);


static uint8_t drvRadioCalcChecksum(const uint8_t *pBuf, uint16_t length);


#ifdef WIN32

/* Radio read loop (runs in its own Windows thread) */
DWORD WINAPI ComReaderProc(LPVOID lpV)
{
    while (bComReaderRun)
    {
        int len;
        unsigned char serbuf[1];
        len = ReadSerialString(serbuf, sizeof(serbuf), 100);
        for (int i = 0; i < len; i++)
        {
            drvRadioRxEnqueue(serbuf[i]);
        }
    }
    return 1;
}

#endif /* WIN32 */


/******************************************************************************
 *
 *  drvRadioRestart
 *
 *  DESCRIPTION:
 *      This driver internal function (re)initializes the Radio driver and
 *      module.  It must be called after power-up and again whenever AC power
 *      is restored to (re)initialize the radio module and receive/transmit
 *      buffers.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvRadioRestart(void)
{
    drvRadioDtr(TRUE);
    drvRadioReset(TRUE);
    drvRadioWriteFlush();
    hwCpu_Delay100US(100);              /* 10ms delay */
    drvRadioReadFlush();
    drvRadioReset(FALSE);
    drvRadioRts(TRUE);
}


/******************************************************************************
 *
 *  drvRadioReset
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function sets the state of the radio
 *      reset signal.
 *
 *  PARAMETERS:
 *      state (in) - 0 = reset deasserted, non-zero = reset asserted
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      This routine must be called twice to first assert then deassert the
 *      radio reset signal.  Ideally, the receive and transmit buffers will
 *      be flushed while the reset signal is asserted, with appropriate delays
 *      to allow the UART buffers to clear.
 *
 *****************************************************************************/
void drvRadioReset(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwRadioReset_ClrVal();          /* assert XBee reset (active low) */
    }
    else
    {
        hwRadioReset_SetVal();          /* deassert XBee reset */
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioDtr
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function sets the state of the DTR
 *      signal.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = asserted, FALSE = deasserted
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvRadioDtr(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwRadioDtr_ClrVal();            /* assert DTR (active low) */
    }
    else
    {
        hwRadioDtr_SetVal();            /* deassert DTR */
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioRts
 *
 *  DESCRIPTION:
 *      This function sets the state of the RTS signal.
 *
 *  PARAMETERS:
 *      state (in) - TRUE = asserted, FALSE = deasserted
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
static inline void drvRadioRts(bool_t state)
{
    EnterCritical();                    /* save and disable interrupts */

    if (state)
    {
        hwRadioRts_ClrVal();            /* assert RTS (active low) */
    }
    else
    {
        hwRadioRts_SetVal();            /* deassert RTS */
    }
    drvRadioRtsState = state;

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioCts
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function returns the current state of
 *      the CTS signal from the XBee radio module.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      CTS signal state (TRUE = asserted, FALSE = deasserted or not present)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      This routine is useful at system initialization time to determine
 *      whether the radio module is present.
 *
 *****************************************************************************/
bool_t drvRadioCts(void)
{
    return drvCtsGet(DRV_CTS_XBEE);
}



/*====== Radio receive circular buffer routines =============================*/


/******************************************************************************
 *
 *  drvRadioRead
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function transfers a message from the
 *      receive buffer to the calling application.  It removes the message
 *      framing and checksum bytes.
 *
 *  PARAMETERS:
 *      pBuf (out)  - application buffer to receive radio message
 *      length (in) - application read buffer size (maximum number of bytes to
 *                    copy out; the remaining portion of the messahe, if any,
 *                    is discarded)
 *
 *  RETURNS:
 *      number of bytes copied (may be less than actual message size)
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      access to the radio receive buffer.
 *
 *****************************************************************************/
int16_t drvRadioRead(void *pBuf, uint16_t length)
{
    int16_t count = 0;
    uint8_t len;
    uint8_t data;
    uint8_t checksum;
    uint8_t frame[128];
    int bufcount;
    int16_t result = 0;
    uint8_t *pByte = (uint8_t *)pBuf;

    while (drvRadioReadFrameSync())
    {
        /* Get first byte of length field from buffer. */
        if (!drvRadioRxPeek(&len, 1))
        {
            /* Not enough chars in buffer. */
            return 0;
        }
        /* First length byte should be zero. */
        if (len != 0)
        {
            /* This is not a valid frame. */
            /* Take the delimiter out of the buffer and resync. */
            drvRadioRxDequeue(frame);
            continue;
        }

        /* Get second byte of length field from buffer. */
        if (!drvRadioRxPeek(&len, 2))
        {
            /* Not enough chars in buffer. */
            return 0;
        }
        /* Second length byte should be less than 128. */
        if ((len & 0x80) == 0x80)
        {
            /* This is not a valid frame. */
            /* Take the delimiter out of the buffer and resync. */
            drvRadioRxDequeue(frame);
            continue;
        }

        /* Length appears reasonable. */
        /* Insure enough data in buffer to verify the frame's checksum. */
        /* 4 bytes for 1-byte delimiter, 2-byte length, 1-byte checksum */
        count = len;
        bufcount = drvRadioRxBufCount();
        if (bufcount < (count + 4))
        {
            /* Not enough chars in buffer. */
            return 0;
        }

        /* Copy buffer payload, including checksum. */
        for (int i = 0; i <= count; i++)
        {
            if (!drvRadioRxPeek(&frame[i], i + 3))
            {
                /* Not enough chars in buffer? */
                /* Since buffer count was checked; this should never happen. */
                return 0;
            }
        }

        /* Frame now has payload and checksum. */
        checksum = drvRadioCalcChecksum(frame, len);
        if (frame[len] == checksum)
        {
            /* Return the payload to the caller. */
            /* And take the message out of the buffer. */
            drvRadioRxDequeue(&data);
            drvRadioRxDequeue(&data);
            drvRadioRxDequeue(&data);
            for (int i = 0; i < count; i++)
            {
                /* Do not exceed caller's length parameter. */
                if (i < length)
                {
                    pByte[i] = frame[i];
                }
                drvRadioRxDequeue(&data);
            }
            drvRadioRxDequeue(&data);
            result = (length < count) ? length : count;
            drvRadioStatRxFrames++;
            return result;
        }
        else
        {
            /* This is not a valid frame. */
            drvRadioStatRxBadChecksums++;
            /* Take the delimiter out of the buffer and resync. */
            drvRadioRxDequeue(frame);
        }
    }

    return 0;
}


/******************************************************************************
 *
 *  drvRadioReadFrameSync
 *
 *  DESCRIPTION:
 *      This internal function discards receive characters up to but not
 *      including the next frame delimiter character, and returns an indication
 *      of whether a sync character was found.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      TRUE if a sync character was found
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      access to the radio receive buffer.
 *
 *****************************************************************************/
static bool_t drvRadioReadFrameSync(void)
{
    uint8_t data;
    bool_t rc = FALSE;

    while (drvRadioRxPeek(&data, 0))
    {
        if (data != 0x7E)
        {
            /* Discard from buffer. */
            drvRadioRxDequeue(&data);
        }
        else
        {
            /* Frame delimiter found. */
            rc = TRUE;
            break;
        }
    }

    return rc;
}


/******************************************************************************
 *
 *  drvRadioReadFlush
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function clears the receive buffer.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvRadioReadFlush(void)
{
    EnterCritical();                    /* save and disable interrupts */
    drvRadioRxInsert = 0;
    drvRadioRxRemove = 0;
    drvRadioRts(TRUE);
    ExitCritical();                     /* restore interrupts */

    return;
}


/******************************************************************************
 *
 *  drvRadioRxEnqueue
 *
 *  DESCRIPTION:
 *      This internal function adds a byte to the end of the receive buffer.
 *      The byte is discarded if the buffer is already full.  The RTS flow
 *      control signal is deasserted to throttle the radio if the buffer has
 *      reached its high-water threshold.
 *
 *  PARAMETERS:
 *      byte (in) - data byte to enqueue
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the receive character ISR.
 *
 *****************************************************************************/
static void drvRadioRxEnqueue(uint8_t byte)
{
    uint16_t newInsert;

    /* compute updated insert pointer for overflow check */
    newInsert = drvRadioRxInsert + 1;
    if (newInsert >= sizeof(drvRadioRxBuf))
    {
        newInsert = 0;
    }

    if (newInsert != drvRadioRxRemove)
    {
        /* space available - put data into RX queue */
        drvRadioRxBuf[drvRadioRxInsert] = byte;
        drvRadioRxInsert = newInsert;
        drvRadioStatRxBytes++;
        /* drop RTS if RX queue is nearing full */
        if (drvRadioRtsState &&
            drvRadioRxBufCount() >= DRV_RADIO_RX_THRESH_HI)
        {
            drvRadioRts(FALSE);
        }
    }
    else
    {
        /* receive buffer would overflow - discard data */
        drvRadioStatRxBufOverflow++;
    }
}


/******************************************************************************
 *
 *  drvRadioRxDequeue
 *
 *  DESCRIPTION:
 *      This function removes a byte from the beginning of the receive buffer.
 *      The RTS flow control signal is reasserted to unblock the radio if it
 *      was deasserted and the buffer has reached its low-water threshold.
 *
 *  PARAMETERS:
 *      pByte (out) - data byte dequeued
 *
 *  RETURNS:
 *      TRUE if data was available
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the task-level read routines.
 *
 *****************************************************************************/
static bool_t drvRadioRxDequeue(uint8_t *pByte)
{
    if (drvRadioRxRemove == drvRadioRxInsert)
    {
        /* receive buffer was empty */
        return FALSE;
    }
    else
    {
        /* receive buffer not empty - remove a byte from the buffer */
        *pByte = drvRadioRxBuf[drvRadioRxRemove];
        if (++drvRadioRxRemove >= sizeof(drvRadioRxBuf))
        {
            drvRadioRxRemove = 0;
        }
        /* reassert RTS if RX queue has sufficiently emptied */
        if (!drvRadioRtsState &&
            drvRadioRxBufCount() <= DRV_RADIO_RX_THRESH_LO)
        {
            drvRadioRts(TRUE);
        }
    }

    return TRUE;
}


/******************************************************************************
 *
 *  drvRadioRxPeek
 *
 *  DESCRIPTION:
 *      This function returns a byte from a specified offset from the logical
 *      start of the receive buffer.  The buffer is not altered and RTS flow
 *      control is not affected.
 *
 *  PARAMETERS:
 *      pByte (out) - data byte retrieved
 *      offset      - offset from logical start of buffer (0 = first byte)
 *
 *  RETURNS:
 *      TRUE if data was available (offset not beyond end of buffer contents)
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the task-level read routines.
 *
 *****************************************************************************/
bool_t drvRadioRxPeek(uint8_t *pByte, uint16_t offset)
{
    uint16_t remove = drvRadioRxRemove;

    /* insure the offset location is available in buffer */
    if (offset >= drvRadioRxBufCount())
    {
        return FALSE;
    }
    /* calculate location of desired offset entry */
    remove += offset;
    if (remove >= sizeof(drvRadioRxBuf))
    {
        remove -= sizeof(drvRadioRxBuf);
    }
    *pByte = drvRadioRxBuf[remove];

    return TRUE;
}


/******************************************************************************
 *
 *  drvRadioRxBufCount
 *
 *  DESCRIPTION:
 *      This function returns the number of bytes that are currently in the
 *      receive buffer.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      number of bytes in receive buffer
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the task-level read routines.
 *
 *****************************************************************************/
static int drvRadioRxBufCount(void)
{
    int count = 0;

    if (drvRadioRxInsert >= drvRadioRxRemove)
    {
        count = drvRadioRxInsert - drvRadioRxRemove;
    }
    else
    {
        count = sizeof(drvRadioRxBuf) - drvRadioRxRemove + drvRadioRxInsert;
    }

    return count;
}


/******************************************************************************
 *
 *  drvRadioOnRxChar
 *
 *  DESCRIPTION:
 *      This function processes a receive character interrupt by moving a
 *      received byte from the UART to the receive buffer.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked from the UART receive character ISR.
 *
 *****************************************************************************/
void drvRadioOnRxChar(void)
{
    uint8_t ch;

    /* move all receive chacaters from the UART to the receive buffer */
    while (hwExpIn_GetCharsInRxBuf() != 0)
    {
        if (hwExpIn_RecvChar(&ch) == ERR_OK)
        {
            drvRadioRxEnqueue(ch);
        }
    }
}



/*====== Radio transmit circular buffer routines ============================*/


/******************************************************************************
 *
 *  drvRadioWrite
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function copies a message from the
 *      calling application to the transmit buffer.  It adds the message
 *      framing and checksum bytes.  If there is insufficient space in the
 *      transmit buffer, then nothing is enqueued.
 *
 *  PARAMETERS:
 *      pBuf (in)   - application buffer containing transmit radio message
 *      length (in) - length of transmit message (not including framing)
 *
 *  RETURNS:
 *      TRUE if the message was successfully enqueued for transmission
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      access to the radio transmit buffer.
 *
 *****************************************************************************/
bool_t drvRadioWrite(const void *pBuf, uint16_t length)
{
    const uint8_t *pByte = (const uint8_t *)pBuf;

    if ((1 + 2 + length + 1) > drvRadioTxBufSpace())
    {
        /* insufficient TX queue space */
        return FALSE;
    }
    /* Place the caller's data packet inside a radio API frame. */
    drvRadioTxEnqueue(0x7E);
    drvRadioTxEnqueue(0x00);
    drvRadioTxEnqueue(length & 0xFF);
    for (int i = 0; i < length; i++)
    {
        drvRadioTxEnqueue(pByte[i]);
    }
    drvRadioTxEnqueue(drvRadioCalcChecksum(pByte, length));
    drvRadioTxStart();
    drvRadioStatTxFrames++;

    return TRUE;
}



/******************************************************************************
 *
 *  drvRadioWriteAT
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function puts the radio into AT command
 *      mode and sends commands to put the radio into API mode and set the 
 *      PAN ID.
 *
 *  PARAMETERS:
 *
 *  RETURNS:
 *      TRUE if the message was successfully enqueued for transmission
 *
 *  NOTES:
 *
 *****************************************************************************/
bool_t drvRadioAPIModeInit(void)
{
      uint8_t pBufAPI[]={'A','T','A','P', ' ','1', '\r'};
      uint8_t pBufCC[]={'+','+','+'};
      uint8_t pBufExit[]={'A','T','C','N','\r'};
      uint8_t pBufWR[]={'A','T','W','R','\r'};
      uint8_t pATBufID[]={'A','T','I','D','0','2','3','4','\r'};
      
      //const char * pTest=config.sys.radioPanId;
      uint8_t rx_byte;
      uint16_t waitCount =0;
    
      
    //send at command mode sequence  
    for (int i = 0; i < 3; i++)
    {
      drvRadioTxEnqueue(pBufCC[i]);
    }
    hwCpu_Delay100US(10000);
    drvRadioReadFlush();
    drvRadioTxStart();   
    drvRadioStatTxFrames++;
    
    waitCount = 0;
    //check for ok from radio before proceeding
    while(rx_byte !=79)
    {
      drvRadioRxPeek(&rx_byte, 0);
      hwCpu_Delay100US(100);
      if(waitCount == MAX_WAIT)
      {
        radioStatus = RADIO_STATUS_FAILURE;
        return FALSE;
      }
      waitCount +=1;
    }
    
    
    //send AT command to device to be in api mode=1
    for (int i = 0; i < 7; i++)
    {
      drvRadioTxEnqueue(pBufAPI[i]);
    }
    rx_byte=0;
    drvRadioTxStart();
    drvRadioStatTxFrames++;
    
    waitCount = 0;
    //check for ok from radio before proceeding
    while(rx_byte !=79)
    {
      drvRadioRxPeek(&rx_byte, 3);
      hwCpu_Delay100US(100);
      if(waitCount == MAX_WAIT)
      {
        radioStatus = RADIO_STATUS_FAILURE;
        return FALSE;
      }
      waitCount +=1;
    }
    

   
    //send AT command to device to set PAN ID
    for (int i = 0; i < 9; i++)
    {
      drvRadioTxEnqueue(pATBufID[i]);
    }
    rx_byte=0;
    drvRadioTxStart();
    drvRadioStatTxFrames++;
    
    waitCount = 0;
    //check for ok from radio before proceeding
    while(rx_byte !=79)
    {
      drvRadioRxPeek(&rx_byte, 6);
      hwCpu_Delay100US(100);
      if(waitCount == MAX_WAIT)
      {
        radioStatus = RADIO_STATUS_FAILURE;
        return FALSE;
      }
      waitCount +=1;
    }
    
    
    //send AT command to device to write value to non-volatile
   /* for (int i = 0; i < 5; i++)
    {
      drvRadioTxEnqueue(pBufWR[i]);
    }
    rx_byte=0;
    drvRadioTxStart();
    drvRadioStatTxFrames++;
    
    //check for ok from radio before proceeding
    while(rx_byte !=79)
    {
      drvRadioRxPeek(&rx_byte, 9);
      hwCpu_Delay100US(100);
    }
         */
    //send AT command to device to exit AT command mode
    for (int i = 0; i < 5; i++)
    {
      drvRadioTxEnqueue(pBufExit[i]);
    }
    rx_byte=0;
    drvRadioTxStart();
    drvRadioStatTxFrames++;
    
    waitCount = 0;
    //check for ok from radio before proceeding
    while(rx_byte !=79)
    {
      drvRadioRxPeek(&rx_byte, 6);
      hwCpu_Delay100US(100);
      if(waitCount == MAX_WAIT)
      {
        radioStatus = RADIO_STATUS_FAILURE;
        return FALSE;
      }
      waitCount +=1;
    }  
    
    
    drvRadioReadFlush();
    drvRadioWriteFlush();
    hwCpu_Delay100US(1000);
    return TRUE;
}

/******************************************************************************
 *
 *  drvRadioWriteFlush
 *
 *  DESCRIPTION:
 *      This radio serial port driver API function clears the transmit buffer.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvRadioWriteFlush(void)
{
    EnterCritical();                    /* save and disable interrupts */
    drvRadioTxInsert = 0;
    drvRadioTxRemove = 0;
    ExitCritical();                     /* restore interrupts */

    return;
}


/******************************************************************************
 *
 *  drvRadioTxEnqueue
 *
 *  DESCRIPTION:
 *      This function adds a byte to the end of the transmit buffer.  The byte
 *      is discarded if the buffer is already full.  The caller is responsible
 *      for (re)starting transmit operation as necessary (e.g. adding a byte to
 *      an empty buffer when the UART transmitter is idle and CTS is true).
 *
 *  PARAMETERS:
 *      byte (in) - data byte to enqueue
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the task-level write routines.
 *
 *****************************************************************************/
void drvRadioTxEnqueue(uint8_t byte)
{
    uint16_t newInsert;

    /* compute updated insert pointer for overflow check */
    newInsert = drvRadioTxInsert + 1;
    if (newInsert >= sizeof(drvRadioTxBuf))
    {
        newInsert = 0;
    }

    if (newInsert != drvRadioTxRemove)
    {
        /* space available - put data into TX queue */
        drvRadioTxBuf[drvRadioTxInsert] = byte;
        drvRadioTxInsert = newInsert;
    }
    else
    {
        /* transmit buffer would overflow - discard data */
        drvRadioStatTxBufOverflow++;
    }

    /* NOTE: caller may need to start TX operation */
}


/******************************************************************************
 *
 *  drvRadioTxDequeue
 *
 *  DESCRIPTION:
 *      This function removes a byte from the beginning of the transmit buffer.
 *
 *  PARAMETERS:
 *      pByte (out) - data byte dequeued
 *
 *  RETURNS:
 *      TRUE if data was available
 *
 *  NOTES:
 *      This can be invoked from any context.
 *      In practice, it is used only from the transmit character and CTS
 *      asserted ISRs.  These operate at different interrupt levels, so
 *      critical-region protection is necessary.
 *
 *****************************************************************************/
static bool_t drvRadioTxDequeue(uint8_t *pByte)
{
    bool_t rc;

    EnterCritical();                    /* save and disable interrupts */

    if (drvRadioTxRemove == drvRadioTxInsert)
    {
        /* transmit buffer was empty */
        rc = FALSE;
    }
    else
    {
        /* transmit buffer not empty - remove a byte from the buffer */
        *pByte = drvRadioTxBuf[drvRadioTxRemove];
        if (++drvRadioTxRemove >= sizeof(drvRadioTxBuf))
        {
            drvRadioTxRemove = 0;
        }
        drvRadioStatTxBytes++;
        rc = TRUE;
    }

    ExitCritical();                     /* restore interrupts */

    return rc;
}


/******************************************************************************
 *
 *  drvRadioTxBufSpace
 *
 *  DESCRIPTION:
 *      This internal function returns the number of bytes that could be added
 *      to the transmit buffer before it would become full.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      number of bytes of space remaining in the transmit buffer
 *
 *  NOTES:
 *      This can be invoked from any context, but it is NOT reentrant.
 *      In practice, it is used only from the task-level write routines.
 *
 *****************************************************************************/
static int drvRadioTxBufSpace(void)
{
    int count;

    if (drvRadioTxInsert < drvRadioTxRemove)
    {
        count = drvRadioTxRemove - drvRadioTxInsert - 1;
    }
    else
    {
        count = sizeof(drvRadioTxBuf) - 1 - drvRadioTxInsert + drvRadioTxRemove;
    }

    return count;
}


/******************************************************************************
 *
 *  drvRadioTxStart
 *
 *  DESCRIPTION:
 *      This function starts transmission if the UART transmitter is not
 *      already active.  It does this by "faking" a transmit interrupt.
 *      The CTS flow control logic is managed by the interrupt service routine.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *      In practice, it is used only from the task-level write routines.
 *
 *****************************************************************************/
void drvRadioTxStart(void)
{
    EnterCritical();                    /* save and disable interrupts */

    if (!drvRadioTxActive)
    {
        drvRadioTxActive = TRUE;
        drvRadioOnTxChar();
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioOnTxChar
 *
 *  DESCRIPTION:
 *      This radio serial port driver internal function processes a transmit
 *      character interrupt by moving the next transmit byte from the transmit
 *      buffer to the UART, if the radio has not blocked transmission
 *      (deasserted CTS) and if the buffer is not empty.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked from the UART transmit character ISR.  It is also used
 *      from the task-level write routines to initiate transmission when a
 *      message is added to an empty buffer.
 *
 *      Critical region protection is used to guard against interaction between
 *      the UART and CTS edge-detection interrupts that operate at different
 *      interrupt levels.
 *
 *****************************************************************************/
void drvRadioOnTxChar(void)
{
    uint8_t ch;

    EnterCritical();                    /* save and disable interrupts */

    if (!drvCtsGet(DRV_CTS_XBEE))
    {
        /* radio has dropped CTS - stop transmission until it reasserts */
        drvCtsIntrEnable(DRV_CTS_XBEE); /* enable CTS edge interrupt */
        drvRadioTxActive = FALSE;
    }
    else if (drvRadioTxDequeue(&ch))
    {
        /* flow control enabled and data available - output the next byte */
        hwExpIn_SendChar(ch);
    }
    else
    {
        /* no data available - mark the transmitter as idle */
        drvRadioTxActive = FALSE;
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioOnCtsAsserted
 *
 *  DESCRIPTION:
 *      This radio serial port driver internal function processes the detection
 *      of the radio's CTS signal going from FALSE to TRUE, signalling that
 *      data transmission to the radio can resume.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked from the CTS driver from its edge-detection ISR.
 *
 *      Critical region protection is used to guard against interaction between
 *      the UART and CTS edge-detection interrupts that operate at different
 *      interrupt levels.
 *
 *****************************************************************************/
void drvRadioOnCtsAsserted(void)
{
    uint8_t ch;

    EnterCritical();                    /* save and disable interrupts */

    if (!drvRadioTxActive && drvRadioTxDequeue(&ch))
    {
        /* restart transmit operation after CTS went true */
        drvRadioTxActive = TRUE;
        hwExpIn_SendChar(ch);
    }

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvRadioCalcChecksum
 *
 *  DESCRIPTION:
 *      This radio serial port driver utility computes the radio serial
 *      protocol checksum over a specified buffer.
 *
 *  PARAMETERS:
 *      pBuf (in)   - buffer over which to compute the checksum
 *      length (in) - number of bytes in the buffer
 *
 *  RETURNS:
 *      8-bit buffer checksum
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
static uint8_t drvRadioCalcChecksum(const uint8_t *pBuf, uint16_t length)
{
    uint32_t checksum = 0;

    for (int i = 0; i < length; i++)
    {
        checksum += pBuf[i];
    }
    checksum &= 0x000000FF;

    return (0x000000FF - checksum) & 0xFF;
}





/* END drvRadio */
