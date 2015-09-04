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
 * Module       : radio.c
 * Description  : This file implements the radio logic.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include <stdio.h>
#include "stdafx.h"

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "datetime.h"
#include "irrigation.h"
#include "radio.h"
#include "crc.h"
#include "drvRadio.h"
#include "drvEeprom.h"
#include "ui.h"
#include "drvKeypad.h"
#include "drvLcd.h"
#include "hwCpu.h"
#include "extFlash.h"
#include "drvSys.h"
#include "drvExtFlash.h"
#include "ui.h"
#include "moisture.h"
#include "drvMoist.h"
#include "drvSolenoid.h"

/* Radio Events */
#define RADIO_EVENT_INIT        SYS_EVENT_RADIO + 1     /* Radio Init */
#define RADIO_EVENT_RESET_CMD   SYS_EVENT_RADIO + 2     /* Reset Command */
#define RADIO_EVENT_MODEM_RESET SYS_EVENT_RADIO + 3     /* Modem Reset Notif */
#define RADIO_EVENT_FACTORY_RST SYS_EVENT_RADIO + 4     /* Factory Reset */
#define RADIO_EVENT_HW_RESET    SYS_EVENT_RADIO + 5     /* Hardware Reset */



/******************************************************************************
 *
 *  COMMAND PENDING QUEUE DEFINITIONS
 *
 *  This command pending queue is used by the AT Command send/retry logic.
 *
 *  NOTE:
 *      The command pending queue is implemented as double-linked list, using
 *      a fixed-size array of queue elements (radioCmdPending_t structures).
 *
 *****************************************************************************/

#define RADIO_CPQ_SIZE          16          /* max number of queue elements */
#define RADIO_CPQ_NONE          0xFF        /* queue element does not exist */
#define RADIO_CPQ_AUTOSEND      0xFFFFFFFF  /* cmd enqueued for auto-send */

//#define RADIO_NHOPS             7          //maximum number of XBee hops

/*
**  Command Pending Queue Element Structure
*/
typedef struct
{
    bool_t inUse;           /* queue element in-use indicator */
    uint8_t prev;           /* previous queue element index */
    uint8_t next;           /* next queue element index */
    uint8_t ident;          /* command identifier */
    uint32_t timeSent;      /* retry info - time cmd sent or auto-send */
} radioCmdPending_t;


/*
** Bootloader Control Structure
*/
typedef struct
{
	dword	Magic;
	dword	CRC;
	dword	ImageLengthBytes;
	byte	UpdateIntFlashFromSPIFlash;
	byte	FirstBoot;
	byte	RunAttempts;
}FW_IMAGE_INFO;

/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/
uint64_t radioMacId;                    /* radio physical address (MAC ID) */
uint32_t radioEtAccumulator = RADIO_ET_INIT;        /* ET Data Accumulator */
uint32_t radioRainAccumulator = RADIO_RAIN_INIT;    /* Rainfall Accumulator */

uint32_t radioStatusAiTime;             /* time ticks on last AI command */
uint32_t radioInitTime;                 /* time ticks since radio init */
uint32_t radioLBTime;                   /* time ticks since last loopback attempt */
uint32_t radioMasterRespTime;           /* time ticks since sent master unit a message */
uint32_t radioExp1RespTime;             /* time ticks since sent expansion 1 a message */
uint32_t radioExp2RespTime;             /* time ticks since sent expansion 2 a message */
uint32_t radioExp3RespTime;             /* time ticks since sent expansion 3 a message */
uint32_t radioCheckExpanStatusTime;     /* time ticks since check radio comm with expansion unit that is irrigating*/
uint32_t radioSnsConCheckinTime;        /* time ticks since sensor concentrator that is suppose to be
                                         * turning irrigation ON or OFF has checked in last
                                         */

bool_t radioSentMasterMsg = FALSE;      /* value to keep track if sent master unit a message */
bool_t radioSentExpan1Msg = FALSE;      /* value to keep track if sent expan 1 unit a message */
bool_t radioSentExpan2Msg = FALSE;      /* value to keep track if sent expan 2 unit a message */
bool_t radioSentExpan3Msg = FALSE;      /* value to keep track if sent expan 3 unit a message */
bool_t radioStartEarlyCmd = FALSE;      /* value to denote if received a message to start irrigation early */

uint8_t dtflag = 0;
uint16_t dtid = 0;



uint16_t temp;
bool_t radioCmdIrrStart = FALSE;        /*denotes that was told to start early */
uint32_t radioSCCheckinTime;            /* time ticks since sent SC sent a status message back */

#ifdef RADIO_ZB
uint64_t radioStatusOp;                 /* last OP command response */
#else
uint16_t radioStatusOp;                 /* last OP command response */
#endif
uint32_t radioStatusVr;                 /* radio firmware version */
uint16_t radioStatusHv;                 /* radio hardware version */
uint8_t radioStatus;                    /* radio status */
uint8_t radioStatusDb;                  /* last received signal strength */
bool_t radioIsJoined;                   /* radio is joined in PAN if TRUE */
bool_t radioYield;                      /* yield poll cycle if TRUE */
uint8_t radioDataFrameIdNext;           /* next data frame ID to use */
uint8_t radioTxDataRetriesRemaining;    /* last data frame retries remaining */
char radioLastMsgDesc[25];              /* last message received description */
char radioLastMsgDate[21];              /* last message received date/time */
uint8_t radioLbState;                   /* loopback test state */
bool_t radioDebug = FALSE;              /* generate debug messages if TRUE */
bool_t radioMonitorRss = TRUE;          /* monitor recv sig strength if TRUE */
uint32_t newFirmwareSize=0;             /* size of new firmware to be downloaded */
extern uint8_t  drvRadioRxBuf[];
extern uint8_t  drvRadioTxBuf[];
uint8_t numLoopbackAttempts =0;
uint32_t magicNum;
uint32_t nBytes;
uint16_t segmentIndex;
uint8_t ImageData[BULK_FW_HDR_SIZE];
FW_IMAGE_INFO ImageInfo;                        /* structure that holds bootloader control values */
sensorConMeasure_t snsConInfo[MAX_NUM_SC];      /* structure to hold rx measurement values from sns con */
bool_t newSensorConcenFound = FALSE;            /* denotes if there is a new sns con to deal with */
bool_t associateSCMode = FALSE;                 /* denotes if in associate mode */
uint64_t unassociatedSnsConMacId;               /* value to store most recent sensor concentrator that tried to associate */
int8_t irrSnsConSolUnitIndex = IRR_SNS_SOL_NONE;/* index into the sensor concentrator list that needs to have solenoid state changed */
uint8_t irrSnsConSolenoidChan;                  /* sensor concentrator solenoid channel to turn on. */
bool_t scCheckedIn= FALSE;                      /* value denoting sc checked in so do put system back into pause */
bool_t scDeleteList[MAX_NUM_SC];                 /* list of SC index to de-associate, FALSE= dont delete, TRUE=delete */
uint8_t scDeleteIndex;                          /* SC index to delete */

//#define TEST_RST  //Test the init retry code
#ifdef TEST_RST
#pragma message ("init retry test!")
uint8_t InitCount = 0;
#endif

/*
**  Radio Command Pending Table
**  Data store used for the AT Command send and retry logic.
*/
uint8_t radioCmdPendingFirst;           /* index of first (oldest) member */
uint8_t radioCmdPendingLast;            /* index of last (newest) member */
radioCmdPending_t radioCmdPending[RADIO_CPQ_SIZE];  /* queue data store */

/*
**  ZigBee Transmit Data Message Buffer
**  Data store used for the last WOIS Command Response transmit packet.
*/
radioTxDataPacket_t radioTxDataPkt;     /* last Tx data packet buffer */
uint16_t radioTxDataPktLen;             /* last Tx data packet length */
uint32_t radioTxDataTime;               /* last Tx data send tick count */

uint8_t expMoistValue[36];
//uint8_t assocflag = 0;
//uint8_t assocack = 0;
//uint16_t statusflag = 0;
//uint16_t statusack = 0;

// for reset logic
bool_t  assocFlag[12];
uint8_t assocCnt[12];



/*
**  LCD_RC - MACRO TO CALCULATE LCD BUFFER OFFSET
*/
#define LCD_RC(row,col)       DRV_LCD_CURSOR_RC((row),(col))

/******************************************************************************
 *
 *  PRIVATE RADIO FUNCTION PROTOTYPES (ONLY FOR USE BY RADIO SUBSYSTEM)
 *
 *****************************************************************************/

static bool_t  radioCommandEnqueue(uint8_t cmdIdent);
static bool_t  radioCommandImmediate(uint8_t cmd);
static bool_t  radioCmdRetryManager(void);
static void    radioCmdPendingInit(void);
static bool_t  radioCmdPendingAdd(uint8_t ident, bool_t send);
static void    radioCmdPendingDelete(uint8_t ident);
static bool_t  radioCmdPendingGet(uint8_t *pIdent);
static bool_t  radioCommandNetworkReset(void);
static bool_t  radioCommandPanIdSet(void);
static bool_t  radioCommandNodeIdSet(void);
static bool_t  radioCommandP0Disable(void);
static bool_t  radioCommandD0Disable(void);
//static bool_t  radioCommandCE(void);
static bool_t  radioCommandD6RtsSet(void);
static void    radioStatusUpdate(void);
static void    radioPacketHandler(const uint8_t *pBuf, int16_t length);
static void    radioPacketZigBeeRxData(const uint8_t *pBuf, int16_t length);
static void    radioPacketAtResponse(const uint8_t *pBuf, int16_t length);
static void    radioPacketModemStatus(const uint8_t *pBuf, int16_t length);
static void    radioPacketZigBeeTxStatus(const uint8_t *pBuf, int16_t length);
static void    radioProtocolCommandHandler(const radioRxDataPacket_t *pPacket);
static void    radioProtocolXferHandler(const radioRxDataPacket_t *pPacket);
static void    radioProtocolSCAssocHandler(const radioRxDataPacket_t *pPacket);
static void    radioProtocolSCStatusHandler(const radioRxDataPacket_t *pPacket);
static void    radioProtocolLoopbackHandler(const radioRxDataPacket_t *pPacket);
static void    radioProtoCmdGetWeatherValues(uint8_t *pData);
static uint8_t radioProtoCmdGetMbValues(uint8_t firstZone, uint8_t *pData);
static bool_t  radioProtoCmdSetMbValues(uint8_t *pData, uint8_t lenData);
static void    radioProtocolFillAckDefaults(packetSCAckData_t *pAckPacket);
static void    radioProtocolAckSend(const radioRxDataPacket_t *pPacket,
                                    uint8_t cmdAck,
                                    const uint8_t *pData,
                                    uint8_t lenData);
static void    radioProtocolRespSend(const radioRxDataPacket_t *pPacket,
                                     const void *pMsg,
                                     uint8_t msgType,
                                     uint8_t length);
static bool_t  radioLoopbackDataSend(const uint8_t *pData,
                                     int16_t lenData);
                      
static void    radioExtCommandHandler(const radioRxDataPacket_t *pPacket);
static void    radioExtAckSend(const radioRxDataPacket_t *pPacket,
                               uint8_t cmdAck,
                               const uint8_t *pData,
                               uint8_t lenData);
static void    radioDebugAtResponse(radioResponsePacket_t *pPacket);
static bool_t  radioCheckSumIsValid(const void *pData,
                                    uint8_t lenData,
                                    uint8_t sumH,
                                    uint8_t sumL);
static void    radioMessageLog(const char *pMsgDesc);
static void    radioLoopbackTest(void);
static uint8_t radioCheckSensorAssocList(uint64_t sensorMAC);
//static uint8_t radioAddSensorAssocList(uint64_t sensorMAC);
//static uint8_t radioRemoveSensorAssocList(uint64_t sensorMAC) ;
static void expansionBusForwardCmd(const radioRxDataPacket_t *pPacket);
void expansionBusCmdHandler(const radioRxDataPacket_t *pPacket);
void expansionSendCfgSeg0(const radioRxDataPacket_t *pPacket);
static void expansionBusSingleCmd(const radioRxDataPacket_t *pPacket, uint64_t macId);
static uint8_t ConvertZone(uint8_t bitZone);

/******************************************************************************
 *
 * radioInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization routine
 *      to initialize the radio subsystem.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine must be called at system initialization.  This routine
 *      may also be called at any time to reset the Radio Logic subsystem.
 *
 *****************************************************************************/
void radioInit(void)
{
    sysEvent(RADIO_EVENT_INIT, 0);
    dtDebug("Initializing radio...\n");

    /* Flush radio serial communication buffers. */
    drvRadioWriteFlush();
    drvRadioReadFlush();

    /* Initialize radio command pending list. */
    radioCmdPendingInit();

    /* Initialize radio status to radio not populated. */
    radioStatus = RADIO_STATUS_NOTPOP;

    /* Clear system fault flag for radio failure. */
    sysFaultClear(SYS_FAULT_RADIO);

    /* Initialize joined indicator to radio not joined. */
    radioIsJoined = FALSE;

    /* Initialize next radio Tx Data frame ID. */
    radioDataFrameIdNext = RADIO_DATA_FRAME_ID_FIRST;

    /* Initialize Tx Data retries remaining to zero. */
    radioTxDataRetriesRemaining = 0;

    /* if config PAN ID is 0 then set to default PAN */
    if(config.sys.radioPanId == 0x0000)
     {
        config.sys.radioPanId = htons(PAN_SYS_DEFAULT);
     }
     
    /* Zero out radio status parameters accessed by UI. */
    radioMacId = 0;         /* radio MAC ID */
    radioStatusVr = 0;      /* sw version */
    radioStatusHv = 0;      /* hw version */
    
    /* Initialize radio yield state. */
    radioYield = FALSE;

    /* Initialize loopback test state. */
    radioLbState = RADIO_LBS_IDLE;
}


/******************************************************************************
 *
 * radioPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to
 *      manage the ZigBee radio and handle all communications with the
 *      radio device driver.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine should be called frequently to avoid loss of radio
 *      packets due to Rx buffer overruns.
 *
 *****************************************************************************/
void radioPoll(void)
{
    uint8_t message[RADIO_MAXPACKET];
    int16_t messageLength;

    /* If status is "NOT POPULATED", test for radio presence. */
    if (radioStatus == RADIO_STATUS_NOTPOP)
    {
        /*
        **  Check for CTS asserted from Radio Module.
        **  Note: This is how radio module population is detected.
        */
        if (drvRadioCts())
        {
            /* Radio is present; send init sequence of AT commands. */
            radioCommandInitSequence();
            /* Change radio status to init ("testing"). */
            radioStatus = RADIO_STATUS_INIT;
            /* Save time of init for use in detecting init timeout. */
            radioInitTime = dtTickCount;
            /* Request LCD screen refresh for new radio status). */
            uiLcdRefresh();
            /* End current radio poll cycle. */
            return;
        }
    }
    
    /* If radio is initializing, check for initialization time-out. */
    if (radioStatus == RADIO_STATUS_INIT)
    {
        // If no response received from radio init commands.... 
        if (dtElapsedSeconds(radioInitTime) > RADIO_INIT_FAIL_SECS)
        {     
            /*
            **  Declare radio failure.
            **  Failure may be caused by bad hardware or wrong radio firmware.
            */
            radioStatus = RADIO_STATUS_FAILURE;
            // Set system fault flag for radio failure. 
            sysFaultSet(SYS_FAULT_RADIO);
            
            //reset radio and retry initing it
            //#pragma message ("radio reset!")
            radioResetHw();
            radioInit();
        }
    }        
      
    if ((radioStatus == RADIO_STATUS_NOTPOP) ||
        (radioStatus == RADIO_STATUS_FAILURE))
    {
        /* Nothing to do; end the poll cycle. */
        return;
    }



    /* Get any new message packets received from the radio module. */
    while ((messageLength = drvRadioRead(message, RADIO_MAXPACKET)) != 0)
    {
        /* Process the received packet. */
        radioPacketHandler(message, messageLength);

        /* Check for yield flag set by handler. */
        if (radioYield)
        {
            /* Yield has been requested, clear yield flag. */
            radioYield = FALSE;
            /* End the current radio poll cycle now. */
            return;
        }
    }


    /* Send or retry pending AT Commands (one per poll cycle). */
    if (radioCmdRetryManager())
    {
        /* Radio AT Command has been sent; end the poll cycle. */
        return;
    }

    /* if radio is scaning for gateway check for timeout */
    if(radioStatus == RADIO_STATUS_SCANNING)
    {
        if ((radioLbState == RADIO_LBS_START))
        {
            // Send loopback test packet.
            radioInitTime = dtTickCount;
            radioLBTime = dtTickCount;
            numLoopbackAttempts = 0; 
            radioLoopbackTest();
        } 
       else if((radioLbState == RADIO_LBS_TESTING) && (numLoopbackAttempts < NUM_LOOPBACK_PKT) 
                  && (dtElapsedSeconds(radioLBTime) > (RADIO_INIT_FAIL_SECS/NUM_LOOPBACK_PKT)))
        {
            radioLBTime = dtTickCount;
            radioLoopbackTest();
            numLoopbackAttempts +=1;
            
        }  
        
        // If no response received from radio init commands.... 
        if (dtElapsedSeconds(radioInitTime) > RADIO_INIT_FAIL_SECS)
        {  
            // Declare radio offline.
            radioLoopbackTestCancel();            
        }
    } 

    /* if in irrigate mode and running sensor concentrators check for 
     * timeout of unit that is suppose to turn on valve. Time out value set in 
     * irrSolenoidControl function
     */
    if((irrSnsConSolUnitIndex != IRR_SNS_SOL_NONE) &&
        (dtElapsedSeconds(radioSnsConCheckinTime) > RADIO_SNSCON_FAIL_SECS))
    {
        // set error flag
        sysFaultSet(SYS_FAULT_SNSCON);
    }


    /* check for timeout on message sent on the expansion bus */
    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
        // If no response received from expansion 1 unit 
        if ((dtElapsedSeconds(radioExp1RespTime) > RADIO_EXPAN_FAIL_SECS) &&
            (radioSentExpan1Msg == TRUE))
        {  
            // set error flag
            sysFaultSet(SYS_FAULT_EXPAN_1);
            
            //change units status to not connected
            radioStatusExpansion1 = EXPANSION_NOT_CONNECTED;
            
            //clear control value
            radioSentExpan1Msg = FALSE;
            
            
            if(((config.sys.numUnits == 1) && (irrCurUnitRunning == UNIT_TYPE_EXPANSION_1)) ||
                ((irrCurUnitRunning == UNIT_TYPE_EXPANSION_1) && (sysState != SYS_STATE_AUTORUN)))
            {
                /* release control variable */
                irrExpRunningProg = IRR_PGM_NONE;
                sysState = SYS_STATE_IDLE;
                expansionIrrState =IRR_STATE_IDLE;
                expansionIrrCurZone = 0;
                irrCurUnitRunning = UNIT_TYPE_MASTER;
            }
            
            if(irrCurUnitRunning == UNIT_TYPE_EXPANSION_1)
            {
                irrCurUnitRunning = UNIT_TYPE_EXPANSION_2;
            }
            
            //if unit was irrigating then set system to non pulsed mode
            config.sys.pulseMode = CONFIG_PULSEMODE_OFF;
            irrPulseMode = CONFIG_PULSEMODE_OFF;
                         
        }
        // If no response received from expansion 2 unit 
        if ((dtElapsedSeconds(radioExp2RespTime) > RADIO_EXPAN_FAIL_SECS) &&
            (radioSentExpan2Msg == TRUE))
        {  
            // set error flag
            sysFaultSet(SYS_FAULT_EXPAN_2);
            
            //change units status to not connected
            radioStatusExpansion2 = EXPANSION_NOT_CONNECTED;
            
            //clear control value
            radioSentExpan2Msg = FALSE;
            
            
            if(((config.sys.numUnits == 2)&&(irrCurUnitRunning == UNIT_TYPE_EXPANSION_2)) ||
                ((irrCurUnitRunning == UNIT_TYPE_EXPANSION_2) && (sysState != SYS_STATE_AUTORUN)))
            {
                /* release control variable */
                irrExpRunningProg = IRR_PGM_NONE;
                sysState = SYS_STATE_IDLE;
                expansionIrrState =IRR_STATE_IDLE;
                expansionIrrCurZone = 0;
                irrCurUnitRunning = UNIT_TYPE_MASTER;
            }
            
            if(irrCurUnitRunning == UNIT_TYPE_EXPANSION_2)
            {
                irrCurUnitRunning = UNIT_TYPE_EXPANSION_3;
            }
            
            //if unit was irrigating then set system to non pulsed mode
            config.sys.pulseMode = CONFIG_PULSEMODE_OFF;
            irrPulseMode = CONFIG_PULSEMODE_OFF;
            
        }
        // If no response received from expansion 1 unit 
        if ((dtElapsedSeconds(radioExp3RespTime) > RADIO_EXPAN_FAIL_SECS) &&
            (radioSentExpan3Msg == TRUE))
        {  
            // set error flag
            sysFaultSet(SYS_FAULT_EXPAN_3);
            
            //change units status to not connected
            radioStatusExpansion3 = EXPANSION_NOT_CONNECTED;
            
            //clear control value
            radioSentExpan3Msg = FALSE; 
            

            if(((config.sys.numUnits == 3)&&(irrCurUnitRunning == UNIT_TYPE_EXPANSION_3)) ||
                ((irrCurUnitRunning == UNIT_TYPE_EXPANSION_3) && (sysState != SYS_STATE_AUTORUN)))
            {
                /* release control variable */
                irrExpRunningProg = IRR_PGM_NONE;
                sysState = SYS_STATE_IDLE;
                expansionIrrState =IRR_STATE_IDLE;
                expansionIrrCurZone = 0;
                irrCurUnitRunning = UNIT_TYPE_MASTER;
            } 
            
            //if unit was irrigating then set system to non pulsed mode
            config.sys.pulseMode = CONFIG_PULSEMODE_OFF;
            irrPulseMode = CONFIG_PULSEMODE_OFF;
                        
        }
        
        // if expansion unit is irrigating then check communications every so often 
        if((irrCurUnitRunning != UNIT_TYPE_MASTER)&&
            (dtElapsedSeconds(radioCheckExpanStatusTime) > RADIO_EXPAN_STATUS_INTERVAL))
        {        
           switch(irrCurUnitRunning)
           {
                case UNIT_TYPE_EXPANSION_1:
                    expansionBusSendCmd(RADIO_CMD_NO_OP,config.sys.expMac1);
                    break;
                case UNIT_TYPE_EXPANSION_2:
                    expansionBusSendCmd(RADIO_CMD_NO_OP,config.sys.expMac2);
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    expansionBusSendCmd(RADIO_CMD_NO_OP,config.sys.expMac3);
                    break;
           }
           radioCheckExpanStatusTime = dtTickCount;
        }
    } 
    else
    {
        // If no response received from master unit 
        if ((dtElapsedSeconds(radioMasterRespTime) > RADIO_EXPAN_FAIL_SECS) &&
            (radioSentMasterMsg == TRUE))
        {  
            // set error flag
            sysFaultSet(SYS_FAULT_EXPAN_MASTER);
            
            //clear control value
            radioSentMasterMsg = FALSE;
            
            //turn pulse mode off if enabled
            config.sys.pulseMode = CONFIG_PULSEMODE_OFF;            
        }
    }
    
    /* If status is "ONLINE", check for Loopback Test start request. */
    if (radioStatus == RADIO_STATUS_ONLINE)
    {
        if (radioLbState == RADIO_LBS_START)
        {
            // Send loopback test packet. 
            radioLoopbackTest();
        }
    }   
}


/******************************************************************************
 *
 * radioReset
 *
 * PURPOSE
 *      This routine is called to reset the RF Module using different means,
 *      depending on the current radio state.  If the radio state indicates
 *      that the XBee RF module is responding satisfactorily to AT commands,
 *      then a network node reset command will be sent; this forces the XBee
 *      RF module to drop from any current PAN and then attempt to associate
 *      with a valid PAN.
 *
 *      In abnormal conditions, when the radio state indicates the XBee module
 *      is not responding to AT commands, this routine re-initializes the WOIS
 *      radio application subsystem; discovery and initialization of the XBee
 *      RF module are then re-attempted.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioReset(void)
{
    /* Trace the radio reset event. */
    sysEvent(RADIO_EVENT_RESET_CMD, radioStatus);

    switch (radioStatus)
    {
        case RADIO_STATUS_ONLINE:
            /* Send a network reset command (network node reset). */
            radioInit();
            radioCommandEnqueue(RADIO_CMD_FR);
            break;
        case RADIO_STATUS_OFFLINE:
        case RADIO_STATUS_SCANNING:
            /* Send a network reset command (network node reset). */
            radioInit();
            radioCommandEnqueue(RADIO_CMD_FR);
            break;

        case RADIO_STATUS_NOTPOP:
        case RADIO_STATUS_FAILURE:
        case RADIO_STATUS_INIT:
        default:
            /* Reinitialize the radio application subsystem. */
            radioInit();
            break;
    }
}


/******************************************************************************
 *
 * radioResetHw
 *
 * PURPOSE
 *      This routine is called to perform a hardware reset of the RF Module.
 *      In addition to the hardware reset, the radio application subsystem is
 *      also re-initialized and the RF Module is reset to its factory default
 *      configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The radio application subsystem re-initialization includes flushing
 *      radio driver's read and write buffers.  The radioInit function is
 *      called between the assert/deassert of the reset pin adding extra delay
 *      to insure the reset pin is asserted for a sufficient amount of time.
 *      (Current RF module specification requires the reset pin to be asserted
 *      for at least 200ns.)
 *
 *****************************************************************************/
void radioResetHw(void)
{
    /* Trace the radio hardware reset event. */
    sysEvent(RADIO_EVENT_HW_RESET, radioStatus);

    /* Assert the RF module reset pin. */
    drvRadioReset(TRUE);
    /* Initialize the radio application subsystem. */
    //radioInit();
    /* Enqueue an XBee factory reset AT command. */
    radioCommandEnqueue(RADIO_CMD_RE);
    /* De-assert the RF module reset pin. */
    drvRadioReset(FALSE);
    
    config.sys.radioPanId = htons(PAN_SYS_DEFAULT);
    
    hwCpu_Delay100US(5000);
    //radioCommandInitSequence();
    dtDebug("Radio Hardware Reset Completed\n");
}


/******************************************************************************
 *
 * radioConfigReset
 *
 * PURPOSE
 *      This routine resets the RF Module to its factory default configuration.
 *      The radio logic follows-up by setting correct non-default parameters.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioConfigReset(void)
{
    /* Enqueue an XBee factory reset AT command. */
    radioCommandEnqueue(RADIO_CMD_RE);
}


/******************************************************************************
 *
 * radioPanIdSet
 *
 * PURPOSE
 *      This routine is called to re-configure the RF module to use the PAN ID
 *      stored in the WaterOptimizer's configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioPanIdSet(void)
{
    switch (radioStatus)
    {
        case RADIO_STATUS_NOTPOP:
        case RADIO_STATUS_FAILURE:
            break;
        default:
            radioStatus = RADIO_STATUS_SCANNING;
            radioLbState = RADIO_LBS_START;
            radioCommandPanIdSet();
            break;
    }
}


/******************************************************************************
 *
 * radioStatusUpdate
 *
 * PURPOSE
 *      This routine is called to update the radio status.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioStatusUpdate(void)
{
    bool_t initComplete;            /* Radio is initialized if TRUE. */

    /* If radio has failed, this routine will not update the status. */
    if (radioStatus == RADIO_STATUS_FAILURE)
    {
        return;
    }

    /* Start with assumption that initialization is complete. */
//#pragma message ("Never inits")
    initComplete = TRUE;

    /* If radio just starting up, verify that initialization is complete. */
    if ((radioStatus == RADIO_STATUS_NOTPOP) ||
        (radioStatus == RADIO_STATUS_INIT))
    {
        /* Test for non-zero software version. */
        if (radioStatusVr == 0)
        {
            /* Initialization is not complete. */
            initComplete = FALSE;
        }
        /* Test for non-zero hardware version. */
        else if (radioStatusHv == 0)
        {
            /* Initialization is not complete. */
            initComplete = FALSE;
        }
        /* Test for non-zero MAC ID. */
        else if (radioMacId == 0)
        {
            /* Initialization is not complete. */
            initComplete = FALSE;
        }
    }

    /* Only update status if initialization complete. */
    if (initComplete && (radioStatus == RADIO_STATUS_INIT))
    {
        radioStatus = RADIO_STATUS_SCANNING;
        radioLbState = RADIO_LBS_START; 
        
        /*set the bootloader control variables */
        drvExtFlashRead(EXT_FLASH_SEC_0,&ImageInfo,sizeof(ImageInfo));
    
        if((ImageInfo.FirstBoot == TRUE)|(ImageInfo.RunAttempts > 4))
        {      
            ImageInfo.UpdateIntFlashFromSPIFlash = FALSE;
            ImageInfo.FirstBoot = FALSE;
            ImageInfo.RunAttempts =0;
            drvExtFlashWrite((char *)&ImageInfo,EXT_FLASH_SEC_0,sizeof(FW_IMAGE_INFO));
        }

        /* if master then send new config to expansion units */
        if(config.sys.unitType == UNIT_TYPE_MASTER) 
        {
            expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, RADIO_EXP_SEND_ALL);     
        }
        else 
        {
            if(config.sys.masterMac != 0) 
            {              
                expansionBusSendCmd(RADIO_CMD_NO_OP, config.sys.masterMac);
                expansionBusSendCmd(RADIO_CMD_EXPANSION_STATUS, config.sys.masterMac);
                expansionBusSendCmd(RADIO_CMD_EXPAN_GET_CONFIG, config.sys.masterMac);
            }
        }

    }
    
}


/******************************************************************************
 *
 * radioCmdRetryManager
 *
 * PURPOSE
 *      This routine manages the auto-send/retry logic for radio AT commands,
 *      sending the next eligible command found in the command pending list.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if a command retry was initiated, otherwise
 *      FALSE is returned.
 *
 *****************************************************************************/
static bool_t radioCmdRetryManager(void)
{
    uint8_t cmdIdent;
    bool_t retry = FALSE;

    if (radioCmdPendingGet(&cmdIdent))
    {
        radioCommandImmediate(cmdIdent);
        retry = TRUE;
    }

    return retry;
}


/******************************************************************************
 *
 * radioPacketHandler
 *
 * PURPOSE
 *      This routine handles received radio packets, processing according
 *      to their API frame identifier field.
 *
 * PARAMETERS
 *      pBuf        IN  pointer to start of packet data
 *      length      IN  length of the packet data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioPacketHandler(const uint8_t *pBuf, int16_t length)
{
    radioRxRemCmdPacket_t *pp = (radioRxRemCmdPacket_t *)pBuf;
    
    /*
    **  Handle received packet according to the API packet type identifier.
    **  (API packet type is always indicated in the first byte.)
    */
    switch (pBuf[0])
    {
        case RADIO_API_RXDATA:
            /* Handle ZigBee RX Data from network. */
            radioPacketZigBeeRxData(pBuf, length);
            break;

        case RADIO_API_RESPONSE:
            /* Handle AT command response from radio module. */
            radioPacketAtResponse(pBuf, length);
            break;

        case RADIO_API_STATUS:
            /* Handle unsolicited modem status from radio module. */
            radioPacketModemStatus(pBuf, length);
            break;

        case RADIO_API_TXSTAT:
            /* Handle ZigBee TX Data transmit status from radio module. */
            radioPacketZigBeeTxStatus(pBuf, length);
            break;

        case RADIO_API_NODE_IDENT:
            /* Received node identification indicator packet. */
            if (radioDebug)
            {
                debugWrite("Received Node Identification Indicator:\n");
                debugHexWrite(pBuf, length, TRUE);
            }
            break;

  
       /* case RADIO_API_REM_RESPONSE:
            // Handle AT command response from radio module. Used for Loopback//
            
            radioProtocolLoopbackHandlerRemote(pp);
            break; */
         

        default:
            /* Ignore unknown API packet types. */
            if (radioDebug)
            {
                debugWrite("Received unknown API packet type:\n");
                debugHexWrite(pBuf, length, TRUE);
            }
            break;
    }
}


/******************************************************************************
 *
 * radioPacketZigBeeRxData
 *
 * PURPOSE
 *      This routine handles received ZigBee Rx Data packets.
 *
 * PARAMETERS
 *      pBuf        IN  pointer to start of packet data
 *      length      IN  length of the packet data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioPacketZigBeeRxData(const uint8_t *pBuf, int16_t length)
{
    radioRxDataPacket_t *pp = (radioRxDataPacket_t *)pBuf;
    radioMsgHeader_t *hdr = (radioMsgHeader_t *)&pp->data[0];
   
    /*
    **  Generate any required debug output.
    */
    if (radioDebug)
    {
        debugWrite("Received ZigBee Data Packet.  API header:  ");
        debugHexWrite(pBuf, woffsetof(radioRxDataPacket_t, data), FALSE);
        debugHexWrite(pp->data, length - woffsetof(radioRxDataPacket_t, data), TRUE);
    }

    /*
    **  Handle Rx Data packet according to WOIS radio protocol message type.
    */
    if (hdr->version == RADIO_PROTOCOL_VER)
    {
        //set radio status to online since received a message over the air
        radioStatus = RADIO_STATUS_ONLINE;
        
        switch (hdr->msgType)
        {
            case RADIO_TYPE_CMD:
                /* Handle WOIS radio command packet. */
                radioProtocolCommandHandler(pp);
                break;


            case RADIO_TYPE_ACK:
                /* Handle WOIS radio command acknowledgement packet. */
                expansionBusCmdHandler(pp);
                break;


            case RADIO_TYPE_XFER:
                /* Handle WOIS bulk data transfer packet. */
                radioProtocolXferHandler(pp);
                break;

            case RADIO_TYPE_LOOPBACK:
                /* Handle loopback reply packet. */
                
                radioProtocolLoopbackHandler(pp);
                break;
            case RADIO_TYPE_SC_ASSOC:
                /* Handle Sensor Concentrator Associate Message. */
                radioProtocolSCAssocHandler(pp);
                
                break;
            case RADIO_TYPE_SC_STATUS :
                /* Handle Sensor Concentrator Status Message */
                radioProtocolSCStatusHandler(pp);
                break;
     /****** For Initial Test Only -- REMOVE  *******/
          case 6:
                 //get the source address
                 uint64_t sensorMacID = U8TOU64(pp->phyAddr[0],
                             pp->phyAddr[1],
                             pp->phyAddr[2],
                             pp->phyAddr[3],
                             pp->phyAddr[4],
                             pp->phyAddr[5],
                             pp->phyAddr[6],
                             pp->phyAddr[7]);
                radioRemoveSensorAssocHandler(sensorMacID);
            default:
                /* Ignore unknown WOIS radio protocol message type. */
                if (radioDebug)
                {
                    debugWrite("Received unknown WOIS message type:\n");
                    debugHexWrite(pBuf, length, TRUE);
                }
                break;
        }
    }

    if (radioMonitorRss)
    {
        /* Get received signal strength. */
        radioCommandEnqueue(RADIO_CMD_DB);
    }
}


/******************************************************************************
 *
 * radioPacketAtResponse
 *
 * PURPOSE
 *      This routine handles received AT Command Response packets.
 *
 * PARAMETERS
 *      pBuf        IN  pointer to start of packet data
 *      length      IN  length of the packet data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioPacketAtResponse(const uint8_t *pBuf, int16_t length)
{
    radioResponsePacket_t *pp = (radioResponsePacket_t *)pBuf;
    uint32_t temp;
    char debugBuf[10];

    /*
    **  Generate any required debug output.
    */
    if (radioDebug ||                       /* Radio debug mode turned on. */
        (pp->frameId == RADIO_CMD_USER))    /* Win32 user sent this AT cmd. */
    {
        debugWrite("Received AT Command Response, ");
        sprintf(debugBuf, "%c%c:  ", pp->atCommand[0], pp->atCommand[1]);
        debugWrite(debugBuf);

        switch (pp->status)
        {
            case RADIO_RESP_OK:
                debugWrite("OK");
                /* The following routine can add more debug info. */
                radioDebugAtResponse(pp);
                /* Terminate the debug line. */
                debugWrite("\n");
                break;
            case RADIO_RESP_ERROR:
                debugWrite("ERROR\n");
                break;
            case RADIO_RESP_BADCMD:
                debugWrite("Invalid Command\n");
                break;
            case RADIO_RESP_BADDATA:
                debugWrite("Invalid Data\n");
                break;
            default:
                debugWrite("Unknown\n");
                break;
        }
        debugHexWrite(pBuf, length, TRUE);
    }

    /*
    **  Process OK response according to the identifier.
    */
    if ((pp->status == RADIO_RESP_BADDATA) ||
        (pp->status == RADIO_RESP_BADCMD))
    {
        if ((pp->frameId & RADIO_CMD_AUTO) != 0)
        {
            /* Radio Failure. */
            /* TBD */
            /*
            ** We should not be giving commands with bad data.
            ** Remove from cmd pending table so we don't endlessly retry.
            */
            radioCmdPendingDelete(pp->frameId);
        }
    }
    else if (pp->status == RADIO_RESP_OK)
    {
        radioCmdPendingDelete(pp->frameId);
        switch (pp->frameId)
        {
            case RADIO_CMD_SH:
                /* Store radio MAC ID high 4 bytes. */
                temp = U8TOU32(pp->data[0], pp->data[1], pp->data[2], pp->data[3]);
                radioMacId = (radioMacId & 0x00000000FFFFFFFF) | ((uint64_t)temp << 32);
                break;

            case RADIO_CMD_SL:
                /* Store radio MAC ID low 4 bytes. */
                temp = U8TOU32(pp->data[0], pp->data[1], pp->data[2], pp->data[3]);
                radioMacId = (radioMacId & 0xFFFFFFFF00000000) | temp;
                break;

            case RADIO_CMD_D6:
                if (pp->data[0] != RADIO_D6_RTS)
                {
                    radioCommandD6RtsSet();
                }
                break;

            case RADIO_CMD_ID:
                if (((pp->data[0] << 8) | pp->data[1]) != ntohs(config.sys.radioPanId))
                {
                    radioCommandPanIdSet();
                }
                break;

            case RADIO_CMD_NI:
                if (memcmp(configSerialNumber, pp->data,
                    sizeof(configSerialNumber)) != 0)
                {
                    radioCommandNodeIdSet();
                }
                break;

            case RADIO_CMD_VR:
                if(pp->data[0] == 0 && pp->data[1] == 0) 
                {              
                  radioStatusVr = U8TOU32(pp->data[0], pp->data[1], pp->data[2], pp->data[3]);   
                } 
                else 
                {                   
                  radioStatusVr = U8TOU32(0,0,pp->data[0], pp->data[1]);
                }    
                break;
            case RADIO_CMD_HV:
                radioStatusHv = U8TOU16(pp->data[0], pp->data[1]);
                break;
            
            //case RADIO_CMD_CE:
            //    radioCE = pp->data[0];
            //    break;

            case RADIO_CMD_DB:
                /* Update last received signal strength. */
                
                if(pp->data[0] == 0)
                {                  
                    radioStatusDb = pp->data[1];
                } 
                else
                {
                    radioStatusDb = pp->data[0];
                }
                  
                break;

            case RADIO_CMD_OP:
                /* Update current Operating PAN. */
#ifdef RADIO_ZB
                radioStatusOp = U8TOU64(pp->data[0], pp->data[1], pp->data[2], pp->data[3],
                                        pp->data[4], pp->data[5], pp->data[6], pp->data[7]);
#else
                radioStatusOp = U8TOU16(pp->data[0], pp->data[1]);
#endif
                break;

            case RADIO_CMD_ID_S:
                radioCommandImmediate(RADIO_CMD_WR);
                break;
            case RADIO_CMD_NI_S:
                /* Now that PAN ID or Node ID was set, issue a Write command */
                radioCommandImmediate(RADIO_CMD_WR);
                break;

            case RADIO_CMD_WR:
                /* Write operation successful. Update the radio status. */
                radioCommandImmediate(RADIO_CMD_WR);
                break;

            case RADIO_CMD_RE:
                /* Factory reset successful. Re-initialize the radio. */
                sysEvent(RADIO_EVENT_FACTORY_RST, radioStatus);
                radioInit();
                radioCommandInitSequence();
                break;
          case  RADIO_CMD_ND:
                //radioNDData = pp->data[0];
                break;
          case RADIO_CMD_RC:
                radioStatusDb = pp->data[0];
                break;
          //case RADIO_CMD_NH_S:
                /* Write operation successful. Update the radio status. */
          //      radioCommandImmediate(RADIO_CMD_WR);
          //      break;
            default:
                break;
        }
        /* Update radio status. */
        radioStatusUpdate();
    }
}


/******************************************************************************
 *
 * radioPacketModemStatus
 *
 * PURPOSE
 *      This routine handles received Modem Status packets.
 *
 * PARAMETERS
 *      pBuf        IN  pointer to start of packet data
 *      length      IN  length of the packet data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioPacketModemStatus(const uint8_t *pBuf, int16_t length)
{
    radioStatusPacket_t *pp = (radioStatusPacket_t *)pBuf;

    /*
    **  Generate any required debug output.
    */
    if (radioDebug)
    {
        debugWrite("Received Modem Status:  ");
        switch (pp->status)
        {
            case RADIO_MDM_HWRESET:
                debugWrite("MODEM HARDWARE RESET\n");
                break;
            case RADIO_MDM_WDRESET:
                debugWrite("MODEM WATCHDOG RESET\n");
                break;
            case RADIO_MDM_ASSOC:
                debugWrite("Modem Associated\n");
                break;
            case RADIO_MDM_DISASSOC:
                debugWrite("Modem Disassociated\n");
                break;
            case RADIO_MDM_LOSTSYNC:
                debugWrite("Modem Synchronization Lost\n");
                break;
            case RADIO_MDM_COOREALIGN:
                debugWrite("Modem Coordinator Realignment\n");
                break;
            case RADIO_MDM_COORESTART:
                debugWrite("Modem Coordinator Started\n");
                break;
            default:
                debugWrite("Unknown\n");
                break;
        }
        debugHexWrite(pBuf, length, TRUE);
    }

    /*
    **  If Hardware or Software Reset is indicated, re-init the Radio logic.
    */
    if ((pp->status == RADIO_MDM_HWRESET) ||
        (pp->status == RADIO_MDM_WDRESET))
    {
        /* Log Modem Reset system event. */
        sysEvent(RADIO_EVENT_MODEM_RESET, pp->status);
        /* Enqueue AT Commands to re-initialize the radio module. */
        radioInit();
        /* Terminate the current radio poll cycle. */
        radioYield = TRUE;
    }

}


/******************************************************************************
 *
 * radioPacketZigBeeTxStatus
 *
 * PURPOSE
 *      This routine handles received ZigBee Tx Status packets.
 *
 * PARAMETERS
 *      pBuf        IN  pointer to start of packet data
 *      length      IN  length of the packet data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioPacketZigBeeTxStatus(const uint8_t *pBuf, int16_t length)
{
    radioTxStatPacket_t *pp = (radioTxStatPacket_t *)pBuf;
    char debugBuf[8];

    if (radioDebug)
    {
        debugWrite("Received ZigBee Transmit Status (FID=");
        sprintf(debugBuf, "%02X):  ", pp->frameId);
        debugWrite(debugBuf);
    }

    switch (pp->status)
    {
        case RADIO_TXSTAT_OK:
            /* Successful transmission */
            /* Clear Tx Data retry counter if success for last message sent. */
            if (pp->frameId == radioTxDataPkt.frameId)
            {
                radioTxDataRetriesRemaining = 0;
            }
            if (radioDebug)
            {
                debugWrite("SUCCESS!  ");
            }
            break;
        case RADIO_TXSTAT_CCAF:
            if (radioDebug)
            {
                debugWrite("FAILURE (CCA failure). ");
            }
            break;
        case RADIO_TXSTAT_INVDEST:
            if (radioDebug)
            {
                debugWrite("FAILURE (invalid destination endpoint). ");
            }
            break;
        case RADIO_TXSTAT_ACKFAIL:
            if (radioDebug)
            {
                debugWrite("FAILURE (network ACK failure). ");
            }
            break;
        case RADIO_TXSTAT_NOTJOINED:
            if (radioDebug)
            {
                debugWrite("FAILURE (not joined to network). ");
            }
            break;
        case RADIO_TXSTAT_SELFADDR:
            if (radioDebug)
            {
                debugWrite("FAILURE (self-addressed). ");
            }
            break;
        case RADIO_TXSTAT_ADDRNFND:
            if (radioDebug)
            {
                debugWrite("FAILURE (address not found). ");
            }
            break;
        case RADIO_TXSTAT_ROUTNFND:
            if (radioDebug)
            {
                debugWrite("FAILURE (route not found). ");
            }
            break;
        default:
            if (radioDebug)
            {
                debugWrite("UNKNOWN STATUS=0x");
                sprintf(debugBuf, "%02X. ", pp->status);
                debugWrite(debugBuf);
            }
            break;
    }

    switch (pp->discovery)
    {
        case RADIO_TXDISC_NOVHD:
            if (radioDebug)
            {
                debugWrite("No discovery overhead.\n");
            }
            break;
        case RADIO_TXDISC_ADDR:
            if (radioDebug)
            {
                debugWrite("Used address discovery.\n");
            }
            break;
        case RADIO_TXDISC_ROUTE:
            if (radioDebug)
            {
                debugWrite("Used route discovery.\n");
            }
            break;
        case RADIO_TXDISC_ADROUTE:
            if (radioDebug)
            {
                debugWrite("Used both address and route discovery.\n");
            }
            break;
        default:
            if (radioDebug)
            {
                debugWrite("Unknown discovery status.\n");
            }
            break;
    }

    if (radioDebug)
    {
        /* Write hex message data to debug output. */
        debugHexWrite(pBuf, length, TRUE);
    }

    /* If failure was on last frame sent, and retries remaining... */
    if ((pp->frameId == radioTxDataPkt.frameId) &&
        (radioTxDataRetriesRemaining > 0))
    {
        /* Decrement Tx Data retry counter. */
        radioTxDataRetriesRemaining--;

        if (radioDebug)
        {
            debugWrite("Re-");
        }

        /* Retry sending last Tx Data packet. */
        radioDataWrite(&radioTxDataPkt, radioTxDataPktLen);
    }
}




/******************************************************************************
 *
 * radioCommandInitSequence
 *
 * PURPOSE
 *      This routine sends commands to the RF module to obtain initial status
 *      information.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioCommandInitSequence(void)
{
    
    //must put the radio into API mode    
    drvRadioAPIModeInit();
    
     /*Set CE */
    //radioCommandEnqueue(RADIO_CMD_CE_S);
    
    /* Set configured D6 (RTS) mode. */
    radioCommandEnqueue(RADIO_CMD_D6_S);
    
    /* Set configured D7 (CTS) mode. */
    radioCommandEnqueue(RADIO_CMD_D7_S);
    
    
    /* Get 64-bit MAC ID from radio. */
    radioCommandEnqueue(RADIO_CMD_SH);
    radioCommandEnqueue(RADIO_CMD_SL);
                                        
    /* Get configured Node ID. */
    radioCommandEnqueue(RADIO_CMD_NI);

    /* Get radio firmware version. */
    #ifdef TEST_RST
    if(InitCount++ > 2)      //only send the command after 2 cycles to see if the retry code works
    #endif
    radioCommandEnqueue(RADIO_CMD_VR);  //testing zone skip on radio failure

    /* Get radio hardware version. */
    radioCommandEnqueue(RADIO_CMD_HV);

    /* Set P0 pin disabled. */
    radioCommandEnqueue(RADIO_CMD_P0_S);

    /* Set D0 pin disabled. */
    radioCommandEnqueue(RADIO_CMD_D0_S);
    
     /*Get CE */
    //radioCommandEnqueue(RADIO_CMD_CE);
    
    //Set number of hops
    //radioCommandEnqueue(RADIO_CMD_NH_S);
    
}


/******************************************************************************
 *
 * radioCommandNetworkReset
 *
 * PURPOSE
 *      This routine resets the network layer parameters on the RF Module.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandNetworkReset(void)
{
    uint8_t data[1] = {0};
    radioInit();
    return radioCommandSend(RADIO_CMD_FR, "FR", data, 1);
}


/******************************************************************************
 *
 * radioCommandP0Disable
 *
 * PURPOSE
 *      This routine configures the RF module for PWM0 configuration (P0 pin)
 *      set to disabled.  (Factory default uses this pin for RSSI PWM.)
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandP0Disable(void)
{
#ifdef WIN32
    uint8_t data[1] = {1};      /* Enable RSSI pin output in Win32 platform. */
#else
    uint8_t data[1] = {0};      /* Disable RSSI pin output in product. */
#endif
    return radioCommandSend(RADIO_CMD_P0_S, "P0", data, 1);
}


/******************************************************************************
 *
 * radioCommandD0Disable
 *
 * PURPOSE
 *      This routine configures the RF module for AD0/DIO0 configuration
 *      (D0 pin) set to disabled.  (Factory default uses this pin for node
 *      identification button.)
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandD0Disable(void)
{
    uint8_t data[1] = {0};

    return radioCommandSend(RADIO_CMD_D0_S, "D0", data, 1);
}

/*****************************************************************************
*
*
*****************************************************************************/
/*
static bool_t radioCommandCE(void)
{
    uint8_t data = 1;

    return radioCommandSend(RADIO_CMD_CE_S, "CE", &data, 1);
}
*/

/******************************************************************************
 *
 * radioCommandD6RtsSet
 *
 * PURPOSE
 *      This routine configures the RF module to support the RTS flow control
 *      signal.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandD6RtsSet(void)
{
    uint8_t data[1] = {RADIO_D6_RTS};  /* Set for RTS flow control. */

    return radioCommandSend(RADIO_CMD_D6_S, "D6", data, 1);
}


/******************************************************************************
 *
 * radioCommandD7CtsSet
 *
 * PURPOSE
 *      This routine configures the RF module to support the RTS flow control
 *      signal.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandD7CtsSet(void)
{
    uint8_t data[1] = {RADIO_D7_CTS};  /* Set for RTS flow control. */

    return radioCommandSend(RADIO_CMD_D6_S, "D7", data, 1);
}




/******************************************************************************
 *
 * radioCommandPanIdSet
 *
 * PURPOSE
 *      This routine configures the RF module for the PAN ID stored in the
 *      WaterOptimizer's configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandPanIdSet(void)
{
    uint8_t data[2];

    data[0] = (uint8_t)(ntohs(config.sys.radioPanId) >> 8);
    data[1] = (uint8_t)(ntohs(config.sys.radioPanId) & 0x00FF);
    return radioCommandSend(RADIO_CMD_ID_S, "ID", data, 2);

}


/******************************************************************************
 *
 * radioCommandNodeIdSet
 *
 * PURPOSE
 *      This routine configures the RF module for a Node ID equal to the
 *      manufacturing serial number.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTES
 *      The manufacturing serial number is specified to comprise exactly
 *      8 ASCII characters, unused characters are encoded as spaces (0x20).
 *
 *****************************************************************************/
static bool_t radioCommandNodeIdSet(void)
{
    return radioCommandSend(RADIO_CMD_NI_S, "NI",
                            configSerialNumber,
                            CONFIG_SN_SIZE);
}


/******************************************************************************
 *
 * radioCommandEnqueue
 *
 * PURPOSE
 *      This routine enqueues an AT command to be sent to the RF module.
 *
 * PARAMETERS
 *      cmdIdent    IN  command identifier
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
static bool_t radioCommandEnqueue(uint8_t cmdIdent)
{
    bool_t result;

    result = radioCmdPendingAdd(cmdIdent, TRUE);
    if (!result)
    {
        debugWrite("radioCommandEnqueue: table overflow\n");
    }
    return result;
}


/******************************************************************************
 *
 * radioCommandImmediate
 *
 * PURPOSE
 *      This routine sends an immediate AT Command to the RF module.
 *
 * PARAMETERS
 *      cmdIdent    IN  command identifier
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTE
 *      Every distinct AT Command that may be sent by the WOIS radio logic to
 *      the XBee RF module is assigned a specific command identifier code.
 *      Many AT Commands are query operations which require no parameter data.
 *      If the specified AT Command requires parameter data, the parameter data
 *      will be implicit in the command identifier.  When AT Commands are used
 *      with parameters, they typically modify the RF module's configuration
 *      settings.  Some command identifiers specify to use fixed "canned"
 *      parameter data, other command identifiers specify to use parameter
 *      data derived from the current WOIS system or configuration data.
 *
 *****************************************************************************/
static bool_t radioCommandImmediate(uint8_t cmdIdent)
{
    char debugbuf[10];
    bool_t result = FALSE;

    switch (cmdIdent)
    {
        case RADIO_CMD_SL:
            result = radioCommandSend(RADIO_CMD_SL, "SL", NULL, 0);
            break;
        case RADIO_CMD_SH:
            result = radioCommandSend(RADIO_CMD_SH, "SH", NULL, 0);
            break;
        case RADIO_CMD_ID:
            result = radioCommandSend(RADIO_CMD_ID, "ID", NULL, 0);
            break;
        case RADIO_CMD_NI:
            result = radioCommandSend(RADIO_CMD_NI, "NI", NULL, 0);
            break;
        case RADIO_CMD_VR:
            result = radioCommandSend(RADIO_CMD_VR, "VR", NULL, 0);
            break;
        case RADIO_CMD_HV:
            result = radioCommandSend(RADIO_CMD_HV, "HV", NULL, 0);
            break;
        //case RADIO_CMD_CE:
        //    result = radioCommandSend(RADIO_CMD_CE, " ", NULL, 0);
        //    break;
        case RADIO_CMD_D6:
            result = radioCommandSend(RADIO_CMD_D6, "D6", NULL, 0);
            break;
        case RADIO_CMD_D7:
            result = radioCommandSend(RADIO_CMD_D7, "D7", NULL, 0);
            break;
        case RADIO_CMD_DB:
            result = radioCommandSend(RADIO_CMD_DB, "DB", NULL, 0);
            break;
        case RADIO_CMD_WR:
            result = radioCommandSend(RADIO_CMD_WR, "WR", NULL, 0);
            break;
        case RADIO_CMD_FR:
            result = radioCommandSend(RADIO_CMD_FR, "FR", NULL, 0);
            break;
        case RADIO_CMD_RE:
            result = radioCommandSend(RADIO_CMD_RE, "RE", NULL, 0);
            break;
        case RADIO_CMD_P0_S:
            result = radioCommandP0Disable();
            break;
        case RADIO_CMD_D0_S:
            result = radioCommandD0Disable();
            break;
        //case RADIO_CMD_CE_S:
        //    result = radioCommandCE();
        //    break;
        case RADIO_CMD_D6_S:
            result = radioCommandD6RtsSet();
            break;
        case RADIO_CMD_ID_S:
            result = radioCommandPanIdSet();
            break;
        case RADIO_CMD_NI_S:
            result = radioCommandNodeIdSet();
            break;
        case RADIO_CMD_D7_S:
            result = radioCommandD7CtsSet();
            break;
        //case RADIO_CMD_NH_S:
        //    uint8_t data[1] = {RADIO_NHOPS};
        //    result = radioCommandSend(RADIO_CMD_NH_S, "NH", data, 1);
        //    break;
        case RADIO_CMD_ND:
            result = radioCommandSend(RADIO_CMD_ND, "ND", NULL, 0);
            break;
        case RADIO_CMD_RC:
            result = radioCommandSend(RADIO_CMD_ND, "RC", 0, 0);
        case RADIO_CMD_USER:
            /* This debug command identifier should never occur here. */
            /* Fall through to default case for error handling. */
        default:
            /* Make sure this command is not retried. */
            radioCmdPendingDelete(cmdIdent);
            /* Write unsupported command debug message. */
            debugWrite("radioCommand: INVALID IDENTIFIER (cmdIdent=");
            sprintf(debugbuf, "%02X)\n", cmdIdent);
            debugWrite(debugbuf);
            break;
    }

    return result;
}


/******************************************************************************
 *
 * radioCommandSend
 *
 * PURPOSE
 *      This routine sends an AT command to the RF module.
 *
 * PARAMETERS
 *      frameId     IN  command identifier used in processing response/retry
 *      pCommand    IN  pointer to 2-char AT command (null term. not required)
 *      pData       IN  pointer to start of command parameter data
 *      lenData     IN  length of the command parameter data
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTES
 *      Failure may be reported for multiple reasons.  Failure return status
 *      does not necessarily mean that the command was not sent.  The command
 *      may actually get transmitted to the RF module but the auto-retry
 *      queue may be full, resulting in a failure of insurance that the
 *      command will be retried after timeout waiting for acknowledgement.
 *
 *****************************************************************************/
bool_t radioCommandSend(uint8_t frameId,
                        const char *pCommand,
                        const uint8_t *pData,
                        int16_t lenData)
{
    radioCommandPacket_t msg;
    char debugBuf[10];
    uint16_t length;
    bool_t result = TRUE;

    /* Add auto commands to command pending table for auto-retry. */
    if ((frameId & RADIO_CMD_AUTO) == RADIO_CMD_AUTO)
    {
        result = radioCmdPendingAdd(frameId, FALSE);
    }

    msg.apiType = RADIO_API_COMMAND;
    msg.frameId = frameId;
    msg.atCommand[0] = pCommand[0];
    msg.atCommand[1] = pCommand[1];
    memcpy(msg.atParams, pData, lenData);

    length = woffsetof(radioCommandPacket_t, atParams) + lenData;

    if (radioDebug)
    {
        debugWrite("Sent AT Command, ");
        sprintf(debugBuf, "%c%c:\n", msg.atCommand[0], msg.atCommand[1]);
        debugWrite(debugBuf);
        debugHexWrite((uint8_t *)&msg, length, TRUE);
    }

    return drvRadioWrite((uint8_t *)&msg, length) && result;
}

/******************************************************************************
 *
 * radioRemoteCommandSend
 *
 * PURPOSE
 *      This routine sends an AT command to the RF module.
 *
 * PARAMETERS
 *      frameId     IN  command identifier used in processing response/retry
 *      pCommand    IN  pointer to 2-char AT command (null term. not required)
 *      pData       IN  pointer to start of command parameter data
 *      lenData     IN  length of the command parameter data
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTES
 *      Failure may be reported for multiple reasons.  Failure return status
 *      does not necessarily mean that the command was not sent.  The command
 *      may actually get transmitted to the RF module but the auto-retry
 *      queue may be full, resulting in a failure of insurance that the
 *      command will be retried after timeout waiting for acknowledgement.
 *
 *****************************************************************************/
/*bool_t radioRemoteCommandSend(uint8_t frameId,
                              uint16_t netAddr,
                              uint32_t phyAddrH,
                              uint32_t phyAddrL,
                              const char *pCommand,
                              const uint8_t *pData,
                              int16_t lenData)
{
    radioRemoteCommandPacket_t msg;
    char debugBuf[10];
    uint16_t length;
    bool_t result = TRUE;

    // Add auto commands to command pending table for auto-retry. //
    if ((frameId & RADIO_CMD_AUTO) == RADIO_CMD_AUTO)
    {
        result = radioCmdPendingAdd(frameId, FALSE);
    }

    msg.apiType = RADIO_API_REM_COMMAND;
    msg.frameId = frameId;
    msg.phyAddr[0] = (uint8_t)(phyAddrH >> 24);
    msg.phyAddr[1] = (uint8_t)((phyAddrH & 0x00FF0000) >> 16);
    msg.phyAddr[2] = (uint8_t)((phyAddrH & 0x0000FF00) >> 8);
    msg.phyAddr[3] = (uint8_t)(phyAddrH & 0x000000FF);
    msg.phyAddr[4] = (uint8_t)(phyAddrL >> 24);
    msg.phyAddr[5] = (uint8_t)((phyAddrL & 0x00FF0000) >> 16);
    msg.phyAddr[6] = (uint8_t)((phyAddrL & 0x0000FF00) >> 8);
    msg.phyAddr[7] = (uint8_t)(phyAddrL & 0x000000FF);
    msg.netAddr[0] = netAddr >> 8;
    msg.netAddr[1] = netAddr & 0x00FF;
    msg.options =0;
    msg.atCommand[0] = pCommand[0];
    msg.atCommand[1] = pCommand[1];
  
    memcpy(msg.atParams, pData, lenData);

    length = woffsetof(radioCommandPacket_t, atParams) + lenData;

    if (radioDebug)
    {
        debugWrite("Sent Remote AT Command, ");
        sprintf(debugBuf, "%c%c:\n", msg.atCommand[0], msg.atCommand[1]);
        debugWrite(debugBuf);
        debugHexWrite((uint8_t *)&msg, length, TRUE);
    }
    
    radioTxDataTime = dtTickCount;
    return drvRadioWrite((uint8_t *)&msg, length) && result;
} */


/******************************************************************************
 *
 * radioDataSend
 *
 * PURPOSE
 *      This routine sends a ZigBee data packet to the RF module, including
 *      the necessary CRC checksum.
 *
 * PARAMETERS
 *      frameId     IN  frame identifier (corelates with any ZigBee Tx Status)
 *      netAddr     IN  destination network address (16-bits)
 *      phyAddrH    IN  destination phy address (high 32-bits)
 *      phyAddrL    IN  destination phy address (low 32-bits)
 *      pData       IN  pointer to start of application payload data
 *      lenData     IN  length of the application payload data
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTES
 *      Specifying frameId of zero will suppress the Tx Status response from
 *      the RF module.
 *
 *****************************************************************************/
bool_t radioDataSend(uint8_t frameId,
                     uint16_t netAddr,
                     uint32_t phyAddrH,
                     uint32_t phyAddrL,
                     const uint8_t *pData,
                     int16_t lenData)
{
    int i;
    uint16_t length;

    radioTxDataPkt.apiType = RADIO_API_TXDATA;
    radioTxDataPkt.frameId = frameId;
    radioTxDataPkt.phyAddr[0] = (uint8_t)(phyAddrH >> 24);
    radioTxDataPkt.phyAddr[1] = (uint8_t)((phyAddrH & 0x00FF0000) >> 16);
    radioTxDataPkt.phyAddr[2] = (uint8_t)((phyAddrH & 0x0000FF00) >> 8);
    radioTxDataPkt.phyAddr[3] = (uint8_t)(phyAddrH & 0x000000FF);
    radioTxDataPkt.phyAddr[4] = (uint8_t)(phyAddrL >> 24);
    radioTxDataPkt.phyAddr[5] = (uint8_t)((phyAddrL & 0x00FF0000) >> 16);
    radioTxDataPkt.phyAddr[6] = (uint8_t)((phyAddrL & 0x0000FF00) >> 8);
    radioTxDataPkt.phyAddr[7] = (uint8_t)(phyAddrL & 0x000000FF);
    radioTxDataPkt.bcastRadius = 0;
    radioTxDataPkt.options = 0x00; // use 0 for production Code, 0x08 tio enable trace route
    radioTxDataPkt.netAddr[0] = netAddr >> 8;
    radioTxDataPkt.netAddr[1] = netAddr & 0x00FF;
    for (i = 0; i < lenData; i++)
    {
        radioTxDataPkt.data[i] = pData[i];
    }

    /* Set packet length. */
    length = woffsetof(radioTxDataPacket_t, data) + lenData;

    /* Set number of retries. */
    radioTxDataRetriesRemaining = RADIO_TXD_N_RETRIES;

    /* Set length of Tx Data packet. */
    radioTxDataPktLen = length;

    /* Send Tx Data packet. */
    return radioDataWrite(&radioTxDataPkt, radioTxDataPktLen);
}


/******************************************************************************
 *
 * radioDataWrite
 *
 * PURPOSE
 *      This routine sends a ZigBee data packet to the RF module.
 *
 * PARAMETERS
 *      msg         IN  pointer to Tx Data Packet to send
 *      length      IN  length of the Tx Data Packet
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 *****************************************************************************/
bool_t radioDataWrite(radioTxDataPacket_t *msg, uint16_t length)
{
    char debugbuf[8];

    if (radioDebug)
    {
        debugWrite("Sent ZigBee Data Packet (FID=");
        sprintf(debugbuf, "%02X", msg->frameId);
        debugWrite(debugbuf);
        debugWrite(").  API header:  ");
        debugHexWrite((uint8_t *)&radioTxDataPkt,
                      woffsetof(radioTxDataPacket_t, data),
                      FALSE);
        debugHexWrite(radioTxDataPkt.data,
                      length - woffsetof(radioTxDataPacket_t, data),
                      TRUE);
    }

    radioTxDataTime = dtTickCount;
    return drvRadioWrite((uint8_t *)msg, length);
}


/******************************************************************************
 *
 * radioLoopbackDataSend
 *
 * PURPOSE
 *      This routine sends a ZigBee data packet to the RF module, using ZigBee
 *      explicit addressing to specify the XBee serial loopback data cluster
 *      ID on the destination endpoint.
 *
 * PARAMETERS
 *      phyAddrH    IN  destination phy address (high 32-bits)
 *      phyAddrL    IN  destination phy address (low 32-bits)
 *      netAddr     IN  destination network address (16-bits)
 *      frameId     IN  frame identifier (corelates with any ZigBee Tx Status)
 *      pData       IN  pointer to start of loopback data
 *      lenData     IN  length of the loopback data
 *
 * RETURN VALUE
 *      Return value is TRUE for success; FALSE for failure.
 *
 * NOTES
 *      Specifying frameId of zero will suppress the Tx Status response from
 *      the RF module.
 *
 *****************************************************************************/

static bool_t radioLoopbackDataSend(const uint8_t *pData, int16_t lenData)
{
    return radioDataSend(RADIO_API_TXDATA,0xFFFE,0x00000000,0x0000FFFF, pData,lenData);
}



/******************************************************************************
 *
 * radioProtocolCommandHandler
 *
 * PURPOSE
 *      This routine handles received WaterOptimizer application command
 *      packets.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of packet
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioProtocolCommandHandler(const radioRxDataPacket_t *pPacket)
{
    radioMsgCmd_t *pMsg = (radioMsgCmd_t *)&pPacket->data[0];
    uint16_t crc;
    uint16_t crcMsg;
    uint8_t cmdAck;
    uint32_t etData;
    uint32_t rainfall;
    int i;
    uint8_t data[55];
    uint8_t lenData = 0;
    int16_t result;
    char debugBuf[40];
    uint64_t expMacID;
    uint16_t tempMoistFailedSensors=0;
    uint8_t offset=0;

    /* Compute the CRC. */
    crc = crc16(((uint8_t *)pMsg) + 4,
                     RADIO_CMD_HEADER_SIZE - 4 + pMsg->dataLen);

    /* Verify the CRC in the command message. */
    
    
    crcMsg = (pMsg->hdr.crcHigh << 8) | pMsg->hdr.crcLow;
    
    if (crcMsg != crc)
    {
        if (radioDebug)
        {
            debugWrite("Received WOIS Cmd with INVALID CRC ");
            sprintf(debugBuf,"%04X (Expected %04X)\n.",
                crcMsg, crc);
            debugWrite(debugBuf);
        }
        return;
    } 
    

    if (pMsg->hdr.msgType == RADIO_TYPE_ACK)
    {
        /* Debug test command acknowledge.  Send to debug test command handler. */
        radioExtCommandHandler(pPacket);
        return;
    }
    
    //dtflag = pMsg->cmd;
    //get the source address
    expMacID = U8TOU64(pPacket->phyAddr[0],
                       pPacket->phyAddr[1],
                       pPacket->phyAddr[2],
                       pPacket->phyAddr[3],
                       pPacket->phyAddr[4],
                       pPacket->phyAddr[5],
                       pPacket->phyAddr[6],
                       pPacket->phyAddr[7]);
    
    /* inform the system that communication was received from the proper expansion */
    if(expMacID ==config.sys.expMac1)
    {
        radioStatusExpansion1 = EXPANSION_CONNECTED;
        radioSentExpan1Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_1);
    }
    else if(expMacID ==config.sys.expMac2)
    {
        radioStatusExpansion2 = EXPANSION_CONNECTED;
        radioSentExpan2Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_2);
    }
    else if(expMacID ==config.sys.expMac3)
    {
        radioStatusExpansion3 = EXPANSION_CONNECTED;
        radioSentExpan3Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_3);
    }
    else if(expMacID ==config.sys.masterMac)
    {
        radioSentMasterMsg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_MASTER);
    }

    /* Handle the command. */
    switch (pMsg->cmd)
    {
        case RADIO_CMD_NO_OP:
            radioMessageLog("No Op");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: NO OP\n");
            }
            /* Do nothing but acknowledge. */
            cmdAck = RADIO_ACK_NO_OP;
            
            /* forward message onto expasion units if master */
            if((config.sys.unitType == UNIT_TYPE_MASTER)&&
               (expMacID != config.sys.expMac1)&&
               (expMacID != config.sys.expMac2)&&
               (expMacID != config.sys.expMac3))
            {
                expansionBusForwardCmd(pPacket);
            }
            /* send updated time to unit that sent NO OP */
            if((config.sys.unitType == UNIT_TYPE_MASTER)&&
               ((expMacID == config.sys.expMac1)||
               (expMacID == config.sys.expMac2)||
               (expMacID == config.sys.expMac3)))
            {
                expansionBusSendCmd(RADIO_CMD_INIT_DATETIME, expMacID);
            }
            break;

        case RADIO_CMD_INHIBIT_ON:
            radioMessageLog("Inhibit On");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: INHIBIT ON\n");
            }
            /* Set Inhibit On. */
            sysInhibitOn();
            cmdAck = RADIO_ACK_INHIBIT_ON;
            
            /* forward message onto expasion units if master */
            if((config.sys.unitType == UNIT_TYPE_MASTER)&&
               (expMacID != config.sys.expMac1)&&
               (expMacID != config.sys.expMac2)&&
               (expMacID != config.sys.expMac3))
            {
                expansionBusForwardCmd(pPacket);
            }
            
            break;

        case RADIO_CMD_INHIBIT_OFF:
            radioMessageLog("Inhibit Off");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: INHIBIT OFF\n");
            }
            /* Set Inhibit Off. */
            sysInhibitOff();
            cmdAck = RADIO_ACK_INHIBIT_OFF;
            
            /* forward message onto expasion units if master */
            if((config.sys.unitType == UNIT_TYPE_MASTER)&
               (expMacID != config.sys.expMac1)&&
               (expMacID != config.sys.expMac2)&&
               (expMacID != config.sys.expMac3))
            {
                expansionBusForwardCmd(pPacket);
            }
            break;

        case RADIO_CMD_FORCE_ON:
            radioMessageLog("Force On");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: FORCE ON\n");
            }
            /* Force On. */
            irrCmdForceOn();
            cmdAck = RADIO_ACK_FORCE_ON;
            break;

        case RADIO_CMD_WEATHER_DATA:
            if (pMsg->dataLen != 8)
            {
                /* Incorrect data length. */
                if (radioDebug)
                {
                    debugWrite("WOIS Cmd: WEATHER UPDATE - Bad data len.\n");
                }
                return;
            }
            /* Extract the 4 bytes of ET data from the command message. */
            etData = U8TOU32(pMsg->data[0],
                             pMsg->data[1],
                             pMsg->data[2],
                             pMsg->data[3]);
            /* Extract the 4 bytes of rainfall data from the command message. */
            rainfall = U8TOU32(pMsg->data[4],
                               pMsg->data[5],
                               pMsg->data[6],
                               pMsg->data[7]);
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: WEATHER UPDATE (ET=0x");
                sprintf(debugBuf, "%08X", etData);
                debugWrite(debugBuf);
                debugWrite("; Rain=0x");
                sprintf(debugBuf, "%08X)\n", rainfall);
                debugWrite(debugBuf);
            }
            cmdAck = RADIO_ACK_WEATHER_DATA;
            lenData = 2;
            /* Attempt to apply weather update data. */
            if (radioProtoCmdWeatherUpdate(etData, rainfall))
            {
                /* Weather update applied successfully. */
                radioMessageLog("Weather Update");
                data[0] = RADIO_RESULT_SUCCESS;
            }
            else
            {
                /* Weather data not applied; accumulator re-sync occurred. */
                radioMessageLog("Weather Sync");
                data[0] = RADIO_RESULT_FAILURE;
            }
            /* Indicate if system is configured for Weather mode. */
            if (config.sys.opMode == CONFIG_OPMODE_WEATHER)
            {
                /* Weather operation mode in use. */
                data[1] = RADIO_WEATHER_MODE_ON;
            }
            else
            {
                /* Weather operation mode not in use. */
                data[1] = RADIO_WEATHER_MODE_OFF;
            }
            
            /* forward message onto expasion units if master */
            if((config.sys.unitType == UNIT_TYPE_MASTER)&&
               (expMacID != config.sys.expMac1)&&
               (expMacID != config.sys.expMac2)&&
               (expMacID != config.sys.expMac3))
            {
                expansionBusForwardCmd(pPacket);
            }
            break;

        case RADIO_CMD_GET_MOIST:
            radioMessageLog("Get Moisture");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: GET MOISTURE VALUES\n");
            }
            cmdAck = RADIO_ACK_GET_MOIST;
            lenData = config.sys.numZones;
            for (i = 0; i < lenData; i++)
            {
                /* Get zone moisture value. */
                data[i] = sysRadioMoistValueGet(i);
            }
            break;

        case RADIO_CMD_GET_EXSTAT:
            radioMessageLog("Get Ext Status");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: GET EXTENDED STATUS\n");
            }
            cmdAck = RADIO_ACK_GET_EXSTAT;
            /* Get Extended System Status data. */
            lenData = sysRadioExtStatusGet(data);
            break;
        case RADIO_CMD_GET_EXSTAT_2:
            radioMessageLog("Get Ext Status Part 2");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: GET EXTENDED STATUS 2\n");
            }
            cmdAck = RADIO_ACK_GET_EXSTAT_2;
            for(i=0; i<(SYS_N_ZONES/2); i++)
            {
                data[2*i]=(irrDailyRuntimePerZone[i] >> 8);
                data[2*i+1]=(irrDailyRuntimePerZone[i] & 0xFF);    
            }
            lenData = 48;
            break;
        
        case RADIO_CMD_GET_EXSTAT_3:
            radioMessageLog("Get Ext Status Part 3");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: GET EXTENDED STATUS 3\n");
            }
            cmdAck = RADIO_ACK_GET_EXSTAT_3;
            for(i=0; i<(SYS_N_ZONES/2); i++)
            {
                data[2*i]=(irrDailyRuntimePerZone[i+24] >> 8);
                data[2*i+1]=(irrDailyRuntimePerZone[i+24] & 0xFF);    
            }
            lenData = 48;
            break;
            
        case RADIO_CMD_CFG_GET_SNAP:
            radioMessageLog("Start Cfg Upload");
            debugWrite("WOIS Cmd: START CONFIGURATION UPLOAD\n");
            cmdAck = RADIO_ACK_CFG_GET_SNAP;
            lenData = 8;
            /* Configuration Version */
            data[0] = (uint8_t)(ntohs(config.sys.version) >> 8);
            data[1] = (uint8_t)(ntohs(config.sys.version) & 0xFF);
            /* Configuration Checksum */
            data[2] = (uint8_t)(ntohs(config.sys.checkSum) >> 8);
            data[3] = (uint8_t)(ntohs(config.sys.checkSum) & 0xFF);
            /* Configuration Data Image Size */
            data[4] = (uint8_t)(CONFIG_IMAGE_SIZE >> 8);
            data[5] = (uint8_t)(CONFIG_IMAGE_SIZE & 0xFF);
            /* Configuration Data Segment Count */
            data[6] = (uint8_t)(RADIO_CFG_SEGS >> 8);
            data[7] = (uint8_t)(RADIO_CFG_SEGS & 0xFF);
            /* Save configuration snapshot to upload buffer. */
            configSnapshotSave();
            break;

        case RADIO_CMD_CFG_PUT_START:
            radioMessageLog("Start Cfg Download");
            debugWrite("WOIS Cmd: START CONFIGURATION DOWNLOAD\n");
            cmdAck = RADIO_ACK_CFG_PUT_START;
            lenData = 8;
            /* Configuration Version */
            data[0] = (uint8_t)(ntohs(config.sys.version) >> 8);
            data[1] = (uint8_t)(ntohs(config.sys.version) & 0xFF);
            /* Configuration Checksum */
            data[2] = (uint8_t)(ntohs(config.sys.checkSum) >> 8);
            data[3] = (uint8_t)(ntohs(config.sys.checkSum) & 0xFF);
            /* Configuration Data Image Size */
            data[4] = (uint8_t)(CONFIG_IMAGE_SIZE >> 8);
            data[5] = (uint8_t)(CONFIG_IMAGE_SIZE & 0xFF);
            /* Configuration Data Segment Count */
            data[6] = (uint8_t)(RADIO_CFG_SEGS >> 8);
            data[7] = (uint8_t)(RADIO_CFG_SEGS & 0xFF);
            /* Clear configuration download buffer to all 0xFF's. */
            configBufferClear();
            break;

        case RADIO_CMD_CFG_PUT_APPLY:
            radioMessageLog("Apply Cfg Download");
            debugWrite("WOIS Cmd: APPLY DOWNLOADED CONFIGURATION\n");
            cmdAck = RADIO_ACK_CFG_PUT_APPLY;
            lenData = 3;
            /* Request to Apply the configuration image in the download buffer. */
            result = configBufferLoad();
            if (result == -1)
            {
                data[0] = RADIO_RESULT_SUCCESS;     /* success */
                data[1] = 0;
                data[2] = 0;
                debugWrite("Configuration applied.\n");
                
               sysFaultClear(SYS_FAULT_EXPAN_MASTER);
               sysFaultClear(SYS_FAULT_EXPAN_1);
               sysFaultClear(SYS_FAULT_EXPAN_2);
               sysFaultClear(SYS_FAULT_EXPAN_3);
            }
            else
            {
                data[0] = RADIO_RESULT_FAILURE;     /* failure */
                data[1] = result >> 8;      /* offset to bad config data MSB */
                data[2] = result & 0x00FF;  /* offset to bad config data LSB */
                debugWrite("Configuration not applied - invalid image.\n");
            }
            break;

        case RADIO_CMD_GET_MB_VALUES:
            radioMessageLog("Get MB Values");
            if (pMsg->dataLen != 1)
            {
                /* Incorrect data length. */
                if (radioDebug)
                {
                    debugWrite("WOIS Cmd: GET MB VALUES - Bad data len.\n");
                }
                return;
            }
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: GET MB VALUES (start zone=");
                sprintf(debugBuf, "%d)\n", pMsg->data[0]);
                debugWrite(debugBuf);
            }
            /* Get Moisture Balance Values data. */
            lenData = radioProtoCmdGetMbValues(pMsg->data[0], data);
            cmdAck = RADIO_ACK_GET_MB_VALUES;
            break;

        case RADIO_CMD_SET_MB_VALUES:
            radioMessageLog("Set MB Values");
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: SET MB VALUES (start zone=");
                sprintf(debugBuf, "%d)\n", pMsg->data[0]);
                debugWrite(debugBuf);
            }
            cmdAck = RADIO_ACK_SET_MB_VALUES;
            lenData = 1;
            /* Set Moisture Balance Values from command data. */
            if (radioProtoCmdSetMbValues(pMsg->data, pMsg->dataLen))
            {
                data[0] = RADIO_RESULT_SUCCESS;
            }
            else
            {
                data[0] = RADIO_RESULT_FAILURE;
            }
            break;

        /* INIT DATE/TIME */
        case RADIO_CMD_INIT_DATETIME:
            radioMessageLog("Init Date/Time");
            //dtflag = 88;
            //dtdata = 
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: Init Date/Time\n");
            }
            cmdAck = RADIO_ACK_INIT_DATETIME;
            lenData = 1;
            /* Initialize response status to indicate failure. */
            data[0] = RADIO_RESULT_FAILURE;
            /* Only allow clock to be set if a date/time error is present. */
            //if (sysErrorOn(SYS_ERROR_DATETIME))
            {
                if (pMsg->dataLen == 6)
                {
                    /* Set date/time in system real time clock. */
                    
                    if ( dtSetClock(  (uint16_t)((int)(pMsg->data[0]) + 2000),
                                       pMsg->data[1],
                                       pMsg->data[2],
                                       pMsg->data[3],
                                       pMsg->data[4],
                                       pMsg->data[5] )   )
                    {
                        /* Update response status to indicate success. */
                        data[0] = RADIO_RESULT_SUCCESS;
                    }
                }
            }
            
            /* forward message onto expasion units if master */
            if(config.sys.unitType == UNIT_TYPE_MASTER)
            {
                expansionBusForwardCmd(pPacket);
            }
            break;
        case RADIO_CMD_SEND_FLOW:
            cmdAck = RADIO_ACK_SEND_FLOW;
            lenData = 4;
            data[0] = RADIO_RESULT_FAILURE;
            if (pMsg->dataLen == 4) {
                slaveGPM = pMsg->data[0];
                slaveFlowDelay = U8TOU16(pMsg->data[1],pMsg->data[2]);
                slaveFindFlow = pMsg->data[3];
                data[0] = RADIO_RESULT_SUCCESS;
            }
            break;
        case RADIO_CMD_DIAGNOSE:
            radioMessageLog("Diagnose");
            if (pMsg->dataLen > 32)
            {
                /* Incorrect data length. */
                if (radioDebug)
                {
                    debugWrite("WOIS Cmd: DIAGNOSE - Data len > 32.\n");
                }
                return;
            }
            memcpy(debugBuf, pMsg->data, pMsg->dataLen);
            debugBuf[pMsg->dataLen] = '\0';
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: DIAGNOSE - '");
                debugWrite(debugBuf);
                debugWrite("'\n");
            }
            /*
            **  NOTE:  No Diagnose commands are currently defined.
            **         Always send "?" for the response string.
            */
            cmdAck = RADIO_ACK_DIAGNOSE;
            lenData = 2;
            data[0] = '?';
            data[1] = '\0';
            break;
            
        case RADIO_CMD_FW_PUT_START:
            radioMessageLog("Start FW Download");
            debugWrite("WOIS Cmd: START FIRWMARE DOWNLOAD\n");
            cmdAck = RADIO_ACK_FW_PUT_START;
            lenData = 8;
            
            /* Extract the 4 bytes of new firmware image size from the command message. */
            newFirmwareSize = U8TOU32(pMsg->data[0],
                                      pMsg->data[1],
                                      pMsg->data[2],
                                      pMsg->data[3]);
            
            /* Current Firmware Version */
            data[0] = (uint8_t)(ntohs(sysFirmwareVer.major));
            data[1] = (uint8_t)(ntohs(sysFirmwareVer.minor));
            data[2] = (uint8_t)(ntohs(sysFirmwareVer.patch));
            data[3] = (uint8_t)(ntohs(sysFirmwareVer.seq));
            
            /* Clear area of flash to all 0xFF's for firmware download. */
            if(extFlashFWErase(EXT_FLASH_SEC_0) == TRUE)
            {
              data[4] = RADIO_RESULT_SUCCESS;  
            } 
            else
            {
              data[4] = RADIO_RESULT_FAILURE;
            }
            
             /* Future Expansion */
            data[5] = 0;            
            data[6] = 0;                   
            data[7] = 0;
            break;

        case RADIO_CMD_FW_PUT_APPLY:
            radioMessageLog("Apply FW Download");
            debugWrite("WOIS Cmd: APPLY DOWNLOADED FIRMWARE\n");
            cmdAck = RADIO_ACK_FW_PUT_APPLY;
            lenData = 3;
            /* Request to start the bootloader again */
            data[0] = RADIO_RESULT_SUCCESS;     /* success */
            data[1] = 0;
            data[2] = 0;
            break;
            
        case RADIO_CMD_IRR_START:
            radioMessageLog("Exp Irr Start");
            debugWrite("WOIS Cmd: IRRIGATION START\n");
            cmdAck = RADIO_ACK_IRR_START;
            lenData = 1;
            
             radioCmdIrrStart = TRUE;             /*denotes that was told to start early */
             
            /* start irrigation program */
            switch(pMsg->data[1])
            {
                case SYS_STATE_TEST:
                      irrCmdTest();
                      data[0] = RADIO_RESULT_SUCCESS;     /* success */
                      break;
                case SYS_STATE_MANUAL:
                      sysManualProgram=pMsg->data[0];
                      sysManualOpMode=pMsg->data[2];
                      sysManualPulseMode=pMsg->data[3];
                      irrCmdManualStart();
                      data[0] = RADIO_RESULT_SUCCESS;     /* success */
                      break;
                case SYS_STATE_FORCE:
                    irrCmdForceOn();
                    data[0] = RADIO_RESULT_SUCCESS;     /* success */
                    break;
                default:
                    if(irrCmdExpPulseOn(pMsg->data[0]))
                    {
                        data[0] = RADIO_RESULT_SUCCESS;     /* success */
                    } 
                    else 
                    {
                        data[0] = RADIO_RESULT_FAILURE;     /* failure */
                    }
                    break;
            }
            break;
            
        case RADIO_CMD_IRR_COMPLETE:
            radioMessageLog("Exp Irr Mode End");
            debugWrite("Expansion Cmd: IRRIGATION COMPLETE\n");
            pMsg->cmd= RADIO_ACK_IRR_COMPLETE;
            lenData=0;
            
            /*check to see if other expansion units to send to */
             if((config.sys.unitType == UNIT_TYPE_MASTER) && (config.sys.numUnits > 0))
             {

                 if((expMacID == config.sys.expMac1)&&(config.sys.numUnits > 1)&&
                    (config.sys.expMac2 != 0x0013A20000000000))
                 {
                    /* pass batan to expansion 2 */
                     expansionBusSendCmd(RADIO_CMD_IRR_START,config.sys.expMac2);
                     irrCurUnitRunning = UNIT_TYPE_EXPANSION_2;
                 } 
                 else if((expMacID == config.sys.expMac2)&&(config.sys.numUnits ==3)&&
                            (config.sys.expMac3 != 0x0013A20000000000))
                 {      
                    /* pass batan to expansion 3 */
                     expansionBusSendCmd(RADIO_CMD_IRR_START,config.sys.expMac3);
                     irrCurUnitRunning = UNIT_TYPE_EXPANSION_3;
                 } 
                 else 
                 {
                     /* release control variable */
                     irrExpRunningProg = IRR_PGM_NONE;
                     sysState = SYS_STATE_IDLE;
                     expansionIrrState =IRR_STATE_IDLE;
                     expansionIrrCurZone = 0;
                     irrCurUnitRunning = UNIT_TYPE_MASTER;
                 }
                 
             }
             break;
          case RADIO_CMD_EXPANSION_STATUS:
              radioMessageLog("Expansion Status");
              debugWrite("Expansion Cmd: EXPANSION STATUS\n");
              pMsg->cmd= RADIO_ACK_EXPANSION_STATUS;
              lenData=0;
              
              /*Expansion irrigation state */              
              if(((irrCurUnitRunning == UNIT_TYPE_EXPANSION_1)&&(expMacID == config.sys.expMac1)) ||
                  ((irrCurUnitRunning == UNIT_TYPE_EXPANSION_2)&&(expMacID == config.sys.expMac2))||
                  ((irrCurUnitRunning == UNIT_TYPE_EXPANSION_3)&&(expMacID == config.sys.expMac3))                  )
              {                
                  /* take care of case where an expansion unit loses comm for a brief period of time
                   * less than the timeout detection and comes back. for example a power glitch
                   */
                  if((pMsg->data[0] == 0) && (pMsg->data[1] == 0) && 
                     (sysState != SYS_STATE_AUTORUN)&&(sysState != SYS_STATE_IDLE))
                  {
                     /* release control variable */
                     irrExpRunningProg = IRR_PGM_NONE;
                     sysState = SYS_STATE_IDLE;
                     expansionIrrState =IRR_STATE_IDLE;
                     expansionIrrCurZone = 0;
                     irrCurUnitRunning = UNIT_TYPE_MASTER;
                  }
                  else 
                  {
                      expansionSysState=pMsg->data[0];
                      expansionIrrState =pMsg->data[1];
                      expansionIrrCurZone =pMsg->data[2];
                  }
              } 
              else if((irrCurUnitRunning == UNIT_TYPE_MASTER)&&(pMsg->data[2] != 0))
              {
                  expansionSysState=pMsg->data[0];
                  expansionIrrState =pMsg->data[1];
                  expansionIrrCurZone =pMsg->data[2];
              }
                
              /* Expansion System Error Flags */
              expansionSysErrorFlags = U8TOU16(pMsg->data[3],
                                               pMsg->data[4]);
              /* Expansion System Fault Flags */
              expansionSysFaultFlags = U8TOU32(pMsg->data[5],
                                               pMsg->data[6],
                                               pMsg->data[7],               
                                               pMsg->data[8]);
              
              /* update list with moisture sensor failures and runtimes from other units */
              if(config.sys.unitType == UNIT_TYPE_MASTER)
              {               
                  /* moisture sensor zone failures */
                  tempMoistFailedSensors = U8TOU16(pMsg->data[9],pMsg->data[10]);
                  if(expMacID == config.sys.expMac1)
                    {
                          offset = 13;
                    }
                    else if(expMacID == config.sys.expMac2)
                    {      
                          offset = 25;
                    }
                    else if(expMacID == config.sys.expMac3)
                    {      
                          offset = 37;
                    }
              
                    for(i=0; i<SYS_N_UNIT_ZONES; i++)
                    {
                        if((tempMoistFailedSensors&(((uint16_t)1)<<i)) != 0)
                        {
                            moistFailureSet(i+offset);
                        }
                        else
                        {
                            moistFailureClear(i+offset);
                        }
                        
                        /*update per zone runtimes */
                        irrDailyRuntimePerZone[i+offset-1] = U8TOU16(pMsg->data[2*i+11],pMsg->data[2*i+12]);
                    }
              }
              
              /* if expansion unit state is idle and is the last unit then have master go to idle*/
              if(expansionSysState == SYS_STATE_IDLE)
              {

                  if(((config.sys.numUnits == 1)&&(expMacID == config.sys.expMac1)) ||
                     ((config.sys.numUnits == 2)&&(expMacID == config.sys.expMac2)) ||
                     ((config.sys.numUnits == 3)&&(expMacID == config.sys.expMac3))) 
                  {                     
                     /* release control variable */
                     irrExpRunningProg = IRR_PGM_NONE;
                     sysState = SYS_STATE_IDLE;
                     expansionIrrState =IRR_STATE_IDLE;
                     expansionIrrCurZone = 0;
                  }
              }
              break;
              
          case RADIO_CMD_IRR_STOP:
              radioMessageLog("Irr Stop");
              debugWrite("Expansion Cmd: IRRIGATION STOP\n");
              pMsg->cmd= RADIO_ACK_IRR_STOP;
              lenData=0;
              irrCmdStop();
              /* release control variables */
              irrExpRunningProg = IRR_PGM_NONE;
              sysState = SYS_STATE_IDLE;
              expansionIrrState =IRR_STATE_IDLE;
              expansionIrrCurZone = 0;
              break;
          case RADIO_CMD_IRR_RESUME:
              radioMessageLog("Irr Resume");
              debugWrite("Expansion Cmd: IRRIGATION RESUME\n");
              pMsg->cmd= RADIO_ACK_IRR_RESUME;
              lenData=0;
              sysResume();
              break;
          case RADIO_CMD_IRR_SKIP:
              radioMessageLog("Irr Skip");
              debugWrite("Expansion Cmd: IRRIGATION SKIP\n");
              pMsg->cmd= RADIO_ACK_IRR_SKIP;
              lenData=0;
              irrCmdSkip();
              break;
          case RADIO_CMD_IRR_OFF:
              radioMessageLog("Irr Off");
              debugWrite("Expansion Cmd: IRRIGATION OFF\n");
              pMsg->cmd= RADIO_ACK_IRR_OFF;
              lenData=0;
              /* Clear any pending auto programs. */
              irrAutoPgmPending = IRR_PGM_NONE;

              /* Stop any irrigation. */
              irrCmdStop();

              /* Set the system Off. */
              sysIsAuto = FALSE;
              break;
          case RADIO_CMD_IRR_AUTO:
              pMsg->cmd= RADIO_ACK_IRR_AUTO;
              lenData=0;
              /* Set the system state to Auto. */
              sysIsAuto = TRUE;
              break;
          case RADIO_CMD_IRR_STOP_OFF:
              radioMessageLog("Irr Stop Off");
              debugWrite("Expansion Cmd: IRRIGATION STOP OFF\n");
              pMsg->cmd= RADIO_ACK_IRR_STOP_OFF;
              lenData=0;
              irrStop = FALSE;
              break;
          case RADIO_CMD_DELETE_SC:
              radioMessageLog("Delete an SC");
              debugWrite("Expansion Cmd: Delete an SC\n");
              pMsg->cmd= RADIO_ACK_DELETE_SC;
              scDeleteList[pMsg->data[0]]=TRUE;       
              break; 
          case RADIO_CMD_SC_IS_REMOVED:
              radioMessageLog("EXP Deleted SC");
              debugWrite("Expansion Cmd: SC Deleted\n");
              pMsg->cmd= RADIO_ACK_SC_IS_REMOVED;
              radioRemoveSensorAssocList(config.sys.assocSensorCon[pMsg->data[0]].macId);       
              break;
          case RADIO_CMD_EXPAN_GET_CONFIG:
              radioMessageLog("EXP Requested Config");
              debugWrite("Expansion Cmd: Requested Configuration\n");
              pMsg->cmd= RADIO_ACK_EXPAN_GET_CONFIG;
              if(config.sys.unitType == UNIT_TYPE_MASTER) 
              {                   
                   expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, expMacID);     
              }
              break;
          default:
             /* Unknown command - call extended command handler. */
             radioExtCommandHandler(pPacket);
             return;
             break;  /* not reached */
    }

    /* Send acknowledgement packet. */
    radioProtocolAckSend(pPacket, cmdAck, data, lenData);

    /* if apply firmware command was sent then reboot processor *
     * this is done here so ACK to the apply command can be sent */
    if(pMsg->cmd==RADIO_CMD_FW_PUT_APPLY)
    {
           sprintf(&uiLcdBuf[LCD_RC(0,0)], "Updating Firmware....Rebooting....      ");
           sprintf(&uiLcdBuf[LCD_RC(1,0)], "                                        ");
           sprintf(&uiLcdBuf[LCD_RC(2,0)], "                                        ");
           sprintf(&uiLcdBuf[LCD_RC(3,0)], "                                        ");
           uiLcdCursor = 160;
            /* Write LCD buffer to hardware device driver. */
           drvLcdWrite(uiLcdBuf, uiLcdCursor);
           hwCpu_Delay100US(5000);
           drvProcessorReboot();
    }
    
    /* Yield control back to the system main polling loop. */
    radioYield = TRUE;
}

/******************************************************************************
 *
 * radioProtocolSCAssocHandler
 *
 * PURPOSE
 *      This routine handles received Association Messages from the 
 *      sensor concentrator.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of packet
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void    radioProtocolSCAssocHandler(const radioRxDataPacket_t *pPacket) 
{
    radioMsgHeader_t *pMsg = (radioMsgHeader_t *)&pPacket->data[0];
    packetSCAckData_t AckPacket;
    uint8_t sensorIndex=MAX_NUM_SC;
    char pBuf[40];
    
    
    //assocflag++;
    // fill with default values as safeguard
    radioProtocolFillAckDefaults(&AckPacket);

    /* get the source address */
    uint64_t sensorMacID = U8TOU64(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3],
                             pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
                             
    /* check to see if Sensor Concentrator is associated or been accepted to be associated
     * if it isnt, but it into temporary storage and notify the sensor concentrator UI that 
     * an sensor concentrator is trying to associate 
     */
    sensorIndex=radioCheckSensorAssocList(sensorMacID);
    if(sensorIndex == MAX_NUM_SC) 
    {     
        if((associateSCMode == TRUE) && (newSensorConcenFound == FALSE))
        {
            unassociatedSnsConMacId = sensorMacID;
            newSensorConcenFound = TRUE;
        }
    }
    else
    {
        /* FOR six associate retry, then RESET */
        if (assocFlag[sensorIndex] == TRUE)
            assocCnt[sensorIndex]++;
        else
            assocCnt[sensorIndex] = 0;
        
        //cnt2 = assocCnt[sensorIndex]; 
        if (assocCnt[sensorIndex] == 6) {
            assocFlag[sensorIndex] = FALSE;
            assocCnt[sensorIndex] = 0;
            radioReset();
        } 
        
        
        /* put message into radio log*/
        //radioMessageLog("SC Associate Message");
        
        sprintf(pBuf,"SC Assoc Msg %08X",(uint32_t)(sensorMacID & 0xFFFFFFFF));
        radioMessageLog(pBuf);
    
        /*build ACK packet*/
        
        /*check to see if unit is in irrigation mode */
        if((sysState != SYS_STATE_IDLE) && (expansionSysState != SYS_STATE_IDLE ))
        {
            AckPacket.sleepTime = SC_SLEEP_TIME_IRR;  /*hardcoded default values stored in config,h*/
        }
        else
        {
            AckPacket.sleepTime = SC_SLEEP_TIME;  /*hardcoded default values stored in config,h*/
        }
        
        AckPacket.command = SC_ASSOCIATE;
        
        /*send MAC address off WO that is in control of it */
        
        switch(config.sys.assocSensorCon[sensorIndex].zoneRange)
        {             
            case UNIT_TYPE_MASTER:
                if(config.sys.unitType == UNIT_TYPE_MASTER)
                {
                    AckPacket.controlMacId = radioMacId;
                }
                else
                {
                    AckPacket.controlMacId = config.sys.masterMac;
                }
                break;
            case UNIT_TYPE_EXPANSION_1:
                AckPacket.controlMacId = config.sys.expMac1;
                break;
            case UNIT_TYPE_EXPANSION_2:
                AckPacket.controlMacId = config.sys.expMac2;
                break;
            case UNIT_TYPE_EXPANSION_3:
                AckPacket.controlMacId = config.sys.expMac3;
                break;
        } 
        //AckPacket.controlMacId = 0xFFFF;
        AckPacket.hibTemp =SC_HIBERNATE_TEMP_C;          /*hardcoded default values stored in config,h*/
        AckPacket.hibThresTime =SC_HIBERNATE_THRES_TIME; /*hardcoded default values stored in config,h*/
        AckPacket.solenoidState = SC_ALL_CHAN_OFF;       /* all solenoids turned off, defined in config.h */
    
        /* Send acknowledgement packet. */
        radioProtocolRespSend(pPacket, &AckPacket, RADIO_TYPE_SC_STATUS,sizeof(packetSCAckData_t));
        //assocack++;
        
        
    }
    
    /* Yield control back to the system main polling loop. */
    radioYield = TRUE;
}

 
/******************************************************************************
 *
 * ConvertZone
 *
 * PURPOSE
 *      Convert the zone from a bitmapped to an index
 *
 * PARAMETERS
 *      bitZone     the current zone in bitmap form, 0100 is valve 3
 *
 * RETURN VALUE
 *      the current zone in ordinal form, 0100 is valve 4.
 *
 *****************************************************************************/
uint8_t ConvertZone(uint8_t bitZone)
{
  uint8_t ret = 0;
  
  //no way to prevent user from passing in bad values

  switch(bitZone)
  {
  case SC_CHAN_1_ON:
    ret = 0;
    break;
  case SC_CHAN_2_ON:
    ret = 1;
    break;
  case SC_CHAN_3_ON:
    ret = 2;
    break;
  case SC_CHAN_4_ON:
    ret = 3;
    break;
  }
  return ret;
}

/******************************************************************************
 *
 * radioProtocolSCStatusHandler
 *
 * PURPOSE
 *      This routine handles received status packets of sensor information
 *      from the sensor concentrator.
 *
 * NOTES
 *      When an SC checks in we have two primary tasks to carry out:
 * 
 *      1) Update internal data structures with information reported
 *         from the SC
 *
 *      2) Push some pieces of state to the SC which tells it how to
 *         operate.
 * 
 *      At any given time there can be a single SC whose state differs
 *      from the status quo.  Basically, one SC whose solenoid will
 *      be instructed to turn on.  For all other SCs, they should be
 *      instructed to be off.  The "current" SC is driven by the zone
 *      logic from irrigation.c
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of packet
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void    radioProtocolSCStatusHandler(const radioRxDataPacket_t *pPacket)
{
    packetSCStatus_t *pMsg = (packetSCStatus_t *)&pPacket->data[0];
    uint16_t crc;
    uint16_t crcMsg;
    packetSCAckData_t AckPacket;
    uint8_t sensorIndex=MAX_NUM_SC;
    uint8_t i;
    uint8_t unitTypeZoneOffset;    
    uint8_t sensorZone;
    char pBuf[40];
    
    //statusflag++;
    
    /* fill with defaults */
    radioProtocolFillAckDefaults(&AckPacket);

     /* Compute the CRC. */
    crc = crc16(((uint8_t *)pMsg) + 4,
                     RADIO_SC_STATUS_HEADER_SIZE - 4);

    /* Verify the CRC in the command message. */
    crcMsg = (pMsg->hdr.crcHigh << 8) | pMsg->hdr.crcLow;
    if (crcMsg != crc)
    {
        /* Ignore this packet. */
        return;
    }
    
    /* get the source address */
    uint64_t sensorMacID = U8TOU64(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3],
                             pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
                             
    /* check to see if MAC is in list and if it isnt tell it to deassociate */
    sensorIndex=radioCheckSensorAssocList(sensorMacID);
    
    if((sensorIndex != MAX_NUM_SC)  && (scDeleteList[sensorIndex] == FALSE)) 
    {    
    
        /*  indicate this hub already associated this controller */
        assocFlag[sensorIndex] = TRUE;
         
        /* make sure that it is associated with this unit */
        if(config.sys.assocSensorCon[sensorIndex].zoneRange == config.sys.unitType)
        {
            /* parse the data packet for the values */
            
            /* system state values */
            snsConInfo[sensorIndex].chargeRate = pMsg->chargeRate;
            snsConInfo[sensorIndex].battVoltage = (pMsg->battVoltage);
            snsConInfo[sensorIndex].solenoidState = pMsg->solenoidState;
            
            /* Setup response as if this is a normal unit */
            AckPacket.solenoidState = SC_ALL_CHAN_OFF;

            /* Check if this SC is the one that the irrigation logic
             * would like to modify state on.
             */
             
            if(irrSnsConSolUnitIndex == sensorIndex)
            {
                /* clear timeout flag */ 
                sysFaultClear(SYS_FAULT_SNSCON);
                
                /* Tell SC to use solenoid state from irrigation logic */
                AckPacket.solenoidState = irrSnsConSolenoidChan;
                
                if(irrSnsConSolenoidChan != SC_ALL_CHAN_OFF)
                {
                        uint8_t currentZone = config.sys.assocSensorCon[sensorIndex].channelZone[ConvertZone(irrSnsConSolenoidChan)];
                        
                        drvSolenoidSet(currentZone + 1, TRUE);
                        
                }
                
                if(sysIsInhibited && sysInhibitStop)
                {          
                    sysIsInhibited = FALSE;
                    sysInhibitStop = FALSE;
                    sysInhibitOffTime = dtTickCount;
                }
                
                //KAV update timeout since the SC for this zone has checked in
                radioSnsConCheckinTime = dtTickCount;
                
                /* release system from soak into water since SC checked in */
                scCheckedIn = TRUE;
                sysResume();
            }

            
            /*set zone offset to account for expansion bus */
            switch(config.sys.unitType)
            {
                case UNIT_TYPE_MASTER:
                    unitTypeZoneOffset = 0;
                    break;
                case UNIT_TYPE_EXPANSION_1:
                    unitTypeZoneOffset = 12;
                    break;  
                case UNIT_TYPE_EXPANSION_2:
                    unitTypeZoneOffset = 24;
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    unitTypeZoneOffset = 36;
                    break;
            }
            
            /* set moisture values for respective zones */
            for(i=0; i<SC_NUM_CHAN_UNIT; i++)
            {
                
                sensorZone = config.sys.assocSensorCon[sensorIndex].channelZone[i];
                
                /*verify that channel is assigned to a valid zone before proceeding */
                if(sensorZone != SC_CHAN_NOT_ASSIGNED) 
                {
                    /*apply offset to account for expansion bus */
                    sensorZone -= unitTypeZoneOffset;
                    
                    /* verify that zone is configured to be a wireless sensor */     
                    if(config.zone[sensorZone].sensorType == SNS_WIRELESS_MOIST)
                    {
                        drvMoistValueSet(sensorZone+1,(uint8_t)pMsg->sensor_chan[i]);
                    }
                }
            }
            
            /*put message into radio log */
            //radioMessageLog("SC Status Message");
            sprintf(pBuf,"SC Status Msg %08X",(uint32_t)(sensorMacID & 0xFFFFFFFF));
            radioMessageLog(pBuf);
    
            /* build ACK packet */
            AckPacket.command = SC_ASSOCIATE;
            
            /* set controller mac id */
            switch(config.sys.assocSensorCon[sensorIndex].zoneRange)
            {             
                case UNIT_TYPE_MASTER:
                    if(config.sys.unitType == UNIT_TYPE_MASTER)
                    {
                        AckPacket.controlMacId = radioMacId;
                    }
                    else
                    {
                        AckPacket.controlMacId = config.sys.masterMac;
                    }
                    break;
                case UNIT_TYPE_EXPANSION_1:
                    AckPacket.controlMacId = config.sys.expMac1;
                    break;
                case UNIT_TYPE_EXPANSION_2:
                    AckPacket.controlMacId = config.sys.expMac2;
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    AckPacket.controlMacId = config.sys.expMac3;
                    break;
            }
        }
    }
    else //sc needs to be deleted
    {
      /* if in the list but commanded to delete then do so */
      if(scDeleteList[sensorIndex] == TRUE)
      {
          scDeleteIndex=sensorIndex;
          expansionBusSendCmd(RADIO_CMD_SC_IS_REMOVED, config.sys.masterMac);
          radioRemoveSensorAssocList(sensorMacID);
           
      }
      
      /* build NACK packet */
      AckPacket.command = SC_DEASSOCIATE;
      AckPacket.controlMacId = 0;
      AckPacket.solenoidState = SC_ALL_CHAN_OFF;       /* all solenoids turned off, defined in config.h */
    }

      
    //do this for all SCs even if we are deleting them  
      
    /*check to see if unit is in irrigation mode */
    if((sysState != SYS_STATE_IDLE) || (expansionSysState != SYS_STATE_IDLE ))
    {
        if(sysIsInhibited || sysIsPaused)
        {
            // Turn off the wireless and wired solenoid if we are paused or inhibited
            AckPacket.solenoidState = SC_ALL_CHAN_OFF;
            
            
            if(irrSnsConSolenoidChan != SC_ALL_CHAN_OFF)
            {
                drvSolenoidSet(config.sys.assocSensorCon[sensorIndex].channelZone[ConvertZone(irrSnsConSolenoidChan)]+1, FALSE);
            }
        }

        // if we are in irrigation but the system is inhibited, we want to use a longer
        // sleep time so as to not kill battery life on the SCs.  For paused, we assume
        // that somebody is at the controller
        AckPacket.sleepTime = sysIsInhibited ? SC_SLEEP_TIME : SC_SLEEP_TIME_IRR;
    } 
    else 
    {
        AckPacket.sleepTime = SC_SLEEP_TIME;
    }

    AckPacket.hibTemp = SC_HIBERNATE_TEMP_C;            /*hardcoded default values stored in config,h*/
    AckPacket.hibThresTime = SC_HIBERNATE_THRES_TIME;    /*hardcoded default values stored in config,h*/
                                                           
    /* Send acknowledgement packet. */
    radioProtocolRespSend(pPacket, &AckPacket, RADIO_TYPE_SC_STATUS,sizeof(packetSCAckData_t));
    //statusack++;
    /* Yield control back to the system main polling loop. */
    radioYield = TRUE;
}



/******************************************************************************
 *
 * radioRemoveSensorAssocHandler
 *
 * PURPOSE
 *      This routine handles received status packets of sensor information
 *      from the sensor concentrator.
 *
 * PARAMETERS
 *      sensorMAC     IN  MAC of sensor concentrator to remove
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioRemoveSensorAssocHandler(uint64_t sensorMAC)
{
   packetSCAckData_t AckPacket;
   uint8_t sensorIndex = radioRemoveSensorAssocList(sensorMAC);
   uint16_t length = sizeof(packetSCAckData_t);
   
   // fill ack with default values
   radioProtocolFillAckDefaults(&AckPacket);
   
   if(sensorIndex != MAX_NUM_SC)
   {
        //build ACK packet
        AckPacket.sleepTime = 5;
        AckPacket.command = SC_DEASSOCIATE;
        AckPacket.controlMacId = 0;
        
        /* Send acknowledgement packet. */
        radioMsgHeader_t *pHdr = (radioMsgHeader_t *)(&AckPacket);
        uint32_t destH = ((sensorMAC >> 32)&0x00000000FFFFFFFF);
        uint32_t destL = (sensorMAC &0x00000000FFFFFFFF);
        uint16_t destN = 0xFFFE;

        pHdr->version = RADIO_PROTOCOL_VER;
        pHdr->msgType = RADIO_TYPE_SC_STATUS;
        radioMsgInsertCrc(pHdr, length);
        radioDataSend(radioDataFrameId(),
                  destN,
                  destH,
                  destL,
                  ((uint8_t *)&AckPacket),
                  length);
   }
   
   /* Yield control back to the system main polling loop. */
    radioYield = TRUE;     
}


/******************************************************************************
 *
 * radioProtocolXferHandler
 *
 * PURPOSE
 *      This routine handles received WaterOptimizer bulk data transfer
 *      packets.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of packet
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioProtocolXferHandler(const radioRxDataPacket_t *pPacket)
{
    radioMsgXfer_t *pMsg = (radioMsgXfer_t *)&pPacket->data[0];
    radioMsgXfer_t resp;
    uint16_t crc;
    uint16_t crcMsg;
    uint32_t i;
    //uint16_t segmentIndex;
    char debugBuf[40];
        /* rebuild the MAC of the source of the packet */
    uint64_t expUnitMacID;
    


    /* Compute the CRC. */
    crc = crc16(((uint8_t *)pMsg) + 4,
                     RADIO_XFER_HEADER_SIZE - 4 + pMsg->dataLen);

    /* Verify the CRC in the received message. */
    crcMsg = (pMsg->hdr.crcHigh << 8) | pMsg->hdr.crcLow;
    if (crcMsg != crc)
    {
        if (radioDebug)
        {
            debugWrite("Received WOIS Xfer Packet with INVALID CRC ");
            sprintf(debugBuf,"%04X (Expected %04X)\n.",
                crcMsg, crc);
            debugWrite(debugBuf);
        }
        /* Ignore this packet. */
        return;
    }

    /* Handle according to the transfer mode. */
    switch (pMsg->xferMode)
    {
        case RADIO_XMODE_CONFIG | RADIO_XMODE_GET_REQ:
            radioMessageLog("Get Cfg Segment");
            /* Get requested segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: GET CONFIG SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_GET_ACK;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_CFG_SEGS)
            {
                /* Send the requested snapshot segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = CONFIG_IMAGE_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                configSnapshotRead(i, resp.data, nBytes);
                resp.dataLen = (uint8_t)nBytes;
            }
            else
            {
                /* Invalid segment index - send ack with no data. */
                resp.dataLen = 0;
            }
            break;

        case RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_REQ:
            radioMessageLog("Put Cfg Segment");
            /* Get segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: PUT CONFIG SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            resp.dataLen = 0;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_CFG_SEGS)
            {
                /* Compute image offset and size of this download segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = CONFIG_IMAGE_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                if (pMsg->dataLen != nBytes)
                {
                    /* Incorrect data length. */
                    debugWrite("ERROR: Bad put config segment data length.\n");
                    resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_NACK;
                }
                else
                {
                    configBufferWrite(&pMsg->data, i, nBytes);
                    resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_ACK;
                }
            }
            else
            {
                /* Invalid segment index. */
                resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_NACK;
            }
            break;
        
        /*
        **  EXPANSION BUS:  Master sending config file to expansion units ACK
        */
        case RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_ACK:
            radioMessageLog("Expan Cfg Seg ACK");
            /* Get requested segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            segmentIndex++;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: GET CONFIG SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_REQ;
            /* Set segment index in response header. */
            resp.segHigh = (segmentIndex >>8);
            resp.segLow = (segmentIndex & 0x00FF);
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_CFG_SEGS)
            {
                /* Send the requested snapshot segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = CONFIG_IMAGE_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                configSnapshotRead(i, resp.data, nBytes);
                resp.dataLen = (uint8_t)nBytes;
            }
            else 
            {
               /* issue the command to the expansion unit to apply
                * the new downloaded configuration */
                expUnitMacID = U8TOU64(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3],
                             pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
                             
               expansionBusSendCmd(RADIO_CMD_CFG_PUT_APPLY, expUnitMacID);
               return; 
            }
            break;
        
        /*
        **  EXPANSION BUS:  Master sending config file to expansion units NACK
        */    
        case RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_NACK:
            break; 
        
        /*
        **  FIRMWARE DOWNLOAD: receiving new firmware from gateway
        */        
        case RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_REQ:
            radioMessageLog("Put FW Segment");
            /* Get segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: PUT FW SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            
            if(segmentIndex ==0)
            {
              newFirmwareSize = U8TOU32(pMsg->data[8],pMsg->data[9],
                                  pMsg->data[10],pMsg->data[11]);
              magicNum = U8TOU32(pMsg->data[0],pMsg->data[1],
                                  pMsg->data[2],pMsg->data[3]);
            }

            resp.dataLen = 0;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            
            
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_FW_SEGS)
            {
                /* Compute image offset and size of this download segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = newFirmwareSize + BULK_FW_HDR_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                if (pMsg->dataLen != nBytes)
                {
                    /* Incorrect data length. */
                    debugWrite("ERROR: Bad put config segment data length.\n");
                    resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_NACK;
                }
                else
                {
                    if(segmentIndex ==0)
                    {
                            //copy off the firmware header to be writtent o external
                            //flash once all packets have been received
                            memcpy(ImageData,&pMsg->data, BULK_FW_HDR_SIZE);
                        
                            if(drvExtFlashWrite((&pMsg->data[BULK_FW_HDR_SIZE]),EXT_FLASH_SEC_1, (nBytes-BULK_FW_HDR_SIZE))) 
                            {
                               resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_ACK;
                            }
                            else 
                            {
                               resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_NACK;
                               //drvExtFlashRead(EXT_FLASH_SEC_1,spiTest,(nBytes-BULK_FW_HDR_SIZE));
                            }

                    } 
                    else
                    {
                        if(extFlashBufferWrite(&pMsg->data, (i-BULK_FW_HDR_SIZE), nBytes))
                        {                      
                            resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_ACK;
                            
                            //check to see if this is the last packet
                            //if so then write the firmware header to external flash
                            if((i-BULK_FW_HDR_SIZE+nBytes)==newFirmwareSize)
                            {
                                if(drvExtFlashWrite(ImageData,FW_NEW_INFO_SPI_ADDR,  BULK_FW_HDR_SIZE) == FALSE) 
                                {
                                    resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_NACK;
                                }
                                
                                
                            }
                        } 
                        else 
                        {
                            resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_NACK;
                        }
                    }
                }
            }
            else
            {
                /* Invalid segment index. */
                resp.xferMode = RADIO_XMODE_WOIS_FW | RADIO_XMODE_PUT_NACK;
            }
            break;
            
        /*
        **  FLOW SENSOR DATA 24hrs:  Read Access to Raw EEPROM Data
        */            
        case RADIO_XMODE_FLOW | RADIO_XMODE_GET_REQ:
            radioMessageLog("Get Flow Segment");
            /* Get requested segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: GET FLOW DATA SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            resp.xferMode = RADIO_XMODE_FLOW | RADIO_XMODE_GET_ACK;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_FLOW_SEGS)
            {
                /* Send the requested snapshot segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = FLOW_DATA_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                drvEepromRead(FLOW_SNS_DATA + i, resp.data, nBytes);
                resp.dataLen = (uint8_t)nBytes;
            }
            else
            {
                /* Invalid segment index - send ack with no data. */
                resp.dataLen = 0;
            }
            break;
        
        /*
        **  LEVEL SENSOR DATA 24hrs:  Read Access to Raw EEPROM Data
        */            
        case RADIO_XMODE_LEVEL | RADIO_XMODE_GET_REQ:
            radioMessageLog("Get LevelSegment");
            /* Get requested segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: GET LEVEL DATA SEGMENT ");
                sprintf(debugBuf, "%d\n", segmentIndex);
                debugWrite(debugBuf);
            }
            resp.xferMode = RADIO_XMODE_LEVEL | RADIO_XMODE_GET_ACK;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_LEVEL_SEGS)
            {
                /* Send the requested snapshot segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = LEVEL_DATA_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                drvEepromRead(LEVEL_SNS_DATA + i, resp.data, nBytes);
                resp.dataLen = (uint8_t)nBytes;
            }
            else
            {
                /* Invalid segment index - send ack with no data. */
                resp.dataLen = 0;
            }
            break;
            
        /*
        **  PROTOCOL EXTENSION FOR DEBUG:  Read Access to Raw EEPROM Data
        */
        case RADIO_XMODE_EEPROM | RADIO_XMODE_GET_REQ:
            /* Get requested segment index from message header. */
            segmentIndex = (pMsg->segHigh << 8) | pMsg->segLow;
            if (radioDebug)
            {
                debugWrite("WOIS Xfer Req: GET EEPROM SEGMENT ");
                sprintf(debugBuf, "%d (address=0x%04X)\n", segmentIndex,
                    segmentIndex * RADIO_MAXSEGMENT);
                debugWrite(debugBuf);
            }
            resp.xferMode = RADIO_XMODE_EEPROM | RADIO_XMODE_GET_ACK;
            /* Set segment index in response header. */
            resp.segHigh = pMsg->segHigh;
            resp.segLow = pMsg->segLow;
            /* Verify segment index is within range. */
            if (segmentIndex < RADIO_EEPROM_SEGS)
            {
                /* Send the requested raw EEPROM segment. */
                i = segmentIndex * RADIO_MAXSEGMENT;
                nBytes = CONFIG_EEPROM_SIZE - i;
                if (nBytes > RADIO_MAXSEGMENT)
                {
                    nBytes = RADIO_MAXSEGMENT;
                }
                drvEepromRead(i, resp.data, nBytes);
                resp.dataLen = (uint8_t)nBytes;
            }
            else
            {
                /* Invalid segment index - send ack with no data. */
                resp.dataLen = 0;
            }
            break;

        default:
            /* Unknown or unsupported transfer mode. */
#ifdef WIN32
            radioTestProtocolXferHandler(pPacket);
#endif
            return;
            break;  /* not reached */
    }

    /* Send acknowledgement. */
    radioProtocolRespSend(pPacket,
                          &resp,
                          RADIO_TYPE_XFER,
                          RADIO_XFER_HEADER_SIZE + resp.dataLen);

    /* Yield control back to the system polling loop. */
    radioYield = TRUE;
}


/******************************************************************************
 *
 * radioProtocolLoopbackHandler
 *
 * PURPOSE
 *      This routine handles received loopback test reply packets.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of packet
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine changes the Loopback Test State from RADIO_LBS_TESTING
 *      to RADIO_LBS_SUCCESS when it receives a packet.  This routine also
 *      logs the received messsage as a "Loopback Reply".  The content of
 *      the packet is not verified.  Justification for not verifying the
 *      packet data is to allow maximum versatility with minimum overhead.
 *      Many data checks have already occurred for the packet to reach this
 *      handler, including proper XBee API framing and checksum as well as
 *      the first two payload data bytes matching the protocol version and
 *      loopback test packet type value specifications.
 *      If the Loopback Test State is not RADIO_LBS_TESTING, the packet is
 *      silently discarded.
 *
 *****************************************************************************/
static void radioProtocolLoopbackHandler(const radioRxDataPacket_t *pPacket)
{
    uint64_t rxSerialNum = U8TOU64(pPacket->data[2],pPacket->data[3],
                                   pPacket->data[4],pPacket->data[5],
                                   pPacket->data[6],pPacket->data[7],
                                   pPacket->data[8],pPacket->data[9]);
    uint64_t configSerialNum = U8TOU64(configSerialNumber[0], configSerialNumber[1],
                                      configSerialNumber[2], configSerialNumber[3],
                                      configSerialNumber[4], configSerialNumber[5],
                                      configSerialNumber[6], configSerialNumber[7]);
                                      
    switch (radioLbState)
    {
        case RADIO_LBS_IDLE:
            /* Ignore the packet. */
            break;
        case RADIO_LBS_START:
            /* Ignore the packet. */
            break;
        case RADIO_LBS_TESTING:
            /* Version check not really needed, just to reference pPacket. */
            if((pPacket->data[0] == RADIO_PROTOCOL_VER) && (rxSerialNum == configSerialNum))
            {
                // Change loopback state to indicate success. //
                radioStatus = RADIO_STATUS_ONLINE;
                radioLbState = RADIO_LBS_SUCCESS;
                radioMessageLog("Loopback Reply");        
                
            }     
            
            break;
        case RADIO_LBS_SUCCESS:
            /* Ignore the packet. */
            break;
        case RADIO_LBS_FAIL:
            /* Ignore the packet. */
            break;
        default:
            /* Ignore the packet. */
            break;
    }
}

/******************************************************************************
 *
 * radioLoopbackTestStart
 *
 * PURPOSE
 *      This routine is called to start a loopback test.  The test sends a
 *      single test packet to the PAN coordinator.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine changes the Loopback Test State (radioLbState) variable
 *      to RADIO_LBS_START, signalling the radio logic to send an XBee loopback
 *      test packet to the coordinator's default MAC address.  Test progress
 *      can be monitored by reading the radioLbState variable.  It is up to
 *      the caller of this routine to determine when the test has completed
 *      or timed out.  The caller must explicitly cancel any loopback test it
 *      starts (see radioLoopbackTestCancel routine).
 *
 *****************************************************************************/
void radioLoopbackTestStart(void)
{
    radioLbState = RADIO_LBS_START;
}


/******************************************************************************
 *
 * radioLoopbackTestStart
 *
 * PURPOSE
 *      This routine is called to cancel a loopback test.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine changes the Loopback Test State (radioLbState) variable
 *      to RADIO_LBS_IDLE, indicating that a loopback test is no longer in
 *      progress.  If a loopback test response packet is received while in the
 *      RADIO_LBS_IDLE state, it will be silently discarded.
 *
 *****************************************************************************/
void radioLoopbackTestCancel(void)
{
    radioLbState = RADIO_LBS_IDLE;
    
    if(radioStatus == RADIO_STATUS_SCANNING)
    {
      radioStatus = RADIO_STATUS_OFFLINE;
    }
}


/******************************************************************************
 *
 * radioLoopbackTest
 *
 * PURPOSE
 *      This routine is called from within the radio logic subsystem to start
 *      a loopback test.  This routine clears the last received message log
 *      and RSSI variables and sends a single test packet to the coordinator's
 *      default MAC address.  The test packet consists of a 2-byte header for
 *      protocol version and message type (indicating this is a loopback test
 *      message) followed by the WOIS serial number for additional test data.
 *      This routine changes the Loopback Test State (radioLbState) variable
 *      to RADIO_LBS_TESTING if the radio driver write call succeeds.  If the
 *      the driver write call fails, the radioLbState is set to RADIO_LBS_FAIL.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      If the loopback test is successful, the radio logic will receive a
 *      ZigBee data packet containing data that is identical to the test
 *      packet sent.  Upon receiving this loopback test response, the radio
 *      subsystem calls the radioProtocolLoopbackHandler routine to
 *      handle the received loopback test response packet.  If the radio
 *      Loopback Test State is RADIO_LBS_TESTING when the response packet
 *      is received, the radioLbState variable is set to RADIO_LBS_SUCCESS.
 *      The radioLbState variable does not change back to RADIO_LBS_IDLE
 *      automatically; a loopback test must be explicitely cancelled (see
 *      radioLoopbackTestCancel routine).
 *
 *****************************************************************************/
static void radioLoopbackTest(void)
{
    uint8_t data[CONFIG_SN_SIZE + 2];

    /*
    ** Construct loopback test data packet.
    */

    /* First byte specifies WOIS protocol version. */
    data[0] = RADIO_PROTOCOL_VER;
    /* Second byte indicates this is a loopback test packet type. */
    data[1] = RADIO_TYPE_LOOPBACK;

    /* Copy WOIS serial number for use as test data. */
    memcpy(&data[2], configSerialNumber, CONFIG_SN_SIZE);

    /* Send loopback test data packet to the coordinator's default address. */
    if (radioLoopbackDataSend(data, sizeof(data)))
    {
        /* Change loopback state to testing, waiting for loopback response. */
        radioLbState = RADIO_LBS_TESTING;
        
        if(radioStatus == RADIO_STATUS_OFFLINE)
        {
          radioStatus = RADIO_STATUS_SCANNING;
        }
    }
    else
    {
        /* Write to radio UART buffer returned failure. */
        radioLbState = RADIO_LBS_FAIL;
    }
}


/******************************************************************************
 *
 * radioProtoCmdWeatherUpdate
 *
 * PURPOSE
 *      This routine is called to process received Weather data.
 *
 * PARAMETERS
 *      etAccum     IN  received ET data accumulator value
 *      rainAccum   IN  received Rainfall accumulator value
 *
 * RETURN VALUE
 *      This routine returns TRUE if the weather data was successfully
 *      applied; otherwise this routine returns FALSE to indicate a
 *      failure.  Failure can occur if either the ET accumulator or
 *      rainfall accumulator are out of range.  When either accumulator
 *      value is out of range, no weather data is extracted however stored
 *      accumulator values are synchronized with the received accumulator
 *      values.
 *
 *****************************************************************************/
bool_t radioProtoCmdWeatherUpdate(uint32_t etAccum, uint32_t rainAccum)
{
    uint32_t etAccumDelta;
    uint32_t rainAccumDelta;
    uint16_t etData;
    uint16_t rainfall;
    bool_t result = FALSE;

    /* Calculate ET and Rainfall deltas. */
    etAccumDelta = etAccum - radioEtAccumulator;
    rainAccumDelta = rainAccum - radioRainAccumulator;

    if (etAccumDelta <= RADIO_ET_MAX_DELTA &&
        rainAccumDelta <= RADIO_RAIN_MAX_DELTA)
    {
        /* ET and Rainfall deltas are positive and within limits. */
        etData = (int16_t)etAccumDelta;
        rainfall = (int16_t)rainAccumDelta;

        /* Apply weather update. */
        irrWeatherUpdate(etData, rainfall);
        /* Set return value to indicate success. */
        result = TRUE;
    }

    /* Update radio ET and Rainfall accumulators with last received values. */
    radioEtAccumulator = etAccum;
    radioRainAccumulator = rainAccum;

    return result;
}


/******************************************************************************
 *
 * radioProtoCmdGetWeatherValues
 *
 * PURPOSE
 *      This routine is called to format the response data for 'Get Weather
 *      Values' WOIS radio commands.
 *
 * PARAMETERS
 *      pData       OUT     points to command-specific response data
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine always formats 8 bytes in the caller's pData buffer.
 *      The first 4 bytes contain the current ET Accumulator value and
 *      the next 4 bytes contain the current Rainfall Accumulator value.
 *
 *****************************************************************************/
void radioProtoCmdGetWeatherValues(uint8_t *pData)
{
    /* Set ET accumulator value in data. */
    pData[0] = (uint8_t)(radioEtAccumulator >> 24);
    pData[1] = (uint8_t)((radioEtAccumulator & 0x00FF0000) >> 16);
    pData[2] = (uint8_t)((radioEtAccumulator & 0x0000FF00) >> 8);
    pData[3] = (uint8_t)(radioEtAccumulator & 0x000000FF);
    /* Set Rainfall accumulator value in data. */
    pData[4] = (uint8_t)(radioRainAccumulator >> 24);
    pData[5] = (uint8_t)((radioRainAccumulator & 0x00FF0000) >> 16);
    pData[6] = (uint8_t)((radioRainAccumulator & 0x0000FF00) >> 8);
    pData[7] = (uint8_t)(radioRainAccumulator & 0x000000FF);
}


/******************************************************************************
 *
 * radioProtoCmdGetMbValues
 *
 * PURPOSE
 *      This routine is called to format the response data for 'Get MB Values'
 *      WOIS radio commands.
 *
 * PARAMETERS
 *      firstZone   IN      specifies the first zone to format
 *      pData       OUT     points to command-specific response data
 *
 * RETURN VALUE
 *      This routine returns the count of command-specific response bytes
 *      copied into the pData buffer.  The return value can range from 1 to 49.
 *      This routine always formats at least 1 byte in the caller's pData
 *      buffer, indicating the firstZone parameter.  The MB values for up to 24
 *      zones can be represented in subsequent 2-byte fields; the actual zone
 *      value count is limited by the number of configured zones.  To retrieve
 *      all MB values in a WOIS system having more than 24 zones, two commands
 *      are needed.  If the firstZone parameter is zero or greater than the
 *      number of configured zones, this routine formats only the firstZone
 *      parameter.
 *
 *****************************************************************************/
uint8_t radioProtoCmdGetMbValues(uint8_t firstZone, uint8_t *pData)
{
    uint8_t lenData;        /* length of command-specific response data */
    uint8_t lastZone;       /* last zone to send */
    uint8_t zone;           /* zone number */
    uint8_t di;             /* data index */
    uint16_t mbValue;       /* MB; represented as an unsigned 2-byte value */

    /* Set First Zone number in data. */
    pData[0] = firstZone;
    /* Set data length to minimum length (with no zone values returned). */
    lenData = 1;
    /* Insure First Zone is within limits. */
    if ((firstZone > 0) &&
        (firstZone <= config.sys.numZones))
    {
        /* Calculate Last Zone, assuming the 24 zone maximum. */
        lastZone = firstZone + 24 - 1;
        /* Insure Last Zone is within configured zone limits. */
        if (lastZone > config.sys.numZones)
        {
            /* Limit Last Zone to last configured zone. */
            lastZone = config.sys.numZones;
        }
        /* Set data index to next available position. */
        di = lenData;
        /* Set MB values in data. */
        for (zone = firstZone; zone <= lastZone; zone++)
        {
            /* Get moisture balance value and set in data buffer. */
            mbValue = (uint16_t)irrMbGet(zone);
            pData[di] = (uint8_t)(mbValue >> 8);
            di++;
            pData[di] = (uint8_t)(mbValue & 0x00FF);
            di++;
            lenData += 2;
        }
    }

    return lenData;
}


/******************************************************************************
 *
 * radioProtoCmdSetMbValues
 *
 * PURPOSE
 *      This routine is called to apply the MB data received by 'Set MB Values'
 *      WOIS radio commands.  This routine supports setting from 1 to 24
 *      contiguous zones.
 *
 * PARAMETERS
 *      pData   IN  points to the command data
 *      lenData IN  length of the command data
 *
 * RETURN VALUE
 *      This routine returns TRUE if the MB data was applied successfully;
 *      otherwise FALSE is returned to indicate a failure.  Failure occurs
 *      if the zone is out of range or a MB value is out of range.
 *
 * NOTES
 *      This routine verifies all MB data before committing any changes.
 *      Failure status indicates no changes were made; success status
 *      indicates all changes were made.
 *      This routine allows the MB to be set for all possible zones, including
 *      those not currently configured.
 *
 *****************************************************************************/
bool_t radioProtoCmdSetMbValues(uint8_t *pData, uint8_t lenData)
{
    uint8_t zoneStart = pData[0];
    uint8_t zoneLimit = zoneStart + ((lenData - 1) / 2);
    uint8_t zone;
    uint8_t di;
    int16_t mbValue;

    /* Verify first zone (1-48). */
    if ((zoneStart < 1) || (zoneStart > SYS_N_ZONES))
    {
        /* First zone is out-of-range. */
        return FALSE;
    }

    /* Verify zone limit (2-49). */
    if ((zoneLimit < 2) || (zoneLimit > SYS_N_ZONES + 1))
    {
        /* Zone limit is out-of-range. */
        return FALSE;
    }

    /*
    **  Verify data length.
    **  There is a 1-byte first zone header, followed by
    **  1 to 24 2-byte zone MB values.
    **  Smallest data size of 3 allows setting one zone MB value.
    **  Largest data size of 49 allows setting 24 zone MB values.
    */

    /* Verify data length is within limits. */
    if ((lenData < 3) || (lenData > 49))
    {
        /* Data length is out-of-range. */
        return FALSE;
    }

    /* Verify data length is odd. */
    if ((lenData + 1) / 2 == (lenData / 2))
    {
        /* Data length is even; there is an unnecessary extra data byte. */
        return FALSE;
    }

    /*
    **  First iterate through all MB values to check validity.
    */

    /* Set data index to first MB value. */
    di = 1;
    /* Verify MB values. */
    for (zone = zoneStart; zone < zoneLimit; zone++)
    {
        /* Get MB value. */
        mbValue = (int16_t)U8TOU16(pData[di], pData[di + 1]);
        if ((mbValue < IRR_MB_MIN) || (mbValue > CONFIG_RZWWS_MAX))
        {
            /* MB value is out-of-range. */
            return FALSE;
        }
        /* Increment data index to next value. */
        di += 2;
    }

    /*
    **  Now that validity is assured, apply the MB values.
    */

    /* Set data index to first MB value. */
    di = 1;
    /* Apply MB values. */
    for (zone = zoneStart; zone < zoneLimit; zone++)
    {
        /* Don't modify zones beyond configuration limit. */
        if (zone > config.sys.numZones)
        {
            break;
        }
        /* Get MB value. */
        mbValue = (int16_t)U8TOU16(pData[di], pData[di + 1]);
        /* Apply MB value to zone. */
        irrMbSet(zone, mbValue);
        /* Increment data index to next value. */
        di += 2;
    }

    return TRUE;
}


/******************************************************************************
 *
 * radioProtocolAckSend
 *
 * PURPOSE
 *      This routine sends command acknowledgement messages in response to
 *      received WaterOptimizer application command packets.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of received command packet
 *      cmdAck      IN  application command acknowledgement response code
 *      pData       IN  pointer to start of command-specific response data
 *      lenData     IN  length of the command-specific response data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioProtocolAckSend(const radioRxDataPacket_t *pPacket,
                                 uint8_t cmdAck,
                                 const uint8_t *pData,
                                 uint8_t lenData)
{
    radioMsgAck_t msg;
    radioMsgCmd_t *cmdMsg = (radioMsgCmd_t *)&pPacket->data[0];

    msg.msgId = cmdMsg->msgId;
    msg.cmd = cmdAck;
    msg.dataLen = RADIO_ACK_MIN_DATA_SIZE + lenData;

    memcpy(msg.data, configSerialNumber, CONFIG_SN_SIZE);

    sysRadioResponseBytes(&msg.data[8], &msg.data[9]);

    memcpy(&msg.data[10], pData, lenData);

    radioProtocolRespSend(pPacket,
                          &msg,
                          RADIO_TYPE_ACK,
                          RADIO_CMD_HEADER_SIZE + msg.dataLen);
}


/******************************************************************************
 *
 * radioProtocolFillAckDefaults
 *
 * PURPOSE
 *      This routine fills an SC ACK packet with default values
 *
 * PARAMETERS
 *      pAckPacket  IN  pointer to uninitialized ACK packet
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioProtocolFillAckDefaults(packetSCAckData_t *pAckPacket)
{
    pAckPacket->command = SC_DEASSOCIATE;
    pAckPacket->controlMacId = 0;
    pAckPacket->hibTemp = SC_HIBERNATE_TEMP_C;
    pAckPacket->hibThresTime = SC_HIBERNATE_THRES_TIME;
    pAckPacket->sleepTime = SC_SLEEP_TIME;
    pAckPacket->solenoidState = SC_ALL_CHAN_OFF;
}


/******************************************************************************
 *
 * radioProtocolRespSend
 *
 * PURPOSE
 *      This routine sends protocol response messages to the destination
 *      address specified in the received data packet.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of received data packet
 *      pMsg        IN  pointer to start of protocol response message
 *      msgType     IN  protocol message type
 *      length      IN  length of the protocol response message
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioProtocolRespSend(const radioRxDataPacket_t *pPacket,
                                  const void *pMsg,
                                  uint8_t msgType,
                                  uint8_t length)
{
    radioMsgHeader_t *pHdr = (radioMsgHeader_t *)pMsg;
    uint32_t destH = U8TOU32(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3]);
    uint32_t destL = U8TOU32(pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
    uint16_t destN = U8TOU16(pPacket->netAddr[0], pPacket->netAddr[1]);

    pHdr->version = RADIO_PROTOCOL_VER;
    pHdr->msgType = msgType;
    radioMsgInsertCrc(pMsg, length);
    radioDataSend(radioDataFrameId(),
                  destN,
                  destH,
                  destL,
                  (uint8_t *)pMsg,
                  length);
}


/******************************************************************************
 *
 * radioMsgInsertCrc
 *
 * PURPOSE
 *      This routine calculates and inserts the correct CRC checksum into
 *      the caller's protocol message.
 *
 * PARAMETERS
 *      pMsg        IN/OUT  pointer to message
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void radioMsgInsertCrc(const void *pMsg, uint8_t length)
{
    uint16_t crc;
    radioMsgHeader_t *hdr = (radioMsgHeader_t *)pMsg;

    crc = crc16((uint8_t *)pMsg + 4, length - 4);

    hdr->crcHigh = (uint8_t)(crc >> 8);
    hdr->crcLow = (uint8_t)(crc & 0x00FF);
}


/******************************************************************************
 *
 * radioDataFrameId
 *
 * PURPOSE
 *      This routine is called to generate a Frame ID to use in the API header
 *      for ZigBee Transmit Data frames.  The Frame ID value is incremented on
 *      each call, until a maximum value is reached and the value wraps around
 *      to the beginning of its range.  This routine returns at least 100
 *      unique values before recycling.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns a value for use in the Frame ID field of the
 *      ZigBee Tx Data API header.
 *
 * NOTES
 *      The Frame ID is used to correlate ZigBee Transmit Status responses
 *      with their respective ZigBee Transmit Data frames.  This allows the
 *      potential for a ZigBee Tx Data frame to be retransmitted when a
 *      transmission failure is reported by the ZigBee radio module.
 *
 *****************************************************************************/
uint8_t radioDataFrameId(void)
{
    uint8_t frameId = radioDataFrameIdNext;

    if (++radioDataFrameIdNext > RADIO_DATA_FRAME_ID_MAX)
    {
        radioDataFrameIdNext = RADIO_DATA_FRAME_ID_FIRST;
    }

    return (frameId);
}


/******************************************************************************
 *
 * radioCmdPendingInit
 *
 * PURPOSE
 *      This routine initializes the command-pending queue, setting all
 *      entries unused.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioCmdPendingInit(void)
{
    uint8_t i;

    /* Initialize radio command pending list to empty. */
    radioCmdPendingFirst = RADIO_CPQ_NONE;
    radioCmdPendingLast = RADIO_CPQ_NONE;

    for (i = 0; i < RADIO_CPQ_SIZE; i++)
    {
        radioCmdPending[i].inUse = FALSE;
        radioCmdPending[i].prev = RADIO_CPQ_NONE;
        radioCmdPending[i].next = RADIO_CPQ_NONE;
    }
}


/******************************************************************************
 *
 * radioCmdPendingAdd
 *
 * PURPOSE
 *      This routine adds a command to the command-pending queue.  If the
 *      command was just sent, the 'send' flag should be set to FALSE
 *      and the current time will be noted for command-retry purposes.
 *      If the caller wishes to enqueue a command for automatic sending,
 *      the 'send' flag should be set to TRUE.  The send time will be
 *      set to zero, informing the Command Retry Manager to send the command
 *      at its next opportunity.
 *
 * PARAMETERS
 *      ident   IN  command identifier
 *      send    IN  Set to TRUE to request command to be enqueued for auto-send
 *                  with auto-retry if no response is received before time-out.
 *                  If the command has just been sent, set to FALSE to request
 *                  the command to be enqueued for auto-retry only.
 *
 * RETURN VALUE
 *      Returns TRUE if the command was successfully added.  Returns FALSE
 *      if the command pending queue is full.
 *
 *****************************************************************************/
static bool_t radioCmdPendingAdd(uint8_t ident, bool_t send)
{
    bool_t wasAdded = FALSE;
    uint8_t i;

    /* First delete any previous instances of this command. */
    radioCmdPendingDelete(ident);

    /* Now add this command to the pending queue. */
    for (i = 0; i < RADIO_CPQ_SIZE; i++)
    {
        /* Find an empty slot in the array. */
        if (!radioCmdPending[i].inUse)
        {
            /* Mark slot in use. */
            radioCmdPending[i].inUse = TRUE;
            /* Store command identifier. */
            radioCmdPending[i].ident = ident;
            if (send)
            {
                /* Set time value to elicit immediate retry. */
                radioCmdPending[i].timeSent = RADIO_CPQ_AUTOSEND;
            }
            else
            {
                /* Store current time tick count. */
                radioCmdPending[i].timeSent = dtTickCount;
            }
            /* Update forward and backward links. */
            radioCmdPending[i].prev = radioCmdPendingLast;
            radioCmdPending[i].next = RADIO_CPQ_NONE;
            if (radioCmdPendingLast != RADIO_CPQ_NONE)
            {
                /* Link last member to this new member. */
                radioCmdPending[radioCmdPendingLast].next = i;
            }
            /* Set last member to this new member. */
            radioCmdPendingLast = i;
            /* Check if a first member already exists. */
            if (radioCmdPendingFirst == RADIO_CPQ_NONE)
            {
                /* Set first member to this new member. */
                radioCmdPendingFirst = i;
            }
            wasAdded = TRUE;
            break;
        }
    }

    return wasAdded;
}


/******************************************************************************
 *
 * radioCmdPendingDelete
 *
 * PURPOSE
 *      This routine removes a command from the command-pending queue.  All
 *      instances of the command are removed.
 *
 * PARAMETERS
 *      ident       IN  command identifier
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioCmdPendingDelete(uint8_t ident)
{
    uint8_t i;

    for (i = 0; i < RADIO_CPQ_SIZE; i++)
    {
        if (radioCmdPending[i].inUse)
        {
            if (radioCmdPending[i].ident == ident)
            {
                /* Unlink this element from the linked-list. */
                if (radioCmdPending[i].prev != RADIO_CPQ_NONE)
                {
                    /* Update previous link member's next index. */
                    radioCmdPending[radioCmdPending[i].prev].next =
                        radioCmdPending[i].next;
                }
                else
                {
                    /* This element was first in list. Update first index. */
                    radioCmdPendingFirst = radioCmdPending[i].next;
                }
                if (radioCmdPending[i].next != RADIO_CPQ_NONE)
                {
                    /* Update next link member's prev index. */
                    radioCmdPending[radioCmdPending[i].next].prev =
                        radioCmdPending[i].prev;
                }
                else
                {
                    /* This element was last in list. Update last index. */
                    radioCmdPendingLast = radioCmdPending[i].prev;
                }
                /* Clean-up this deleted array element. */
                radioCmdPending[i].inUse = FALSE;
                radioCmdPending[i].prev = RADIO_CPQ_NONE;
                radioCmdPending[i].next = RADIO_CPQ_NONE;
            }
        }
    }
}


/******************************************************************************
 *
 * radioCmdPendingGet
 *
 * PURPOSE
 *      This routine searches the command pending queue to find the next
 *      eligible command to send.  The queue is first searched for the oldest
 *      command that has been enqueued for auto-send.
 *      If no auto-send commands are found, then the queue is searched for the
 *      the oldest timed-out command.  If an eligible command is found, it is
 *      removed from the command-pending queue and returned to the caller.
 *
 * PARAMETERS
 *      pIdent  OUT     If the return value is TRUE, this parameter is set to
 *                      the identifier of a command to be sent or retried.
 *
 * RETURN VALUE
 *      This routine returns TRUE if an eligible command was found to send or
 *      retry.  This routine returns FALSE if no eligible command was found.
 *
 *****************************************************************************/
static bool_t radioCmdPendingGet(uint8_t *pIdent)
{
    uint8_t i;

    /* First search for a command enqueued for auto-send. */
    i = radioCmdPendingFirst;
    while (i != RADIO_CPQ_NONE)
    {
        if (radioCmdPending[i].timeSent == RADIO_CPQ_AUTOSEND)
        {
            *pIdent = radioCmdPending[i].ident;
            radioCmdPendingDelete(radioCmdPending[i].ident);
            return TRUE;
        }
        i = radioCmdPending[i].next;
    }
    /* Next search for a command that has timed-out. */
    i = radioCmdPendingFirst;
    while (i != RADIO_CPQ_NONE)
    {
        if (dtElapsedSeconds(radioCmdPending[i].timeSent) > RADIO_CMD_RETRY_SECS)
        {
            *pIdent = radioCmdPending[i].ident;
            radioCmdPendingDelete(radioCmdPending[i].ident);
            return TRUE;
        }
        i = radioCmdPending[i].next;
    }

    return FALSE;
}


/******************************************************************************
 *
 * radioDebugAtResponse
 *
 * PURPOSE
 *      This routine formats the AT command response received from an XBee
 *      API AT command response packet and writes it to the debug output.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to XBee API AT Command response packet
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine currently supports only "AI" command responses.
 *
 *****************************************************************************/
static void radioDebugAtResponse(radioResponsePacket_t *pPacket)
{
    if (pPacket->frameId == RADIO_CMD_AI)
    {
        switch (pPacket->data[0])
        {
            case RADIO_AISTAT_JOINED:
                debugWrite(" - AI status = JOINED");
                break;
            case RADIO_AISTAT_NOPANS:
                debugWrite(" - AI status = NO PANS FOUND");
                break;
            case RADIO_AISTAT_NOVALPANS:
                debugWrite(" - AI status = NO VALID PANS FOUND");
                break;
            case RADIO_AISTAT_NJEXP:
                debugWrite(" - AI status = PAN NOT ALLOWING JOINS");
                break;
            case RADIO_AISTAT_NJFAIL:
                debugWrite(" - AI status = JOIN ATTEMPT FAILED");
                break;
            case RADIO_AISTAT_COFAIL:
                debugWrite(" - AI status = COORDINATOR START FAILED");
                break;
            case RADIO_AISTAT_CHECKING:
                debugWrite(" - AI status = CHECKING FOR EXISTING COORDINATOR");
                break;
            case RADIO_AISTAT_SCANNING:
                debugWrite(" - AI status = SCANNING FOR PARENT");
                break;
            default:
                debugWrite(" - AI status = (unknown)");
                break;
        }
    }
}


/******************************************************************************
 *
 * radioMessageLog
 *
 * PURPOSE
 *      This routine logs a received radio message for radio communications
 *      audit and debug use.
 *
 * PARAMETERS
 *      pMsgDesc    IN  pointer to message description string (up to 20 chars)
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The message description string must be terminated with a null and
 *      is truncated if longer than 20 characters.
 *      Currently, only the last message is saved for audit purposes, however
 *      if this routine is modified in the future to maintain a list of
 *      previous messages, a more efficient date/time storage mechanism would
 *      be desirable.
 *
 *****************************************************************************/
static void radioMessageLog(const char *pMsgDesc)
{
    char buf[41];
    int len;

    /* Safely copy message description string to global memory. */
    len = (int)strlen(pMsgDesc);
    if (len > 25)
    {
        len = 25;
    }
    memcpy(radioLastMsgDesc, pMsgDesc, len);
    radioLastMsgDesc[len] = '\0';

    /* Safely copy date/time string to global memory. */
    dtFormatDebugDateTime(buf);
    len = (int)strlen(buf);
    if (len > 20)
    {
        len = 20;
    }
    memcpy(radioLastMsgDate, buf, len);
    radioLastMsgDate[len] = '\0';
}



/******************************************************************************
 *
 *  RADIO FEATURE DEMO AND WINDOWS-BASED TEST EXTENSIONS
 *
 *****************************************************************************/


/******************************************************************************
 *
 * radioExtCommandHandler
 *
 * PURPOSE
 *      This routine is called by the WOIS protocol command handler to handle
 *      commands it doesn't recognize.  This routine provides Debug and Demo
 *      command implementations outside the normal command handler.
 *
 * PARAMETERS
 *      pPacket     IN      pointer to received command packet
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine contains hooks used on the Windows-based WOIS simulation
 *      platform; these hooks are only compiled if the preprocessor definition
 *      for "WIN32" is set.
 *
 *****************************************************************************/
static void radioExtCommandHandler(const radioRxDataPacket_t *pPacket)
{
    radioMsgCmd_t *pMsg = (radioMsgCmd_t *)&pPacket->data[0];
    uint8_t data[50];
    bool_t msgSent = FALSE;
    int i;

    /* Handle the command. */
    switch (pMsg->cmd)
    {
        case RADIO_CMD_RC_TEST:
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: RC TEST\n");
            }
            radioProtocolAckSend(pPacket, RADIO_ACK_RC_TEST, NULL, 0);
            break;

        case RADIO_CMD_RC_EVENT:
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: RC EVENT\n");
            }
            for (i = 0; i < pMsg->dataLen; i++)
            {
                drvKeypadPut(pMsg->data[i]);
            }

            /* DESIGN CHANGE NOTE:  Don't ACK remote control event commands. */
            /* radioExtAckSend(pPacket, RADIO_ACK_RC_EVENT, NULL, 0); */

            /* Yield remaining radio poll cycle so UI can process the event. */
            radioYield = TRUE;
            break;

        case RADIO_CMD_RC_GETLCD:
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: RC GET LCD\n");
            }
            data[0] = uiPosition;
            data[1] = (uiLcdCursor == DRV_LCD_CURSOR_OFF) ? 0xFF : uiLcdCursor;

            /*
            **  The Get LCD command sends a checksum of each of its four lines
            **  of LCD data.  Compare these against checksums of the current
            **  LCD content and send LCD data for each line that is different.
            */
            if (!radioCheckSumIsValid(uiLcdBuf, 40, pMsg->data[0], pMsg->data[1]))
            {
                data[2] = 0;
                memcpy(&data[3], uiLcdBuf, 40);
                radioExtAckSend(pPacket, RADIO_ACK_RC_GETLCD, data, 43);
                msgSent = TRUE;
            }

            if (!radioCheckSumIsValid(uiLcdBuf + 40, 40, pMsg->data[2], pMsg->data[3]))
            {
                data[2] = 1;
                memcpy(&data[3], uiLcdBuf + 40, 40);
                radioExtAckSend(pPacket, RADIO_ACK_RC_GETLCD, data, 43);
                msgSent = TRUE;
            }

            if (!radioCheckSumIsValid(uiLcdBuf + 80, 40, pMsg->data[4], pMsg->data[5]))
            {
                data[2] = 2;
                memcpy(&data[3], uiLcdBuf + 80, 40);
                radioExtAckSend(pPacket, RADIO_ACK_RC_GETLCD, data, 43);
                msgSent = TRUE;
            }

            if (!radioCheckSumIsValid(uiLcdBuf + 120, 40, pMsg->data[6], pMsg->data[7]))
            {
                data[2] = 3;
                memcpy(&data[3], uiLcdBuf + 120, 40);
                radioExtAckSend(pPacket, RADIO_ACK_RC_GETLCD, data, 43);
                msgSent = TRUE;
            }

            if (!msgSent)
            {
                /* No screen data changed, just send cursor and function. */
                radioExtAckSend(pPacket, RADIO_ACK_RC_GETLCD, data, 2);
            }

            /* Yield for other subsystems to have a poll opportunity. */
            radioYield = TRUE;
            break;

        /* SAVE EVENT LOG */
        case RADIO_CMD_EVENT_LOG:
            if (pMsg->dataLen == 1)
            {
                if (pMsg->data[0] == 1)
                {
                    debugWrite("WOIS Cmd: Event Log Save\n");
                    configEventLogSave();
                }
                else
                {
                    debugWrite("WOIS Cmd: Event Log Query\n");
                }
                data[0] = (uint8_t)(SYS_EVENT_LIMIT >> 8);
                data[1] = (uint8_t)(SYS_EVENT_LIMIT & 0x00FF);
                data[2] = (uint8_t)(sizeof(sysEventLog_t) >> 8);
                data[3] = (uint8_t)(sizeof(sysEventLog_t) & 0x00FF);
                data[4] = (uint8_t)(CONFIG_EVENT_LOG >> 8);
                data[5] = (uint8_t)(CONFIG_EVENT_LOG & 0x00FF);
                radioProtocolAckSend(pPacket, RADIO_ACK_EVENT_LOG, data, 6);
            }
            break;

        case RADIO_CMD_GET_FW_VER:
            if (radioDebug)
            {
                debugWrite("WOIS Cmd: Get Firmware Version\n");
            }
            data[0] = sysFirmwareVer.major;
            data[1] = sysFirmwareVer.minor;
            data[2] = sysFirmwareVer.patch;
            data[3] = sysFirmwareVer.seq;
            radioProtocolAckSend(pPacket, RADIO_ACK_GET_FW_VER, data, 4);
            break;

#ifdef WIN32
        // These cases are for instrumenting remote control on the Windows platform.
        case RADIO_ACK_RC_TEST:
            RemoteControlTestAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_RC_EVENT:
            RemoteControlEventAck(pMsg->msgId);
            break;

        case RADIO_ACK_RC_GETLCD:
            RemoteControlGetLcdAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_GET_FW_VER:
            GetFwVerAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_CFG_PUT_START:
            CloneStartAck();
            break;

        case RADIO_ACK_CFG_PUT_APPLY:
            CloneApplyAck(pMsg->data[10], U8TOU16(pMsg->data[11], pMsg->data[12]));
            break;

        case RADIO_ACK_CFG_GET_SNAP:
            CloneUpStartAck();
            break;

        case RADIO_ACK_EVENT_LOG:
            CloneEvtStartAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_GET_MOIST:
            GetMoistAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_GET_EXSTAT:
            GetExStatAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_GET_MB_VALUES:
            GetMbValuesAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_SET_MB_VALUES:
            SetMbValuesAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_WEATHER_DATA:
            WeatherUpdateAck((char *)pMsg->data, pMsg->dataLen);
            break;

        case RADIO_ACK_INIT_DATETIME:
            InitDateTimeAck((char *)pMsg->data, pMsg->dataLen);
            break;
#endif

        default:
            /* Unknown command. */
            break;  /* not reached */
    }
}


/******************************************************************************
 *
 * radioCheckSumIsValid
 *
 * PURPOSE
 *      This routine is called to verify the CRC-16 checksum of a block of
 *      data matches the high and low CRC bytes provided.
 *
 * PARAMETERS
 *      pData   IN  pointer to start of data
 *      lenData IN  length of data
 *      sumH    IN  high byte of CRC checksum
 *      sumL    IN  low byte of CRC checksum
 *
 * RETURN VALUE
 *      This routine returns TRUE if the CRC-16 checksum of the provided data
 *      matches the high and low CRC bytes provided; otherwise FALSE is
 *      returned.
 *
 * NOTES
 *      This routine is used by the remote control LCD screen get command.
 *
 *****************************************************************************/
static bool_t radioCheckSumIsValid(const void *pData,
                                   uint8_t lenData,
                                   uint8_t sumH,
                                   uint8_t sumL)
{
    uint16_t sum = crc16(pData, lenData);

    return ((sumH == (sum >> 8)) && (sumL == (sum & 0x00FF)));
}


/******************************************************************************
 *
 * radioExtAckSend
 *
 * PURPOSE
 *      This routine provides the same interface as the radioProtocolAckSend
 *      function but sends command acknowledgement messages without the 10-byte
 *      standard response data (serial number and current status).  The only
 *      data sent in the command acknowledgement is the data provided by the
 *      caller.  This routine allows smaller, more efficient response packets
 *      and is intended for use with debug and remote control commands.
 *
 * PARAMETERS
 *      pPacket     IN  pointer to start of received command packet
 *      cmdAck      IN  application command acknowledgement response code
 *      pData       IN  pointer to start of command response data
 *      lenData     IN  length of the command response data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void radioExtAckSend(const radioRxDataPacket_t *pPacket,
                            uint8_t cmdAck,
                            const uint8_t *pData,
                            uint8_t lenData)
{
    radioMsgAck_t msg;
    radioMsgCmd_t *cmdMsg = (radioMsgCmd_t *)&pPacket->data[0];

    msg.msgId = cmdMsg->msgId;
    msg.cmd = cmdAck;
    msg.dataLen = lenData;

    memcpy(msg.data, pData, lenData);

    radioProtocolRespSend(pPacket,
                          &msg,
                          RADIO_TYPE_ACK,
                          RADIO_CMD_HEADER_SIZE + msg.dataLen);
}



/******************************************************************************
 *
 *  RADIO SENSOR CONCENTRATOR FUNCTIONS
 *
 *****************************************************************************/

/******************************************************************************
 *
 * radioCheckSensorAssocList
 *
 * PURPOSE
 *      This routine provides the same interface as the radioProtocolAckSend
 *      function but sends command acknowledgement messages without the 10-byte
 *      standard response data (serial number and current status).  The only
 *      data sent in the command acknowledgement is the data provided by the
 *      caller.  This routine allows smaller, more efficient response packets
 *      and is intended for use with debug and remote control commands.
 *
 * PARAMETERS
 *      sensorMAC     IN  MAC ID of sensor concentrator that sent message
 *
 * RETURN VALUE
 *      uint8_t    returns the index in the list at which MAC was found.
 *                 returns MAX_NUM_SC if not found.
 *
 *****************************************************************************/
static uint8_t radioCheckSensorAssocList(uint64_t sensorMAC)
{
   uint8_t  i;
   uint8_t retIndex = MAX_NUM_SC;
   
   for(i=0; i<MAX_NUM_SC; i++)
   {
      if(config.sys.assocSensorCon[i].macId == sensorMAC)
      {
          retIndex =i;
          break;
      }
   }
   
   return retIndex;      
}


/******************************************************************************
 *
 * radioAddSensorAssocList
 *
 * PURPOSE
 *      This routine provides the same interface as the radioProtocolAckSend
 *      function but sends command acknowledgement messages without the 10-byte
 *      standard response data (serial number and current status).  The only
 *      data sent in the command acknowledgement is the data provided by the
 *      caller.  This routine allows smaller, more efficient response packets
 *      and is intended for use with debug and remote control commands.
 *
 * PARAMETERS
 *      sensorMAC     IN  MAC ID of sensor concentrator that sent message
 *
 * RETURN VALUE
 *      uint_8    index in associated sensor list where it was added.
 *
 *****************************************************************************/
uint8_t radioAddSensorAssocList(uint64_t sensorMAC)
{
   uint8_t  i;
   bool_t retIndex = MAX_NUM_SC;
   
   for(i=0; i<MAX_NUM_SC; i++)
   {
      if(config.sys.assocSensorCon[i].macId == 0)
      {
          retIndex=i;
          config.sys.assocSensorCon[i].macId = sensorMAC;
          config.sys.numSensorCon += 1;
          break;
      }
   }
   
   return retIndex;      
}

/******************************************************************************
 *
 * radioRemoveSensorAssocList
 *
 * PURPOSE
 *      This routine removes a sensor concentrator MAC ID from the associated
 *      list.  It does this by setting that list index to 0. This returns the 
 *      that it was removed from.
 *
 * PARAMETERS
 *      sensorMAC     IN  MAC ID of sensor concentrator that sent message
 *
 * RETURN VALUE
 *      uint_8    index in associated sensor list where it was removed from.
 *
 *****************************************************************************/
uint8_t radioRemoveSensorAssocList(uint64_t sensorMAC)
{
    uint8_t  i, j;
    bool_t retIndex = MAX_NUM_SC;
    uint8_t chanZone;
    int8_t assocSensorConIndex;

    // find the index in config.sys.assocSensorCon of the provided sensorMAC
    assocSensorConIndex = -1;
    for(i = 0; i < MAX_NUM_SC; i++)
    {
        if(config.sys.assocSensorCon[i].macId == sensorMAC)
        {
            assocSensorConIndex = i;
            break;
        }
    }

    // clear out measurement storage as well
    snsConInfo[assocSensorConIndex].chargeRate = 0;
    snsConInfo[assocSensorConIndex].battVoltage = 0;
    snsConInfo[assocSensorConIndex].solenoidState = 0;
    config.sys.assocSensorCon[assocSensorConIndex].macId = 0;
    scDeleteList[assocSensorConIndex] = FALSE;
    
    //Look to see if any other SCs after this one are marked for deleteion
            
    for(i= assocSensorConIndex; i<MAX_NUM_SC; i++)
    {
      if(scDeleteList[i] == TRUE)
      {
        //Move remaining SCs up one in the list to keep the position correct
        while( i< MAX_NUM_SC)
        {
          scDeleteList[i-1] = scDeleteList[i]; 
          i++;
        }
        break;
      }
    }

    
    if(config.sys.numSensorCon != 0)
    {
        config.sys.numSensorCon -= 1;
    }

    // change the zone settings back to defaults and no sensor for each zone
    // associated with this sensor concentrator
    for(i = 0; i < SC_NUM_CHAN_UNIT; i++)
    {
        chanZone = config.sys.assocSensorCon[assocSensorConIndex].channelZone[i];
        if(chanZone != SC_CHAN_NOT_ASSIGNED)
        {
            config.zone[chanZone].snsConChan = SC_NONE_SELECTED;
            config.zone[chanZone].snsConTableIndex = ZONE_SC_INDEX_NONE;
            config.zone[chanZone].sensorType = SNS_NONE;
            config.zone[chanZone].group = CONFIG_GROUP_NONE;
        }
        config.sys.assocSensorCon[assocSensorConIndex].channelZone[i] = SC_CHAN_NOT_ASSIGNED;
    }

    // Clean up items from config.zone where the snsConTableIndex to be
    // removed is referenced (but not a part of channelZone).
    for(i = 0; i < SYS_N_ZONES; i++)
    {
        if (config.zone[i].snsConTableIndex == assocSensorConIndex)
        {
            config.zone[i].snsConChan = SC_NONE_SELECTED;
            config.zone[i].snsConTableIndex = ZONE_SC_INDEX_NONE;
            config.zone[i].sensorType = SNS_NONE;
            config.zone[i].group = CONFIG_GROUP_NONE;
        }
    }

    // Move the remaining SCs down one.  This allows us to maintain the
    // invariant that is always the lowest indices of assocSensorCon that
    // are filled.  For each sensor concentrator shifted, we must also change
    // the index pointers (for now, just zones)
    for(i = assocSensorConIndex; i < MAX_NUM_SC; i++)
    {
         if(config.sys.assocSensorCon[i+1].macId != 0)
         {
             // update assocSensorCon
             config.sys.assocSensorCon[i].macId = config.sys.assocSensorCon[i+1].macId;
             config.sys.assocSensorCon[i + 1].macId = 0;
             config.sys.assocSensorCon[i].zoneRange = config.sys.assocSensorCon[i+1].zoneRange;
             config.sys.assocSensorCon[i + 1].zoneRange = 0;
             for (j = 0; j < sizeof(config.sys.assocSensorCon[i + 1].channelZone); j++)
             {
                 config.sys.assocSensorCon[i].channelZone[j] = config.sys.assocSensorCon[i+1].channelZone[j];
                 config.sys.assocSensorCon[i + 1].channelZone[j] = 0;
             }

             // update snsConInfo array
             snsConInfo[i].battVoltage = snsConInfo[i + 1].battVoltage;
             snsConInfo[i + 1].battVoltage = 0;
             snsConInfo[i].chargeRate = snsConInfo[i + 1].chargeRate;
             snsConInfo[i + 1].chargeRate = 0;
             snsConInfo[i].solenoidState = snsConInfo[i + 1].solenoidState;
             snsConInfo[i + 1].solenoidState = 0;

             // scan zones for any zones referencing the old index.  Update that
             // zone to point to index x - 1 where x is the previous index.
             for (j = 0; j < MAX_NUM_SC; j++)
             {
                 if (config.zone[j].snsConTableIndex == (i + 1))
                 {
                     config.zone[j].snsConTableIndex = i;
                 }
             }
         }
    }

   return assocSensorConIndex;
}


/******************************************************************************
 *
 *  RADIO EXPANSION BUS FUNCTIONS
 *
 *****************************************************************************/
 
 
/******************************************************************************
 *
 * expansionBusForwardCmd
 *
 * PURPOSE
 *      This routine forwards on a WOIS command message to the expansion
 *      units for them to respond to. This is also used for the master to send
 *      a command to all of the expansion units at the same time.
 *
 * PARAMETERS
 *      pPacket   the packet that was received and needs to be forwarded on
 *
 * RETURN VALUE
 *      void
 *
 *****************************************************************************/
static void expansionBusForwardCmd(const radioRxDataPacket_t *pPacket)
{
    radioMsgCmd_t *pMsg = (radioMsgCmd_t *)&pPacket->data[0];
    uint16_t destN= U8TOU16(pPacket->netAddr[0], pPacket->netAddr[1]);
    uint32_t destH;
    uint32_t destL;
    
    switch(config.sys.numUnits)
    {
      case 3:
          if(config.sys.expMac3 != 0x0013A20000000000)
          {
            radioExp3RespTime = dtTickCount;  /* set time counter for timeout response */
            radioSentExpan3Msg = TRUE;
            destH = (uint32_t)(config.sys.expMac3 >>32);
            destL = (uint32_t)(config.sys.expMac3 & 0xFFFFFFFF); 
            radioDataSend(radioDataFrameId(),
                        destN,
                        destH,
                        destL,
                        (uint8_t *)pMsg,
                        RADIO_CMD_HEADER_SIZE + pMsg->dataLen);
          }
      case 2:
          if(config.sys.expMac2 != 0x0013A20000000000)
          {
            radioExp2RespTime = dtTickCount;  /* set time counter for timeout response */
            radioSentExpan2Msg = TRUE;
            destH = (uint32_t)(config.sys.expMac2 >>32);
            destL = (uint32_t)(config.sys.expMac2 & 0xFFFFFFFF); 
            radioDataSend(radioDataFrameId(),
                        destN,
                        destH,
                        destL,
                        (uint8_t *)pMsg,
                        RADIO_CMD_HEADER_SIZE + pMsg->dataLen);
          }

      case 1:
          if(config.sys.expMac1 != 0x0013A20000000000)
          {
            radioExp1RespTime = dtTickCount;  /* set time counter for timeout response */
            radioSentExpan1Msg = TRUE;
            destH = (uint32_t)(config.sys.expMac1 >>32);
            destL = (uint32_t)(config.sys.expMac1 & 0xFFFFFFFF); 
            radioDataSend(radioDataFrameId(),
                        destN,
                        destH,
                        destL,
                        (uint8_t *)pMsg,
                        RADIO_CMD_HEADER_SIZE + pMsg->dataLen);
          }
      default:
          break;
    }

}

/******************************************************************************
 *
 * expansionBusSingleCmd
 *
 * PURPOSE
 *      This routine forwards on a WOIS command message to the expansion
 *      units for them to respond to
 *
 * PARAMETERS
 *      pPacket   the packet that needs to be sent to a single expansion unit
 *      macId     the MAC ID of the unit to receive the command
 *
 * RETURN VALUE
 *      void
 *
 *****************************************************************************/
static void expansionBusSingleCmd(const radioRxDataPacket_t *pPacket, uint64_t macId)
{
    radioMsgCmd_t *pMsg = (radioMsgCmd_t *)&pPacket->data[0];
    uint16_t destN= U8TOU16(pPacket->netAddr[0], pPacket->netAddr[1]);
    uint32_t destH;
    uint32_t destL;
    
     destH = (uint32_t)(macId >>32);
     destL = (uint32_t)(macId & 0xFFFFFFFF); 
     
     if(macId != 0x0013A20000000000)
     {
        radioDataSend(radioDataFrameId(),
                        destN,
                        destH,
                        destL,
                        (uint8_t *)pMsg,
                        RADIO_CMD_HEADER_SIZE + pMsg->dataLen);
     }
}


/******************************************************************************
 *
 * expansionBusSendCmd
 *
 * PURPOSE
 *      This routine sends a WOIS command message from the master to each of the 
 *      expansion units associated with the master.
 *
 * PARAMETERS
 *      command   Value of the command that is to be sent
 *      macId     mac ID of the unit that the command is to be sent to.
 *                if sending it to all units then sent the value to 0
 *
 * RETURN VALUE
 *      void
 *
 *****************************************************************************/
void expansionBusSendCmd(uint8_t command, uint64_t macId)
{
    radioRxDataPacket_t pPacket;
    radioMsgCmd_t *pMsg = (radioMsgCmd_t *)&pPacket.data[0];
    radioMsgHeader_t *pHdr = (radioMsgHeader_t *)pMsg;
    uint8_t length=RADIO_CMD_HEADER_SIZE;
    uint8_t i;
    
    /*if(macId == radioMacId)
    {
        if(macId == config.sys.expMac1)
        {
            config.sys.expMac1 = 0x0013A20000000000;
        }
        else if(macId == config.sys.expMac2)
        {
            config.sys.expMac2 = 0x0013A20000000000;
        }
        else if(macId == config.sys.expMac3)
        {
            config.sys.expMac3 = 0x0013A20000000000;
        }
        return;
    }    */
    
    switch(command)
    {
      case RADIO_CMD_NO_OP:
          pMsg->cmd= RADIO_CMD_NO_OP;
          pMsg->dataLen=0;
          break;
      case RADIO_CMD_GET_MOIST:
          pMsg->cmd= RADIO_CMD_GET_MOIST;
          pMsg->dataLen=0;
          break;
      case RADIO_CMD_GET_EXSTAT:
          pMsg->cmd= RADIO_CMD_GET_EXSTAT;
          pMsg->dataLen=0;
          break;
      case RADIO_CMD_CFG_PUT_START:
          pMsg->cmd= RADIO_CMD_CFG_PUT_START;
          pMsg->dataLen=0;
          break;
      case RADIO_CMD_CFG_PUT_APPLY:
          pMsg->cmd= RADIO_CMD_CFG_PUT_APPLY;
          pMsg->dataLen=0;
          break;
      case RADIO_CMD_GET_MB_VALUES:          
          /* each expansion unit controls valves 1-12 on its 
           *own system, but take up different zone numbers in the
           *greater scheme of things.  */
          pMsg->cmd= RADIO_CMD_GET_MB_VALUES;
          pMsg->dataLen=1;
          
          /*set the first moisture balance zone to be 1.
           *each expansion controls on its own unit from 1 to 12
           *want the data for all configured zones from each of the expansions */
          pMsg->data[0]= 1;
          length +=1;
          break;
      case RADIO_CMD_INIT_DATETIME:
          pMsg->cmd= RADIO_CMD_INIT_DATETIME;
          pMsg->dataLen=6;
          pMsg->data[0]= (uint8_t)(dtYear - 2000);
          pMsg->data[1]= dtMon+1;
          pMsg->data[2]= dtMday;
          pMsg->data[3]= dtHour;
          pMsg->data[4]= dtMin;
          pMsg->data[5]= dtSec;          
          length +=6;
          break;
      case RADIO_CMD_SEND_FLOW:
          pMsg->cmd= RADIO_CMD_SEND_FLOW;
          pMsg->dataLen=4;
          pMsg->data[0]= GPM;
          pMsg->data[1]= (flowDelay >> 8);
          pMsg->data[2]= (flowDelay & 0x00FF);
          pMsg->data[3]= findFlow;        
          length +=4;  
          break;
      case RADIO_CMD_IRR_START:
          pMsg->cmd= RADIO_CMD_IRR_START;
          pMsg->dataLen=4;
          pMsg->data[0]=irrExpRunningProg;
          pMsg->data[1]=sysState;
          pMsg->data[2]=irrOpMode;
          pMsg->data[3]=irrPulseMode;
          length +=4;
          
          radioCheckExpanStatusTime = dtTickCount;
          break; 
      case RADIO_CMD_IRR_COMPLETE:
          pMsg->cmd= RADIO_CMD_IRR_COMPLETE;
          pMsg->dataLen=1;
          pMsg->data[0]=irrProgram;
          length +=1;
          break;
      case RADIO_CMD_EXPANSION_STATUS:
          pMsg->cmd= RADIO_CMD_EXPANSION_STATUS;
          pMsg->dataLen=35;
          length +=35;
          /*irrigation state */
          pMsg->data[0]=sysState;
          pMsg->data[1]=irrState;
          if(config.sys.unitType == UNIT_TYPE_EXPANSION_1)
          {
              pMsg->data[2]=irrCurZone+12;
          }
          else if(config.sys.unitType == UNIT_TYPE_EXPANSION_2)
          {
              pMsg->data[2]=irrCurZone+24;
          } 
          else if(config.sys.unitType == UNIT_TYPE_EXPANSION_3)
          {
              pMsg->data[2]=irrCurZone+36;
          } 
          /* System Error Flags */
          pMsg->data[3]=(uint8_t)(sysErrorFlags >> 8);
          pMsg->data[4]=(uint8_t)(sysErrorFlags & 0xFF);
          /* System Fault Flags */
          pMsg->data[5]=(uint8_t)(sysFaultFlags >> 24);
          pMsg->data[6]=(uint8_t)((sysFaultFlags & 0x00FF0000) >> 16); 
          pMsg->data[7]=(uint8_t)((sysFaultFlags & 0x0000FF00) >> 8);
          pMsg->data[8]=(uint8_t)(sysFaultFlags & 0x000000FF);
          /* moisture sensor zone failures */
          pMsg->data[9]=(uint8_t)((moistFailedSensors & 0x0000000000000F00) >> 8);
          pMsg->data[10]=(uint8_t)((moistFailedSensors & 0x00000000000000FF));
          
          /*send zone runtimes to master */
          for(i=0; i<SYS_N_UNIT_ZONES; i++)
          {
                pMsg->data[2*i+11]=(irrDailyRuntimePerZone[i] >> 8);
                pMsg->data[2*i+12]=(irrDailyRuntimePerZone[i] & 0xFF);    
          }         
          break;
       case RADIO_CMD_IRR_STOP:
          pMsg->cmd= RADIO_CMD_IRR_STOP;
          pMsg->dataLen=0;
          /* release control variable */
          irrExpRunningProg = IRR_PGM_NONE;
          sysState = SYS_STATE_IDLE;
          expansionIrrState =IRR_STATE_IDLE;
          expansionIrrCurZone = 0;
          break;
       case RADIO_CMD_IRR_RESUME:
          pMsg->cmd= RADIO_CMD_IRR_RESUME;
          pMsg->dataLen=0;
          break;
       case RADIO_CMD_IRR_SKIP:
          pMsg->cmd= RADIO_CMD_IRR_SKIP;
          pMsg->dataLen=0;
          break;
       case RADIO_CMD_IRR_OFF:
          pMsg->cmd= RADIO_CMD_IRR_OFF;
          pMsg->dataLen=0;
          break; 
       case RADIO_CMD_IRR_AUTO:
          pMsg->cmd= RADIO_CMD_IRR_AUTO;
          pMsg->dataLen=0;
          break;
       case RADIO_CMD_IRR_STOP_OFF:
           pMsg->cmd= RADIO_CMD_IRR_STOP_OFF;
           pMsg->dataLen=0;
           break; 
       case RADIO_CMD_DELETE_SC:
           pMsg->cmd= RADIO_CMD_DELETE_SC;
           pMsg->dataLen=1;           
           /* index of SC to delete */
           pMsg->data[0]=scDeleteIndex;
           length +=1;
           break;
        case RADIO_CMD_SC_IS_REMOVED:
           pMsg->cmd= RADIO_CMD_SC_IS_REMOVED;
           pMsg->dataLen=1;           
           /* index of SC to delete */
           pMsg->data[0]=scDeleteIndex;
           length +=1;
           break;
         case RADIO_CMD_EXPAN_GET_CONFIG:
           pMsg->cmd= RADIO_CMD_EXPAN_GET_CONFIG;
           pMsg->dataLen=0;
           break; 
      default:
          /* not a supported command */
          return;
    }
    
    pHdr->version = RADIO_PROTOCOL_VER;
    pHdr->msgType = RADIO_TYPE_CMD;
    radioMsgInsertCrc(pMsg, length);
    
    if(macId == RADIO_EXP_SEND_ALL )
    {      
        /* send command to each expansion unit */
        expansionBusForwardCmd(&pPacket);

    }
    else 
    {
        /* set time counter for timeout response */
        if(macId == config.sys.masterMac)
        {
            radioMasterRespTime = dtTickCount;
            radioSentMasterMsg = TRUE;
        }
        else if(macId == config.sys.expMac1)
        {
            radioExp1RespTime = dtTickCount;
            radioSentExpan1Msg = TRUE;
        }
        else if(macId == config.sys.expMac2)
        {
            radioExp2RespTime = dtTickCount;
            radioSentExpan2Msg = TRUE;
        } 
        else if(macId == config.sys.expMac3){          
            radioExp3RespTime = dtTickCount;
            radioSentExpan3Msg = TRUE;
        }
        /*send command to only 1 expansion unit */
        expansionBusSingleCmd(&pPacket, macId);
    }
}


/******************************************************************************
 *
 * expansionBusCmdHandler
 *
 * PURPOSE
 *      This routine handles responses to commands that were issued from this unit
 *      to expansion units. It determines what unit sent the response back and the
 *      command it it responding to. It then acts accordingly to the command.
 *
 * PARAMETERS
 *      pPacket     a pointer to the received radio packet 
 *
 * RETURN VALUE
 *      void
 *
 *****************************************************************************/
void expansionBusCmdHandler(const radioRxDataPacket_t *pPacket)
{
    radioMsgAck_t *pMsg = (radioMsgAck_t *)&pPacket->data[0]; 
    uint8_t moistStart=0;
    uint8_t moistEnd=0;
    uint8_t i;
    
    /* rebuild the MAC of the source of the packet */
    uint64_t expUnitMacID = U8TOU64(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3],
                             pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
    
    /* inform the system that communication was received from the proper expansion */
    if(expUnitMacID ==config.sys.expMac1)
    {
        radioStatusExpansion1 = EXPANSION_CONNECTED;
        radioSentExpan1Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_1);
    }
    else if(expUnitMacID ==config.sys.expMac2)
    {
        radioStatusExpansion2 = EXPANSION_CONNECTED;
        radioSentExpan2Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_2);
    }
    else if(expUnitMacID ==config.sys.expMac3)
    {
        radioStatusExpansion3 = EXPANSION_CONNECTED;
        radioSentExpan3Msg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_3);
    }
    else if(expUnitMacID ==config.sys.masterMac)
    {
        radioSentMasterMsg = FALSE;
        sysFaultClear(SYS_FAULT_EXPAN_MASTER);
    }
    
    
    /* handle each response */                         
    switch(pMsg->cmd)
    {
      case RADIO_CMD_NO_OP:
          radioMessageLog("Expan: No Op ACK");
          break;
      case RADIO_CMD_INHIBIT_ON:
          radioMessageLog("Expan: Inhibit On ACK");
          break;
      case RADIO_CMD_INHIBIT_OFF:
          radioMessageLog("Expan: Inhibit Off ACK");
          break;
      case RADIO_CMD_FORCE_ON:
          radioMessageLog("Expan: Force On ACK");
          break;
      case RADIO_CMD_WEATHER_DATA:
          radioMessageLog("Expan: Weather Data ACK");
          break;
      case RADIO_CMD_INIT_DATETIME:
          radioMessageLog("Expan: Init DateTime ACK");
          break;
      case RADIO_CMD_GET_MOIST:
          radioMessageLog("Expan: Get Moist ACK");
          
          /* determine which unit responded and load
           * the data into the correct zones */
          if(expUnitMacID ==config.sys.expMac1)
          {   
                moistStart = 13;
                moistEnd = 24;
          }
          else if(expUnitMacID ==config.sys.expMac2)
          {
                moistStart = 25;
                moistEnd = 36;
          }
          else if(expUnitMacID ==config.sys.expMac3)
          {
                moistStart = 37;
                moistEnd = 48;
          }
          
          /* load moisture data from expansion unit into moisture array */
          for(i= moistStart; i < moistEnd;i++)
          {
             //drvMoistValueSet(i,pMsg->data[i-moistStart+10]);
             expMoistValue[i-13] =  pMsg->data[i-moistStart+10];
          }
          break;
      case RADIO_CMD_GET_EXSTAT:
          radioMessageLog("Expan: Get Ext Stat ACK");
          
          
          break;
      case RADIO_CMD_CFG_PUT_START:
          radioMessageLog("Expan: Put Cfg Start ACK");
          
          /* update the configuration snapshot buffer before reading from it
           * this puts the latest version of the configuration image into the
           * snapshot buffer which is what is sent to the expansion units. */
          configSnapshotSave();
          
          /* start the bulk transfer of the config file 
           * by sending segment 0 of the configuration file */
          expansionSendCfgSeg0(pPacket);
          break;
      case RADIO_CMD_CFG_PUT_APPLY:
          radioMessageLog("Expan: Cfg Apply ACK");
          
          /*check to see if the configuration update of expansion unit 
           *was successful */
          if(pMsg->data[0] == RADIO_RESULT_FAILURE)
          {
              /* resend the configuration from the beginning*/
              expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, expUnitMacID);
          } 
          else
          {
              expansionBusSendCmd(RADIO_CMD_INIT_DATETIME, expUnitMacID);
          }
          break;
      case RADIO_CMD_GET_MB_VALUES:
          radioMessageLog("Expan: Get MB Val ACK");
         
          /*account for which unit data came from so it is placed in the
          *master zone list in the proper zone */
          if(expUnitMacID ==config.sys.expMac1)
          {
              pMsg->data[0] += 12;
          }
          else if(expUnitMacID ==config.sys.expMac2)
          {
              pMsg->data[0] += 24;
          }
          else if(expUnitMacID ==config.sys.expMac3)
          {
              pMsg->data[0] += 36;
          }
         
          /* Set Moisture Balance Values from command data. */ 
          if(radioProtoCmdSetMbValues(pMsg->data, pMsg->dataLen)==FALSE) 
          {
              /* failure since zone or values are out of bounds */
          }
            
          break;
      default:
          /* not a supported command */
          return;
    }
    
    /* Yield control back to the system polling loop. */
    radioYield = TRUE;
        
}


/******************************************************************************
 *
 * expansionSendCfgSeg0
 *
 * PURPOSE
 *      When an expansion unit responds to the command to start the configuration
 *      download it will then send the first segment (segment 0) of the configuration
 *      data image to kick off the complete data transfer. When the expansion unit
 *      responds to this packet the bulkDataTransferHandler will take control of 
 *      sending the remaining packets of the data image.
 * PARAMETERS
 *      pPacket     a pointer to the received radio packet 
 *
 * RETURN VALUE
 *      void
 *
 *****************************************************************************/
void expansionSendCfgSeg0(const radioRxDataPacket_t *pPacket)
{
   radioMsgXfer_t resp;   
   configSys_t  * pExpanConfigSys = (configSys_t  *)resp.data;
   configImage_t expanConfig;
   
   /* rebuild the MAC of the source of the packet */
    uint64_t expUnitMacID = U8TOU64(pPacket->phyAddr[0],
                             pPacket->phyAddr[1],
                             pPacket->phyAddr[2],
                             pPacket->phyAddr[3],
                             pPacket->phyAddr[4],
                             pPacket->phyAddr[5],
                             pPacket->phyAddr[6],
                             pPacket->phyAddr[7]);
                             
   resp.xferMode = RADIO_XMODE_CONFIG | RADIO_XMODE_PUT_REQ;
   
   /* Set segment index in response header. */
   resp.segHigh = 0x00;
   resp.segLow = 0x00;
   configSnapshotRead(0, resp.data, RADIO_MAXSEGMENT);
   resp.dataLen = (uint8_t)RADIO_MAXSEGMENT;
   
   
   configSnapshotRead(0, &expanConfig, sizeof(configImage_t));
   /*set the master mac id */
   expanConfig.sys.masterMac = radioMacId;
   pExpanConfigSys->masterMac = radioMacId;
    
   if(expUnitMacID ==config.sys.expMac1)
   {
      expanConfig.sys.unitType = UNIT_TYPE_EXPANSION_1;
      pExpanConfigSys->unitType = UNIT_TYPE_EXPANSION_1; 
   }
   else if(expUnitMacID ==config.sys.expMac2)
   {
      expanConfig.sys.unitType = UNIT_TYPE_EXPANSION_2;
      pExpanConfigSys->unitType = UNIT_TYPE_EXPANSION_2;
   }
   else if(expUnitMacID ==config.sys.expMac3)
   {
      expanConfig.sys.unitType = UNIT_TYPE_EXPANSION_3;
      pExpanConfigSys->unitType = UNIT_TYPE_EXPANSION_3;
   }
   
   
    /* recompute the CRC checksum */                                        
    pExpanConfigSys->checkSum=configMemorySnapShotChecksumCalc((uint8_t *)&expanConfig);
   
    /* Send segment 0 of config to expansion unit */
    radioProtocolRespSend(pPacket,
                          &resp,
                          RADIO_TYPE_XFER,
                          RADIO_XFER_HEADER_SIZE + resp.dataLen);   
}




