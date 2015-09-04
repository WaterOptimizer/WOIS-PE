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
 * Module       : system.c
 * Description  : This file implements the system manager.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "datetime.h"
#include "radio.h"
#include "ui.h"
#include "irrigation.h"
#include "moisture.h"
#include "drvLed.h"
#include "drvSys.h"
#include "drvRtc.h"
#include "drvExtFlash.h"



/* System Events */
#define SYS_EVENT_INIT_START    SYS_EVENT_SYS + 1   /* App. Init Start */
#define SYS_EVENT_INIT_FINISH   SYS_EVENT_SYS + 2   /* App. Init Finish */
#define SYS_EVENT_PWR_FAIL      SYS_EVENT_SYS + 3   /* Main Power Failure */
#define SYS_EVENT_PWR_RECOV     SYS_EVENT_SYS + 4   /* Main Power Recovery */
#define SYS_EVENT_PWR_RECOV_H   SYS_EVENT_SYS + 5   /* Main Pwr Recov secs H */
#define SYS_EVENT_24V_FAIL      SYS_EVENT_SYS + 6   /* 24V Power Failure */
#define SYS_EVENT_24V_RECOV     SYS_EVENT_SYS + 7   /* 24V Power Recovery */
#define SYS_EVENT_INHIB_ON      SYS_EVENT_SYS + 8   /* Inhibit On Command */
#define SYS_EVENT_INHIB_OFF     SYS_EVENT_SYS + 9   /* Inhibit Off Command */
#define SYS_EVENT_PAUSE         SYS_EVENT_SYS + 10  /* Pause Command */
#define SYS_EVENT_RESUME        SYS_EVENT_SYS + 11  /* Resume Command */
#define SYS_EVENT_SHUTDOWN      SYS_EVENT_SYS + 12  /* Shutdown Start */
 
/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/

uint32_t sysPowerFailTime;              /* Time of last power failure */
uint32_t sysInhibitOnTime;              /* Last Inhibit On Time (in ticks) */
uint32_t sysInhibitOffTime;             /* Last Inhibit Off Time (in ticks) */
uint32_t sysFaultFlags = 0;             /* System Fault Flags */
uint16_t sysErrorFlags = 0;             /* System Error Flags */
uint8_t sysState = SYS_STATE_IDLE;      /* current operating state */
uint8_t sysMaxZones = SYS_N_UNIT_ZONES;  /* max number of zones supported */
uint8_t sysTestRunTime = 1;             /* per-zone test run-time in mins */
uint8_t sysManualProgram = 0;           /* selected manual program A=0..D=3 */
uint8_t sysManualOpMode = CONFIG_OPMODE_RUNTIME;    /* manual irr. op mode */
uint8_t sysManualPulseMode = CONFIG_PULSEMODE_OFF;  /* manual irr pulse mode */
bool_t  sysIsAuto = FALSE;              /* system is ON/AUTO if TRUE */
bool_t sysIsInhibited = FALSE;          /* Inhibit Flag On=TRUE */
bool_t sysIsPaused = FALSE;             /* Paused Flag On=TRUE */
bool_t sysResetRequest = FALSE;         /* Request a system reset if TRUE */
sysEventLog_t sysEventLog;              /* System Event Log Data Store */
bool_t sysInhibitStop = FALSE;          /* Inhibit off command recieved, wait for SC to checkin before clearing */

/*
** Expansion Units State and Status Data
*/
uint8_t expansionIrrState;                       /* Expansion unit Irrigation State */
uint8_t expansionIrrCurZone;                     /* Expansion Current Irrigation Zone (13..48) */
uint8_t expansionSysState = SYS_STATE_IDLE;      /* Expansion current operating state */
uint32_t expansionSysFaultFlags = 0;             /* Expansion System Fault Flags */
uint16_t expansionSysErrorFlags = 0;             /* Expansion System Error Flags */
uint8_t radioStatusExpansion1 = EXPANSION_NOT_CONNECTED;       /* Expansion radio connection status */
uint8_t radioStatusExpansion2 = EXPANSION_NOT_CONNECTED;       /* Expansion radio connection status */
uint8_t radioStatusExpansion3 = EXPANSION_NOT_CONNECTED;       /* Expansion radio connection status */


/* Firmware Version */
//#pragma message( "Building Version 23.04 Alpha 0" )
const sysVersion_t sysFirmwareVer = { 0, 23, 04, SYS_SEQ_ALPHA + 0 };
const char sysVersionDate[12] = "11-14-2014";



/******************************************************************************
 *
 * sysInit
 *
 * PURPOSE
 *      This routine is called by the device driver initialization function to
 *      initialize all of the application layer subsystems and begin the main
 *      polling loop.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      Upon completion, this routine calls the sysPoll function to enter
 *      the system's main polling loop, which never returns.
 *
 *****************************************************************************/
void sysInit(void)
{
    
    
    /*
    ** Initialize all application subsystems.
    */

    /* Trace system init start event. */
    sysEvent(SYS_EVENT_INIT_START, drvSysResetReason());

    debugWrite("\n\nPowering up...\n");


    
    /* Initialize configuration subsystem. */
    configInit();

    /* Initialize user interface subsystem. */
    uiInit();

    /* Initialize date/time subsystem. */
    dtInit();

    /* Initialize moisture sensor subsystem. */
    moistInit();

    /* Initialize irrigation subsystem. */
    irrInit();

    /* Initialize radio subsystem. */
    radioInit();

    /* Default to AUTO operation after cold start. */
    sysIsAuto = TRUE;

    /* Trace system init finished event. */
    sysEvent(SYS_EVENT_INIT_FINISH, 0);

    drvSysWatchDogClear();
    debugWrite("WaterOptimizer system initialized.\n");


#ifndef WIN32
    /* Transfer control to system polling loop. */
    sysPoll();
#endif

    /* Never reached. */
}


/******************************************************************************
 *
 * sysPoll
 *
 * PURPOSE
 *      This routine is called by the system manager initialization function to
 *      run the system main polling loop.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine tests for power failure before each subsystem is polled.
 *      This routine never exits (except if running in Windows simulation).
 *
 *****************************************************************************/
void sysPoll(void)
{
#if 0
    /* FOLLOWING FLAGS SET FOR TESTING/DEBUG ONLY */
    sysErrorFlags |= 1;
    sysFaultFlags |= 1;
#endif

#ifndef WIN32
    /* Run this loop forever. */
    for (;;)
    {
#endif

        /* Test for system reset request. */
        if (sysResetRequest)
        {
            /* Loop until watchdog reset occurs. */
            for ( ; ; )
            {
                /* Allow configuration manager to finish synchronization. */
                if ((configState == CONFIG_STATE_WRITING1) ||
                    (configState == CONFIG_STATE_WRITING2))
                {
                    configPoll();
                }
            }
        }

        /* Clear the watchdog counter. */
        drvSysWatchDogClear();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the date & time subsystem. */
        dtPoll();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the moisture sensor subsystem. */
        moistPoll();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the irrigation subsystem. */
        irrPoll();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the user interface subsystem. */
        uiPoll();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the radio subsystem. */
        radioPoll();

        /*** TEST POWER ***/
        sysPollPowerCheck();

        /* Poll the configuration subsystem. */
        configPoll();

#ifndef WIN32
    }
#endif
}


/******************************************************************************
 *
 * sysPollPowerCheck
 *
 * PURPOSE
 *      This routine is called by the system manager main poll loop to check
 *      for loss of power and initiate low-power mode for battery operation.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      In case of power failure, this routine does not return until AC power
 *      is restored.
 *
 *****************************************************************************/
void sysPollPowerCheck(void)
{
    uint8_t powerRestoredSecs = 0;
    uint32_t powerOutSecs;

    if (!drvSys12vdcOk())
    {
        /* System main power failure detected. */

        /* Insure watchdog is freshly-tamed. */
        drvSysWatchDogClear();

        /* Trace system main power failure event. */
        sysEvent(SYS_EVENT_PWR_FAIL, 0);

        dtDebug("*** MAIN POWER FAILED ***\n");

        /* Set system to pause mode. */
        sysPause();

        /* If currently watering, poll irrigation app to process the pause. */
        if (irrState == IRR_STATE_WATERING)
        {
            irrPoll();
        }

        /* Insure watchdog is tamed again before more debug writes. */
        drvSysWatchDogClear();

        debugWrite("Entering low-power mode...\n\n");

        /* Save Time of power failure. */
        sysPowerFailTime = dtTickCount;

        /* Trace system shutdown start. */
        sysEvent(SYS_EVENT_SHUTDOWN, 0);

        /* Shutdown the device drivers and switch to low-power mode. */
        drvSysShutdown();

        /* Operate in STOP3 mode until power is restored for 3 seconds. */
        while (powerRestoredSecs < 3)
        {
            /* Enter STOP3 state. */
            drvSysStop();

            /* Wake-up due to RTC interrupt. */
            /* Maintain real time clock. */
            dtPollRtc();
            /* Maintain total watering time daily roll-over operation. */
            dtNewMinute();

            /* Test for AC power restored. */
            if (drvSys12vdcOk())
            {
                /* Maintain power-restored counter. */
                powerRestoredSecs++;
            }
            else
            {
                /* Reset power-restored counter. */
                powerRestoredSecs = 0;
            }
        }
        /* System main power restored for 3 successive seconds. */

        /* Trace system main power restored event and power-out sec count. */
        powerOutSecs = dtElapsedSeconds(sysPowerFailTime);
        if (powerOutSecs > 0xFFFF)
        {
            sysEvent(SYS_EVENT_PWR_RECOV_H, (uint16_t)(powerOutSecs >> 16));
        }
        sysEvent(SYS_EVENT_PWR_RECOV, (uint16_t)(powerOutSecs & 0x0000FFFF));

        /* Restart the device drivers and resume normal power mode. */
        drvSysRestart();

        /* Insure watchdog is tamed before debug write. */
        drvSysWatchDogClear();

        dtDebug("*** MAIN POWER RESTORED ***\n");

        /* Re-initialize the radio. */
        radioInit();

        /* Restart the configuration manager. */
        configRestart();

        /* Get moisture sample updates. */
        moistSampleAll();

        /* Resume irrigation only if 24VAC is also okay. */
        if (drvSys24vacOk())
        {
            /* Resume irrigation. */
            sysResume();
        }
    }

//KAV remove for Mains powered WO
//#warning "!!!*** Remember to build both Solar and AC powered versions!"
//
//#define SOLAR_POWERED
#ifndef SOLAR_POWERED
//#warning "!!**Non Solar Build"

    if (!drvSys24vacOk())
    {
        /* 24 VAC power has failed.  Did it just fail? */
        if (!sysFaultOn(SYS_FAULT_24VAC))
        {
            /* Trace 24VAC power failure event. */
            sysEvent(SYS_EVENT_24V_FAIL, 0);

            /* Insure watchdog is tamed before debug write. */
            drvSysWatchDogClear();
            dtDebug("*** 24V AC FAILED ***\n");

            /* Set 24VAC hardware fault. */
            sysFaultSet(SYS_FAULT_24VAC);
        }
        /* Pause any active irrigation program. */
        sysPause();
    }
    else
    {
        /* 24 VAC power is okay.  Did it just recover? */
        if (sysFaultOn(SYS_FAULT_24VAC))
        {
            /* Trace 24VAC power restored event. */
            sysEvent(SYS_EVENT_24V_RECOV, 0);

            /* Insure watchdog is tamed before debug write. */
            drvSysWatchDogClear();
            dtDebug("*** 24 VAC RESTORED ***\n");

            /* Clear 24VAC hardware fault. */
            sysFaultClear(SYS_FAULT_24VAC);

            /* Resume irrigation. */
            sysResume();
        }
    }
    #else
    //#warning "Solar build"
    #endif
}


/******************************************************************************
 *
 * sysRadioResponseBytes
 *
 * PURPOSE
 *      This routine returns 2 bytes of system status information for use
 *      in radio command acknowledgement response.
 *
 * PARAMETERS
 *      pException  OUT     system exception flags
 *      pStatus     OUT     system status
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysRadioResponseBytes(uint8_t *pException, uint8_t *pStatus)
{
    uint8_t exception = 0;  /* exception flags */
    uint8_t status;         /* system status */
    uint8_t opMode;         /* current operating mode */
    uint8_t pulseMode;      /* current pulse mode */

    /* Use configured operation mode as default. */
    opMode = config.sys.opMode;
    pulseMode = config.sys.pulseMode;

    switch (sysState)
    {
        case SYS_STATE_IDLE:
            status = RADIO_SYSTAT_IDLE;
            break;
        case SYS_STATE_AUTORUN:
            status = RADIO_SYSTAT_AUTORUN;
            break;
        case SYS_STATE_TEST:
            status = RADIO_SYSTAT_TEST;
            break;
        case SYS_STATE_MANUAL:
            status = RADIO_SYSTAT_MANUAL;
            /* Override configured operation mode with irrigation mode. */
            opMode = irrOpMode;
            pulseMode = irrPulseMode;
            break;
        case SYS_STATE_FORCE:
            status = RADIO_SYSTAT_FORCE;
            /* Override configured operation mode with irrigation mode. */
            opMode = irrOpMode;
            pulseMode = irrPulseMode;
            break;
        default:
            /* This case should not occur - default status to Idle. */
            status = RADIO_SYSTAT_IDLE;
            break;
    }

    /* Add current mode of operation to high status nibble. */
    switch (opMode)
    {
        case CONFIG_OPMODE_RUNTIME:
            status |= RADIO_SYSTAT_RUNTIME;
            break;
        case CONFIG_OPMODE_SENSOR:
            status |= RADIO_SYSTAT_SENSOR;
            break;
        case CONFIG_OPMODE_WEATHER:
            status |= RADIO_SYSTAT_WEATHER;
            break;
        default:
            /* This case should not occur. */
            break;
    }
    if (pulseMode == CONFIG_PULSEMODE_ON)
    {
        status |= RADIO_SYSTAT_PULSE;
    }

    if (!sysIsAuto)
    {
        exception |= RADIO_EXCEP_OFF;
    }
    if (sysIsInhibited)
    {
        exception |= RADIO_EXCEP_INHIBITED;
    }
    if (sysErrorFlags != 0)
    {
        exception |= RADIO_EXCEP_SYSERROR;
    }
    if (sysFaultFlags != 0)
    {
        exception |= RADIO_EXCEP_HWFAULT;
    }
    if (sysIsPaused)
    {
        exception |= RADIO_EXCEP_PAUSED;
    }

    *pException = exception;
    *pStatus = status;
}


/******************************************************************************
 *
 * sysRadioExtStatusGet
 *
 * PURPOSE
 *      This routine returns radio command response data for the Get Extended
 *      System Status command.
 *
 * PARAMETERS
 *      pData   OUT     pointer to extended status data buffer
 *
 * RETURN VALUE
 *      This routine returns a count of the number of extended status bytes
 *      written to the buffer starting at pData.
 *
 * NOTES
 *      The Extended System Status is 17 bytes includes the following:
 *          - System Error Flags (2 bytes)
 *          - System Fault Flags (4 bytes)
 *          - Configuration Data Checksum (2 bytes)
 *          - Date/Time (Y, M, D, h, m - 5 bytes)
 *          - Total Watering Minutes Today (2 bytes)
 *          - Total Watering Minutes Yesterday (2 bytes)
 *
 *****************************************************************************/
uint8_t sysRadioExtStatusGet(uint8_t *pData)
{
    /* System Error Flags */
    pData[0] = (uint8_t)(sysErrorFlags >> 8);
    pData[1] = (uint8_t)(sysErrorFlags & 0xFF);
    /* System Fault Flags */
    pData[2] = (uint8_t)(sysFaultFlags >> 24);
    pData[3] = (uint8_t)((sysFaultFlags & 0x00FF0000) >> 16);
    pData[4] = (uint8_t)((sysFaultFlags & 0x0000FF00) >> 8);
    pData[5] = (uint8_t)(sysFaultFlags & 0x000000FF);
    /* Configuration Data Checksum */
    pData[6] = (uint8_t)(ntohs(config.sys.checkSum) >> 8);
    pData[7] = (uint8_t)(ntohs(config.sys.checkSum) & 0xFF);
    /* Date/Time */
    pData[8] = (uint8_t)(dtYear - 2000);
    pData[9] = dtMon + 1;
    pData[10] = dtMday;
    pData[11] = dtHour;
    pData[12] = dtMin;
    /* Total Watering Minutes Today */
    pData[13] = (uint8_t)(irrMinsToday >> 8);
    pData[14] = (uint8_t)(irrMinsToday & 0xFF);
    /* Total Watering Minutes Yesterday */
    pData[15] = (uint8_t)(irrMinsYesterday >> 8);
    pData[16] = (uint8_t)(irrMinsYesterday & 0xFF);

    /* Return length of extended status data. */
    return 17;
}


/******************************************************************************
 *
 * sysRadioMoistValueGet
 *
 * PURPOSE
 *      This routine returns a zone's current moisture sensor reading (or a
 *      failure code) for use in the "Get Moisture Values" radio command
 *      acknowledgement response.
 *
 * PARAMETERS
 *      zi      IN      zone index (0-47)
 *
 * RETURN VALUE
 *      This routine returns a moisture percentage, ranging from 0-100.
 *      A value of 128 (0x80) indicates that the zone has no moisture sensor
 *      configured.  A value of 129 (0x81) indicates that a sensor failure
 *      has been detected for the zone.
 *
 *****************************************************************************/
uint8_t sysRadioMoistValueGet(uint8_t zi)
{
    int8_t moisture;
    uint8_t value;
    uint8_t zone = zi + 1;

    if (moistSensorConfigured(zone))
    {
        moisture = moistValueGet(zone);
        if (moisture >= 0)
        {
            value = moisture;
        }
        else
        {
            /* sensor failure */
            value = RADIO_SENSOR_FAILURE;
        }
    }
    else
    {
        /* sensor not configured */
        value = RADIO_SENSOR_NOT_CFG;
    }
    return value;
}


/******************************************************************************
 *
 * sysInhibitOn
 *
 * PURPOSE
 *      This routine is called to execute the Inhibit-On command.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysInhibitOn(void)
{
    sysIsInhibited = TRUE;
    sysInhibitOnTime = dtTickCount;
    /* Trace system inhibit on event. */
    sysEvent(SYS_EVENT_INHIB_ON, 0);
    dtDebug("Command to INHIBIT ON\n");
}


/******************************************************************************
 *
 * sysInhibitOff
 *
 * PURPOSE
 *      This routine is called to execute the Inhibit-Off command.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysInhibitOff(void)
{
    //if the current zone has a wireless sensor
    //wait for it to checkin
    if(config.zone[irrSnsConSolUnitIndex].sensorType == SNS_WIRELESS_MOIST )
    {
        sysInhibitStop = TRUE;
    }
    else
    {
        sysInhibitStop = FALSE;
        sysIsInhibited = FALSE;
        sysInhibitOffTime = dtTickCount;    }
    
    /* Trace system inhibit off event. */
    sysEvent(SYS_EVENT_INHIB_OFF, 0);
    dtDebug("Command to INHIBIT OFF\n");
}


/******************************************************************************
 *
 * sysPause
 *
 * PURPOSE
 *      This routine is called to pause irrigation.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysPause(void)
{
    if (!sysIsPaused)
    {
        sysIsPaused = TRUE;
        /* Trace system pause event. */
        sysEvent(SYS_EVENT_PAUSE, 0);
        dtDebug("Command to PAUSE\n");
    }
}


/******************************************************************************
 *
 * sysResume
 *
 * PURPOSE
 *      This routine is called to resume irrigation after a pause command.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysResume(void)
{
    if (sysIsPaused)
    {
        sysIsPaused = FALSE;
        /* Trace system resume event. */
        sysEvent(SYS_EVENT_RESUME, 0);
        dtDebug("Command to RESUME\n");
    }
}


/******************************************************************************
 *
 * sysErrorSet
 *
 * PURPOSE
 *      This routine is called to set System Error flags.
 *
 * PARAMETERS
 *      error   IN      system error bit to set
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysErrorSet(uint8_t error)
{
    if (error < SYS_ERROR_LIMIT)
    {
        /* Set the error bit. */
        sysErrorFlags |= ((uint16_t)1) << error;
    }

    if (sysErrorFlags != 0)
    {
        /* Turn on error LED. */
        drvLedError(TRUE);
    }
}


/******************************************************************************
 *
 * sysErrorClear
 *
 * PURPOSE
 *      This routine is called to clear System Error flags.
 *
 * PARAMETERS
 *      error   IN      system error bit to clear
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysErrorClear(uint8_t error)
{
    if (error < SYS_ERROR_LIMIT)
    {
        /* Clear the error bit. */
        sysErrorFlags &= ~(((uint16_t)1) << error);
    }

    if (sysErrorFlags == 0)
    {
        /* Turn off error LED. */
        drvLedError(FALSE);
    }
}


/******************************************************************************
 *
 * sysErrorOn
 *
 * PURPOSE
 *      This routine is called to test System Error flags.
 *
 * PARAMETERS
 *      error   IN      system error bit to test
 *
 * RETURN VALUE
 *      This routine returns TRUE if the error bit is set; otherwise FALSE.
 *
 *****************************************************************************/
bool_t sysErrorOn(uint8_t error)
{
    bool_t errorOn = FALSE;
    if (error < SYS_ERROR_LIMIT)
    {
        errorOn = (sysErrorFlags & (((uint16_t)1) << error)) != 0;
    }
    return errorOn;
}


/******************************************************************************
 *
 * sysFaultSet
 *
 * PURPOSE
 *      This routine is called to set Hardware Fault flags.
 *
 * PARAMETERS
 *      fault   IN      hardware fault bit to set
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysFaultSet(uint8_t fault)
{
    if (fault < SYS_FAULT_LIMIT)
    {
        /* Set the fault bit. */
        sysFaultFlags |= ((uint32_t)1) << fault;
    }

    if (sysFaultFlags != 0)
    {
        /* Turn on fault LED. */
        drvLedFault(TRUE);
    }
}


/******************************************************************************
 *
 * sysFaultClear
 *
 * PURPOSE
 *      This routine is called to clear Hardware Fault flags.
 *
 * PARAMETERS
 *      fault   IN      hardware fault bit to clear
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void sysFaultClear(uint8_t fault)
{
    if (fault < SYS_FAULT_LIMIT)
    {
        /* Clear the fault bit. */
        sysFaultFlags &= ~(((uint32_t)1) << fault);
    }

    if (sysFaultFlags == 0)
    {
        /* Turn off fault LED. */
        drvLedFault(FALSE);
    }
}


/******************************************************************************
 *
 * sysFaultOn
 *
 * PURPOSE
 *      This routine is called to test Hardware Fault flags.
 *
 * PARAMETERS
 *      fault   IN      hardware fault bit to test
 *
 * RETURN VALUE
 *      This routine returns TRUE if the fault bit is set; otherwise FALSE.
 *
 *****************************************************************************/
bool_t sysFaultOn(uint8_t fault)
{
    bool_t faultOn = FALSE;
    if (fault < SYS_FAULT_LIMIT)
    {
        faultOn = (sysFaultFlags & (((uint32_t)1) << fault)) != 0;
    }
    return faultOn;
}


/******************************************************************************
 *
 * sysFormatFirmwareVer
 *
 * PURPOSE
 *      This routine is called to format the system firmware version.
 *
 *      Format examples:
 *          "0.1.0"
 *          "1.0.0-alpha1"
 *          "1.0.0-beta1"
 *          "1.0.0-rc1"
 *
 * PARAMETERS
 *      buf     IN  buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted firmware
 *      version string.
 *
 *****************************************************************************/
char *sysFormatFirmwareVer(char *buf)
{
    uint8_t seqType = sysFirmwareVer.seq & SYS_SEQ_MASK;
    uint8_t seqNum = sysFirmwareVer.seq & ~SYS_SEQ_MASK;
    char seqName[8];

    switch (seqType)
    {
        //removed at customer request in email 12/3/2010
        /*case SYS_SEQ_ALPHA:
            sprintf(seqName, "-alpha%d", seqNum);
            break;
        case SYS_SEQ_BETA:
            sprintf(seqName, "-beta%d", seqNum);
            break;
        case SYS_SEQ_RC:
            sprintf(seqName, "-rc%d", seqNum);
            break;
            */
        default:
            seqName[0] = '\0';
            break;
    }

    sprintf(buf,
        "%d.%d.%d%s",
        sysFirmwareVer.major,
        sysFirmwareVer.minor,
        sysFirmwareVer.patch,
        seqName);

    return (buf);
}


/******************************************************************************
 *
 * sysExecutionExtend
 *
 * PURPOSE
 *      This routine is called to notify the system manager that a subsystem
 *      is not expecting to yield control for an extended time.  This routine
 *      insures that the watchdog timer will not force a system reset and also
 *      allows checking for power failure.  If this routine detects a main
 *      power failure, it will initiate transition to low-power (battery)
 *      operation and not return until main power is restored.
 *
 * PARAMETERS
 *     None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The WOIS system uses a cooperative multi-tasking execution model.
 *      A well-behaved subsystem should not run for more than 50-100 ms per
 *      poll cycle in order to maintain reasonable user interface response
 *      times.  This routine is provided for special cases when an operation
 *      must run longer than 100 ms to complete a critical task that cannot
 *      be time-sliced into multiple poll cycles.  In such cases when this
 *      routine must be used, it should be called at frequent intervals, at
 *      least every 50 ms.
 *
 *****************************************************************************/
void sysExecutionExtend(void)
{
    if (!sysResetRequest)
    {
        /* Reset the watchdog timer. */
        drvSysWatchDogClear();
    }
    /* Check for, and handle, any power failures. */
    sysPollPowerCheck();
}


/******************************************************************************
 *
 * sysEvent
 *
 * PURPOSE
 *      This routine is called to add an entry to the system event log.
 *
 * PARAMETERS
 *      eventType   IN      event type (subsystem event code)
 *      eventData   IN      event data (additional event information)
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine uses a high-resolution timestamp for each log entry.
 *      The timestamp value is stored in millisecond units; however precision
 *      is limited by the 2ms hardware timer update interval.
 *
 *****************************************************************************/
void sysEvent(uint16_t eventType, uint16_t eventData)
{
    sysEventLog.ev[sysEventLog.mn.next].time = htonl(drvMSGet());
    sysEventLog.ev[sysEventLog.mn.next].type = htons(eventType);
    sysEventLog.ev[sysEventLog.mn.next].data = htons(eventData);

    if (++sysEventLog.mn.next >= SYS_EVENT_LIMIT)
    {
        sysEventLog.mn.next = 0;
        sysEventLog.mn.wrapped = TRUE;
    }
}
