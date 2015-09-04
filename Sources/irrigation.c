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
 * Module       : irrigation.c
 * Description  : This file implements the irrigation logic.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "irrigation.h"
#include "datetime.h"
#include "ui.h"
#include "moisture.h"
#include "drvSolenoid.h"
#include "drvSys.h"
#include "radio.h"

/* Irrigation Events */
#define IRR_EVENT_IRR_START     SYS_EVENT_IRR + 1   /* Irrigation Start */
#define IRR_EVENT_IRR_SENSED    SYS_EVENT_IRR + 2   /* Irr Start Sensor Mode */
#define IRR_EVENT_IRR_FINISH    SYS_EVENT_IRR + 3   /* Irrigation Finish */
#define IRR_EVENT_IRR_SKIP      SYS_EVENT_IRR + 4   /* Irrigation Skipped */
#define IRR_EVENT_IRR_STOP      SYS_EVENT_IRR + 5   /* Irrigation Stopped */
#define IRR_EVENT_ET_DATA       SYS_EVENT_IRR + 6   /* Weather Update +ET */
#define IRR_EVENT_FORCE_BSY     SYS_EVENT_IRR + 7   /* Force On Busy/Ignore */
#define IRR_EVENT_RAIN          SYS_EVENT_IRR + 8   /* Weather Update +rain */



/******************************************************************************
 *
 *  IRRIGATION ALGORITHM CONSTANTS
 *
 *****************************************************************************/

/* Plant Species Factors (units of 1/10) */
static const uint8_t irrPlantTypeFactor[4][3] =
{
    /* columns: Low DT, Avg DT, High DT */
    9, 5, 2,        /* Trees */
    7, 5, 2,        /* Shrubs */
    9, 5, 2,        /* Ground Cover */
    9, 5, 2,        /* Mixture */
};


/* Plant Microclimate Factors (units of 1/10) */
static const uint8_t irrPlantClimateFactor[4][3] =
{
    /* columns: Full Sun, 50% Sun, 75% Sun */
    14, 10, 5,      /* Trees */
    13,  8, 5,      /* Shrubs */
    11, 10, 5,      /* Ground Cover */
    11, 10, 5,      /* Mixture */
};


/* Plant Density Factors (units of 1/10) */
static const uint8_t irrPlantDensityFactor[4][3] =
{
    /* columns: Sparse, Avg Density, Highly Dense */
    5, 10, 13,      /* Trees */
    5, 10, 11,      /* Shrubs */
    5, 10, 11,      /* Ground Cover */
    5, 11, 13,      /* Mixture */
};


/* Crop Coefficient Factors for Fescue and Bermuda turf (units of 1/100) */
static const uint8_t irrCropFactor[2][3][12] =
{
    /* columns: Jan, Feb, ..., Dec */
    61, 69, 77, 84, 90, 93, 93, 89, 83, 75, 67, 59, /* Fescue - Full Sun */
    51, 58, 65, 70, 75, 78, 78, 75, 70, 63, 56, 50, /* Fescue - 50% Shade */
    41, 46, 52, 56, 60, 62, 62, 60, 56, 50, 45, 40, /* Fescue - 75% Shade */

    52, 64, 70, 73, 73, 71, 69, 67, 64, 60, 57, 53, /* Bermuda - Full Sun */
    44, 54, 59, 61, 61, 60, 58, 56, 54, 50, 48, 45, /* Bermuda - 50% Shade */
    35, 43, 47, 49, 49, 48, 46, 45, 43, 40, 38, 36, /* Bermuda - 75% Shade */
};


/* Allowable Surface Accumulation (units of 1/100 inch per hour) */
static const uint8_t irrAsaFactor[7][4] =
{
    /* slope: 0-3%, 4-6%, 7-12%, >12% */
    20, 15, 10, 10,     /* Clay */
    23, 19, 16, 13,     /* Silty Clay */
    26, 22, 18, 15,     /* Clay Loam */
    30, 25, 21, 17,     /* Loam */
    33, 29, 24, 20,     /* Sandy Loam */
    36, 30, 26, 22,     /* Loamy Sand */
    40, 35, 30, 25,     /* Sand */
};


/* Infiltration Rate (units of 1/100 inch per hour) */
static const uint8_t irrIrFactor[7] =
{
    10,                 /* Clay */
    15,                 /* Silty Clay */
    20,                 /* Clay Loam */
    35,                 /* Loam */
    40,                 /* Sandy Loam */
    50,                 /* Loamy Sand */
    60,                 /* Sand */
};



/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/

/*
** System Irrigation State Data
*/
static uint32_t irrLastPollTime;   /* Last Poll Time (tick count) */
static uint32_t irrElapsedSecs;    /* Seconds elapsed since last poll */
static uint32_t irrLastEtDataTime; /* Tick Count on last ET/Rainfall data received */
uint32_t irrPgmStartTime;   /* Tick Count on last program start */
static uint32_t irrSenStartTime;   /* Sensor pgm start time, used for sensor wait */
uint16_t irrMinsToday;      /* Total Watering Minutes Today */
uint16_t irrMinsYesterday;  /* Total Watering Minutes Yesterday */
uint8_t irrState;           /* Irrigation State */
uint8_t irrAutoPgmPending;  /* Program Auto-Start "Queue" (holds 1 entry) */
uint8_t irrNumZones;        /* Snapshot of Configured Number of Zones */
uint8_t irrProgram;         /* Current Irrigation Program */
uint8_t irrOpMode;          /* Current Program Operating Mode */
uint8_t irrPulseMode;       /* Current Program Pulse Mode */
uint8_t irrCurZone;         /* Current Irrigation Zone (1..48) */
uint8_t irrNextZone;        /* Next Zone (1..48) */
static bool_t irrSkip;             /* Skip current zone when TRUE */
bool_t irrStop;             /* Stop current irrigation program when TRUE */
uint8_t irrExpRunningProg;   /* current program running on expansion units. */
uint8_t irrStateOld;         /* the old irrigation state used to determine if master unit needs to be updated */
uint8_t irrCurUnitRunning;   /* keeps track if another unit is irrigating. 0 means no other unit is running */
static uint32_t irrWateringStartTime;  //   current zone watering start time
bool_t flowFlag = FALSE;
bool_t timeFlag = FALSE;
uint32_t eTime = 0xFFFFFFFF;
uint8_t GPM = 0;
uint8_t slaveGPM = 0;
uint8_t flowIndex = 0;
uint16_t flowDelay = 0;
uint16_t slaveFlowDelay = 0;
uint8_t  findFlow = 0;
uint8_t slaveFindFlow = 0;
bool_t TestSkipFlag = FALSE;

/*
** Zone Irrigation State Data
*/
int16_t irrMoistureBalance[SYS_N_ZONES];    /* Moisture Balance per-zone */
irrZoneState_t irrZone[SYS_N_ZONES];        /* Irrigation Scorecard per-zone */
uint16_t irrDailyRuntimePerZone[SYS_N_ZONES];    /*keep runtime per zone per day. reset at midnight */


/******************************************************************************
 *
 *  IRRIGATION FUNCTION PROTOTYPES
 *
 *****************************************************************************/

static void     irrPollActionIdle(void);
static void     irrPollActionWatering(void);
static void     irrPollActionSoaking(void);
static void     irrPollActionSensorWait(void);
static void     irrZoneTransition(bool_t forceTransition);
static bool_t   irrIsWirelessZone(uint8_t zone);
static void     irrCycleStart(void);
static void     irrProgramFinished(void);
static uint8_t  irrSelectNextZone(void);
static void     irrWeatherUpdateZone(uint8_t zi, uint16_t etData, uint16_t rainfall);
static bool_t   irrStart(uint8_t cause, uint8_t program, uint8_t opMode, uint8_t pulseMode);
static void     irrZoneClear(void);
static void     irrStartTestInit(void);
static void     irrStartZoneInit(void);
static void     irrStartSensorInit(void);
static void     irrStartWeatherInit(void);
static void     irrStartPulseInit(void);
static uint8_t  irrFindNextRunnableZone(void);
static uint8_t  irrFindNextRunnableSensorZone(void);
static void     irrSolenoidControl(uint8_t zone, bool_t state);
static uint8_t  irrCropCoefficient(uint8_t zi, uint8_t type);
static uint16_t irrPulseTimeCalc(uint8_t zi);
static uint16_t irrSoakTimeCalc(uint8_t zi);
static uint32_t irrZoneFinishTime(uint8_t zi);
static void     irrSensorGroupRuntimeAdjust(uint8_t leadZone);
static uint8_t  irrSensorGroupCount(uint8_t leadZone);
static void     irrSoakTimeIncrement(uint32_t elapsedSecs);
static void     irrSensorThresholdCheck(void);
static void     irrStartedDebug(void);
static bool_t   irrHaveRunnableProgram(void);
static bool_t   flowMinMax(uint8_t zi); 


/******************************************************************************
 *
 * irrInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization function to
 *      initialize the irrigation subsystem and irrigation state data store.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine must be called at system initialization.
 *
 *****************************************************************************/
void irrInit(void)
{
    irrStateOld = IRR_STATE_IDLE; 
    irrState = IRR_STATE_IDLE;
    irrProgram = IRR_PGM_NONE;
    irrExpRunningProg = IRR_PGM_NONE;
    irrSkip = FALSE;
    irrStop = FALSE;
    irrCurZone = 0;
    irrAutoPgmPending = IRR_PGM_NONE;
    /* Set last ET data time to force initial "no weather data" error. */
    irrLastEtDataTime = (uint32_t)0 - DT_SECS_24_HOURS;
}


/******************************************************************************
 *
 * irrPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to
 *      start, stop and manage progress of irrigation programs and cycles.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrPoll(void)
{
       
    irrElapsedSecs = dtElapsedSeconds(irrLastPollTime);
    irrLastPollTime = dtTickCount;

    /* Check for no weather data received in last 24 hours. */
    if ((config.sys.opMode == CONFIG_OPMODE_WEATHER) &&
        (dtElapsedSeconds(irrLastEtDataTime) > DT_SECS_24_HOURS))
    {
        sysErrorSet(SYS_ERROR_WEATHERDATA);
    }
    else
    {
        sysErrorClear(SYS_ERROR_WEATHERDATA);
    }

    /* Check for no runnable programs. */
    if (sysIsAuto && !irrHaveRunnableProgram())
    {
        sysErrorSet(SYS_ERROR_NOPROGRAM);
    }
    else
    {
        sysErrorClear(SYS_ERROR_NOPROGRAM);
    }

    /* if irrigation state has changed notify the master unit of it */
    if(irrStateOld != irrState)
    {
        irrStateOld = irrState;
        /* if expansion report status back to master */
        if((config.sys.unitType != UNIT_TYPE_MASTER))
        {
            expansionBusSendCmd(RADIO_CMD_EXPANSION_STATUS, config.sys.masterMac);
        }
    }
      
    switch (irrState)
    {
        case IRR_STATE_IDLE:
            irrPollActionIdle();
            break;
        case IRR_STATE_SENSING:
            irrPollActionSensorWait();
            break;
        case IRR_STATE_WATERING:
            irrPollActionWatering();
            break;
        case IRR_STATE_SOAKING:
            irrPollActionSoaking();
            break;
        default:
            /* Invalid state.  Should never happen. */
            break;
    }
}


/*
**  POLL ACTION ROUTINES FOR EACH IRRIGATION STATE
*/

/******************************************************************************
 *
 * irrPollActionIdle
 *
 * PURPOSE
 *      This routine is called from the irrigation poll routine to perform
 *      action routine for the IRR_STATE_IDLE irrigation state.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrPollActionIdle(void)
{
    uint8_t opMode = config.sys.opMode;

    /* If Auto mode and there is a program in the queue. */
    if (sysIsAuto && (irrAutoPgmPending != IRR_PGM_NONE))
    {
        /* Change weather mode to runtime mode if no weather data in 24 hrs. */
        if ((opMode == CONFIG_OPMODE_WEATHER) &&
            sysErrorOn(SYS_ERROR_WEATHERDATA))
        {
            opMode = CONFIG_OPMODE_RUNTIME;
        }

        /* Start automatic irrigation. */
        irrStart(SYS_STATE_AUTORUN,
                 irrAutoPgmPending,
                 opMode,
                 config.sys.pulseMode);

        /* Remove program from queue. */
        irrAutoPgmPending = IRR_PGM_NONE;
    }
}


/******************************************************************************
 *
 * irrPollActionSensorWait
 *
 * PURPOSE
 *      This routine is called from the irrigation poll routine to perform
 *      action routine for the IRR_STATE_SENSING irrigation state.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrPollActionSensorWait(void)
{
    /* Test for sensor wait complete */
    if (dtElapsedSeconds(irrSenStartTime) > IRR_SENSING_SECS)
    {
        /* Start waiting sensor program. */
        irrStart(sysState, irrProgram, irrOpMode, irrPulseMode);
    }
}


/******************************************************************************
 *
 * irrPollActionWatering
 *
 * PURPOSE
 *      This routine is called from the irrigation poll routine to perform
 *      action routine for the IRR_STATE_WATERING irrigation state.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrPollActionWatering(void)
{
    uint8_t zi = irrCurZone - 1;        /* use zero-based zone index */
    bool_t isZoneCycleComplete = FALSE;
    
    /* Maintain watering timers. */
    irrZone[zi].elapsedTime += (uint16_t)irrElapsedSecs;
    irrDailyRuntimePerZone[zi] += (uint16_t)irrElapsedSecs;
    
    if (irrPulseMode == CONFIG_PULSEMODE_ON)
    {
        irrZone[zi].elapsedPulseTime += (uint16_t)irrElapsedSecs;
        irrSoakTimeIncrement(irrElapsedSecs);
    }

    /* Check if current cycle is complete. */

    if (irrStop)
    {
        /* Set zone stopped event flag. */
        irrZone[zi].flags |= IRR_ZF_STOPPED;
        /* Set current zone's time limit to force an immediate end. */
        irrZone[zi].actualTimeLimit = irrZone[zi].elapsedTime;
    }

    if (irrSkip)
    {
        /* Clear the skip flag. */
        irrSkip = FALSE;
        /* Set zone skipped event flag. */
        irrZone[zi].flags |= IRR_ZF_SKIPPED;
        /* Set current zone's time limit to force an immediate end. */
        irrZone[zi].actualTimeLimit = irrZone[zi].elapsedTime;
    }
    
    /* if zone is configured for a generic sensor (nonmoisture) then skip over zone */
    if(
        (config.zone[zi].sensorType == SNS_PRESSURE) ||
        (config.zone[zi].sensorType == SNS_RAIN_GAUGE) 
        //(config.zone[zi].sensorType == SNS_FLOW)
       ) 
    {
        /* Set current zone's time limit to force an immediate end. */
        irrZone[zi].actualTimeLimit = irrZone[zi].elapsedTime;
        
        /* Set moisture sensor sample frequency to active rate (1/min). */
        moistFreqSet(irrCurZone, MOIST_FREQ_ACTIVE);
            
    }
    
    /* Check sensors for threshold reached. */
    if (irrOpMode == CONFIG_OPMODE_SENSOR)
    {
        /* Check sensors for all zones. */
        irrSensorThresholdCheck();

        /* Check time remaining for current zone. */
        if (irrRemainingZoneSecs(irrCurZone) == 0)
        {
            /* Set moisture sensor sample frequency to inactive rate. */
            if(irrIsWirelessZone(irrCurZone))
            {
                moistFreqSet(irrCurZone, MOIST_FREQ_INACTIVE);
            }
            /* End irrigation because zone runtime limit is reached. */
            isZoneCycleComplete = TRUE;
        }
    }

    if (irrPulseMode == CONFIG_PULSEMODE_ON)
    {
        /* Determine if pulse cycle is complete. */
        if (irrZone[zi].elapsedPulseTime >= irrZone[zi].pulseTimeLimit)
        {
            isZoneCycleComplete = TRUE;
        }
    }

    /* 
     * End cycle if zone runtime limit is reached.
     * 
     * If this is a wireless sensor zone, then we go into soaking until
     * the SC reports in (and we can turn off valves in sync) or we
     * time out waiting for the SC (we will move to next zone).
     */
    if (irrRemainingZoneSecs(irrCurZone) == 0)
    {
        /* Correct for any overshoot (clock speed-up problem) */
        irrZone[zi].elapsedTime = irrZone[zi].actualTimeLimit;

        if (irrIsWirelessZone(irrCurZone))
        {
            // if this is a wireless.  We want to wait for the SC to check in.  By
            // marking that it has not checked in and calling solenoid control, we
            // will queue up the action turning off solenoids in sync.  irrSolenoidControl
            // will put the system into soaking.  Since the elapsed time is expired,
            // when we time out or the sc checks in we will move on to the next zone.
            scCheckedIn = FALSE;
            irrSolenoidControl(irrCurZone, IRR_SOL_OFF);
        }
        else
        {
            isZoneCycleComplete = TRUE;
        }
    }
    
    /* Check if inhibited or paused. */
    if ((sysIsInhibited || sysIsPaused))
    {
        /* Stop watering for now. */
        /* Stop the current zone. */
        if(irrIsWirelessZone(irrCurZone) == FALSE)
        {
            irrSolenoidControl(irrCurZone, IRR_SOL_OFF);

            /* Stop the Master Valve/Pump. */
            irrSolenoidControl(IRR_SOL_MASTER, IRR_SOL_OFF);
        }

        irrState = IRR_STATE_SOAKING;
        return;
    }

    
    /* check flow shut-off  */
    
    if ( (flowMinMax(irrCurZone) == FALSE) && (flowFlag == FALSE) )
    {
      flowFlag = TRUE;
      // shut off master valve 
      irrSolenoidControl(IRR_SOL_MASTER, IRR_SOL_OFF);   
      // shut off current zone valve 
      irrSolenoidControl(irrCurZone, IRR_SOL_OFF);       
      
      // Call routine to skip watering to next zone 
      irrCmdSkip();
      
      // send command to other units 
      if(config.sys.unitType == UNIT_TYPE_MASTER)
      {
          expansionBusSendCmd(RADIO_CMD_IRR_SKIP, RADIO_EXP_SEND_ALL);
      }
      
    } 
      
    
    
    if (isZoneCycleComplete)
    {
        /* Zero elapsed soak and pulse times for current zone. */
        irrZone[zi].elapsedSoakTime = 0;
        irrZone[zi].elapsedPulseTime = 0;
        irrZoneTransition(FALSE);
    }
}


/******************************************************************************
 *
 * irrPollActionSoaking
 *
 * PURPOSE
 *      This routine is called from the irrigation poll routine to perform
 *      action routine for the IRR_STATE_SOAKING irrigation state.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrPollActionSoaking(void)
{
    /* Maintain soak timers. */
    irrSoakTimeIncrement(irrElapsedSecs);
    static bool FirstTime = FALSE;

    /* Check if program stop requested. */
    if (irrStop)
    {
        /* Clear stop flag */
        //irrStop = FALSE;
        /* Program is finished. */
        irrProgramFinished();
        return;
    }

    /* Check sensors for threshold reached. */
    if (irrOpMode == CONFIG_OPMODE_SENSOR)
    {
        irrSensorThresholdCheck();
    }

    /*
    **  Check if watering can resume.
    */

    /* Check for inhibit or pause. */
    if (sysIsInhibited)
    {
      return;
    }
    
    if(sysIsPaused)
    {
        // if we are waiting for an SC to check in (paused)
        // and have timed out
        // move to the next zone and start over
        if(sysFaultOn(SYS_FAULT_SNSCON) || ( TestSkipFlag == TRUE ))
        {
            // the zoneIndex is the actual index of the current zone (e.g. 1)
            // in data structures containing information about the zone.
            // 
            // That is, zone 1 is actually index 0 in most data structures dealing
            // with the zones
            uint8_t zoneIndex = irrCurZone - 1;
          
            // clear connection failure
            sysFaultClear(SYS_FAULT_SNSCON);
            
            //clear the current solenoid state so it doesn't 
            //turn it on whe this SC finally checks in
            irrSnsConSolenoidChan = SC_ALL_CHAN_OFF;
            
            // Reset timer used for checking for timeout on next node
            radioSnsConCheckinTime = dtTickCount;
            
            // we are moving on to the next zone, change system state to unpaused
            sysResume();
            
            // mark the zone which just timed out as skipped and set the time
            // limit for the zone to be however much has elaped (make it expire).
            // Without this, the same zone would be selected again when
            // irrSelectZone() is executed
            irrZone[zoneIndex].flags |= IRR_ZF_SKIPPED;
            irrZone[zoneIndex].actualTimeLimit = irrZone[zoneIndex].elapsedTime;
            
            // Trace irrigation skip event
            sysEvent(IRR_EVENT_IRR_SKIP, irrCurZone);
                      
            irrZoneTransition(TRUE);
            return;
        }
        else
        {
          /* Watering is inhibited or paused. */
          return;
        }
    }
    else
    {
        /*
         *  Note: The current zone is zero when all zones are soaking.
         *        Non-zero value indicates that watering had been inhibited or
         *        paused before a zone's irrigation cycle was completed.
         */
         irrZoneTransition(FALSE);
    }
}



/*
**  IRRIGATION SUBROUTINES
*/

/******************************************************************************
 *
 * irrZoneTransition
 *
 * PURPOSE
 *      This routine is called to transition from one zone to the next.  In
 *      particular, it does the following:
 * 
 *        1) Cleans up state from the current (old) zone.  This includes
 *           setting all wired and wireless valves in the zone off.
 *
 *        2) Set irrNextZone to the next appropriate zone (could be 0) by
 *           calling irrSelectNextZone()
 *        
 *        3) If there is a next zone:
 *           Initialize state for next zone/cycle by calling irrCycleStart()
 *  
 *        4) If there is no next zone:
 *           Change soaking or finished (IDLE) based on remaining program time.
 *
 * PARAMETERS
 *      forceTransition  Determines whether or not we should "force" a
 *                       transition to the next zone bypassing things like
 *                       waiting for wireless sensors.  This is useful in
 *                       cases where we have already given up on the zone..
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrZoneTransition(bool_t forceTransition)
{
    uint32_t remainingProgramSeconds = irrRemainingProgramSecs();
    uint8_t zi = irrCurZone - 1;
    
   
    if (forceTransition)
    {
        /* FORCE: just turn off the current zone wired solenoid */
        drvSolenoidSet(irrCurZone, FALSE);
    }
    else
    {
        /* Stop the current zone. */
        irrSolenoidControl(irrCurZone, IRR_SOL_OFF);
    }
    
    /* Find the next zone to water. */
    irrNextZone = irrSelectNextZone();

    if (irrStop)
    {
        /* Indicate no more zones left to water. */
        irrNextZone = 0;
        remainingProgramSeconds = 0;
    }

    if (irrNextZone != 0)
    {
        irrCycleStart();
    }
    else
    {
        /* Stop the Master Valve/Pump. */
        irrSolenoidControl(IRR_SOL_MASTER, IRR_SOL_OFF);
        
        if (remainingProgramSeconds == 0)
        {
            /* Program is finished. */
            irrProgramFinished();
        }
        else
        {
            /* Pause for soak. */
            irrCurZone = 0;
            irrState = IRR_STATE_SOAKING;
            
            /* if expansion report status back to master */
            if(config.sys.unitType != UNIT_TYPE_MASTER)
            {
                expansionBusSendCmd(RADIO_CMD_EXPANSION_STATUS, config.sys.masterMac);
            }
        }
    }
}


/******************************************************************************
 *
 * irrCycleStart
 *
 * PURPOSE
 *      This routine is called to start a new irrigation zone watering cycle.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrCycleStart(void)
{
    /* For wireless zones, mark the SC */
    TestSkipFlag = FALSE;
    flowFlag = FALSE;
    //findFlow = 0;
    timeFlag = FALSE;
    if (irrIsWirelessZone(irrNextZone) &&
        irrCurZone != irrNextZone)
    {
        scCheckedIn = FALSE;
    }
    
    /* Start the Master Valve/Pump. */

    /* Start the Zone. */
    /* Send zone start command to solenoid driver. */
    irrSolenoidControl(irrNextZone, IRR_SOL_ON);
    
    //irrWateringStartTime = dtTickCount;

    /* Set the current zone to the next zone chosen for watering. */
    irrCurZone = irrNextZone;
    irrState = IRR_STATE_WATERING;

    if ((irrOpMode == CONFIG_OPMODE_SENSOR) &&
        (irrIsCurrentGroupLeader(irrCurZone)))
    {
        moistFreqSet(irrCurZone, MOIST_FREQ_ACTIVE);
    }

    /* if expansion report status back to master */
    if(config.sys.unitType != UNIT_TYPE_MASTER)
    {
          expansionBusSendCmd(RADIO_CMD_EXPANSION_STATUS, config.sys.masterMac);
    }
    /* Request LCD screen refresh. */
    uiLcdRefresh();

}


/******************************************************************************
 *
 * irrProgramFinished
 *
 * PURPOSE
 *      This routine is called to perform 'housekeeping' operations necessary
 *      upon completion of an irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrProgramFinished(void)
{
    uint8_t zi;                     /* zone index */
    uint32_t mbAdjust;              /* amount to adjust zone's MB budget */
    uint32_t timeProduct;           /* product of elapsed time and app rate */

    /* Trace irrigation finished event. */
    sysEvent(IRR_EVENT_IRR_FINISH, 0);

    /* Deduct watering time from any existing moisture balance budget. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        /*
        **  Adjust zone's moisture balance deficit/surplus amount.
        **  Note: appRate is zero for test programs.
        */
        if ((irrZone[zi].appRate != 0) &&
            (irrZone[zi].elapsedTime > 0))
        {
            /* Multiply elapsed secs by actual application rate. */
            timeProduct =
                irrZone[zi].elapsedTime * irrZone[zi].appRate * irrZone[zi].appEff;
            /* Convert from secs to hours; correct for percent multiplier. */
            mbAdjust = timeProduct / (3600 * 100);
            irrMoistureBalance[zi] += (uint16_t)mbAdjust;
            /* Don't exceed the RZWWS limit. */
            if (irrMoistureBalance[zi] > ntohs(config.zone[zi].rzwws))
            {
                irrMoistureBalance[zi] = ntohs(config.zone[zi].rzwws);
            }
        }
    }
    
    /* if unit is master and running in pulse mode then pass start batan to next unit */

        if((config.sys.unitType == UNIT_TYPE_MASTER)) 
        {
            if((config.sys.numUnits >0)&& (irrStop == FALSE))
            {              
                irrExpRunningProg = irrProgram;
                /* send start command to expansion 1 if in pulse mode */
                if(config.sys.expMac1 != 0x0013A20000000000)
                {
                    expansionBusSendCmd(RADIO_CMD_IRR_START,config.sys.expMac1);
                    irrCurUnitRunning = UNIT_TYPE_EXPANSION_1;
                }
                else if((config.sys.expMac2 != 0x0013A20000000000) && (config.sys.numUnits >1))
                {
                    expansionBusSendCmd(RADIO_CMD_IRR_START,config.sys.expMac2);
                    irrCurUnitRunning = UNIT_TYPE_EXPANSION_2;
                }
                else if((config.sys.expMac3 != 0x0013A20000000000) && (config.sys.numUnits >2))
                {
                    expansionBusSendCmd(RADIO_CMD_IRR_START,config.sys.expMac3);
                    irrCurUnitRunning = UNIT_TYPE_EXPANSION_3;
                }                
             }
            /* Clear the stop flag. */
            irrStop = FALSE;
        } 
        else 
        {
            /* inform master that complete */
            expansionBusSendCmd(RADIO_CMD_IRR_COMPLETE,config.sys.masterMac);
            irrStateOld = irrState;  
        }

    
    /* if unit is a master and there are other units that need to run mainting running display
     * to have 1 point for the user to get information about the entire system
     */
    if((config.sys.unitType == UNIT_TYPE_MASTER) && (config.sys.numUnits >0)&&(irrStop == FALSE)) 
    {
        /* leave sysState in the same state as it was */
        
        /* set expansion program running */
        irrExpRunningProg = irrProgram;
    }
    else
    {
      sysState = SYS_STATE_IDLE;
    }
    
    irrProgram = IRR_PGM_NONE;
    irrState = IRR_STATE_IDLE;
    irrCurZone = 0;
    /* Set moisture sample frequency to inactive for all sensored zones. */
    irrMoistConfigUpdate();
    /* Request LCD screen refresh. */
    uiLcdRefresh();
}


/******************************************************************************
 *
 * irrSelectNextZone
 *
 * PURPOSE
 *      This routine is calculates the next zone to run in the currently
 *      active irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the number of the next zone to run or zero to
 *      indicate that irrigation is complete.
 *
 *****************************************************************************/
static uint8_t irrSelectNextZone(void)
{
    uint8_t nextZone = 0;           /* best choice for next zone to run */

    if (irrOpMode == CONFIG_OPMODE_SENSOR)
    {
        /* Try to run sensor zones first. */
        nextZone = irrFindNextRunnableSensorZone();
    }
    
    if (nextZone == 0)
    {
        nextZone = irrFindNextRunnableZone();
    }

    return nextZone;
}


/******************************************************************************
 *
 * irrCmdSkip
 *
 * PURPOSE
 *      This routine is called to skip watering operations for the current
 *      zone in the currently executing irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrCmdSkip(void)
{
    /* Only skip if watering. */
    if (irrState == IRR_STATE_WATERING)
    {
        irrSkip = TRUE;
        /* Trace irrigation skip event. */
        sysEvent(IRR_EVENT_IRR_SKIP, irrCurZone);
        dtDebug("Command to SKIP\n");
    }
}


/******************************************************************************
 *
 * irrCmdStop
 *
 * PURPOSE
 *      This routine is called to stop the currently executing irrigation
 *      program or irrigation test.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrCmdStop(void)
{
    sysIsPaused = FALSE;
    if (irrState != IRR_STATE_IDLE)
    {
        irrStop = TRUE;
        /* Trace irrigation stop event. */
        sysEvent(IRR_EVENT_IRR_STOP, irrCurZone);
        dtDebug("Command to STOP\n");
    }
        
    irrSnsConSolenoidChan = SC_ALL_CHAN_OFF; //KAV make sure to tell the SC we are done and to shut everything off
        
    //Force all solenoids off
      drvSolenoidSet(0, FALSE);
    
    if(config.sys.unitType != UNIT_TYPE_MASTER)
    {
        /* notify system that stop command was received */
        irrStop = TRUE;
        /* cancel any scehduled program */
        irrAutoPgmPending = IRR_PGM_NONE;
    }
}


/******************************************************************************
 *
 * irrCmdTest
 *
 * PURPOSE
 *      This routine is called to execute the irrigation Test command.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrCmdTest(void)
{
    irrStop = FALSE;
    irrStart(SYS_STATE_TEST,
             IRR_PGM_NONE,
             CONFIG_OPMODE_RUNTIME,
             CONFIG_PULSEMODE_OFF);
}


/******************************************************************************
 *
 * irrRemainingZoneSecs
 *
 * PURPOSE
 *      This routine calculates the number of seconds remaining for the
 *      specified zone in the currently active irrigation program.
 *
 * PARAMETERS
 *      zone    IN  the zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds remaining for the specified
 *      zone in the currently active irrigation program.
 *
 *****************************************************************************/
int32_t irrRemainingZoneSecs(uint8_t zone)
{
    int32_t secsRemaining = 0;      /* zone watering seconds remaining */

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        secsRemaining = irrZone[zone - 1].actualTimeLimit -
                        irrZone[zone - 1].elapsedTime;
        secsRemaining = (secsRemaining < 0) ? 0 : secsRemaining;
    }

    return secsRemaining;
}


/******************************************************************************
 *
 * irrRemainingProgramSecs
 *
 * PURPOSE
 *      This routine calculates the number of seconds remaining in
 *      the current irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the number of seconds remaining in the currently
 *      active irrigation program.
 *
 *****************************************************************************/
int32_t irrRemainingProgramSecs(void)
{
    int32_t secsRemaining = 0;      /* program watering seconds remaining */
    int32_t secs;                   /* zone watering seconds, temp store */
    uint8_t zi;                     /* zone index */

    /* Compute run-time total for current  program. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        secs = irrZone[zi].actualTimeLimit - irrZone[zi].elapsedTime;
        /* Ignore any negative values. */
        if (secs > 0)
        {
            secsRemaining += secs;
        }
    }

    return secsRemaining;
}


/******************************************************************************
 *
 * irrRemainingSoakSecs
 *
 * PURPOSE
 *      This routine is called to calculate the time required before any
 *      watering can resume.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the number of seconds remaining in the currently
 *      active irrigation program before the next irrigation pulse cycle can
 *      can begin.
 *
 *****************************************************************************/
uint32_t irrRemainingSoakSecs(void)
{
    uint32_t secsRemaining = 0;     /* system soak seconds remaining */
    uint32_t secs;                  /* zone soak seconds, temp store */
    uint8_t zi;                     /* zone index */

    /* First find the largest remaining soak time. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        if (irrZone[zi].elapsedTime < irrZone[zi].actualTimeLimit)
        {
            secs = irrRemainingZoneSoakSecs(zi + 1);
            if (secs > secsRemaining)
            {
                secsRemaining = secs;
            }
        }
    }

    /* Now find unfinished zone with least remaining soak time. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        if (irrZone[zi].elapsedTime < irrZone[zi].actualTimeLimit)
        {
            secs = irrRemainingZoneSoakSecs(zi + 1);
            if (secs < secsRemaining)
            {
                secsRemaining = secs;
            }
        }
    }

    return secsRemaining;
}


/******************************************************************************
 *
 * irrRemainingZoneSoakSecs
 *
 * PURPOSE
 *      This routine is called to calculate the remaining soak time for a
 *      zone's current soak cycle.
 *
 * PARAMETERS
 *      zone    IN  the zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds remaining in the current
 *      soak cycle for a zone.  This routine returns zero if the zone is not
 *      currently soaking.
 *
 *****************************************************************************/
uint16_t irrRemainingZoneSoakSecs(uint8_t zone)
{
    int32_t secs = 0;               /* seconds of soak time remaining */
    uint8_t zi = zone - 1;          /* zone index */

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        secs = irrZone[zi].soakTimeLimit - irrZone[zi].elapsedSoakTime;
    }

    return (uint16_t)(secs < 0 ? 0 : secs);
}


/******************************************************************************
 *
 * irrRemainingZonePulseSecs
 *
 * PURPOSE
 *      This routine is called to calculate the remaining pulse time for the
 *      specified zone.
 *
 * PARAMETERS
 *      zone    IN  the zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds remaining in the current
 *      pulse cycle for a zone.  This routine returns zero if the zone is not
 *      currently watering.
 *
 *****************************************************************************/
uint16_t irrRemainingZonePulseSecs(uint8_t zone)
{
    uint8_t zi = zone - 1;          /* zone index */
    int32_t remainingSecs = 0;      /* seconds of watering time remaining */

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        remainingSecs =
            irrZone[zi].pulseTimeLimit - irrZone[zi].elapsedPulseTime;
    }

    return (uint16_t)(remainingSecs < 0 ? 0 : remainingSecs);
}


/******************************************************************************
 *
 * irrRemainingZonePulses
 *
 * PURPOSE
 *      This routine is called to calculate the number of pulses remaining in
 *      the active irrigation program for a specified zone.
 *
 * PARAMETERS
 *      zone    IN  the zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the number of pulses remaining in the currently
 *      active irrigation program for the specified zone.  The number of
 *      pulses includes the current pulse cycle if the zone is watering.
 *
 *****************************************************************************/
uint16_t irrRemainingZonePulses(uint8_t zone)
{
    uint8_t zi = zone - 1;          /* zone index */
    uint32_t totalPulses = 0;       /* number of pulses remaining */
    int32_t remainingTime;          /* seconds of watering time remaining */

    /* Insure that critical parameters are in range. */
    if ((zone > 0) &&
        (zone <= SYS_N_ZONES) &&
        (irrZone[zi].pulseTimeLimit != 0))
    {
        remainingTime = irrZone[zi].actualTimeLimit - irrZone[zi].elapsedTime;
        if (zone == irrCurZone)
        {
            /* Count current pulse and subtract from remaining time calc. */
            totalPulses++;
            remainingTime -= irrRemainingZonePulseSecs(zone);
        }
        if (remainingTime > 0)
        {
            totalPulses += (remainingTime / irrZone[zi].pulseTimeLimit) +
                (((remainingTime % irrZone[zi].pulseTimeLimit) > 0) ? 1 : 0);
        }
    }
    return (uint16_t)totalPulses;
}


/******************************************************************************
 *
 * irrProgramRuntime
 *
 * PURPOSE
 *      This routine is called to calculate the time required to finish
 *      irrigation of a zone.
 *
 * PARAMETERS
 *      program     IN  the irrigation program (0=A,..3=D)
 *      opMode      IN  the mode of operation
 *
 * RETURN VALUE
 *      This routine returns the estimated number of seconds to complete
 *      the irrigation program.
 *
 *****************************************************************************/
uint32_t irrProgramRuntime(uint8_t program, uint8_t opMode)
{
    uint32_t runtime = 0;       /* Program Runtime */
    uint32_t runtimeW;          /* Weather-based Zone Runtime */
    uint32_t runtimeC;          /* Configured Zone Runtime */
    uint8_t zi;

    if (program < SYS_N_PROGRAMS)
    {
        for (zi = 0; zi < config.sys.numZones; zi++)
        {
            runtimeC = config.zone[zi].runTime[program] * 60;
            if (opMode == CONFIG_OPMODE_WEATHER)
            {
                runtimeW = irrWeatherRuntimeCalc(zi);
                runtime +=
                    runtimeW < runtimeC ? runtimeW : runtimeC;
            }
            else
            {
                runtime += runtimeC;
            }
        }
    }

    return runtime;
}


/******************************************************************************
 *
 * irrFindNextRunnableZone
 *
 * PURPOSE
 *      This routine is called to find the next runnable zone in the
 *      currently active irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the zone number if a runnable zone was found;
 *      a value of zero indicates no zone was found.
 *
 *****************************************************************************/
static uint8_t irrFindNextRunnableZone(void)
{
    uint8_t nextZone = 0;               /* zone with longest finish time */
    uint32_t maxFinishTime = 0;         /* longest zone finish time */
    uint32_t zoneFinishTime;            /* zone finish time, temp store */
    uint8_t zi;                         /* zone index */

    for (zi = 0; zi < irrNumZones; zi++)
    {
        /* Find a zone that can run. */
        if (irrZone[zi].elapsedTime < irrZone[zi].actualTimeLimit)
        {
            if (irrPulseMode == CONFIG_PULSEMODE_OFF)
            {
                nextZone = zi + 1;
                break;
            }
            /* Find zone with longest finish time. */
            else if (irrZone[zi].elapsedSoakTime >= irrZone[zi].soakTimeLimit)
            {
                zoneFinishTime = irrZoneFinishTime(zi);
                if (zoneFinishTime > maxFinishTime)
                {
                    maxFinishTime = zoneFinishTime;
                    nextZone = zi + 1;
                }
            }
        }
    }

    return nextZone;
}


/******************************************************************************
 *
 * irrFindNextRunnableSensorZone
 *
 * PURPOSE
 *      This routine is called to find the next runnable zone that is a
 *      sensor group leader (with followers) in the currently active
 *      irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the zone number if a runnable group leader zone
 *      (with at least one follower) was found; a value of zero indicates no
 *      zone was found.
 *
 *****************************************************************************/
static uint8_t irrFindNextRunnableSensorZone(void)
{
    uint8_t nextZone = 0;               /* best sensor lead zone to run */
    uint32_t maxFinishTime = 0;         /* longest zone finish time */
    uint32_t zoneFinishTime;            /* zone finish time, temp store */
    uint8_t zi;                         /* zone index */

    for (zi = 0; zi < irrNumZones; zi++)
    {
        /* Find a sensor group leader (with followers) that can run. */
        if ((irrZone[zi].elapsedTime < irrZone[zi].actualTimeLimit) &&
            (irrIsCurrentGroupLeader(zi + 1)) &&
            (irrSensorGroupCount(zi + 1) > 1))
        {
            if (irrPulseMode == CONFIG_PULSEMODE_OFF)
            {
                nextZone = zi + 1;
                break;
            }
            /* Find zone with longest finish time. */
            else if (irrZone[zi].elapsedSoakTime >= irrZone[zi].soakTimeLimit)
            {
                zoneFinishTime = irrZoneFinishTime(zi);
                if (zoneFinishTime > maxFinishTime)
                {
                    maxFinishTime = zoneFinishTime;
                    nextZone = zi + 1;
                }
            }
        }
    }

    return nextZone;
}


/******************************************************************************
 *
 * irrZoneFinishTime
 *
 * PURPOSE
 *      This routine is called to calculate the time required to finish
 *      irrigation of a zone.
 *
 * PARAMETERS
 *      zi      IN  the zone's index (0-47)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds to complete irrigation
 *      of a zone (including soak time), calculated as if it were the only
 *      zone being watered.
 *
 *****************************************************************************/
static uint32_t irrZoneFinishTime(uint8_t zi)
{
    uint32_t finishTime = 0;        /* estimated number of seconds to finish */
    uint32_t totalPulses;           /* number of irrigation pulses remaining */
    uint32_t remainingTime;         /* seconds of watering time remaining */

    if (zi < SYS_N_ZONES)
    {
        remainingTime = irrZone[zi].actualTimeLimit - irrZone[zi].elapsedTime;
        totalPulses = (remainingTime / irrZone[zi].pulseTimeLimit) +
            (((remainingTime % irrZone[zi].pulseTimeLimit) > 0) ? 1 : 0);
        finishTime = remainingTime +
            (irrZone[zi].soakTimeLimit * (totalPulses - 1));
    }

    return finishTime;
}


/******************************************************************************
 *
 * irrSensorGroupCount
 *
 * PURPOSE
 *      This routine is called to count the number of zones in a sensor group.
 *
 * PARAMETERS
 *      leadZone    IN  the group leader's zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the number of zones in the sensor group, including
 *      the group leader.
 *
 *****************************************************************************/
static uint8_t irrSensorGroupCount(uint8_t leadZone)
{
    uint8_t zi;                 /* zone index */
    uint8_t count = 0;          /* count of zones in sensor group */

    for (zi = 0; zi < irrNumZones; zi++)
    {
        if (irrZone[zi].group == (leadZone - 1))
        {
            count++;
        }
    }
    return count;
}


/******************************************************************************
 *
 * irrCmdManualStart
 *
 * PURPOSE
 *      This routine is called to execute the Manual Start irrigation command.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrCmdManualStart(void)
{
    irrStop = FALSE;
    irrStart(SYS_STATE_MANUAL,
             sysManualProgram,
             sysManualOpMode,
             sysManualPulseMode);
}


/******************************************************************************
 *
 * irrCmdForceOn
 *
 * PURPOSE
 *      This routine is called to execute the Force-On command and start
 *      the Force-On irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The Force-On irrigation program is defined as the one with the longest
 *      total runtime.  If more than one program have equally long runtime,
 *      the first program is run (i.e., lowest number where A=0, ..., D=3).
 *
 *****************************************************************************/
void irrCmdForceOn(void)
{
    int32_t runtime;                    /* sum of zone runtimes */
    int32_t maxRuntime = 0;             /* max program runtime */
    uint8_t maxProgram = 0;             /* program with max runtime */
    uint8_t zi;                         /* zone index */
    uint8_t pi;                         /* program index */

    irrStop = FALSE;

    if ((irrState != IRR_STATE_IDLE) | (irrExpRunningProg != IRR_PGM_NONE))
    {
        /* Trace Force On Busy/Ignore event. */
        sysEvent(IRR_EVENT_FORCE_BSY, 0);

        dtDebug("Irrigation is active.  Ignoring Force-On.\n");
        return;
    }

    /* Find program with longest runtime. */
    for (pi = 0; pi < SYS_N_PROGRAMS; pi++)
    {
        runtime = 0;
        for (zi = 0; zi < config.sys.numZones; zi++)
        {
            runtime += config.zone[zi].runTime[pi];
        }
        if (runtime > maxRuntime)
        {
            maxRuntime = runtime;
            maxProgram = pi;
        }
    }
    /* Start longest runtime program, if any was found. */
    if (maxRuntime > 0)
    {
        irrStart(SYS_STATE_FORCE,
                 maxProgram,
                 CONFIG_OPMODE_RUNTIME,
                 CONFIG_PULSEMODE_OFF);
    }
}


/******************************************************************************
 *
 * irrCmdExpPulseOn
 *
 * PURPOSE
 *      This routine is called to execute the expansion unit pulse mode start/
 *      command. In pulse mode the master tells each expansion unit when they
 *      can run the program
 *
 * PARAMETERS
 *      program   program to run
 *
 * RETURN VALUE
 *      True      if program scheduled
 *      False     if program not scheduled due to another program scheduled
 *
 * NOTES
 *
 *****************************************************************************/
bool_t irrCmdExpPulseOn(uint8_t program)
{
    irrStop = FALSE;
    if (irrState != IRR_STATE_IDLE)
    {
        /* Trace Force On Busy/Ignore event. */
        sysEvent(IRR_EVENT_FORCE_BSY, 0);

        dtDebug("Irrigation is active.  Ignoring Expansion Pulse Start Command\n");
        return FALSE;
    }
    
    if (irrAutoPgmPending == IRR_PGM_NONE)
    {
        /* Schedule program to auto-start. */
        irrAutoPgmPending = program;
        return TRUE;
    }
    
    return FALSE;
}

/******************************************************************************
 *
 * irrStart
 *
 * PURPOSE
 *      This routine is called to start an irrigation program.
 *
 * PARAMETERS
 *      cause       IN  the irrigation start cause (system state)
 *      program     IN  the irrigation program to start
 *      opMode      IN  the irrigation operation mode to use
 *      pulseMode   IN  the irrigation pulse mode to use
 *
 * RETURN VALUE
 *      This routine returns TRUE if the program was successfully started;
 *      otherwise FALSE.
 *
 *****************************************************************************/
static bool_t irrStart(uint8_t cause,
                       uint8_t program,
                       uint8_t opMode,
                       uint8_t pulseMode)
{
    bool_t started;         /* program started indication return code */

    /* Check for invalid irrigation state. */
    if ((irrState != IRR_STATE_IDLE) &&
        (irrState != IRR_STATE_SENSING))
    {
        /* Can't start a new program when one is already starting or active. */
        dtDebug("Irrigation Start Failed\n");
        return FALSE;
    }

    /* Trace irrigation start event. */
    if (irrState == IRR_STATE_SENSING)
    {
        sysEvent(IRR_EVENT_IRR_SENSED, 0);
    }
    else
    {
        sysEvent(IRR_EVENT_IRR_START,
                  ((sysTestRunTime & 0x00FF) << 8) |
                  ((program & 0x0003)<< 6) |
                  ((opMode & 0x0003) << 4) |
                  ((pulseMode & 0x0001) << 3) |
                  (cause & 0x0007));
    }

      
    /* in case first zone is a set to a sensor concetrator set flag to false so will wait until
        SC checks in */
    
    scCheckedIn = FALSE;
    
    //if(cause == SYS_STATE_TEST) scCheckedIn = TRUE;
            
    //reset timeout counter
    radioSnsConCheckinTime = dtTickCount;
    
    /*make sure system isnt paused from a stop command */
    sysResume();
    
    /* Set the system state to indicate irrigation start cause. */
    sysState = cause;

    /* Save irrigation program start parameters. */
    irrProgram = program;
    irrOpMode = opMode;
    irrPulseMode = pulseMode;

    /* Check if moisture sensors must be sampled before starting. */
    if ((irrState != IRR_STATE_SENSING) &&
        (opMode == CONFIG_OPMODE_SENSOR))
    {
        /* Change irrigation state to Sensing. */
        irrState = IRR_STATE_SENSING;
        /* Initiate sampling of all configured moisture sensors. */
        moistSampleAll();
        /* Sampling is asynchronous and can take several seconds. */
        /* Save sample start time for use by sensor wait logic. */
        irrSenStartTime = dtTickCount;
        /* Return to caller with success indication. */
        /* Note: This routine is called again after sensor wait is up. */
        return TRUE;
    }

    /*
    **  Calculate irrigation program parameters and initialize the
    **  zone parameter snapshot data store.
    */
    irrNumZones = config.sys.numZones;

    /* Clear all zone snap-shot parameters. */
    irrZoneClear();

    if (cause == SYS_STATE_TEST)
    {
        /* Initialize zone parameters for test operation. */
        irrStartTestInit();
    }
    else
    {
        /* Initialize zone parameters for runtime operation. */
        irrStartZoneInit();
    }

    /* Initialize for weather or sensor based irrigation. */
    if (irrOpMode == CONFIG_OPMODE_WEATHER)
    {
        /* Set weather runtimes. */
        irrStartWeatherInit();
    }
    else if (irrOpMode == CONFIG_OPMODE_SENSOR)
    {
        /* Set moisture sensor max values. */
        irrStartSensorInit();
    }

    /* Initialize for pulse mode irrigation. */
    if (irrPulseMode == CONFIG_PULSEMODE_ON)
    {
        /* Set pulse mode parameters. */
        irrStartPulseInit();
    }

    /*
    **  Find and select the first zone to water.
    */
    irrNextZone = irrSelectNextZone();




    /* If a waterable zone was found, start irrigation. */
    if (irrNextZone != 0)
    {
        /* Write irrigation start debug message. */
        irrStartedDebug();

        /* Save irrigation program start time. */
        irrPgmStartTime = dtTickCount;

        /* Start the first irrigation watering cycle. */
        irrCycleStart();

        /* Set return code to indicate program was successfully started. */
        started = TRUE;
    }
    else
    {
        /* Write irrigation aborted debug message. */
        dtDebug("Irrigation Start Aborted - No Watering Required\n");

        /* Irrigation program is finished - no runnable zones. */
        irrProgramFinished();

        /* Set return code to indicate program was not started. */
        started = FALSE;
    }

    return started;
}


/******************************************************************************
 *
 * irrZoneClear
 *
 * PURPOSE
 *      This routine is called to clear the count and time parameters in
 *      the irrigation zone "snap-shot" data store.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrZoneClear(void)
{
    uint8_t zi;             /* zone index */

    /* Clear all counters/timers in zone snap-shot. */
    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        irrZone[zi].actualTimeLimit = 0;
        irrZone[zi].elapsedPulseTime = 0;
        irrZone[zi].elapsedSoakTime = 0;
        irrZone[zi].elapsedTime = 0;
        irrZone[zi].pulseTimeLimit = 0;
        irrZone[zi].soakTimeLimit = 0;
        irrZone[zi].group = -1;
        irrZone[zi].appRate = 0;
        irrZone[zi].appEff = 0;
        irrZone[zi].maxMoist = 0;
        irrZone[zi].flags = 0;
    }
}


/******************************************************************************
 *
 * irrStartTestInit
 *
 * PURPOSE
 *      This routine is called to initialize irrigation settings for an
 *      irrigation test operation.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartTestInit(void)
{
    uint16_t testSeconds;       /* number of seconds to test each zone */
    uint8_t zi;                 /* zone index */

    /* Compute per-zone test time in seconds. */
    testSeconds = sysTestRunTime * 60;

    /* Initialize each zone to run for test time. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        irrZone[zi].actualTimeLimit = testSeconds;
    }
}


/******************************************************************************
 *
 * irrStartZoneInit
 *
 * PURPOSE
 *      This routine is called to initialize a "snap-shot" of basic
 *      irrigation settings for each zone.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartZoneInit(void)
{
    uint8_t zi;                 /* zone index */

    /*
    **  Take a snap-shot of current zone configuration settings for use in
    **  irrigation program operation.
    */

    for (zi = 0; zi < irrNumZones; zi++)
    {
        /*
        **  Store runtime for zone (in seconds).
        **  NOTE:  Actual time limit is initialized to configured program
        **  runtime for the zone.
        */
        irrZone[zi].actualTimeLimit =
            config.zone[zi].runTime[irrProgram] * 60;

        /*
        **  Store sensor group and maximum moisture value for use
        **  in Sensor-based operating mode.
        */
        irrZone[zi].group = config.zone[zi].group;
        irrZone[zi].maxMoist = config.zone[zi].maxMoist;

        /*
        **  Store application rate and application efficiency for use
        **  in Weather-based operating mode.
        **  NOTE:  These values are also used for moisture balance accounting
        **  which is maintained across all operating modes.
        */
        irrZone[zi].appRate = ntohs(config.zone[zi].appRate);
        irrZone[zi].appEff = config.zone[zi].appEff;
    }
}


/******************************************************************************
 *
 * irrStartSensorInit
 *
 * PURPOSE
 *      This routine is called to initialize settings for an irrigation cycle
 *      running in sensor mode.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartSensorInit(void)
{
    uint8_t zi;             /* zone index */
    int i;                  /* iteration index */
    int8_t moisture;        /* current zone moisture value */

    /* Initialize moisture sensor sample frequency. */
    moistFreqSetAll(MOIST_FREQ_INACTIVE);

    /* Initialize moisture sensor thresholds. */
    moistSensorThreshInit();

    /* Don't run zones with moisture greater than the minimum value. */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        if (irrIsConfigGroupLeader(zi + 1))
        {
            /* Get current moisture reading for group leader. */
             moisture = moistValueGet(zi + 1);
             if (moisture >= config.zone[zi].minMoist)
             {
                 /* Moisture above minimum - don't water. */
                 /* Zero the runtime for all zones in sensor group. */
                 for (i = 0; i < irrNumZones; i++)
                 {
                     if (config.zone[i].group == zi)
                     {
                         irrZone[i].actualTimeLimit = 0;
                         irrZone[i].flags |= IRR_ZF_MIN_MET;
                     }
                 }
             }
        }
    }
}


/******************************************************************************
 *
 * irrStartWeatherInit
 *
 * PURPOSE
 *      This routine is called to calculate weather-based runtimes for each
 *      zone, updating the actual time limit set in the "snap-shot".
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartWeatherInit(void)
{
    uint8_t zi;             /* zone index */
    uint16_t mbRunTime;     /* weather-based run time calculation */

    /*
    **  Note:  Each zone's actualTimeLimit is already initialized to the
    **  configured runtime for the current program.
    */
    for (zi = 0; zi < irrNumZones; zi++)
    {
        mbRunTime = irrWeatherRuntimeCalc(zi);
        /* Use calculated runtime if less than configured runtime. */
        if (mbRunTime < irrZone[zi].actualTimeLimit)
        {
            irrZone[zi].actualTimeLimit = mbRunTime;
        }
    }
}


/******************************************************************************
 *
 * irrWeatherRuntimeCalc
 *
 * PURPOSE
 *      This routine is called to calculate the zone runtime in weather mode,
 *      based on application rate and Moisture Balance deficit.  If there is
 *      no MB deficit, the runtime will be zero.
 *
 * PARAMETERS
 *      zi      IN  the zone index (0-47)
 *
 * RETURN VALUE
 *      This routine returns the number of irrigation seconds required to
 *      satisfy the MB deficit for the specified zone.
 *
 * NOTES
 *      If this routine calculates a number of irrigation seconds greater
 *      than the maximum configurable runtime, the return value is capped
 *      at the maximum configurable runtime seconds.
 *
 *****************************************************************************/
uint16_t irrWeatherRuntimeCalc(uint8_t zi)
{
    uint32_t runtime = 0;       /* calculated zone runtime */
    uint32_t appVal;            /* product of app rate and efficiency */

    if (zi < SYS_N_ZONES)
    {
        appVal = ntohs(config.zone[zi].appRate) * config.zone[zi].appEff;

        /* Test for Moisture Balance deficit. */
        if (irrMoistureBalance[zi] < 0)
        {
            /*
            **  Calculate runtime in seconds needed to bring MB value to zero.
            **  Note: appVal - 1 is added to numerator to round-up the result.
            */
            runtime =
                ((-irrMoistureBalance[zi] * 3600 * 100) + appVal - 1) / appVal;

            /* Don't let calculated runtime exceed max configurable runtime. */
            if (runtime > CONFIG_RUNTIME_MAX_SECS)
            {
                runtime = CONFIG_RUNTIME_MAX_SECS;
            }
        }
    }

    return (uint16_t)runtime;
}


/******************************************************************************
 *
 * irrCropCoefficient
 *
 * PURPOSE
 *      This routine is called to calculate the crop coefficient for a turf
 *      plant type.
 *
 * PARAMETERS
 *      zi      IN  the zone index (0-47)
 *      type    IN  the plant type species
 *
 * RETURN VALUE
 *      This routine returns the number of crop coefficient factor for
 *      the zone.  This routine returns zero if the zone is not configured
 *      for a turf plant type.
 *
 * NOTES
 *      The zone must be configured for either Fescue or Bermuda turf.
 *
 *****************************************************************************/
static uint8_t irrCropCoefficient(uint8_t zi, uint8_t type)
{
    uint8_t factor = 0;

    if (zi < SYS_N_ZONES)
    {
        switch (type)
        {
            case CONFIG_PLANTTYPE_FESCUE:
                factor = irrCropFactor[0][config.zone[zi].climate][dtMon];
                break;

            case CONFIG_PLANTTYPE_BERMUDA:
                factor = irrCropFactor[1][config.zone[zi].climate][dtMon];
                break;

            default:
                /* Zone is not configured for a turf plant type. */
                break;
        }
    }

    return factor;
}


/******************************************************************************
 *
 * irrWeatherUpdate
 *
 * PURPOSE
 *      This routine is called to update the moisture balance for each zone
 *      with new weather data received from the radio.
 *
 * PARAMETERS
 *      etData      IN  the new ET data value (in 1/100 inch units)
 *      rainfall    IN  the new rainfall value (in 1/100 inch units)
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine uses the irrWeatherUpdateZone helper routine to perform
 *      the Weather-based irrigation algorithm calculation and update zone MB
 *      values.
 *
 *****************************************************************************/
void irrWeatherUpdate(uint16_t etData, uint16_t rainfall)
{
    uint8_t zi;                 /* zone index */

    /* Update last ET/Rainfall Data received time. */
    irrLastEtDataTime = dtTickCount;

    /* Trace Weather Update event and data. */
    sysEvent(IRR_EVENT_ET_DATA, etData);
    if (rainfall > 0)
    {
        sysEvent(IRR_EVENT_RAIN, rainfall);
    }

    /*
    **  Apply the ET/Rainfall data to update MB value for each configured zone.
    */
    for (zi = 0; zi < config.sys.numZones; zi++)
    {
        irrWeatherUpdateZone(zi, etData, rainfall);
    }
}


/******************************************************************************
 *
 * irrWeatherUpdateZone
 *
 * PURPOSE
 *      This helper routine is called by irrWeatherUpdate to perform the
 *      Weather-based irrigation algorithm calculation for the specified
 *      ET and rainfall data and to update the MB value for the specified zone.
 *
 * PARAMETERS
 *      zi          IN  the zone index (0-47)
 *      etData      IN  the new ET data value (in 1/100 inch units)
 *      rainfall    IN  the new rainfall value (in 1/100 inch units)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrWeatherUpdateZone(uint8_t zi, uint16_t etData, uint16_t rainfall)
{
    uint8_t type;               /* plant type species */
    uint8_t density;            /* plant type density */
    uint8_t dt;                 /* plant type drought tolerance */
    uint32_t rain;              /* rainfall * 10 */
    uint32_t et;                /* et data * 10 */
    int32_t mb;                 /* mb adjust * 10 */

    /* Insure zone index is in valid range. */
    if (zi >= SYS_N_ZONES)
    {
        return;
    }

    /* Note: The rainfall and etData values are scaled by 10x for precision. */
    rain = 8 * rainfall;
    et = 10 * etData;

    configZonePlantTypeGet(zi, &type, &density, &dt);

    switch (type)
    {
        case CONFIG_PLANTTYPE_TREES:
        case CONFIG_PLANTTYPE_SHRUBS:
        case CONFIG_PLANTTYPE_GROUNDCOVER:
        case CONFIG_PLANTTYPE_MIXTURE:
            mb = rain - ((et *
                irrPlantTypeFactor[type][dt] *
                irrPlantClimateFactor[type][config.zone[zi].climate] *
                irrPlantDensityFactor[type][density]) / 1000);
            break;
        case CONFIG_PLANTTYPE_FESCUE:
        case CONFIG_PLANTTYPE_BERMUDA:
            mb = rain - ((et * irrCropCoefficient(zi, type)) / 100);
            break;
        default:
            /* should never reach */
            break;
    }

    /* Update the zone's MB value, scaling mb down from 10x. */
    irrMoistureBalance[zi] += (int16_t)(mb / 10);

    /* Insure the MB value is constrained within the allowed range. */
    if (irrMoistureBalance[zi] > ntohs(config.zone[zi].rzwws))
    {
        irrMoistureBalance[zi] = ntohs(config.zone[zi].rzwws);
    }
    else if (irrMoistureBalance[zi] < IRR_MB_MIN)
    {
        irrMoistureBalance[zi] = IRR_MB_MIN;
    }
}


/******************************************************************************
 *
 * irrStartPulseInit
 *
 * PURPOSE
 *      This routine is called to initialize zone parameters at start of an
 *      irrigation program running in pulse mode.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartPulseInit(void)
{
    uint8_t zi;

    for (zi = 0; zi < irrNumZones; zi++)
    {
        irrZone[zi].pulseTimeLimit = irrPulseTimeCalc(zi);
        irrZone[zi].elapsedPulseTime = 0;
        irrZone[zi].soakTimeLimit = irrSoakTimeCalc(zi);
        irrZone[zi].elapsedSoakTime = irrZone[zi].soakTimeLimit;
    }
}


/******************************************************************************
 *
 * irrPulseTimeCalc
 *
 * PURPOSE
 *      This routine is called to calculate the maximum pulse time for a
 *      zone, based on its configured soil type and slope.
 *
 * PARAMETERS
 *      zi      IN  the zone index (0-47)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds allowed for a pulse cycle.
 *
 *****************************************************************************/
static uint16_t irrPulseTimeCalc(uint8_t zi)
{
    uint32_t pulseTime = 0;
    int16_t div;
    uint32_t runtime;       /* configured runtime (in seconds) */

    /* Insure zone index is within valid range. */
    if (zi >= SYS_N_ZONES)
    {
        return 0;
    }

    runtime = config.zone[zi].runTime[irrProgram] * 60;

    div =
        ntohs(config.zone[zi].appRate) - irrIrFactor[config.zone[zi].soilType];
    if (div <= 0)
    {
        pulseTime = runtime;
    }
    else
    {
        pulseTime =
            (irrAsaFactor[config.zone[zi].soilType][config.zone[zi].slope] *
            3600) / div;
    }

    return (uint16_t)(pulseTime < runtime ? pulseTime : runtime);
}


/******************************************************************************
 *
 * irrSoakTimeCalc
 *
 * PURPOSE
 *      This routine is called to calculate the required soak time for a zone,
 *      based on its configured soil type and slope.
 *
 * PARAMETERS
 *      zi      IN  the zone index (0-47)
 *
 * RETURN VALUE
 *      This routine returns the number of seconds required for soak time
 *      between pulse cycles.
 *
 *****************************************************************************/
static uint16_t irrSoakTimeCalc(uint8_t zi)
{
    uint32_t soakTime = 0;
    uint32_t div;

    if (zi < SYS_N_ZONES)
    {
        div = irrIrFactor[config.zone[zi].soilType];
        if (div > 0)
        {
            soakTime =
                (irrAsaFactor[config.zone[zi].soilType][config.zone[zi].slope] *
                3600) / div;
        }
    }

    return (uint16_t)soakTime;
}


/******************************************************************************
 *
 * irrSolenoidControl
 *
 * PURPOSE
 *      This routine is used to control the master and zone solenoids.
 *
 * PARAMETERS
 *      zone        IN  zone to control (0=master solenoid)
 *      state       IN  state to set (FALSE=off, TRUE=on)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrSolenoidControl(uint8_t zone, bool_t state)
{
    char debugBuf[30];
    char zoneName[15];


    /* Make sure watchdog doesn't timeout on long debug message writes. */
    sysExecutionExtend();

    if (zone == 0)
    {
        sprintf(zoneName, "Master Valve");
    }
    else
    {
        sprintf(zoneName, "Zone %d", zone);
    }
    
    sprintf(debugBuf, "%s turned %s\n",
        zoneName,
        state ? "ON" : "OFF");
    dtDebug(debugBuf);

    /*if zone is assigned to a wireless sensor then have it turn on the solenoid 
    * there as well as the wired connection. The user will then be responsible for
    * making sure that only 1 valve is in fact connected. 
    * The wired valve wont be turned on until the Sensor Concentrator checks in and is told to turn on.
    * This was done to keep the watering times consistent */
    if(zone == 0)
    {   
        drvSolenoidSet(zone, state);    
    }
    else if(irrIsWirelessZone(zone))
    {            
        /* next time proper SC checks in tell it to turn on/off solenoid */
        irrSnsConSolUnitIndex = config.zone[zone-1].snsConTableIndex;
        
        if(state == TRUE)
        {
            switch(config.zone[zone-1].snsConChan)
            {
                case 0:
                    irrSnsConSolenoidChan = SC_CHAN_1_ON;
                    break;
                case 1:
                    irrSnsConSolenoidChan = SC_CHAN_2_ON;
                    break;
                case 2:
                    irrSnsConSolenoidChan = SC_CHAN_3_ON;
                    break;
                case 3:
                    irrSnsConSolenoidChan = SC_CHAN_4_ON;
                    break;
                default:
                    irrSnsConSolenoidChan = SC_ALL_CHAN_OFF;
            }
        }
        else if(state == FALSE)
        {
            /* place system into SOAK mode via the pause command until SC checks in */
            irrSnsConSolenoidChan = SC_ALL_CHAN_OFF;
        }
        
        /* place system into SOAK mode via the pause command until SC checks in */
       
        if(scCheckedIn == FALSE)
        {
            sysPause();
        }

    }
    else
    {
        // Always turn on the wired solenoid
        drvSolenoidSet(zone, state);
    }

}


/******************************************************************************
 *
 * irrSensorGroupRuntimeAdjust
 *
 * PURPOSE
 *      This routine is called in sensor mode to adjust runtimes of all zones
 *      in a sensor group once  the group leader has reached its maximum
 *      moisture setpoint (i.e., threshold was met).
 *
 * PARAMETERS
 *      leadZone    IN  group leader zone number (1-48)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrSensorGroupRuntimeAdjust(uint8_t leadZone)
{
    uint8_t zi;                     /* zone index */
    uint32_t runtimeRatio;          /* calculated sensor runtime ratio */

    /* Insure leadZone parameter is within valid range. */
    if ((leadZone == 0) || (leadZone > SYS_N_ZONES))
    {
        return;
    }

    runtimeRatio =
        (irrZone[leadZone - 1].elapsedTime * 1000) /
        irrZone[leadZone - 1].actualTimeLimit;

    for (zi = 0; zi < irrNumZones; zi++)
    {
        if (irrZone[zi].group == (leadZone - 1))
        {
            if (zi == (leadZone - 1))
            {
                /* This is the lead zone. */
                irrZone[zi].flags |= IRR_ZF_THRESH_MET;
                irrZone[zi].actualTimeLimit = irrZone[zi].elapsedTime;
            }
            else
            {
                /* This is a follower zone. */
                irrZone[zi].flags |= IRR_ZF_THRESH_ADJ;
                irrZone[zi].actualTimeLimit =
                    (uint16_t)((runtimeRatio * irrZone[zi].actualTimeLimit) /
                    1000);
            }
        }
    }
}


/******************************************************************************
 *
 * irrSoakTimeIncrement
 *
 * PURPOSE
 *      This routine is called to increment soak time for all zones.
 *
 * PARAMETERS
 *      elapsedSecs IN  seconds elapsed since last increment
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrSoakTimeIncrement(uint32_t elapsedSecs)
{
    uint8_t zi;                     /* zone index */

    if ((elapsedSecs != 0) &&
        (irrPulseMode == CONFIG_PULSEMODE_ON))
    {
        /* Increment soak times for all zones. */
        for (zi = 0; zi < irrNumZones; zi++)
        {
            if (irrZone[zi].elapsedSoakTime < irrZone[zi].soakTimeLimit)
            {
                irrZone[zi].elapsedSoakTime += (uint16_t)elapsedSecs;
            }
        }
    }
}

/******************************************************************************
 *
 * irrIsWirelessZone
 *
 * PURPOSE
 *      This routine is called to determine if a zone contains wireless
 *      sensors.  If zone is considered to be wireless if it points to
 *      a valid sensor concentrator and its sensor type is either
 *      SNS_WIRELESS_MOIST or SNS_WIRELESS_VALVE.
 *
 * PARAMETERS
 *      zone    IN  the zone number to test (1-48)
 *
 * RETURN VALUE
 *      This routine returns TRUE if the zone contains wireless sensors and
 *      FALSE if not.
 *
 *****************************************************************************/
static bool_t irrIsWirelessZone(uint8_t zone)
{
    bool_t isWireless = ((config.zone[zone-1].sensorType == SNS_WIRELESS_MOIST) ||
                         (config.zone[zone-1].sensorType == SNS_WIRELESS_VALVE));
    bool_t isValid = (config.zone[zone-1].snsConTableIndex >= 0);
    
    return (isWireless && isValid);
}


/******************************************************************************
 *
 * irrIsCurrentGroupLeader
 *
 * PURPOSE
 *      This routine is called to test if a zone is a sensor group leader
 *      in the currently active irrigation program.
 *
 * PARAMETERS
 *      zone    IN  the zone number to test (1-48)
 *
 * RETURN VALUE
 *      This routine returns TRUE if the zone is a group leader in the
 *      currently active irrigation program.
 *
 *****************************************************************************/
bool_t irrIsCurrentGroupLeader(uint8_t zone)
{
    bool_t isLeader = FALSE;        /* return value */

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        isLeader = (irrZone[zone - 1].group == (zone - 1));
    }

    return isLeader;
}


/******************************************************************************
 *
 * irrIsConfigGroupLeader
 *
 * PURPOSE
 *      This routine is called to test if a zone is configured as a sensor
 *      group leader.
 *
 * PARAMETERS
 *      zone    IN  the zone number to test (1-48)
 *
 * RETURN VALUE
 *      This routine returns TRUE if the zone is configured as a group leader.
 *
 *****************************************************************************/
bool_t irrIsConfigGroupLeader(uint8_t zone)
{
    uint8_t zi = zone - 1;          /* zone index */
    bool_t isLeader = FALSE;        /* return value */

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        isLeader = (config.zone[zi].group == zi);
    }

    return isLeader;
}


/******************************************************************************
 *
 * irrMoistConfigUpdate
 *
 * PURPOSE
 *      This routine is called to update the sample frequency for all moisture
 *      sensors in the system to match the current configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrMoistConfigUpdate(void)
{
    /* Only update if not currently running an irrigation program. */
    /* Otherwise, the update will occur when program finishes. */
    if (irrState == IRR_STATE_IDLE)
    {
        if (moistSensorsConfigured())
        {
            moistFreqSetAll(MOIST_FREQ_INACTIVE);
        }
        else
        {
            moistFreqSetAll(MOIST_FREQ_OFF);
        }
    }
}


/******************************************************************************
 *
 * irrZoneMoisture
 *
 * PURPOSE
 *      This routine is called to get the current moisture sensor reading
 *      for a zone.
 *
 * PARAMETERS
 *      zone    IN  the zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the most recent moisture sensor reading
 *      as a percentage value (0-100).  This routine returns -1 if the
 *      sensor has failed.
 *
 *****************************************************************************/
int8_t irrZoneMoisture(uint8_t zone)
{
    return moistValueGet(zone);
}


/******************************************************************************
 *
 * irrOpModeConfigChanged
 *
 * PURPOSE
 *      This routine is called to notify the irrigation subsystem that
 *      the configured mode of operation has changed.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrOpModeConfigChanged(void)
{
    /* RFU: Nothing to do in current design. */
}


/******************************************************************************
 *
 * irrSensorThresholdCheck
 *
 * PURPOSE
 *      This routine is called to check if any sensor thresholds have been
 *      met and adjust follower zones accordingly.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrSensorThresholdCheck(void)
{
    uint8_t zi;                 /* zone index */

    if (irrOpMode == CONFIG_OPMODE_SENSOR)
    {
        /* In sensor mode, check group leader zones for completion. */
        for (zi = 0; zi < irrNumZones; zi++)
        {
            if (irrIsCurrentGroupLeader(zi + 1))
            {
                /* This zone has a sensor. */
                if (irrZoneMoisture(zi + 1) < 0)
                {
                    /* Moisture sensor has failed. */
                    /* Water zone until standard runtime limit. */
                }
                else if ((irrRemainingZoneSecs(zi + 1) > 0) &&
                    moistThresholdMet(zi + 1))
                {
                    /* Adjust runtimes for other members of same sensor group. */
                    irrSensorGroupRuntimeAdjust(zi + 1);
                    /* Set moisture sensor sample frequency to inactive rate. */
                    moistFreqSet(zi + 1, MOIST_FREQ_INACTIVE);
                }
            }
        }
    }
}


/******************************************************************************
 *
 * irrStartedDebug
 *
 * PURPOSE
 *      This routine is called to write a debug message on starting an
 *      irrigation program.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void irrStartedDebug(void)
{
    char debugBuf[100];
    char tmpBuf[41];

    /* Print time-stamped debug message. */
    sprintf(debugBuf, "Irrigation Started - %s\n",
        uiFormatSystemMode(tmpBuf));
    dtDebug(debugBuf);
}


/******************************************************************************
 *
 * irrHaveMbDeficit
 *
 * PURPOSE
 *      This routine is called to determine if the system has any zones
 *      with a Moisture balance deficit (i.e., if Weather-based operation
 *      might require irrigation).
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if any configured zone has a negative
 *      Moisture Balance value.
 *
 *****************************************************************************/
bool_t irrHaveMbDeficit(void)
{
    uint8_t zi;                     /* zone index */
    bool_t haveCredit = FALSE;      /* have ET credit if TRUE */

    for (zi = 0; zi < config.sys.numZones; zi++)
    {
        if (irrMoistureBalance[zi] < 0)
        {
            haveCredit = TRUE;
            break;
        }
    }

    return haveCredit;
}


/******************************************************************************
 *
 * irrHaveRunnableProgram
 *
 * PURPOSE
 *      This routine is called to check if a runnable program exists in the
 *      current configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if any runnable program exists; otherwise
 *      FALSE is returned.
 *
 * NOTES
 *      A program is runnable if two conditions are met:
 *         1) There is at least one start time configured for the program.
 *         2) There is at least one zone with non-zero run time configured
 *            for the program.
 *
 *****************************************************************************/
static bool_t irrHaveRunnableProgram(void)
{
    uint8_t day;                            /* day index */
    uint8_t pi;                             /* program index */
    uint8_t zi;                             /* zone index */
    bool_t pgmStarts;                       /* pgm has a start time if TRUE */
    bool_t haveRunnableProgram = FALSE;     /* have runnable pgm if TRUE */

    for (pi = 0; pi < SYS_N_PROGRAMS; pi++)
    {
        pgmStarts = FALSE;
        for (day = 0; day < CONFIG_SCHED_DAY_LIMIT; day++)
        {
            if (config.sched[day][pi].startTime == htons(CONFIG_SCHED_START_DISABLED))
            {
                continue;
            }
            pgmStarts = TRUE;
            break;
        }
        if (pgmStarts)
        {
            for (zi = 0; zi < config.sys.numZones; zi++)
            {
                if (config.zone[zi].runTime[pi] != 0)
                {
                    haveRunnableProgram = TRUE;
                    break;
                }
            }
        }
    }

    return haveRunnableProgram;
}


/******************************************************************************
 *
 * irrMbFix
 *
 * PURPOSE
 *      This routine is called to fix any discrepancies in the Moisture
 *      Balance data store.  Each zone's Moisture Balance is checked against
 *      the current RZWWS configuration value.  If a Moisture Balance value
 *      exceeds the current RZWWS value, it is changed to the RZWWS value.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void irrMbFix(void)
{
    uint8_t zi;                     /* zone index */

    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        if (irrMoistureBalance[zi] > ntohs(config.zone[zi].rzwws))
        {
            irrMoistureBalance[zi] = ntohs(config.zone[zi].rzwws);
        }
    }
}


/******************************************************************************
 *
 * irrMbGet
 *
 * PURPOSE
 *      This routine is called to get the current Moisture Balance value for
 *      a zone.
 *
 * PARAMETERS
 *      zone    IN      zone number (1-48)
 *
 * RETURN VALUE
 *      This routine returns the current MB value for the given zone.
 *
 *****************************************************************************/
int16_t irrMbGet(uint8_t zone)
{
    int16_t mbValue = 0;

    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        mbValue =  irrMoistureBalance[zone - 1];
    }

    return mbValue;
}


/******************************************************************************
 *
 * irrMbSet
 *
 * PURPOSE
 *      This routine is called to set the current Moisture Balance value for
 *      a zone.
 *
 * PARAMETERS
 *      zone        IN      zone number (1-48)
 *      mbValue     IN      moisture balance value
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine checks for (and corrects) MB values that are outside the
 *      allowed range for the zone.  MB values outside the allowed range are
 *      capped at their respective minimum or maximum values before they are
 *      applied.
 *
 *****************************************************************************/
void irrMbSet(uint8_t zone, int16_t mbValue)
{
    uint8_t zi = zone - 1;          /* zone index */

    /* Insure zone number is within limits. */
    if ((zone > 0) && (zone <= SYS_N_ZONES))
    {
        /* Insure that MB value is not above the zone's RZWWS limit. */
        if (mbValue > ntohs(config.zone[zi].rzwws))
        {
            irrMoistureBalance[zi] = ntohs(config.zone[zi].rzwws);
        }
        /* Insure that MB value is not below system's minimum allowed MB. */
        else if (mbValue < IRR_MB_MIN)
        {
            irrMoistureBalance[zi] = IRR_MB_MIN;
        }
        else
        {
            irrMoistureBalance[zi] = mbValue;
        }
    }
}
/******************************************************************************
*
*  detect the gallon per min, shut off when flow exceeds max or below min value 
*  PARAMETERS
*  
*     zi           IN    zone index, current zone
*  
*  RETURN VALUE:  false when error 
*
*
*******************************************************************************/
static bool_t flowMinMax(uint8_t zi)    // zi: 1--12
{
   //if ( config.zone[zi-1].sensorType == SNS_FLOW )  0--11
   
   int i;
   //int flowIndex; // 1--12
   
     
   for (i=0; i<12; i++) {
        if ( config.zone[i].sensorType == SNS_FLOW ) {
            flowIndex = i+1;
            findFlow = 1;
            break;
        } 
   }
   
   // can not find a flow meter
   //if (i == 12) {
   //     GPM = 0;
   //     findFlow = 0;
   //     return TRUE;
   //}
   
   // read flow value if find a flow meter  
   if( (config.sys.unitType == UNIT_TYPE_MASTER) ) //&& (findFlow == 1) )
   {   
            if (i == 12){
                GPM = 0;
                findFlow = 0;
                return TRUE;
            }                 
            else {    
                moistSample(flowIndex);
                GPM = moistValueGet(flowIndex);
            }
   } 
   else {   
         GPM = slaveGPM;  
         flowDelay = slaveFlowDelay;
         findFlow = slaveFindFlow;    
   }
   
   // expansion unit 
   if (GPM == 0)
        return TRUE;
  
  
   if ((GPM < config.zone[zi-1].minGPM || GPM > config.zone[zi-1].maxGPM) && (!timeFlag))
   { 
        eTime = dtTickCount;
        timeFlag = TRUE;
   }
   
   if ( (flowDelay != 0) && ((dtTickCount - eTime) > flowDelay) && timeFlag )
        return FALSE;
   else 
        return TRUE;
   
   /*
   if (zi == 1 && irrZone[0].elapsedTime < 5) // zone 1 delay 5 seconds reading
   {        
        GPM = 1;
   } else {   
        GPM = moistValueGet(zin);
   }
   */
}

















