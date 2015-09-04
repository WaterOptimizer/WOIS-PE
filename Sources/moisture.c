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
 * Module       : moisture.c
 * Description  : This file implements the moisture sensor logic.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include "global.h"
#include "system.h"
#include "config.h"
#include "moisture.h"
#include "drvMoist.h"
#include "datetime.h"



/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/

uint64_t moistFailedSensors = 0;    /* bitmap of failed sensors (1=failed) */
uint32_t moistLastTest = 0;         /* tick count at last sensor check */



/******************************************************************************
 *
 *  MOISTURE LOGIC FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void moistFailureSet(uint8_t zone);
void moistFailureClear(uint8_t zone);
static void moistFailureClearUnconfigured(void);



/******************************************************************************
 *
 * moistInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization function to
 *      initialize the moisture sensor logic and moisture sensor state data
 *      store.
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
void moistInit(void)
{
    if (moistSensorsConfigured())
    {
        /* Initialize moisture sensor sample frequency. */
        moistFreqSetAll(MOIST_FREQ_INACTIVE);

        /* Get initial moisture samples. */
        moistSampleAll();
    }
    else
    {
        /* Disable moisture sensor sampling. */
        moistFreqSetAll(MOIST_FREQ_OFF);
    }
}


/******************************************************************************
 *
 * moistPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to
 *      manage the moisture sensor logic.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistPoll(void)
{
    uint8_t zi;
    bool_t sensorsConfigured = moistSensorsConfigured();

    /* Check if sensor operating mode and if any sensors are configured. */
    if ((config.sys.opMode == CONFIG_OPMODE_SENSOR) &&
        !sensorsConfigured)
    {
        /* Set no sensors configured error. */
        sysErrorSet(SYS_ERROR_NOSENSORS);
    }
    else
    {
        /* Clear no sensors configured error. */
        sysErrorClear(SYS_ERROR_NOSENSORS);
    }

    /* Test moisture sensors for failure (or recovery from failure). */
    if (dtElapsedSeconds(moistLastTest) > MOIST_TEST_SECS)
    {
        moistLastTest = dtTickCount;
        /* Clear failures for unconfigured sensors. */
        moistFailureClearUnconfigured();
        /* Only test sensors if sensors configured. */
        if (sensorsConfigured)
        {
            /* Check all configured sensors. */
            for (zi = 0; zi < config.sys.numZones; zi++)
            {
                if (moistSensorConfigured(zi + 1))
                {
                    /* Calling function is sufficient to check for failure. */
                    moistValueGet(zi + 1);
                }
            }
        }
    }

    /* Maintain moisture sensor fault alarm condition. */
    if (moistFailedSensors == 0)
    {
        /* Set moisture sensor fault alarm. */
        sysFaultClear(SYS_FAULT_MOIST);
    }
    else
    {
        /* Clear moisture sensor fault alarm. */
        sysFaultSet(SYS_FAULT_MOIST);
    }

    

}


/******************************************************************************
 *
 * moistSensorThreshInit
 *
 * PURPOSE
 *      This routine is called to initialize moisture sensor thresholds.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistSensorThreshInit(void)
{
    uint8_t zi;

    /* Only initialize zones with sensors. */
    for (zi = 0; zi < config.sys.numZones; zi++)
    {
        if (moistSensorConfigured(zi + 1))
        {
            moistThresholdSet(zi + 1, config.zone[zi].maxMoist);
        }
    }
}


/******************************************************************************
 *
 * moistThresholdSet
 *
 * PURPOSE
 *      This routine is called to set the threshold for a moisture sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *      threshold   IN  percent moisture threshold (0-100)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistThresholdSet(uint8_t zone, uint8_t threshold)
{
    if (zone <= SYS_N_UNIT_ZONES)
    {
        drvMoistThresholdSet(zone, threshold);
    }
    else
    {
        /* FUTURE: set expansion unit sensor threshold. */
#ifdef WIN32
        drvMoistThresholdSet(zone, threshold);
#endif
    }
}


/******************************************************************************
 *
 * moistThresholdGet
 *
 * PURPOSE
 *      This routine is called to get the threshold for a moisture sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      This routine returns the current threshold value for the specified
 *      zone's moisture sensor.
 *
 *****************************************************************************/
uint8_t moistThresholdGet(uint8_t zone)
{
    uint8_t threshold = 0;

    if (zone <= SYS_N_UNIT_ZONES)
    {
        threshold = drvMoistThresholdGet(zone);
    }
    else
    {
        /* FUTURE: set expansion unit sensor threshold. */
#ifdef WIN32
        threshold = drvMoistThresholdGet(zone);
#endif
    }

    return threshold;
}


/******************************************************************************
 *
 * moistSampleAll
 *
 * PURPOSE
 *      This routine is called to initiate sampling of all configured
 *      moisture sensors in the system.  The sensors are read asynchronously,
 *      and typically within one second.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistSampleAll(void)
{
    uint8_t zi;
    
    for (zi = 0; zi < config.sys.numZones; zi++)
    {
        if (moistSensorConfigured(zi + 1))
        {
            moistSample(zi + 1);
        }
    }
}


/******************************************************************************
 *
 * moistSample
 *
 * PURPOSE
 *      This routine is called to initiate sampling for a moisture sensor.
 *      The sensor moisture value will be read asynchronously, and typically
 *      within one second.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistSample(uint8_t zone)
{
    if (zone <= SYS_N_UNIT_ZONES)
    {
        drvMoistSample(zone);
    }
    else
    {
        /* FUTURE: sample expansion unit sensor. */
#ifdef WIN32
        drvMoistSample(zone);
#endif
    }
}


/******************************************************************************
 *
 * moistValueGet
 *
 * PURPOSE
 *      This routine is called to get the current moisture value for a sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      This routine returns the current moisture sensor value, in percent
 *      moisture (0-100).  A return value of -1 indicates a sensor failure.
 *
 *****************************************************************************/
int8_t moistValueGet(uint8_t zone)
{
    int8_t moistValue = -1;

    if (zone <= SYS_N_UNIT_ZONES)
    {
        moistValue = drvMoistValueGet(zone);
    }
    else
    {
        /* FUTURE: get expansion unit sensor data. */
//#ifdef WIN32
//        moistValue = drvMoistValueGet(zone);
//#endif
		moistValue = expMoistValue[zone-13];
    }

    /* Check and manage moisture sensor failure status. */
    if (moistValue == -1)
    {
        /* Mark moisture sensor as failed. */
        moistFailureSet(zone);
    }
    else
    {
        /* Insure moisture sensor not marked as failed. */
        moistFailureClear(zone);
    }

    return moistValue;
}


/******************************************************************************
 *
 * moistThresholdMet
 *
 * PURPOSE
 *      This routine is called to test if the threshold has been met for
 *      a moisture sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      This routine returns TRUE if moisture sensor for zone has met its
 *      threshold value.
 *
 *****************************************************************************/
bool_t moistThresholdMet(uint8_t zone)
{
    bool_t thresholdMet = FALSE;

    if (zone <= SYS_N_UNIT_ZONES)
    {
        thresholdMet = drvMoistThresholdMet(zone);
    }
    else
    {
        /* FUTURE: get expansion unit sensor data. */
#ifdef WIN32
        thresholdMet = drvMoistThresholdMet(zone);
#endif
    }

    return thresholdMet;
}


/******************************************************************************
 *
 * moistSensorsConfigured
 *
 * PURPOSE
 *      This routine indicates whether any moisture sensors are configured in
 *      the system.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if any zone has a moisture sensor configured.
 *
 *****************************************************************************/
bool_t moistSensorsConfigured(void)
{
    uint8_t zi;
    bool_t configured = FALSE;
    uint8_t endZone=config.sys.numZones;

    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
        endZone=(config.sys.numUnits +1) *12;
    }
    
    for (zi = 0; zi < endZone; zi++)
    {
        if (config.zone[zi].group == zi)
        {
            configured = TRUE;
            break;
        }
    }

    return configured;
}


/******************************************************************************
 *
 * moistSensorConfigured
 *
 * PURPOSE
 *      This routine indicates whether a moisture sensor is configured for
 *      a zone.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      This routine returns TRUE if zone has a moisture sensor configured.
 *
 *****************************************************************************/
bool_t moistSensorConfigured(uint8_t zone)
{
    uint8_t zi = zone - 1;
    bool_t configured = FALSE;
    uint8_t endZone=config.sys.numZones;

    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
        endZone=(config.sys.numUnits +1) *12;
    }
    
    if ((zi < endZone) &&
        (config.zone[zi].group == zi))
    {
        configured = TRUE;
    }

    return configured;
}


/******************************************************************************
 *
 * moistFreqSet
 *
 * PURPOSE
 *      This routine is called to set the sample frequency for a moisture
 *      sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *      freq        IN  moisture sensor sample frequency
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistFreqSet(uint8_t zone, moistFreq_t freq)
{
    if (freq == MOIST_FREQ_NOCHANGE)
    {
        /* No change to this zone's current frequency setting. */
        return;
    }
    if (zone <= SYS_N_UNIT_ZONES)
    {
        drvMoistFreqSet(zone, (drvMoistFreq_t)freq);
    }
    else
    {
        /* FUTURE: set expansion unit sensor. */
#ifdef WIN32
        drvMoistFreqSet(zone, (drvMoistFreq_t)freq);
#endif
    }
}


/******************************************************************************
 *
 * moistFreqSetAll
 *
 * PURPOSE
 *      This routine is called to set all moisture sensors to a sample
 *      frequency.
 *
 * PARAMETERS
 *      freq        IN  moisture sensor sample frequency
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine disables sampling for all non-configured moisture
 *      sensor zones.
 *
 *****************************************************************************/
void moistFreqSetAll(moistFreq_t freq)
{
    uint8_t zi;

    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        if ((zi < config.sys.numZones) &&
            (moistSensorConfigured(zi + 1)))
        {
            if((config.zone[zi].sensorType == SNS_FLOW) ||
                (config.zone[zi].sensorType == SNS_PRESSURE) ||
                (config.zone[zi].sensorType == SNS_RAIN_GAUGE))
            {
                moistFreqSet(zi + 1, MOIST_FREQ_ACTIVE);
            }
            else
            {
                moistFreqSet(zi + 1, freq);
            }
                
        }
        else
        {
            /* No sensor or beyond configured number of zones - set to off. */
            moistFreqSet(zi + 1, MOIST_FREQ_OFF);
        }
    }
}


/******************************************************************************
 *
 * moistFailureSet
 *
 * PURPOSE
 *      This routine is called to set the failure flag for a configured
 *      moisture sensor.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine does not set a failure flag if the zone is not
 *      configured with a moisture sensor.
 *
 *****************************************************************************/
void moistFailureSet(uint8_t zone)
{
    if (moistSensorConfigured(zone))
    {
        moistFailedSensors |= ((uint64_t)1) << (zone - 1);
    }
}


/******************************************************************************
 *
 * moistFailureClear
 *
 * PURPOSE
 *      This routine is called to clear the moisture sensor failure flag for
 *      zone.
 *
 * PARAMETERS
 *      zone        IN  moisture sensor zone (1-48)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void moistFailureClear(uint8_t zone)
{
    moistFailedSensors &= ~(((uint64_t)1) << (zone - 1));
}


/******************************************************************************
 *
 * moistFailureClearUnconfigured
 *
 * PURPOSE
 *      This routine is called to clear the moisture sensor failure flags for
 *      all unconfigured zones.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine is used to clear moisture sensor failure flags for any
 *      zones configured to not have a sensor.
 *
 *****************************************************************************/
static void moistFailureClearUnconfigured(void)
{
    uint8_t zi;

    if (moistFailedSensors == 0)
    {
        /* Nothing to do. */
        return;
    }

    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        if ((moistFailedSensors & (((uint64_t)1) << zi)) != 0)
        {
            if (!moistSensorConfigured(zi + 1))
            {
                moistFailureClear(zi + 1);
            }
        }
    }
}


/******************************************************************************
 *
 * moistFailureZoneGet
 *
 * PURPOSE
 *      This routine is called to get the identity of the lowest numbered
 *      zone with a failed moisture sensor.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the first (lowest numbered) zone with a failed
 *      moisture sensor.  A value of zero indicates there are no failures.
 *
 *****************************************************************************/
uint8_t moistFailureZoneGet(void)
{
    uint8_t zi;
    uint8_t failedZone = 0;

    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        if ((moistFailedSensors & (((uint64_t)1) << zi)) != 0)
        {
            failedZone = zi + 1;
            break;
        }
    }
    return failedZone;
}
