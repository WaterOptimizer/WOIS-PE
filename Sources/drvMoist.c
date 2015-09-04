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
 * Module       : drvMoist.c
 * Description  : This file implements the driver for the moisture sensors,
 *                including support for switching sensor power and performing
 *                ADC readings.
 *
 *****************************************************************************/

/* MODULE drvMoist */

#include "global.h"
#include "drv.h"
#include "drvRtc.h"
#include "hwAdc.h"
#include "hwBusData.h"
#include "hwBusLatch2.h"
#include "config.h"
#include "system.h"
#include "radio.h"
#include "drvMoist.h"
#include "moisture.h"
#include "drvEeprom.h"
#include "datetime.h"

#define DRV_MOIST_NZONES        SYS_N_UNIT_ZONES
#if DRV_MOIST_NZONES > 12
#error "The moisture sensor driver does not support more than 12 sensors."
#endif

#define DRV_MOIST_ACTIVE_PERIOD         60
#define DRV_MOIST_INACTIVE_PERIOD       (60 * 60)

#define DRV_MOIST_NSAMPLES      2       /* at-threshold count hysterisis */

/*
 * Per Decagon:
 *   Water Content (%) = (3.72 * I) - 31
 *     where I is the current through the probe in milliamps
 *
 * The hardware should convert 1mA to an ADC input of 0.1V, so:
 *   Water Content (%) = (37.2 * V) - 31
 *     where V is the voltage at the ADC input in volts
 * and:
 *   V = (P + 31) / 37.2
 *     where P is the water content in percent
 * Note, however, that the actual results may be different due to the effects
 * of the filtering and overcurrent/overvoltage protection circuits.
 *
 * Substituting:
 *   ADC = V / 3.25 * 4095
 *   V = ADC / 4095 * 3.25
 * yields:
 *   Water Content (%) = (37.2 * ADC / 4095 * 3.25) - 31
 *                       (ADC * 0.02952381) - 31
 *
 * Testing was done on an EVT2 unit (serial #PROTO101, DB101, TB101, AC101),
 * zones 1 & 2).  The input was set to various DC current values (4mA, 8mA,
 * 12mA, 16mA, and 20mA) and the ADC readings measured.  Test results confirmed
 * that the ADC current measurements were within 1.3% of the expected values,
 * with the largest errors at the low end of the current range (values that
 * correspond to soil moisture levels below 0%).
 *
 * Using the above formulas based on an ADC Vref+ of 3.25V and a 100 ohm
 * resistor, this yields the following ADC values:
 *    0% = 0.833V = 1050 ADC
 *   40% = 1.909V = 2405 ADC
 *   60% = 2.446V = 3082 ADC
 *  100% = 3.521V = 4437 ADC
 */
#define DRV_MOIST_PERCENT_LO    0       /* minimum percent to report */
#define DRV_MOIST_PERCENT_HI    60      /* maximum percent to report */
#define DRV_MOIST_ADC_LO        1050    /* ADC value for min percent */
#define DRV_MOIST_ADC_HI        3082    /* ADC value for max percent */
#define DRV_MOIST_LO_LIMIT      -10     /* minimum percent w/o error */
#define DRV_MOIST_HI_LIMIT      65      /* maximum percent w/o error */
#define DRV_GENERIC_ADC_LO      505       /* ADC min value for generic sensor (4mA)*/
#define DRV_GENERIC_ADC_HI      2520    /* ADC max value for generic sensor (20mA)*/
#define DRV_GENERIC_LO_LIMIT    0     /* minimum percent w/o error */
#define DRV_GENERIC_HI_LIMIT    100      /* maximum percent w/o error */

#if DRV_MOIST_PERCENT_HI != SYS_MOISTURE_MAX
#error "Upper ADC/percent data point does not match SYS_MOISTURE_MAX."
#endif

#if DRV_MOIST_LO_LIMIT >= DRV_MOIST_PERCENT_LO
#error "Lower error threshold must be below valid percentage range."
#endif

#if DRV_MOIST_HI_LIMIT <= DRV_MOIST_PERCENT_HI
#error "Upper error threshold must be above valid percentage range."
#endif


typedef struct
{
    int8_t         lastValue;           /* last sample, as a percentage */
    uint8_t        threshold;           /* moisture level detect threshold */
    uint8_t        count;               /* # consecutive samples >= threshold */
    drvMoistFreq_t freq;                /* sampling frequence (and on/off) */
    uint32_t       nextTime;            /* RTC time to take next sample */
} drvMoistZone_t;

/* per-zone sensor configuration and state */
static drvMoistZone_t drvMoistZones[DRV_MOIST_NZONES] =
{
};

/*
 * Current zone being sampled:
 *   1-12 = zone # being sampled
 *   0    = idle
 *   -1   = shutdown
 */
static int8_t drvMoistActive;


/******************************************************************************
 *
 *  drvMoistRestart
 *
 *  DESCRIPTION:
 *      This driver internal function (re)initializes the Moisture driver.  It
 *      must be called after power-up and again whenever AC power is restored
 *      to (re)initialize the state of the current sensor being sampled.
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
void drvMoistRestart(void)
{
    drvMoistActive = 0;
}


/******************************************************************************
 *
 *  drvMoistShutdown
 *
 *  DESCRIPTION:
 *      This driver internal function shuts down the moisture sensor sampling
 *      operations prior to the transition into a low-power mode as a result of
 *      the failure of AC power.
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
void drvMoistShutdown(void)
{
    drvMoistActive = -1;
    drvMoistPower(0);
}


/******************************************************************************
 *
 *  drvMoistSample
 *
 *  DESCRIPTION:
 *      This driver API function schedules one or all enabled moisture sensors
 *      to be sampled as soon as possible.
 *
 *  PARAMETERS:
 *      zone (in) - If 0, then schedule all enabled sensors to be sampled.
 *                  Else, if 1..12, then schedule the zone (if enabled) to be
 *                  sampled.
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvMoistSample(uint8_t zone)
{
    uint32_t now = drvRtcGet();

    if (zone == 0)
    {
        for (int i = 0; i < DRV_MOIST_NZONES; i++)
        {
            if (drvMoistZones[i].freq != DRV_MOIST_FREQ_OFF)
            {
                drvMoistZones[i].nextTime = now;
            }
        }
    }
    else if (zone <= DRV_MOIST_NZONES)
    {
        if (drvMoistZones[zone - 1].freq != DRV_MOIST_FREQ_OFF)
        {
            drvMoistZones[zone - 1].nextTime = now;
        }
    }
}


/******************************************************************************
 *
 *  drvMoistValueGet
 *
 *  DESCRIPTION:
 *      This driver API function returns the most recent value sampled for a
 *      specific zone.
 *
 *  PARAMETERS:
 *      zone (in) - zone to retrieve (1..12)
 *
 *  RETURNS:
 *      moisture percentage level (0..60), or -1 for error
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      This is decoupled from the actual sensor I/O, which is done by the
 *      timer-based ISR.  This routine only reports the last value recorded.
 *
 *****************************************************************************/
int8_t drvMoistValueGet(uint8_t zone)
{
    return drvMoistZones[zone - 1].lastValue;
}


/******************************************************************************
 *
 *  drvMoistValueSet
 *
 *  DESCRIPTION:
 *      This driver API function sets the moisture value for a
 *      specific zone. this is used for expansion bus and sensor concentrator
 *
 *  PARAMETERS:
 *      zone (in) - zone to set (1..48)
 *      value (in) - moisture percentage level (0..60), or -1 for error
 *
 *  RETURNS:
 *      
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      This is decoupled from the actual sensor I/O, which is done by the
 *      timer-based ISR.  This routine only reports the last value recorded.
 *
 *****************************************************************************/
void drvMoistValueSet(uint8_t zone, uint8_t value)
{
     
   if(value !=RADIO_SENSOR_NOT_CFG)
   {
      if(value == RADIO_SENSOR_FAILURE)
      {
          drvMoistZones[zone - 1].lastValue = -1;
          
          /* Mark moisture sensor as failed. */
          moistFailureSet(zone);
      }
      else 
      {
          drvMoistZones[zone - 1].lastValue = (int8_t)value;
          
          /* Insure moisture sensor not marked as failed. */
          moistFailureClear(zone);
      }
   }
}

/******************************************************************************
 *
 *  drvMoistThresholdSet
 *
 *  DESCRIPTION:
 *      This driver API function configures the target moisture level for a
 *      specific zone.
 *
 *  PARAMETERS:
 *      zone (in)      - zone to configure (1..12)
 *      threshold (in) - new threshold value (0..60)
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      Calling this routine resets the threshold count for this zone, even if
 *      the threshold value is not changed.
 *
 *****************************************************************************/
void drvMoistThresholdSet(uint8_t zone, uint8_t threshold)
{
    
    if ((zone > 0) )//&& (zone < DRV_MOIST_NZONES))
    {
        
        if(zone <= DRV_MOIST_NZONES)
        {
            drvMoistZones[zone-1].count = 0;
            drvMoistZones[zone-1].threshold = threshold;
        }
    }
}


/******************************************************************************
 *
 *  drvMoistThresholdGet
 *
 *  DESCRIPTION:
 *      This driver API function returns the target moisture level for a
 *      specific zone configured by the drvMoistThresholdSet routine.
 *
 *  PARAMETERS:
 *      zone (in) - zone to retrieve (1..12)
 *
 *  RETURNS:
 *      configured threshold value (0..60)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
uint8_t drvMoistThresholdGet(uint8_t zone)
{
    return drvMoistZones[zone - 1].threshold;
}


/******************************************************************************
 *
 *  drvMoistThresholdMet
 *
 *  DESCRIPTION:
 *      This driver API function returns whether the target moisture level for
 *      a specific zone has been met.  The sampled moisture level must be at or
 *      above the configured threshold for at least the last DRV_MOIST_NSAMPLES
 *      for the threshold to be met.
 *
 *  PARAMETERS:
 *      zone (in) - zone to retrieve (1..12)
 *
 *  RETURNS:
 *      zone threshold status (TRUE = threshold has been met)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
bool_t drvMoistThresholdMet(uint8_t zone)
{
    return (drvMoistZones[zone - 1].count >= DRV_MOIST_NSAMPLES);
}


/******************************************************************************
 *
 *  drvMoistFreqSet
 *
 *  DESCRIPTION:
 *      This driver API function configures the sampling frequency for a
 *      specific zone.
 *
 *  PARAMETERS:
 *      zone (in) - zone to configure (1..12)
 *      freq (in) - new sampling frequency (off, 1/minute, 1/hour)
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *      The scheduled time for the next reading is changed to be sooner only if
 *      necessary to satify the new sampling frequency, else it is left
 *      unchanged.  Specifically, calling this routine to set the frequency to
 *      its current value will not affect the timing of the next sample.
 *
 *****************************************************************************/
void drvMoistFreqSet(uint8_t zone, drvMoistFreq_t freq)
{
    if (zone >= 1 && zone <= DRV_MOIST_NZONES)
    {
        uint32_t now = drvRtcGet();
        drvMoistZone_t *pZone = &drvMoistZones[zone - 1];

        pZone->freq = freq;
        switch (freq)
        {
            case DRV_MOIST_FREQ_OFF:
                break;
            case DRV_MOIST_FREQ_INACTIVE:
                if ((int32_t)(pZone->nextTime - now) > DRV_MOIST_INACTIVE_PERIOD)
                {
                    pZone->nextTime = now + DRV_MOIST_INACTIVE_PERIOD;
                }
                break;
            case DRV_MOIST_FREQ_ACTIVE:
                if ((int32_t)(pZone->nextTime - now) > DRV_MOIST_ACTIVE_PERIOD)
                {
                    pZone->nextTime = now + DRV_MOIST_ACTIVE_PERIOD;
                }
                break;
        }
    }
}


/******************************************************************************
 *
 *  drvMoistIsr
 *
 *  DESCRIPTION:
 *      This driver internal function is invoked by a 20ms timer ISR to perform
 *      periodic sampling of moisture sensors.  The drvMoistActive global
 *      variable is used to keep track of the driver "state".  This variable
 *      indicated which sensor has been power (by the previous ISR invocation
 *      20ms ago).  If set, then its value is sampled and the sensor is powered
 *      off.  The ISR then enables power to another sensor if any sensor is due
 *      (or past due) for reading.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is called from a timer ISR.
 *
 *****************************************************************************/
void drvMoistIsr(void)
{
    uint32_t now = drvRtcGet();
    drvMoistZone_t *pZone;

    if (drvMoistActive > 0)
    {
        pZone = &drvMoistZones[drvMoistActive - 1];

        /* read active sensor */
        /* if sensor is wireless then dont update value from wired sensor value */
        if(config.zone[drvMoistActive-1].sensorType == SNS_WIRED_MOIST) 
        {
            pZone->lastValue = drvMoistAdc2Percent(drvMoistRead(drvMoistActive));
        }
        /* for generic sensor compute the value according to sensor type */
        else if(config.zone[drvMoistActive-1].sensorType != SNS_WIRELESS_MOIST)
        {
            pZone->lastValue = ((drvGenericAdc2Percent(drvMoistRead(drvMoistActive)) *
                                config.zone[drvMoistActive-1].maxMoist))/100;
            
            /* if sensor is a flow or level sensor then save value into EEPROM
             * for nonvolatile storage. Each sensor reading value is only 1 byte.
             * use the RTC minute counter since midnight for the offset
             */                    
            if(config.zone[drvMoistActive-1].sensorType == SNS_FLOW)
            {
                 drvEepromWrite(&pZone->lastValue, FLOW_SNS_DATA + dtMin +dtHour*60, 1);    
            }
            else if(config.zone[drvMoistActive-1].sensorType == SNS_RAIN_GAUGE)
            {
                 drvEepromWrite(&pZone->lastValue, LEVEL_SNS_DATA + dtMin +dtHour*60, 1);    
            }
        }
        
        drvMoistPower(0);
        drvMoistActive = 0;
        /* update threshold status */
        if (pZone->lastValue < pZone->threshold)
        {
            pZone->count = 0;
        }
        else if (pZone->count < DRV_MOIST_NSAMPLES)
        {
            pZone->count++;
        }

        switch (pZone->freq)
        {
            case DRV_MOIST_FREQ_OFF:
                break;
            case DRV_MOIST_FREQ_INACTIVE:
                pZone->nextTime = now + DRV_MOIST_INACTIVE_PERIOD;
                break;
            case DRV_MOIST_FREQ_ACTIVE:
                pZone->nextTime = now + DRV_MOIST_ACTIVE_PERIOD;
                break;
        }
    }

    if (drvMoistActive == 0)
    {
        /* determine if another zone needs to be read; if so, activate its power */
        pZone = drvMoistZones;
        for (int zone = 1; zone <= DRV_MOIST_NZONES; zone++, pZone++)
        {
            if (pZone->freq != DRV_MOIST_FREQ_OFF && pZone->nextTime <= now)
            {
                drvMoistPower(zone);
                drvMoistActive = zone;
                break;
            }
        }
    }
}


/******************************************************************************
 *
 *  drvMoistAdc2Percent
 *
 *  DESCRIPTION:
 *      This driver internal function converts an ADC 12-bit value to a
 *      moisture percentage value.  Valid values are 0 to 60%.  A negative
 *      value indicates a sensor error (ADC value out of range).
 *
 *  PARAMETERS:
 *      adcValue (in) - 12-bit value from moisture sensor ADC channel
 *
 *  RETURNS:
 *      moisture percentage level (0..60), or -1 for error
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
int8_t drvMoistAdc2Percent(uint16_t adcValue)
{
    int32_t percent;

    percent = adcValue;
    percent -= DRV_MOIST_ADC_LO;
    percent *= DRV_MOIST_PERCENT_HI - DRV_MOIST_PERCENT_LO;
    percent /= DRV_MOIST_ADC_HI - DRV_MOIST_ADC_LO;
    percent += DRV_MOIST_PERCENT_LO;

    if (percent > DRV_MOIST_HI_LIMIT || percent < DRV_MOIST_LO_LIMIT)
    {
        percent = -1;                   /* mark as error */
    }
    else if (percent > DRV_MOIST_PERCENT_HI)
    {
        percent = DRV_MOIST_PERCENT_HI; /* bring within range */
    }
    else if (percent < DRV_MOIST_PERCENT_LO)
    {
        percent = DRV_MOIST_PERCENT_LO; /* bring within range */
    }

    return (int8_t)percent;
}


/******************************************************************************
 *
 *  drvGenericAdc2Percent
 *
 *  DESCRIPTION:
 *      This driver internal function converts an ADC 12-bit value to a
 *      generic percentage value.  Valid values are 0 to 100%.  A negative
 *      value indicates a sensor error (ADC value out of range).
 *
 *  PARAMETERS:
 *      adcValue (in) - 12-bit value from moisture sensor ADC channel
 *
 *  RETURNS:
 *      generic percentage level (0..6100), or -1 for error
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
int8_t drvGenericAdc2Percent(uint16_t adcValue)
{
    int32_t percent;

    percent = adcValue;
    percent -= DRV_GENERIC_ADC_LO;
    percent *= 100;
    percent /= DRV_GENERIC_ADC_HI - DRV_GENERIC_ADC_LO;
    

    if (percent > DRV_GENERIC_HI_LIMIT)
    {
        percent = DRV_GENERIC_HI_LIMIT; /* bring within range */
    }
    else if (percent < DRV_GENERIC_LO_LIMIT)
    {
        percent = DRV_GENERIC_LO_LIMIT; /* bring within range */
    }
    
    return (int8_t)percent;
}

/******************************************************************************
 *
 *  drvMoistPower
 *
 *  DESCRIPTION:
 *      This driver internal function applies power to the specified zone.
 *      Specifying an invalid zone (outside the range of 1..12) turns off
 *      moisture sensor power.  The hardware design is such that at most one
 *      sensor can be powered at a time.  Moisture sensor power should be turned
 *      off except when actively reading the moisture level.
 *
 *  PARAMETERS:
 *      zone (in) - Value (1..12) of moisture sensor to activate; any other
 *                  value (e.g. 0) turns off power.
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvMoistPower(uint8_t zone)
{
    EnterCritical();                    /* save and disable interrupts */

    drvLatch2 = (uint8_t)((drvLatch2 & ~0xF0) | ((zone & 0x0F) << 4));
    hwBusData_PutVal(drvLatch2);
    hwBusLatch2_SetVal();
    hwBusLatch2_ClrVal();

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvMoistRead
 *
 *  DESCRIPTION:
 *      This driver internal function reads the value from the specified zone.
 *      The sensor power for that zone must have been enabled previously,
 *      allowed time to "settle", and should be deactivated after this function
 *      completes.
 *
 *  PARAMETERS:
 *      zone (in) - Value (1..12) of moisture sensor to read.
 *
 *  RETURNS:
 *      12-bit ADC value (0..4091)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
uint16_t drvMoistRead(uint8_t zone)
{
    uint16_t value;
    uint8_t rtn = ERR_COMMON;

    EnterCritical();                    /* save and disable interrupts */

    if (zone >= 1 && zone <= DRV_MOIST_NZONES)
    {
        rtn = hwAdc_MeasureChan(TRUE,
                                (uint8_t)(zone - 1 + hwAdc_CHANNEL_SENSOR_ZONE1));
        if (rtn == ERR_OK)
        {
            rtn = hwAdc_GetChanValue16((uint8_t)(zone - 1 + hwAdc_CHANNEL_SENSOR_ZONE1),
                                       &value);
        }
    }

    ExitCritical();                     /* restore interrupts */

    if (rtn != ERR_OK)
    {
        value = 0;
    }

    return (uint16_t)(value >> 4);      /* right-justify 12-bit result */
}


/* END drvMoist */
