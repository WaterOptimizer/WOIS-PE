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
 * Module       : drvMoist.h
 * Description  : This file declares the interface to the driver for the
 *                moisture sensors, including support for switching sensor
 *                power and performing ADC readings.
 *
 *****************************************************************************/

#ifndef __drvMoist_H
#define __drvMoist_H

/* MODULE drvMoist */


/*
 * Application Interface
 */

typedef enum
{
    DRV_MOIST_FREQ_OFF = 0,
    DRV_MOIST_FREQ_INACTIVE,
    DRV_MOIST_FREQ_ACTIVE,
} drvMoistFreq_t;

#define drvMoistSampleAll() drvMoistSample(0)

void    drvMoistSample(uint8_t zone);
int8_t  drvMoistValueGet(uint8_t zone);
void    drvMoistThresholdSet(uint8_t zone, uint8_t threshold);
uint8_t drvMoistThresholdGet(uint8_t zone);
bool_t  drvMoistThresholdMet(uint8_t zone);
void    drvMoistFreqSet(uint8_t zone, drvMoistFreq_t freq);
void    drvMoistValueSet(uint8_t zone, uint8_t value);

/*
 * Internal Driver Interfaces
 */
void     drvMoistRestart(void);
void     drvMoistShutdown(void);
void     drvMoistIsr(void);
void     drvMoistPower(uint8_t zone);
int8_t   drvMoistAdc2Percent(uint16_t adcValue);
int8_t drvGenericAdc2Percent(uint16_t adcValue);
uint16_t drvMoistRead(uint8_t zone);


/* END drvMoist */

#endif
