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
 * Module       : moisture.h
 * Description  : This file defines the moisture sensor logic interfaces.
 *
 *****************************************************************************/

#ifndef __moisture_H
#define __moisture_H

/* MODULE moisture */

#include "global.h"

/******************************************************************************
 *
 *  IMPLEMENTATION VALUES
 *
 *****************************************************************************/

#define MOIST_TEST_SECS     5   /* number of seconds between sensor tests */

/*
**  MOISTURE SENSOR SAMPLE FREQUENCY VALUES
*/
typedef enum
{
    MOIST_FREQ_OFF = 0,         /* frequency: no sampling */
    MOIST_FREQ_INACTIVE,        /* frequency: inactive sample rate */
    MOIST_FREQ_ACTIVE,          /* frequency: active sample rate */
    MOIST_FREQ_NOCHANGE = 255,  /* no change to the frequency currently set */
} moistFreq_t;



/******************************************************************************
 *
 *  MOISTURE SENSOR GLOBAL MEMORY
 *
 *****************************************************************************/

/*
**  FAILED SENSOR FLAGS
**  Note: This 64-bit failure flag bitmap has bit on (1) for failed sensor.
**  Sensor for zone 1 is bit 0, sensor for zone 2 is bit 1, and so on.
*/
extern uint64_t moistFailedSensors;



/******************************************************************************
 *
 *  FUNCTION PROTOTYPES
 *
 *****************************************************************************/
void moistInit(void);
void moistPoll(void);
void moistSensorThreshInit(void);
void moistThresholdSet(uint8_t zone, uint8_t threshold);
uint8_t moistThresholdGet(uint8_t zone);
bool_t moistThresholdMet(uint8_t zone);
void moistSampleAll(void);
int8_t moistValueGet(uint8_t zone);
bool_t moistSensorsConfigured(void);
bool_t moistSensorConfigured(uint8_t zone);
void moistFreqSet(uint8_t zone, moistFreq_t freq);
void moistFreqSetAll(moistFreq_t freq);
void moistSample(uint8_t zone);
uint8_t moistFailureZoneGet(void);
void moistFailureSet(uint8_t zone);
void moistFailureClear(uint8_t zone);

/* END moisture */

#endif
