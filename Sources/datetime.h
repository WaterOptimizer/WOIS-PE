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
 * Module       : datetime.h
 * Description  : This file defines the date/time manager interfaces.
 *
 *****************************************************************************/

#ifndef __datetime_H
#define __datetime_H

/* MODULE datetime */

#include "global.h"


/******************************************************************************
 *
 *  IMPLEMENTATION CONSTANTS
 *
 *****************************************************************************/

#define DT_SECS_24_HOURS    86400       /* number of seconds in 24 hours */
#define DT_YEAR_MIN         2008        /* minimum settable year */
#define DT_YEAR_MAX         2099        /* maximum settable year */

#define DEAD_TIME           1           /*amount of dead time for safety margin between
                                         *switching from 1 unit to another */
#define TIME_BEFORE_SCHED_TIME  2       /* time before scheduled runtime to confirm comm with master */
/******************************************************************************
 *
 *  DATETIME GLOBAL VARIABLES
 *
 *****************************************************************************/

/* Date & Time */
extern uint16_t dtYear;     /* Year (4-digit) */
extern uint8_t  dtMon;      /* Month (0..11) */
extern uint8_t  dtMday;     /* Day of Month (1..31) */
extern uint8_t  dtWday;     /* Day of Week (0=Sun, ..., 6=Sat) */
extern uint8_t  dtHour;     /* Hour (0..23) */
extern uint8_t  dtMin;      /* Minute (0..59) */
extern uint8_t  dtSec;      /* Second (0..59) */

extern uint16_t dtPrevTime;     /* previous time check (minutes since 00:00) */
extern uint16_t dtCurTime;      /* current time check (minutes since 00:00) */

/* Tick Count */
extern uint32_t dtTickCount;    /* # seconds since system was initialized */
extern uint32_t dtTickCountMs;  /* MS Tick Count (milliseconds since init) */



/******************************************************************************
 *
 *  DATETIME FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void dtInit(void);
void dtPoll(void);
void dtPollRtc(void);
uint8_t dtDayOfWeek(uint16_t year, uint8_t month, uint8_t mday);
uint8_t dtDaysPerMonth(uint8_t month);
void dtIncrement(void);
char *dtFormatHourMin(char *buf, uint8_t hour, uint8_t min);
char *dtFormatMinutes(char *buf, uint16_t minutes);
char *dtFormatRunTime(char *buf, uint16_t minutes);
char *dtFormatRunTimeSecs(char *buf, uint32_t seconds);
char *dtFormatCurrentTimeDate(char *buf);
char *dtFormatCurrentDateTime(char *buf);
char *dtFormatDebugDateTime(char *buf);
char *dtFormatUpTime(char *buf);
uint32_t dtElapsedSeconds(uint32_t oldTickCount);
uint8_t dtIrrWday(void);
bool_t dtNewMinute(void);
void dtDebug(const char *pData);
bool_t dtSetClock(uint16_t year, uint8_t mon, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);


/* END datetime */

#endif
