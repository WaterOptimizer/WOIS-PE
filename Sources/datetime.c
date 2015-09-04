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
 * Module       : datetime.c
 * Description  : This file implements the date/time manager
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
#include "drvRtc.h"



/* Date/Time Events */
#define DT_EVENT_AUTO_START_Q       SYS_EVENT_DT + 1   /* Auto-start queued */
#define DT_EVENT_AUTO_START_QF      SYS_EVENT_DT + 2   /* Auto-start Q-full */

#define DT_MIDNIGHT 0       /*dtCurTime setting for midnight */


/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/

uint16_t dtYear;            /* year: 4-digit */
uint8_t  dtMon;             /* month: 0..11 */
uint8_t  dtMday;            /* day of month: 1..31 */
uint8_t  dtWday;            /* day of week: 0=Sun, ..., 6=Sat */
uint8_t  dtHour;            /* hour: 0..23 */
uint8_t  dtMin;             /* minute: 0..59 */
uint8_t  dtSec;             /* second: 0..59 */
uint32_t dtTickCount;       /* Tick Count (seconds since system init) */
uint32_t dtTickCountMs;     /* MS Tick Count (milliseconds since init) */
uint16_t dtPrevTime;        /* previous time check (minutes since 00:00) */
uint16_t dtCurTime;         /* current time check (minutes since 00:00) */



/******************************************************************************
 *
 *  DATETIME CONSTANTS
 *
 *****************************************************************************/

/* Day Names */
static const char * const dtDayNames[] =
{
    "Sun",
    "Mon",
    "Tue",
    "Wed",
    "Thu",
    "Fri",
    "Sat",
};


/* Month Names */
static const char * const dtMonthNames[] =
{
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec",
};



/******************************************************************************
 *
 * dtInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization routine
 *      to initialize the Date/Time subsystem.
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
void dtInit(void)
{
    dtTickCount = 0;

    /* Set an initial default value for date & time. */
    dtYear = DT_YEAR_MIN;   /* minimum settable year (2008) */
    dtMon  = 1 - 1;         /* January, as 0-based index */
    dtMday = 1;             /* first day of month */
    dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);    /* computed day of week */
    dtHour = 0;             /* midnight: hour */
    dtMin  = 0;             /* midnight: minute */
    dtSec  = 0;             /* midnight: second */

    /* Initialize irrigation auto-start time reference. */
    dtPrevTime = 0;

    /* Turn on system error for no date & time set. */
    sysErrorSet(SYS_ERROR_DATETIME);
}


/******************************************************************************
 *
 * dtPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to
 *      maintain the real time clock and second tick count.  This routine
 *      also detects start times for scheduled irrigation programs.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine should be called frequently to keep the system clock
 *      up-to-date and to insure that the intervals between tick count
 *      one-second increments are reasonably precise.  To maintain precision
 *      of +/- 5%, this routine needs to be called at least once every 100
 *      ms.
 *
 *****************************************************************************/
void dtPoll(void)
{
    uint8_t pgm;
    char debugBuf[2];

    /* Maintain real time clock. */
    dtPollRtc();

    /* Manage automatic irrigation start. */
    if (dtNewMinute())
    {
       
        /* Check configuration to see if an irrigation program should start. */
        if (sysIsAuto)
        {
            for (pgm = 0; pgm < SYS_N_PROGRAMS; pgm++)
            {
               /* confirm communication with master prior to schedule start
                * time if in pulse mode. if no communication then revert to runtime based system
                */
                if((config.sys.pulseMode ==CONFIG_PULSEMODE_ON) & (config.sys.unitType != UNIT_TYPE_MASTER))
                {
                    if((ntohs(config.sched[dtIrrWday()][pgm].startTime)-TIME_BEFORE_SCHED_TIME) == dtCurTime)
                    {
                        /* send No Op command to Master to test communication */
                        expansionBusSendCmd(RADIO_CMD_NO_OP, config.sys.masterMac);
    
                    } 

                    return;

                }
                
                if ((ntohs(config.sched[dtIrrWday()][pgm].startTime) == dtCurTime) & 
                    (irrStop == FALSE)& (radioCmdIrrStart==FALSE))             
                {                      
                    /* Make sure watchdog doesn't timeout on long debug message writes. */
                    sysExecutionExtend();
                    if ((irrAutoPgmPending == IRR_PGM_NONE) | (irrExpRunningProg == IRR_PGM_NONE))
                    {
                        /* Schedule program to auto-start. */
                        irrAutoPgmPending = pgm;
                        /* Trace auto start queued event. */
                        sysEvent(DT_EVENT_AUTO_START_Q, pgm);
                        dtDebug("PGM '");
                        sprintf(debugBuf, "%c", 'A' + pgm);
                        debugWrite(debugBuf);
                        debugWrite("' queued for auto-start.\n");
                        
                        if(config.sys.unitType == UNIT_TYPE_MASTER)
                        {
                          expansionBusSendCmd(RADIO_CMD_IRR_STOP_OFF,RADIO_EXP_SEND_ALL);
                        }
                    }
                    else
                    {
                        /* Already one program waiting in queue. */
                        /* Trace auto start - queue full event. */
                        sysEvent(DT_EVENT_AUTO_START_QF, pgm);
                        dtDebug("PGM '");
                        sprintf(debugBuf, "%c", 'A' + pgm);
                        debugWrite(debugBuf);
                        debugWrite("' auto-start Failed.  Queue is full.\n");
                    }
                }
                
                /*if a stop command was received before scheduled time cancel
                 * scheduled program and once passed scheduled time clear the stop flag 
                 */
                if (ntohs(config.sched[dtIrrWday()][pgm].startTime) < dtCurTime)
                {
                    irrStop = FALSE;
                    radioCmdIrrStart=FALSE;
                }
                  
            }
        }
    }
}


/******************************************************************************
 *
 * dtPollRtc
 *
 * PURPOSE
 *      This routine is called to maintain the real time clock and second tick
 *      count.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine may be called from low-power (battery) mode; it must
 *      not perform any functions that are unavailable in low-power mode.
 *
 *****************************************************************************/
void dtPollRtc(void)
{
#ifndef WIN32
    
    static uint32_t lastTick = 0;

    while (lastTick < drvRtcGet())
    {
        dtIncrement();
        lastTick++;
    }
#endif
}


/******************************************************************************
 *
 * dtNewMinute
 *
 * PURPOSE
 *      This routine is called to advance the time check logic and indicate
 *      if the clock has advanced to a the next minute.  This routine is used
 *      by the date/time polling logic to facilitate time matching for time
 *      scheduled operations.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if the minute value changed since the last
 *      time this routine was called.
 *
 * NOTES
 *      This routine must be called regularly to detect advance to a new
 *      minute.  When advance to midnight (00:00) is detected, this routine
 *      manages roll-over of the irrigation total watering time counters.
 *      This routine may be called from low-power (battery) mode; it must
 *      not perform any functions that are unavailable in low-power mode.
 *
 *****************************************************************************/
bool_t dtNewMinute(void)
{
    bool_t isNewMinute = FALSE;
    uint8_t zi;

    dtCurTime = (dtHour * 60) + dtMin;
    if (dtPrevTime != dtCurTime)
    {
        isNewMinute = TRUE;
        dtPrevTime = dtCurTime;
        /* Test if currently watering. */
        if (irrState == IRR_STATE_WATERING)
        {
            /* Increment total watering minutes. */
            irrMinsToday++;
        }
        /* Check for midnight (00:00). */
        if (dtCurTime == 0)
        {
            /* Copy today's total watering time to yesterday's data store. */
            irrMinsYesterday = irrMinsToday;
            /* Zero out today's total watering time. */
            irrMinsToday = 0;
            
            /* zero out todays per zone watering time */
            for(zi=0; zi<SYS_N_ZONES;zi++)
            {
                irrDailyRuntimePerZone[zi]=0;
            }
        }
    }

    return isNewMinute;
}


/******************************************************************************
 *
 * dtDayOfWeek
 *
 * PURPOSE
 *      This routine calculates the day of week, based on calendar date.
 *
 * PARAMETERS
 *      year    IN  year (e.g., 2008)
 *      month   IN  month (0=Jan, 1=Feb, ..., 11=Dec)
 *      day     IN  day of month (1-31)
 *
 * RETURN VALUE
 *      This routine returns the day of week as a value ranging from 0 to 6,
 *      where: 0=Sun, 1=Mon, ..., 6=Sat.
 *
 * NOTES
 *      1) Works only for Gregorian dates (1582 onward).
 *      2) Does not account for skipping 9/3/1752 through 9/13/1752, so
 *         in effect, it is valid only for dates from 9/14/1752 onward.
 *      3) Implements Zeller's congruence algorithm.
 *      4) Reference: http://en.wikipedia.org/wiki/Zeller's_congruence
 *
 *****************************************************************************/
uint8_t dtDayOfWeek(uint16_t year, uint8_t month, uint8_t mday)
{
    static uint8_t monthCode[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

    if (month < 2)
    {
        year--;
    }
    return (uint8_t)((year + year/4 - year/100 + year/400 + monthCode[month] + mday) % 7);
}


/******************************************************************************
 *
 * dtDaysPerMonth
 *
 * PURPOSE
 *      This routine returns the number of days in a month.
 *
 * PARAMETERS
 *      month   IN  month (0=Jan, 1=Feb, ..., 11=Dec)
 *
 * RETURN VALUE
 *      This routine returns the (maximum) number of days in the month.
 *
 * NOTES
 *      This routine does not handle leap year differences.  February
 *      is always 29 days.
 *
 *****************************************************************************/
uint8_t dtDaysPerMonth(uint8_t month)
{
    static const uint8_t daysPerMonth[] =
    {
        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    return daysPerMonth[month];
}


/******************************************************************************
 *
 * dtIncrement
 *
 * PURPOSE
 *      This routine increments the stored real time clock by one second.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void dtIncrement(void)
{
    /* Increment system tick count. */
    dtTickCount++;
    dtTickCountMs = drvMSGet();

    if (++dtSec >= 60)
    {
        dtSec = 0;
        if (++dtMin >= 60)
        {
            dtMin = 0;
            if (++dtHour >= 24)
            {
                dtHour = 0;
                dtMday++;
                if (dtMday > dtDaysPerMonth(dtMon) ||
                    (dtMday > 28 && dtMon == 1 && (dtYear % 4) != 0))
                {
                    dtMday = 1;
                    if (++dtMon >= 12)
                    {
                        dtMon = 0;
                        dtYear++;
                    }
                }
                if (++dtWday >= 7)
                {
                    dtWday = 0;
                }
            }
        }
    }
}


/******************************************************************************
 *
 * dtFormatHourMin
 *
 * PURPOSE
 *      This routine returns a formatted hour/minute string, based on the
 *      configured time format (12-hour, 24-hour US, 24-hour Canada).
  *
 *      Format examples:
 *          12-hour         "1:09pm"  
 *          24-hour (US)    "13:09"  
 *          24-hour (CDN)   "13h09"  
*
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *      hour    IN  hours (0-23)
 *      min     IN  minutes (0-59)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted hour/minute
 *      string.
 *
 *****************************************************************************/
char *dtFormatHourMin(char *buf, uint8_t hour, uint8_t min)
{
    switch (config.sys.timeFmt)
    {
        case CONFIG_TIMEFMT_12HR:
            {
                /* 12-hour format (e.g. " 1:00am" and "11:59pm") */
                const char *pSuffix = (hour < 12 || hour == 24) ? "am" : "pm";

                if (hour > 12)
                {
                    hour -= 12;
                }
                if (hour == 0)
                {
                    hour = 12;
                }
                sprintf(buf, "%2d:%02d%s", hour, min, pSuffix);
            }
            break;

        case CONFIG_TIMEFMT_24HR_US:
            /* 24-hour US format (e.g. "01:00" and "23:59") */
            sprintf(buf, "%02d:%02d", hour, min);
            break;

        case CONFIG_TIMEFMT_24HR_CA:
            /* 24-hour CANADA format (e.g. "01h00" and "23h59") */
            sprintf(buf, "%02dh%02d", hour, min);
            break;
    }

    return buf;
}


/******************************************************************************
 *
 * dtFormatMinutes
 *
 * PURPOSE
 *      This routine returns a formatted hour/minute string, based on the
 *      configured time format (12-hour, 24-hour US, 24-hour Canada).
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *      minutes IN  minutes since midnight (valid range: 0-1439)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted hour/minute
 *      string.
 *
 *****************************************************************************/
char *dtFormatMinutes(char *buf, uint16_t minutes)
{
    uint8_t hour = (uint8_t)(minutes / 60);
    uint8_t min  = (uint8_t)(minutes % 60);

    return dtFormatHourMin(buf, hour, min);
}


/******************************************************************************
 *
 * dtFormatRunTime
 *
 * PURPOSE
 *      This routine returns a formatted irrigation program runtime string.
 *      The string is formated as "HH:MM".  Any leading zero is suppressed
 *      for the hour.
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *      minutes IN  minutes since midnight (valid range: 0-1439).
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted hour/minute
 *      string.
 *
 *****************************************************************************/
char *dtFormatRunTime(char *buf, uint16_t minutes)
{
    uint8_t hour = (uint8_t)(minutes / 60);
    uint8_t min  = (uint8_t)(minutes % 60);

    sprintf(buf, "%2d:%02d", hour, min);

    return buf;
}


/******************************************************************************
 *
 * dtFormatRunTimeSecs
 *
 * PURPOSE
 *      This routine returns a formatted irrigation program runtime string.
 *      The string is formated as "HH:MM:SS".  Any leading zero is suppressed
 *      for the hour.
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *      seconds IN  runtime in seconds
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted hour/minute/sec
 *      string.
 *
 *****************************************************************************/
char *dtFormatRunTimeSecs(char *buf, uint32_t seconds)
{
    uint8_t hours = (uint8_t)(seconds / 3600);
    uint8_t mins  = (uint8_t)((seconds / 60) % 60);
    uint8_t secs = (uint8_t)((seconds % 60) % 60);

    sprintf(buf, "%d:%02d:%02d", hours, mins, secs);

    return buf;
}


/******************************************************************************
 *
 * dtFormatCurrentTimeDate
 *
 * PURPOSE
 *      This routine returns a formatted time/date string for the current
 *      real time clock date and time, using the configured time format.
 *
 *      Format examples:
 *          12-hour         "1:09pm Tue Jan 01, 2008"  
 *          24-hour (US)    "13:09 Tue Jan 01, 2008"  
 *          24-hour (CDN)   "13h09 Tue Jan 01, 2008"  
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted time/date
 *      string.
 *
 *****************************************************************************/
char *dtFormatCurrentTimeDate(char *buf)
{
    char tmpbuf[41];
    const char *pTimeStr = dtFormatHourMin(tmpbuf, dtHour, dtMin);

    sprintf(buf, "%s %3.3s %3.3s %02d, %04d",
            pTimeStr,
            dtDayNames[dtWday],
            dtMonthNames[dtMon],
            dtMday,
            dtYear);

    return buf;
}


/******************************************************************************
 *
 * dtFormatCurrentDateTime
 *
 * PURPOSE
 *      This routine returns a formatted date/time string for the current
 *      real time clock date and time, using the configured time format.
 *
 *      Format examples:
 *          12-hour         "2008-Jan-01 Tue 1:09pm"
 *          24-hour (US)    "2008-Jan-01 Tue 13:09"
 *          24-hour (CDN)   "2008-Jan-01 Tue 13h09"
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted date/time
 *      string.
 *
 *****************************************************************************/
char *dtFormatCurrentDateTime(char *buf)
{
    char tmpbuf[41];
    const char *pTimeStr = dtFormatHourMin(tmpbuf, dtHour, dtMin);

    sprintf(buf, "%04d-%3.3s-%02d %3.3s %s",
            dtYear,
            dtMonthNames[dtMon],
            dtMday,
            dtDayNames[dtWday],
            pTimeStr);

    return buf;
}


/******************************************************************************
 *
 * dtFormatDebugDateTime
 *
 * PURPOSE
 *      This routine returns a formatted date/time string for the current
 *      real time clock date and time, including seconds, in a concise,
 *      machine-readable format.
 *
 *      Format example:
 *          "2008-01-01 13:09:00"
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted date/time
 *      string.
 *
 *****************************************************************************/
char *dtFormatDebugDateTime(char *buf)
{
    sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            dtYear,
            dtMon + 1,
            dtMday,
            dtHour,
            dtMin,
            dtSec);

    return buf;
}


/******************************************************************************
 *
 * dtFormatUpTime
 *
 * PURPOSE
 *      This routine returns a formatted system up-time string.
 *
 * PARAMETERS
 *      buf     OUT buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted up-time
 *      string.
 *
 *****************************************************************************/
char *dtFormatUpTime(char *buf)
{
    uint32_t days = dtTickCount / DT_SECS_24_HOURS;
    uint8_t hours = (uint8_t)((dtTickCount % DT_SECS_24_HOURS) / 3600);
    uint8_t mins  = (uint8_t)(((dtTickCount % DT_SECS_24_HOURS) / 60) % 60);
    uint8_t secs = (uint8_t)(((dtTickCount % DT_SECS_24_HOURS) % 60) % 60);

    if (days == 0)
    {
        sprintf(buf, "%02d:%02d:%02d", hours, mins, secs);
    }
    else
    {
        sprintf(buf,
                "%d day%s %02d:%02d:%02d",
                days,
                (days == 1) ? "" : "s",
                hours,
                mins,
                secs);
    }

    return buf;
}


/******************************************************************************
 *
 * dtIrrWday
 *
 * PURPOSE
 *      This routine returns the current irrigation day of week.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns the day of week (0=Mon, 1=Tue, ..., 6=Sun).
 *
 * NOTES
 *      The irrigation scheduling logic uses a different day numbering
 *      convention than the real time clock.
 *
 *****************************************************************************/
uint8_t dtIrrWday(void)
{
    uint8_t irrWday;

    /* convert dt day to irr day */
    if (dtWday == 0)
    {
        irrWday = 6;
    }
    else
    {
        irrWday = dtWday - 1;
    }

    return irrWday;
}


/******************************************************************************
 *
 * dtElapsedSeconds
 *
 * PURPOSE
 *      This routine returns the number of seconds elapsed since the caller's
 *      specified tick count.
 *
 * PARAMETERS
 *      oldTickCount    IN  tick count to compute elapsed seconds for
 *
 * RETURN VALUE
 *      This routine returns the number of seconds elapsed.
 *
 *****************************************************************************/
uint32_t dtElapsedSeconds(uint32_t oldTickCount)
{
    /* Return elapsed seconds since oldTickCount. */
    return (dtTickCount - oldTickCount);
}


/******************************************************************************
 *
 * dtDebug
 *
 * PURPOSE
 *      This routine writes a time-stamped debug message.  The debug message
 *      string provided by the caller is prefixed with the current date & time
 *      using the following format:  "YYYY-MM-DD HH:MM:SS  ".
 *
 * PARAMETERS
 *      pData   IN  pointer to debug message
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The caller is responsible for providing any linefeed termination and
 *      messages longer than 100 characters will be truncated.
 *
 *****************************************************************************/
void dtDebug(const char *pData)
{
    char debugBuf[144];
    char tmpBuf[41];

    /* Print a time-stamped debug message. */
    sprintf(debugBuf, "%s  %.100s",
            dtFormatDebugDateTime(tmpBuf),
            pData);
    debugWrite(debugBuf);
}


/******************************************************************************
 *
 * dtSetClock
 *
 * PURPOSE
 *      This routine sets date and time for the system clock.
 *
 * PARAMETERS
 *      year    IN  year (4-digits, e.g., 2008)
 *      mon     IN  month (1-12)
 *      day     IN  day of month (1-31)
 *      hour    IN  hour (0-23)
 *      min     IN  minute (0-59)
 *      sec     IN  second (0-59)
 *
 * RETURN VALUE
 *      This routine returns TRUE if the date and time parameters were
 *      accepted and the clock updated.  This routine returns FALSE if any
 *      of the date/time parameters are out of bounds.
 *
 *****************************************************************************/
bool_t dtSetClock(uint16_t year,
                  uint8_t mon,
                  uint8_t day,
                  uint8_t hour,
                  uint8_t min,
                  uint8_t sec)
{
   bool_t valueCheckSuccess = TRUE;

    /* Check all values are in bounds. */
   if ((year < DT_YEAR_MIN) || (year > DT_YEAR_MAX))
   {
       valueCheckSuccess = FALSE;
   }
   else if ((mon < 1) || (mon > 12))
   {
       valueCheckSuccess = FALSE;
   }
   else if ((day < 1) ||
            (day > dtDaysPerMonth(mon - 1)) ||
            ((day > 28) && (mon == 2) && ((year % 4) != 0)))
   {
       valueCheckSuccess = FALSE;
   }
   else if (hour > 23)
   {
       valueCheckSuccess = FALSE;
   }
   else if (min > 59)
   {
       valueCheckSuccess = FALSE;
   }
   else if (sec > 59)
   {
       valueCheckSuccess = FALSE;
   }

   if (valueCheckSuccess)
   {
       /* Update date and time settings. */
       dtYear = year;
       dtMon = mon - 1;
       dtMday = day;
       dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
       dtHour = hour;
       dtMin = min;
       dtSec = sec;
       /* Clear date/time error flag, if set. */
       if (sysErrorOn(SYS_ERROR_DATETIME))
       {
            sysErrorClear(SYS_ERROR_DATETIME);
       }
   }

   return valueCheckSuccess;
}
