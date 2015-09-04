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
 * Module       : system.h
 * Description  : This file defines the system manager interfaces.
 *
 *****************************************************************************/

#ifndef __system_H
#define __system_H

/* MODULE system */

#include "global.h"



/******************************************************************************
 *
 *  IMPLEMENTATION VALUES
 *
 *****************************************************************************/

#define SYS_N_ZONES             48      /* number of zones in system (<=99) */
#define SYS_N_UNITS             4      /* number of zones in system (<=99) */
#define SYS_N_UNIT_ZONES        12      /* number of zones in one wois unit */
#define SYS_N_PROGRAMS          4       /* number of irrig. programs (<=4) */
#define SYS_TEST_RUNTIME_MAX    30      /* maximum per-zone test run-time */
#define SYS_MOISTURE_MAX        60      /* maximum moisture percentage */
#define SYS_EVENT_LIMIT         100     /* Number of Event Log Entries */


/*
**  SYSTEM STATES
*/
#define SYS_STATE_IDLE           0       /* IDLE */
#define SYS_STATE_AUTORUN        1       /* Running Auto-Started Program */
#define SYS_STATE_TEST           2       /* Running Test Program */
#define SYS_STATE_MANUAL         3       /* Running Manual-Started Program */
#define SYS_STATE_FORCE          4       /* Running FORCE-ON Program */


/*
**  SYSTEM ERROR FLAGS
**
**  Each error flag is a bit in the 16-bit sysErrorFlags field.
**  Error flags must be assigned contiguously, starting with bit 0.
*/
#define SYS_ERROR_DATETIME      0   /* date & time not set */
#define SYS_ERROR_WEATHERDATA   1   /* no weather data in 24 hrs */
#define SYS_ERROR_NOSENSORS     2   /* sensor mode but no sensors */
#define SYS_ERROR_NOPROGRAM     3   /* no runnable program scheduled */
#define SYS_ERROR_LIMIT         4


/*
**  HARDWARE FAULT FLAGS
**
**  Each fault flag is a bit in the 32-bit sysFaultFlags field.
**  Fault flags must be assigned contiguously, starting with bit 0.
*/
#define SYS_FAULT_MANUFDATA     0   /* invalid manufacturing data */
#define SYS_FAULT_24VAC         1   /* no 24VAC power - check fuse */
#define SYS_FAULT_MOIST         2   /* moisture sensor failure */
#define SYS_FAULT_RADIO         3   /* radio failure */
#define SYS_FAULT_EXPAN_MASTER  4   /* lost communication with master unit */
#define SYS_FAULT_EXPAN_1       5   /* lost communication with expansion 1 */
#define SYS_FAULT_EXPAN_2       6   /* lost communication with expansion 2 */
#define SYS_FAULT_EXPAN_3       7   /* lost communication with expansion 3 */
#define SYS_FAULT_SNSCON        8   /* lost communication with active sensor concentrator */
#define SYS_FAULT_LIMIT         9



/******************************************************************************
 *
 *  SYSTEM DATA STRUCTURES
 *
 *****************************************************************************/

/*
**  FIRMWARE VERSION STRUCT
*/
typedef struct
{
    uint8_t major;              /* major release number (major features) */
    uint8_t minor;              /* minor release number (minor features) */
    uint8_t patch;              /* patch release number (bug fixes) */
    uint8_t seq;                /* sequence number for alpha/beta/rc */
} sysVersion_t;

/*
**  FIRMWARE VERSION SEQUENCE TYPES
**
**  The firmware version sequence type is identified in the MSB 2 bits
**  of the 8-bit sequence number field.  The remaining 6 bits of the sequence
**  number allow specification of up to 64 different sequence numbers for
**  each sequence type (alpha, beta, rc).
*/
#define SYS_SEQ_MASK    0xC0    /* mask for sequence type in the MSB 2 bits */
#define SYS_SEQ_ALPHA   0x80    /* sequence type: alpha release (alpha) */
#define SYS_SEQ_BETA    0x40    /* sequence type: beta release (beta) */
#define SYS_SEQ_RC      0xC0    /* sequence type: release candidate (rc) */


/*
**  SYSTEM EVENT LOG ENTRY STRUCT
*/
typedef struct
{
    uint32_t time;          /* time in milliseconds since system init */
    uint16_t type;          /* event type (subsystem event code) */
    uint16_t data;          /* event data (additional event information) */
} sysEventLogEntry_t;


/*
**  SYSTEM EVENT LOG MAINTENANCE INFO STRUCT
*/
typedef struct
{
    uint8_t next;                           /* Next Event Log entry index */
    bool_t wrapped;                         /* Event Log wrapped if TRUE */
    uint8_t year;                           /* Save to EEPROM 2-dig year */
    uint8_t month;                          /* Save to EEPROM month */
    uint8_t day;                            /* Save to EEPROM day */
    uint8_t hour;                           /* Save to EEPROM hour */
    uint8_t min;                            /* Save to EEPROM minute */
    uint8_t sec;                            /* Save to EEPROM second */
} sysEventLogMaint_t;


/*
**  SYSTEM EVENT LOG STRUCT
*/
typedef struct
{
    sysEventLogMaint_t mn;                  /* System Event Log Maint Info */
    sysEventLogEntry_t ev[SYS_EVENT_LIMIT]; /* System Event Log Entry */
} sysEventLog_t;

/* Size of Data Storage used by the System Event Log */
#define SYS_EVENT_LOG_SIZE  sizeof(sysEventLog_t)

/*
**  SYSTEM EVENT LOG ENTRY SUBSYSTEM IDENTIFIERS
**
**  The subsystem is identified in high nibble of the 16-bit event type.
*/
#define SYS_EVENT_SYS       0x1000      /* System Manager Event */
#define SYS_EVENT_DT        0x2000      /* Date/Time Logic Event */
#define SYS_EVENT_CONFIG    0x3000      /* Configuration Manager Event */
#define SYS_EVENT_UI        0x4000      /* User Interface Event */
#define SYS_EVENT_RADIO     0x5000      /* Radio Logic Event */
#define SYS_EVENT_IRR       0x6000      /* Irrigation Logic Event */
#define SYS_EVENT_MOIST     0x7000      /* Moisture Sensor Logic Event */





/******************************************************************************
 *
 *  SYSTEM GLOBAL VARIABLES
 *
 *****************************************************************************/

extern const sysVersion_t sysFirmwareVer;   /* System Firmware Version  */
extern const char sysVersionDate[12];       /* System Firmware date string */
extern uint8_t sysState;            /* Current System State */
extern uint8_t sysTestRunTime;      /* per-zone test run-time */
extern uint8_t sysManualProgram;    /* selected manual program A=0..D=3 */
extern uint8_t sysManualOpMode;     /* manual program operation mode */
extern uint8_t sysManualPulseMode;  /* manual program pulse mode */
extern bool_t sysIsAuto;            /* System Auto flag, TRUE=on/automatic */
extern bool_t sysIsInhibited;       /* Irrigation is inhibited when TRUE */
extern bool_t sysIsPaused;          /* Irrigation is paused when TRUE */
extern uint32_t sysInhibitOnTime;   /* Last Inhibit On Time (in time-ticks) */
extern uint32_t sysInhibitOffTime;  /* Last Inhibit Off Time (in time-ticks) */
extern uint16_t sysErrorFlags;      /* System Error Flags */
extern uint32_t sysFaultFlags;      /* System Fault Flags */
extern bool_t sysResetRequest;      /* Request a system reset if TRUE */
extern uint8_t sysMaxZones;         /* Max number of zones supported */
extern sysEventLog_t sysEventLog;   /* System Event Log Data Store */
extern uint8_t expansionIrrState;           /* Expansion unit Irrigation State */
extern uint8_t expansionIrrCurZone;         /* Expansion Current Irrigation Zone (13..48) */
extern uint8_t expansionSysState;           /* Expansion current operating state */
extern uint32_t expansionSysFaultFlags;     /* Expansion System Fault Flags */
extern uint16_t expansionSysErrorFlags;     /* Expansion System Error Flags */
extern uint8_t radioStatusExpansion1;       /* Expansion radio connection status */
extern uint8_t radioStatusExpansion2;       /* Expansion radio connection status */
extern uint8_t radioStatusExpansion3;       /* Expansion radio connection status */
extern uint8_t expansionIrrStop;            /* value to keep track if stop command was sent to expansion */
extern bool_t sysInhibitStop;               /* Inhibit off command recieved, wait for SC to checkin before clearing */

/******************************************************************************
 *
 *  FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void sysInit(void);
void sysPoll(void);
void sysPollPowerCheck(void);
void sysRadioResponseBytes(uint8_t *pException, uint8_t *pStatus);
uint8_t sysRadioExtStatusGet(uint8_t *pData);
uint8_t sysRadioMoistValueGet(uint8_t zi);
void sysInhibitOn(void);
void sysInhibitOff(void);
void sysPause(void);
void sysResume(void);
void sysErrorSet(uint8_t error);
void sysErrorClear(uint8_t error);
void sysFaultSet(uint8_t fault);
void sysFaultClear(uint8_t fault);
bool_t sysErrorOn(uint8_t error);
bool_t sysFaultOn(uint8_t fault);
char *sysFormatFirmwareVer(char *buf);
void sysExecutionExtend(void);
void sysEvent(uint16_t eventType, uint16_t eventData);


/* END system */

#endif
