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
 * Module       : irrigation.h
 * Description  : This file defines the irrigation logic interfaces.
 *
 *****************************************************************************/
#ifndef __irrigation_H
#define __irrigation_H

/* MODULE irrigation */

#include "global.h"
#include "system.h"


/******************************************************************************
 *
 *  IRRIGATION STATE AND IMPLEMENTATION VALUES
 *
 *****************************************************************************/

#define IRR_ET_MAX          1000    /* ET Max Value */
#define IRR_SENSING_SECS    1       /* Seconds to wait for sensor readings */

/* Irrigation State */
#define IRR_STATE_IDLE      0       /* Idle */
#define IRR_STATE_SENSING   1       /* Wait for sensor readings before start */
#define IRR_STATE_WATERING  2       /* Zone watering in progress */
#define IRR_STATE_SOAKING   3       /* Watering paused; all zones soaking */

/* Irrigation Program */
#define IRR_PGM_A           0x00    /* Program A */
#define IRR_PGM_B           0x01    /* Program B */
#define IRR_PGM_C           0x02    /* Program C */
#define IRR_PGM_D           0x03    /* Program D */
#define IRR_PGM_NONE        0xFF    /* No Program */

/* Irrigation Solenoid Ident */
#define IRR_SOL_MASTER      0       /* Master solenoid */
#define IRR_SOL_ZONE_1      1       /* Zone 1 solenoid */

/* Irrigation Solenoid State */
#define IRR_SOL_OFF         0       /* Solenoid Off */
#define IRR_SOL_ON          1       /* Solenoid On  */

/* Irrigation Zone Event Flags */
#define IRR_ZF_STOPPED      0x01    /* Stop Command Received */
#define IRR_ZF_SKIPPED      0x02    /* Skip Command Received */
#define IRR_ZF_THRESH_MET   0x04    /* Moisture Sensor Threshold Met */
#define IRR_ZF_THRESH_ADJ   0x08    /* Sensor Group Runtime Adjusted */
#define IRR_ZF_MIN_MET      0x10    /* Moisture Minimum Already Met */

/* Moisture Balance Limits */
#define IRR_MB_MIN          (-10 * 100) /* minimum MB is -10.00 inches */



/******************************************************************************
 *
 *  IRRIGATION DATA STRUCTURES
 *
 *****************************************************************************/

/*
**  Zone Irrigation State
**
**  This structure is used to keep the per-zone "scorecard" for irrigation
**  program start parameters and progress info.
*/
typedef struct
{
    uint16_t elapsedTime;           /* elapsed watering time */
    uint16_t actualTimeLimit;       /* actual time limit */
    uint16_t pulseTimeLimit;        /* pulse time limit */
    uint16_t elapsedPulseTime;      /* elapsed pulse time */
    uint16_t soakTimeLimit;         /* soak time limit */
    uint16_t elapsedSoakTime;       /* elapsed soak time */
    uint16_t appRate;               /* application rate (1/100 in/hr) */
    uint8_t appEff;                 /* application efficiency (1-100) */
    uint8_t maxMoist;               /* maximum moisture threshold (1-99) */
    int8_t group;                   /* moisture sense group leader */
    uint8_t flags;                  /* zone event flags */
} irrZoneState_t;



/******************************************************************************
 *
 *  IRRIGATION GLOBAL VARIABLES
 *
 *****************************************************************************/

extern uint8_t irrState;            /* Irrigation State */
extern uint8_t irrProgram;          /* Current Program */
extern uint8_t irrNumZones;         /* Snapshot of Configured Number of Zones */
extern uint8_t irrOpMode;           /* Snapshot of Operating Mode */
extern uint8_t irrPulseMode;        /* Snapshot of Pulse Mode */
extern uint8_t irrCurZone;          /* Current Zone (1..48) */
extern uint32_t irrPgmStartTime;    /* Tick Count on last program start */
extern uint16_t irrMinsToday;       /* Total Watering Minutes Today */
extern uint16_t irrMinsYesterday;   /* Total Watering Minutes Yesterday */
extern uint8_t irrAutoPgmPending;   /* Auto Program Start Queue (1 entry) */
extern int16_t irrMoistureBalance[SYS_N_ZONES]; /* Moisture Balance per-zone */
extern irrZoneState_t irrZone[SYS_N_ZONES]; /* Irrigation Scorecard per-zone */
extern uint8_t irrExpRunningProg;   /* current program running on expansion units. */
extern bool_t irrStop;              /* Stop current irrigation program when TRUE */
extern uint8_t irrCurUnitRunning;   /* keeps track if another unit is irrigating. 0 means no other unit is running */
extern uint16_t irrDailyRuntimePerZone[SYS_N_ZONES]; /*keep runtime per zone per day. reset at midnight */
extern uint8_t flowFlag;
extern uint8_t GPM;
extern uint8_t slaveGPM;
extern uint8_t flowIndex;
extern uint16_t flowDelay;
extern uint16_t slaveFlowDelay;
extern bool_t timeFlag;
extern uint8_t findFlow;
extern uint8_t slaveFindFlow;
extern bool_t TestSkipFlag;
extern uint8_t irrNextZone;
/******************************************************************************
 *
 *  IRRIGATION FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void irrInit(void);
void irrPoll(void);
void irrCmdTest(void);
void irrCmdManualStart(void);
void irrCmdForceOn(void);
void irrCmdSkip(void);
void irrCmdStop(void);
int32_t irrRemainingZoneSecs(uint8_t zone);
int32_t irrRemainingProgramSecs(void);
uint32_t irrProgramRuntime(uint8_t program, uint8_t opMode);
void irrWeatherUpdate(uint16_t etData, uint16_t rainfall);
uint32_t irrRemainingSoakSecs(void);
uint16_t irrRemainingZoneSoakSecs(uint8_t zone);
uint16_t irrRemainingZonePulseSecs(uint8_t zone);
uint16_t irrRemainingZonePulses(uint8_t zone);
uint16_t irrWeatherRuntimeCalc(uint8_t zi);
bool_t irrIsCurrentGroupLeader(uint8_t zone);
bool_t irrIsConfigGroupLeader(uint8_t zone);
void irrMoistConfigUpdate(void);
int8_t irrZoneMoisture(uint8_t zone);
void irrOpModeConfigChanged(void);
bool_t irrHaveMbDeficit(void);
int16_t irrMbGet(uint8_t zone);
void irrMbSet(uint8_t zone, int16_t mbValue);
void irrMbFix(void);
bool_t irrCmdExpPulseOn(uint8_t program);
//static bool_t flowMinMax(uint8_t zi); 
/* END irrigation */

#endif
