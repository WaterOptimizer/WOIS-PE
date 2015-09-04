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
 * Module       : ui.c
 * Description  : This file implements the user interface logic.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include <stdlib.h>

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "radio.h"
#include "datetime.h"
#include "irrigation.h"
#include "moisture.h"
#include "ui.h"
#include "drvKeypad.h"
#include "drvLcd.h"
#include "drvLed.h"
#include "drvRtc.h"
#include "drvSys.h"



/******************************************************************************
 *
 *  UI MACROS
 *
 *****************************************************************************/

/*
**  LCD_RC - MACRO TO CALCULATE LCD BUFFER OFFSET
*/
#define LCD_RC(row,col)       DRV_LCD_CURSOR_RC((row),(col))

/* Turn off LCD cursor display. */
#define LCD_CURSOR_OFF        DRV_LCD_CURSOR_OFF


/******************************************************************************
 *
 *  UI IMPLEMENTATION VALUES
 *
 *****************************************************************************/

//#define UI_NEW_APP_RATE         1       /* use single-field App Rate UI */
#define UI_MBVAL_FAST_DELTA     50      /* step value for MB fast inc/dec */
#define UI_STATUS_N_REPS        6       /* # secs to cycle line-3 status */

/* Screen Timeout - Idle time before returning to Status Screen. */
#define UI_SCREEN_TIMEOUT       (5 * 60)    /* timeout after 5 minutes */

/*
**  FRONT PANEL FUNCTION SWITCH & LED DEFINITIONS
*/
#define UI_FUNC_HOME            0       /* function switch: HOME */
#define UI_FUNC_SETDT           1       /* function switch: SET DATE/TIME */
#define UI_FUNC_SETUP           2       /* function switch: CONTROLLER SETUP */
#define UI_FUNC_OPMODE          3       /* function switch: MODE OF OPERATION */
#define UI_FUNC_IRRDAYS         4       /* function switch: IRRIGATION DAY(S) */
#define UI_FUNC_RUNTIME         5       /* function switch: RUN TIMES */
#define UI_FUNC_ADVANCED        6       /* function switch: ADVANCED SETTINGS */
#define UI_FUNC_RFU1            7       /* function switch: (not populated) */
#define UI_FUNC_APPRATE         8       /* function switch: APPLICATION RATE */
#define UI_FUNC_SOILTYPE        9       /* function switch: SOIL TYPE */
#define UI_FUNC_SLOPE           10      /* function switch: SLOPE */
#define UI_FUNC_MOIST           11      /* function switch: SETPOINTS */
#define UI_FUNC_GROUPS          12      /* function switch: GROUPS */
#define UI_FUNC_CLIMATE         13      /* function switch: CLIMATE */
#define UI_FUNC_PLANTTYPES      14      /* function switch: PLANT TYPES */
#define UI_FUNC_RFU2            15      /* function switch: (not populated) */

/*
**  FRONT PANEL SOFT KEY DEFINITIONS
*/
#define UI_N_SOFTKEYS           6       /* number of soft keys */

/* Soft-Key Mask Values (bit on = mask off the key) */
#define UI_KM_KEY1              0x01    /* Mask off Soft Key 1 */
#define UI_KM_KEY2              0x02    /* Mask off Soft Key 2 */
#define UI_KM_KEY3              0x04    /* Mask off Soft Key 3 */
#define UI_KM_KEY4              0x08    /* Mask off Soft Key 4 */
#define UI_KM_KEY5              0x10    /* Mask off Soft Key 5 */
#define UI_KM_KEY6              0x20    /* Mask off Soft Key 6 */
#define UI_KM_ALL               0x3F    /* Mask allowing no Soft Keys */
#define UI_KM_NONE              0x00    /* Mask allowing all Soft Keys */

/*
**  CONTROLLER STATUS FUNCTION MANUAL START SCREEN FIELDS
*/
#define UI_MANUAL_FIELD_PROGRAM     0       /* manual start: program */
#define UI_MANUAL_FIELD_OPMODE      1       /* manual start: mode */
#define UI_MANUAL_FIELD_PULSE       2       /* manual start: pulse mode */
#define UI_MANUAL_FIELD_LIMIT       3

/*
**  SET DATE/TIME FUNCTION SCREEN FIELDS
*/
#define UI_DT_FIELD_YEAR            0       /* date & time: year */
#define UI_DT_FIELD_MONTH           1       /* date & time: month */
#define UI_DT_FIELD_MDAY            2       /* date & time: day of month */
#define UI_DT_FIELD_HOUR            3       /* date & time: hour */
#define UI_DT_FIELD_MIN             4       /* date & time: minute */
#define UI_DT_FIELD_TIMEFMT         5       /* time format */
#define UI_DT_FIELD_LIMIT           6

/*
**  CONTROLLER SETUP SCREEN FIELDS
*/
#define UI_SETUP_FIELD_NUNITS       0       /* number of units */
#define UI_SETUP_FIELD_UNIT_TYPE    1       /* unit type */
#define UI_SETUP_FIELD_NZONES       2       /* number of zones */
#define UI_SETUP_FIELD_LIMIT        3

/*
**  CONTROLLER SETUP EXPANSION SCREEN FIELDS
*/
#define UI_SETUP_EXPAN_FIELD_LIMIT        1
#define UI_SETUP_EXPAN_MAC_FIELD_LIMIT    16
#define UI_SETUP_EXPAN_NZONES             0

/*
**  CONTROLLER SETUP SENSOR CONCENTRATOR SCREEN FIELDS
*/
#define UI_SETUP_SENSOR_CON_FIELD_LIMIT     12

/*
**  MODE OF OPERATION SCREEN FIELDS
*/
#define UI_OPMODE_FIELD_OPMODE      0       /* mode (runtime, moisture, ET) */
#define UI_OPMODE_FIELD_PULSE       1       /* pulse mode (standard, pulse) */
#define UI_OPMODE_FIELD_LIMIT       2

/*
**  GROUPS SCREEN FIELDS
*/
#define UI_GROUPS_FIELD_TYPE        0       /* sensor type */
#define UI_GROUPS_FIELD_PARAM       1       /* type-specific parameter Like MAC addr*/
#define UI_GROUPS_FIELD_PARAM_2     2       // Group a, b, c, d etc
#define UI_GROUPS_FIELD_PARAM_3     3       //New group field for wireless valves
#define UI_GROUPS_FIELD_LIMIT       4

/*
**  PLANT TYPES SCREEN FIELDS
*/
#define UI_PT_FIELD_TYPE            0       /* plant type */
#define UI_PT_FIELD_DENSITY         1       /* density */
#define UI_PT_FIELD_DT              2       /* drought tolerance */
#define UI_PT_FIELD_LIMIT           3

/*
**  CONTROLLER STATUS MESSAGE IDENTIFIERS - USED FOR LCD LINE 3 STATUS DISPLAY
**  (These identifiers are used by the uiFormatSystemStatus function.)
*/
#define UI_SMSG_ERROR           0       /* offset to first error message */
#define UI_SMSG_ERROR_LIMIT     UI_SMSG_ERROR + SYS_ERROR_LIMIT

#define UI_SMSG_FAULT           16      /* offset to first fault message */
#define UI_SMSG_FAULT_LIMIT     UI_SMSG_FAULT + SYS_FAULT_LIMIT

#define UI_SMSG_FLOW            47      /* display flow */
#define UI_SMSG_DATE_TIME       48      /* Date/Time message */
#define UI_SMSG_SENSOR          49      /* Sensor message */
#define UI_SMSG_SENSORFAIL      50      /* Sensor failure message */
#define UI_SMSG_PULSE           51      /* Pulse message */
#define UI_SMSG_AUTO_OFF        52      /* auto mode off message */
#define UI_SMSG_TOT_WATER       53      /* total watering time remaining */
#define UI_SMSG_RADIO_OFFLINE   54      /* radio is offline */
#define UI_SMSG_LIMIT           55

//Message start locations
#define UI_WS_MAC_ADR_START     22
#define UI_WV_MAC_ADR_START     22
#define UI_WS_CHAN_STRT         31
#define UI_WV_CHAN_STRT         31
#define UI_WV_GRP_STRT          33


/******************************************************************************
 *
 *  UI STRUCTURE DEFINITIONS
 *
 *****************************************************************************/

/*
**  Menu Action Function Types
*/
typedef void (*uiDisplayAction_t)(void);
typedef void (*uiNavDialAction_t)(void);
typedef void (*uiSoftKeyAction_t)(uint8_t eventType, uint8_t eventKey);

/*
**  Soft Key Definition Structure
*/
typedef struct
{
    uiSoftKeyAction_t  pAction;         /* softkey action (NULL if unused) */
    const char        *pLabel;          /* softkey label - SHOULD BE MSG #! */
} uiSoftKey_t;

/*
**  Menu Definition Structure
*/
typedef struct
{
    uiDisplayAction_t pDisplay;         /* update LCD display action */
    uiNavDialAction_t pDialCw;          /* option-dial-CW event action */
    uiNavDialAction_t pDialCcw;         /* option-dial-CCW event action */
    uiSoftKey_t softKey[UI_N_SOFTKEYS]; /* softkey labels & actions */
} uiMenuDb_t;

/*
**  Radio Signal Strength Bar-Graph Table Entry Structure
*/
typedef struct
{
    uint8_t     threshold;      /* strongest signal value for this label */
    const char *pText;
} uiRadioSignalBars_t;



/******************************************************************************
 *
 *  UI GLOBAL VARIABLES
 *
 *****************************************************************************/

/*
**  Current Function and Menu State
*/
static uint8_t unpin[4];

uint8_t uiPosition;                 /* current function select position */
static const uiMenuDb_t *uiCurMenu; /* current menu */
static uint8_t uiKeyMask;           /* current softkey bit mask, 1 = disable */


/*
**  Current Menu General Purpose State Variables
*/

static uint8_t pd;
static uint8_t uiField;             /* current field within menu */
static uint8_t uiField2;            /* current secondary field index */
static uint8_t uiFirstField;        /* field to display at top of page */
static uint8_t uiTemp8;             /* temporary register 8-bits */
static uint64_t uiTemp64;           /* temporary register 64-bits */
static uint16_t uiTemp16;           /* temporary register 16-bits */
static uint16_t uiTemp162;
//static uint32_t snsConTime
static bool_t lock = FALSE;

static uint8_t pinIndex = 0;
static uint8_t unpinIndex = 0;
static uint8_t pin[4];





/*
**  LCD Display Composition Variables
*/
char uiLcdBuf[160];                 /* LCD screen buffer */
uint8_t uiLcdCursor;                /* current hardware cursor location */


/*
**  LCD Refresh Control Variables
*/
static bool_t uiLcdRefreshReq = TRUE;   /* flag to force immediate LCD refresh */
static bool_t uiLcdRefreshTick = FALSE; /* flag indicating time-tick LCD refresh */
static uint8_t uiLcdLastUpdateSec;  /* seconds value on last LCD update */
static uint8_t uiLcdLastUpdateMin;  /* minutes value on last LCD update */
static uint32_t uiLastEventTime;    /* Time of last front panel switch event */
static uint32_t uiScreenTimeout;    /* # secs inactivity before return to home */
static int8_t uiStatusCycle = 0;    /* Current status cycle message */
static uint8_t uiStatusReps = 0;    /* # of reps completed for current cycle */
static bool_t uiRtcIsFast = FALSE;  /* Clock speed-up for software testing */


uint8_t navEndZone;           /* control variable to set how many zones can navigate through */
bool_t timeChanged;          /* keep track if user change the time */

/******************************************************************************
 *
 *  UI FUNCTION PROTOTYPES
 *
 *****************************************************************************/

/*
**  Common Utility Routines
*/
static void  uiEventFunction(uint8_t position, bool_t altFunc);
static void  uiEventDial(uint8_t direction);
static void  uiEventSoft(uint8_t eventType, uint8_t eventKey);
static void  uiMenuSet(const uiMenuDb_t *menu);
static char *uiFormatIrrMode(char *buf);
static char *uiFormatSystemStatus(char *buf);
static void  uiFirstFieldFix(uint8_t nColumns);
static void uiFirstFieldFix2(uint8_t nColumns);
static void  uiLcdUpdate(void);


/*
**  Menu Action Routines
*/
static void uiUnimplementedDisplay(void);

static void uiStatusDisplay(void);
static void uiStatusNext(void);
static void uiStatusPrev(void);

static void uiStatusOff(uint8_t eventType, uint8_t eventKey);
static void uiStatusAuto(uint8_t eventType, uint8_t eventKey);
static void uiStatusManual(uint8_t eventType, uint8_t eventKey);
static void uiStatusTest(uint8_t eventType, uint8_t eventKey);
static void uiStatusStop(uint8_t eventType, uint8_t eventKey);
static void uiStatusPause(uint8_t eventType, uint8_t eventKey);
static void uiStatusResume(uint8_t eventType, uint8_t eventKey);
static void uiStatusSkip(uint8_t eventType, uint8_t eventKey);

static void uiStatusTestDisplay(void);
static void uiStatusTestInc(uint8_t eventType, uint8_t eventKey);
static void uiStatusTestDec(uint8_t eventType, uint8_t eventKey);
static void uiStatusTestStart(uint8_t eventType, uint8_t eventKey);
static void uiStatusTestSkip(uint8_t eventType, uint8_t eventKey);

static void uiStatusManualDisplay(void);
static void uiStatusManualNext(void);
static void uiStatusManualPrev(void);
static void uiStatusManualInc(uint8_t eventType, uint8_t eventKey);
static void uiStatusManualDec(uint8_t eventType, uint8_t eventKey);
static void uiStatusManualStart(uint8_t eventType, uint8_t eventKey);

static void uiStatusCancel(uint8_t eventType, uint8_t eventKey);

static void uiStatusResumeDisplay(void);
static void uiStatusResumeOk(uint8_t eventType, uint8_t eventKey);
static void uiStatusResumeTestOk(uint8_t eventType, uint8_t eventKey);
static void uiStatusResumeManOk(uint8_t eventType, uint8_t eventKey);
static void uiStatusResumeCancel(uint8_t eventType, uint8_t eventKey);

//  lock and unlock screen
static void uiLock(uint8_t eventType, uint8_t eventKey);
static void uiUnlock(uint8_t eventType, uint8_t eventKey);





static void uiDtDisplay(void);
static void uiDtNext(void);
static void uiDtPrev(void);
static void uiDtInc(uint8_t eventType, uint8_t eventKey);
static void uiDtDec(uint8_t eventType, uint8_t eventKey);

static void uiSetupDisplay(void);
static void uiSetupNext(void);
static void uiSetupPrev(void);
static void uiSetupInc(uint8_t eventType, uint8_t eventKey);
static void uiSetupDec(uint8_t eventType, uint8_t eventKey);

       
static void uiSetupExpanNext(void);
static void uiSetupExpanPrev(void);
static void uiSetupExpanMACNext(void);
static void uiSetupExpanMACPrev(void);
static void uiSetupExpanMACDisplay(void);
static void uiSetupExpanDisplay(void);
static void uiSetupExpanMAC(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanConfig(uint8_t eventType, uint8_t eventKey);
static void uiSetupEx1(uint8_t eventType, uint8_t eventKey);
static void uiSetupEx2(uint8_t eventType, uint8_t eventKey);
static void uiSetupEx3(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanMacClear(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanMacAccept(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanMacCancel(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanMACDec(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanMACInc(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanBack(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanDec(uint8_t eventType, uint8_t eventKey);
static void uiSetupExpanInc(uint8_t eventType, uint8_t eventKey);

static void uiSetupSnsCon(uint8_t eventType, uint8_t eventKey);
static void uiSetupSensorConDisplay(void);
static void uiSetupSensorConNext(void);
static void uiSetupSensorConPrev(void);
static void uiSetupSensorConAdd(uint8_t eventType, uint8_t eventKey);
static void uiSetupSensorConDelete(uint8_t eventType, uint8_t eventKey);
static void uiSetupSensorConAccept(uint8_t eventType, uint8_t eventKey); 
static void uiSetupSensorConDeny(uint8_t eventType, uint8_t eventKey); 
static void uiSetupSensorConBack(uint8_t eventType, uint8_t eventKey);
static void uiSetupSensorConInc(uint8_t eventType, uint8_t eventKey);

static void uiOpModeDisplay(void);
static void uiOpModeNext(void);
static void uiOpModePrev(void);
static void uiOpModeInc(uint8_t eventType, uint8_t eventKey);
static void uiOpModeDec(uint8_t eventType, uint8_t eventKey);

static void uiIrrDaysSummaryDisplay(void);
static void uiIrrDaysSummaryNext(void);
static void uiIrrDaysSummaryPrev(void);
static void uiIrrDaysSummaryToday(uint8_t eventType, uint8_t eventKey);

static void uiIrrDaysDisplay(void);
static void uiIrrDaysNext(void);
static void uiIrrDaysPrev(void);
static void uiIrrDaysInc(uint8_t eventType, uint8_t eventKey);
static void uiIrrDaysDec(uint8_t eventType, uint8_t eventKey);
static void uiIrrDaysOff(uint8_t eventType, uint8_t eventKey);

static void uiRunTimeSummaryDisplay(void);
static void uiRunTimeSummaryNext(void);
static void uiRunTimeSummaryPrev(void);
static void uiRunTimeSummaryGotoA(uint8_t eventType, uint8_t eventKey);
#if SYS_N_PROGRAMS > 1
static void uiRunTimeSummaryGotoB(uint8_t eventType, uint8_t eventKey);
#endif
#if SYS_N_PROGRAMS > 2
static void uiRunTimeSummaryGotoC(uint8_t eventType, uint8_t eventKey);
#endif
#if SYS_N_PROGRAMS > 3
static void uiRunTimeSummaryGotoD(uint8_t eventType, uint8_t eventKey);
#endif

static void uiRunTimeDisplay(void);
static void uiRunTimeNext(void);
static void uiRunTimePrev(void);
static void uiRunTimeInc(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeDec(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeOff(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeCopy(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeGroup2(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeGroup3(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeGroup4(uint8_t eventType, uint8_t eventKey);


static void uiRunTimeCopyDisplay(void);
static void uiRunTimeCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiRunTimeCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiAppRateSummaryDisplay(void);
static void uiAppRateSummaryNext(void);
static void uiAppRateSummaryPrev(void);
static void uiAppRateSummaryGoto(uint8_t eventType, uint8_t eventKey);

static void uiAppRateDisplay(void);
static void uiAppRateNext(void);
static void uiAppRatePrev(void);
static void uiAppRateInc(uint8_t eventType, uint8_t eventKey);
static void uiAppRateDec(uint8_t eventType, uint8_t eventKey);
static void uiAppRateCopy(uint8_t eventType, uint8_t eventKey);

static void uiAppRateCopyDisplay(void);
static void uiAppRateCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiAppRateCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiAppEffSummaryDisplay(void);
static void uiAppEffSummaryNext(void);
static void uiAppEffSummaryPrev(void);
static void uiAppEffSummaryGoto(uint8_t eventType, uint8_t eventKey);

static void uiAppEffDisplay(void);
static void uiAppEffNext(void);
static void uiAppEffPrev(void);
static void uiAppEffInc(uint8_t eventType, uint8_t eventKey);
static void uiAppEffDec(uint8_t eventType, uint8_t eventKey);
static void uiAppEffCopy(uint8_t eventType, uint8_t eventKey);

static void uiAppEffCopyDisplay(void);
static void uiAppEffCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiAppEffCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiSoilTypeDisplay(void);
static void uiSoilTypeNext(void);
static void uiSoilTypePrev(void);
static void uiSoilTypeInc(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeDec(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeCopy(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeGroup2(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeGroup3(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeGroup4(uint8_t eventType, uint8_t eventKey);

static void uiSoilTypeCopyDisplay(void);
static void uiSoilTypeCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiSoilTypeCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiSlopeDisplay(void);
static void uiSlopeNext(void);
static void uiSlopePrev(void);
static void uiSlopeInc(uint8_t eventType, uint8_t eventKey);
static void uiSlopeDec(uint8_t eventType, uint8_t eventKey);
static void uiSlopeCopy(uint8_t eventType, uint8_t eventKey);
static void uiSlopeGroup2(uint8_t eventType, uint8_t eventKey);
static void uiSlopeGroup3(uint8_t eventType, uint8_t eventKey);
static void uiSlopeGroup4(uint8_t eventType, uint8_t eventKey);

static void uiSlopeCopyDisplay(void);
static void uiSlopeCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiSlopeCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiMoistDisplay(void);
static void uiMoistNext(void);
static void uiMoistPrev(void);
static void uiMoistInc(uint8_t eventType, uint8_t eventKey);
static void uiMoistDec(uint8_t eventType, uint8_t eventKey);
static void uiMoistCopy(uint8_t eventType, uint8_t eventKey);
static void uiMoistGroup2(uint8_t eventType, uint8_t eventKey);
static void uiMoistGroup3(uint8_t eventType, uint8_t eventKey);
static void uiMoistGroup4(uint8_t eventType, uint8_t eventKey);

static void uiMoistCopyDisplay(void);
static void uiMoistCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiMoistCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiMoistValDisplay(void);
static void uiMoistValNext(void);
static void uiMoistValPrev(void);
static void uiMoistValGroup1(uint8_t eventType, uint8_t eventKey);
static void uiMoistValGroup2(uint8_t eventType, uint8_t eventKey);
static void uiMoistValGroup3(uint8_t eventType, uint8_t eventKey);
static void uiMoistValGroup4(uint8_t eventType, uint8_t eventKey);

///////////////////////////////////////////////
/// FLOW SETTINGS

static void uiFlowSetting(uint8_t eventType, uint8_t eventKey);
static void uiFlowDisplay(void);
static void uiFlowNext(void);
static void uiFlowPrev(void);
static void uiFlowInc(uint8_t eventType, uint8_t eventKey);
static void uiFlowDec(uint8_t eventType, uint8_t eventKey);
static void uiFlowCopy(uint8_t eventType, uint8_t eventKey);
static void uiFlowGroup2(uint8_t eventType, uint8_t eventKey);
static void uiFlowGroup3(uint8_t eventType, uint8_t eventKey);
static void uiFlowGroup4(uint8_t eventType, uint8_t eventKey);

static void uiFlowCopyDisplay(void);
static void uiFlowCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiFlowCopyNo(uint8_t eventType, uint8_t eventKey);

///////////////////////////////////////////////////
static void uiGroupsSummaryDisplay(void);
static void uiGroupsSummaryNext(void);
static void uiGroupsSummaryPrev(void);

static void uiGroupsDisplay(void);
static void uiGroupsNext(void);
static void uiGroupsPrev(void);
static void uiGroupsInc(uint8_t eventType, uint8_t eventKey);
static void uiGroupsDec(uint8_t eventType, uint8_t eventKey);
static void uiGroupsCopy(uint8_t eventType, uint8_t eventKey);
static void uiGroupsGroup2(uint8_t eventType, uint8_t eventKey);
static void uiGroupsGroup3(uint8_t eventType, uint8_t eventKey);
static void uiGroupsGroup4(uint8_t eventType, uint8_t eventKey);

static void uiGroupsUngroupDisplay(void);
static void uiGroupsUngroupYes(uint8_t eventType, uint8_t eventKey);
static void uiGroupsUngroupNo(uint8_t eventType, uint8_t eventKey);

static void uiGroupsCopyDisplay(void);
static void uiGroupsCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiGroupsCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiGroupsCheckZone(uint8_t zone);
static void uiGroupsCheckAll(void);
static int  uiGroupsCountFollowers(uint8_t zone);

static void uiClimateDisplay(void);
static void uiClimateNext(void);
static void uiClimatePrev(void);
static void uiClimateInc(uint8_t eventType, uint8_t eventKey);
static void uiClimateDec(uint8_t eventType, uint8_t eventKey);
static void uiClimateCopy(uint8_t eventType, uint8_t eventKey);
static void uiClimateGroup2(uint8_t eventType, uint8_t eventKey);
static void uiClimateGroup3(uint8_t eventType, uint8_t eventKey);
static void uiClimateGroup4(uint8_t eventType, uint8_t eventKey);

static void uiClimateCopyDisplay(void);
static void uiClimateCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiClimateCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiMbValDisplay(void);
static void uiMbValNext(void);
static void uiMbValPrev(void);
static void uiMbValInc(uint8_t eventType, uint8_t eventKey);
static void uiMbValDec(uint8_t eventType, uint8_t eventKey);
static void uiMbValIncFast(uint8_t eventType, uint8_t eventKey);
static void uiMbValDecFast(uint8_t eventType, uint8_t eventKey);
static void uiMbValSetZero(uint8_t eventType, uint8_t eventKey);
static void uiMbValCopy(uint8_t eventType, uint8_t eventKey);

static void uiMbValCopyDisplay(void);
static void uiMbValCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiMbValCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiPlantTypeDisplay(void);
static void uiPlantTypeNext(void);
static void uiPlantTypePrev(void);
static void uiPlantTypeInc(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeDec(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeCopy(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeGroup2(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeGroup3(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeGroup4(uint8_t eventType, uint8_t eventKey);


static void uiPlantTypeCopyDisplay(void);
static void uiPlantTypeCopyYes(uint8_t eventType, uint8_t eventKey);
static void uiPlantTypeCopyNo(uint8_t eventType, uint8_t eventKey);

static void uiRzwwsDisplay(void);
static void uiRzwwsNext(void);
static void uiRzwwsPrev(void);
static void uiRzwwsGroup1(uint8_t eventType, uint8_t eventKey);
static void uiRzwwsGroup2(uint8_t eventType, uint8_t eventKey);
static void uiRzwwsGroup3(uint8_t eventType, uint8_t eventKey);
static void uiRzwwsGroup4(uint8_t eventType, uint8_t eventKey);


static void uiAdvSummaryDisplay(void);
static void uiAdvSummaryNext(void);
static void uiAdvSummaryPrev(void);
static void uiAdvSummaryHidden0(uint8_t eventType, uint8_t eventKey);
static void uiAdvSummaryHidden1(uint8_t eventType, uint8_t eventKey);

static void uiAdvDefaultsDisplay(void);
static void uiAdvDefaultsYes(uint8_t eventType, uint8_t eventKey);
static void uiAdvDefaultsNo(uint8_t eventType, uint8_t eventKey);

static void uiAdvProduct1Display(void);
static void uiAdvProduct1Next(void);
static void uiAdvProduct1Prev(void);

static void uiAdvProductDbg0(uint8_t eventType, uint8_t eventKey);
static void uiAdvProductDbg1(uint8_t eventType, uint8_t eventKey);
static void uiAdvProductDbg2(uint8_t eventType, uint8_t eventKey);

static void uiAdvProductDbgDisplay(void);
static void uiAdvProductDbgNext(void);
static void uiAdvProductDbgPrev(void);
static void uiAdvProductDbgRadio(uint8_t eventType, uint8_t eventKey);
static void uiAdvProductDbgEvSave(uint8_t eventType, uint8_t eventKey);
static void uiAdvProductDbgEvLog(uint8_t eventType, uint8_t eventKey);
static void uiAdvProductDbgReset(uint8_t eventType, uint8_t eventKey);

static void uiAdvEventLogDisplay(void);
static void uiAdvEventLogNext(void);
static void uiAdvEventLogPrev(void);

static void uiAdvResetDisplay(void);
static void uiAdvResetYes(uint8_t eventType, uint8_t eventKey);
static void uiAdvResetNo(uint8_t eventType, uint8_t eventKey);

static void uiAdvProduct2Display(void);
static void uiAdvProduct2Next(void);
static void uiAdvProduct2Prev(void);

static void uiAdvRadioDisplay(void);
static void uiAdvRadioNext(void);
static void uiAdvRadioPrev(void);
static void uiAdvRadioReset(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEdit(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioHwReset(uint8_t eventType, uint8_t eventKey);

static void uiAdvRadioEditDisplay(void);
static void uiAdvRadioEditNext(void);
static void uiAdvRadioEditPrev(void);
static void uiAdvRadioEditInc(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEditDec(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEditAny(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEditOp(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEditAccept(uint8_t eventType, uint8_t eventKey);
static void uiAdvRadioEditCancel(uint8_t eventType, uint8_t eventKey);

static void uiAdvRadio2Display(void);
static void uiAdvRadio2Next(void);
static void uiAdvRadio2Prev(void);
static void uiAdvRadio2Test(uint8_t eventType, uint8_t eventKey);

//// lock/unlock
static void uiLockDisplay( void );
static void uiLockDisplayNext( void );
static void uiLockDisplayPrev( void );
static void uiLockAdd(uint8_t eventType, uint8_t eventKey);
static void uiLockDel(uint8_t eventType, uint8_t eventKey);
static void uiLockOK(uint8_t eventType, uint8_t eventKey);
static void uiLockDisplayOK( void );



static void uiUnlockDisplay( void );
static void uiUnlockDisplayNext( void );
static void uiUnlockDisplayPrev( void );
static void uiUnlockAdd(uint8_t eventType, uint8_t eventKey);
static void uiUnlockDel(uint8_t eventType, uint8_t eventKey);
static void uiUnlockOK(uint8_t eventType, uint8_t eventKey);
static void uiUnlockDisplayOK( void );




////////////////////////////////////////////////////
/******************************************************************************
 *
 *  UI STRING CONSTANTS
 *
 *****************************************************************************/

/*
**  SYSTEM ERROR MESSAGE TABLE
**  Note: String pointer array index correlates with System Error bit number.
*/
static const char * const uiSystemErrorMsg[SYS_ERROR_LIMIT] =
{
/*  "1234567890123456789012345678901234567890" - max msg size is 40 chars */
    "Clock is not set. Please set date/time.",      /* SYS_ERROR_DATETIME */
    "No weather data received in 24 hours.",        /* SYS_ERROR_WEATHERDATA */
    "No moisture sensors are configured.",          /* SYS_ERROR_NOSENSORS */
    "No runnable program is scheduled.",            /* SYS_ERROR_NOPROGRAM */
};

/*
**  SYSTEM FAULT MESSAGE TABLE
**  Note: String pointer array index correlates with System Fault bit number.
*/
static const char * const uiSystemFaultMsg[SYS_FAULT_LIMIT] =
{
/*  "1234567890123456789012345678901234567890" - max msg size is 40 chars */
    "Invalid manufacturing data.",                  /* 0+27, SYS_FAULT_MANUFDATA */
    "24V AC failure. Please check fuse 1.",          /* 28+35, SYS_FAULT_24VAC */
    "", /* SYS_FAULT_MOIST handled as special case by uiFormatSystemStatus. */
    "Radio has failed.",                            /* 63+28, SYS_FAULT_RADIO */
    "Loss of Communications with zones 1-12.",        /* 92+39, expansion bus loss of comm with master */
    "Loss of Communications with zones 13-24.",        /* 131+40 expansion bus loss of comm with expansion 1 */
    "Loss of Communications with zones 25-36.",        /* 171+40 expansion bus loss of comm with expansion 2 */
    "Loss of Communications with zones 37-48.",        /* 211+40 expansion bus loss of comm with expansion 3 */
    "Loss of Comm with SC: "                  /* 251+274 sensor concentrator loss of communication */
};

/*
**  DAY FULL NAMES (used with irrigation schedules day index)
*/
static const char * const uiDayFullNames[] =
{
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday",
};

/*
**  TIME FORMAT NAMES
*/
static const char * const uiTimeFmtNames[] =
{
    "12-Hour",
    "24-Hour (US)",
    "24-Hour (Canada)",
};

/*
**  MODE OF OPERATION NAMES
*/
static const char * const uiOpModeNames[] =
{
    "Runtime Based",
    "Sensor Based",
    "Weather Based",
};

/*
**  SOIL TYPE NAMES
*/
static const char * const uiSoilTypeNames[] =
{
    "Clay",
    "Silty Clay",
    "Clay/Loam",
    "Loam",
    "Sandy Loam",
    "Loamy Sand",
    "Sand",
};

/*
**  SLOPE VALUE NAMES
*/
static const char * const uiSlopeNames[] =
{
    "0-3%",
    "4-6%",
    "7-12%",
    ">12%",
};

/*
**  CLIMATE VALUE NAMES
*/
static const char * const uiClimateNames[] =
{
    "Full Sun",
    "50% Shade",
    "75% Shade",
};

/*
**  PLANT TYPE SPECIES NAMES
*/
static const char * const uiPlantTypeNames[] =
{
    "Trees",
    "Shrubs",
    "Ground Cover",
    "Mixture",
    "Fescue Turf",
    "Bermuda Turf",
};

/*
**  PLANT TYPE DENSITY NAMES
*/
static const char * const uiPlantTypeDensities[] =
{
    "Sparse",
    "Average",
    "Highly Dense",
};

/*
**  PLANT TYPE DROUGHT TOLERANCE NAMES
*/
static const char * const uiPlantTypeDroughtTolerances[] =
{
    "Low DT",
    "Avg DT",
    "High DT",
};

/*
**  RADIO STATUS NAMES
*/
static const char * const uiRadioStatus[] =
{
    "NOT PRESENT",          /* RADIO_STATUS_NOTPOP */
    "ONLINE",               /* RADIO_STATUS_ONLINE */
    "OFFLINE",              /* RADIO_STATUS_OFFLINE */
    "SCANNING",             /* RADIO_STATUS_SCANNING */
    "FAILURE",              /* RADIO_STATUS_FAILURE */
    "INITIALIZING",         /* RADIO_STATUS_INIT */
};


/*
**  RADIO SIGNAL STRENGTH BAR-GRAPH TABLE
*/
static const uiRadioSignalBars_t uiRadioSignalBars[] =
{
    {80, "\377"},                       /* 1 bar:  80 and up */
    {70, "\377\377"},                   /* 2 bars: 70-79     */
    {60, "\377\377\377"},               /* 3 bars: 60-69     */
    {40, "\377\377\377\377"},           /* 4 bars: 40-59     */
    {0,  "\377\377\377\377\377"},       /* 5 bars:  0-39     */
};



/******************************************************************************
 *
 *  MENU TEMPLATE DEFINITIONS
 *
 *****************************************************************************/

/* Unimplemented function menu */
static const uiMenuDb_t uiMenuUnimplemented =
{
    uiUnimplementedDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Controller Status Function Main Menu - System Idle (no program in progress)*/
static const uiMenuDb_t uiMenuStatusIdle =
{
    uiStatusDisplay,
    uiStatusNext,
    uiStatusPrev,
    {
        {uiStatusOff,           "OFF"},
        {uiStatusAuto,          "AUTO"},
        {uiStatusManual,        "MANUAL"},
        {uiStatusTest,          "TEST"},
        {NULL,                    NULL},
        {NULL,                    NULL},
    }
};


/* Controller Status Function Main Menu - Program Active (watering/soaking) */
static const uiMenuDb_t uiMenuStatusActive =
{
    uiStatusDisplay,
    uiStatusNext,
    uiStatusPrev,
    {
        {uiStatusOff,            "OFF"},
        {uiStatusAuto,          "AUTO"},
        {NULL,                    NULL},
        {uiStatusStop,          "STOP"},
        {NULL,                    NULL},
        {uiStatusSkip,          "SKIP"},
    }
};


/* Controller Status Function Main Menu - Program Interrupted (inhibit/pause) */
static const uiMenuDb_t uiMenuStatusPaused =
{
    uiStatusDisplay,
    uiStatusNext,
    uiStatusPrev,
    {
        {uiStatusOff,             "OFF"},
        {uiStatusAuto,           "AUTO"},
        {NULL,                     NULL},
        {uiStatusStop,           "STOP"},
        {uiStatusResume,       "RESUME"},
        {uiStatusTestSkip,       "NEXT"},
    }
};


/* Controller Status Function Menu - Test Start */
static const uiMenuDb_t uiMenuStatusTest =
{
    uiStatusTestDisplay,
    NULL,
    NULL,
    {
        {uiStatusTestDec,       "-"},
        {uiStatusTestInc,       "+"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiStatusTestStart,     "START"},
        {uiStatusCancel,        "CANCEL"},
    }
};


/* Controller Status Function Menu - Manual Start */
static const uiMenuDb_t uiMenuStatusManual =
{
    uiStatusManualDisplay,
    uiStatusManualNext,
    uiStatusManualPrev,
    {
        {uiStatusManualDec,     "-"},
        {uiStatusManualInc,     "+"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiStatusManualStart,   "START"},
        {uiStatusCancel,        "CANCEL"},
    }
};


/* Controller Status Function Menu - Resume Interrupted Program from Inhibit */
static const uiMenuDb_t uiMenuStatusResume =
{
    uiStatusResumeDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiStatusResumeOk,      "YES"},
        {uiStatusResumeCancel,  "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Controller Status Function Menu - Resume Test Start from Inhibit */
static const uiMenuDb_t uiMenuStatusResumeTest =
{
    uiStatusResumeDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiStatusResumeTestOk,  "YES"},
        {uiStatusResumeCancel,  "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Controller Status Function Menu - Resume Manual Start from Inhibit */
static const uiMenuDb_t uiMenuStatusResumeMan =
{
    uiStatusResumeDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiStatusResumeManOk,   "YES"},
        {uiStatusResumeCancel,  "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Controller Setup Function Menu */
static const uiMenuDb_t uiMenuSetup =
{
    uiSetupDisplay,
    uiSetupNext,
    uiSetupPrev,
    {
        {uiSetupDec,            "-"},
        {uiSetupInc,            "+"},
        {uiSetupEx1,            "13-24"},
        {uiSetupEx2,            "25-36"},
        {uiSetupEx3,            "37-48"},
        {uiSetupSnsCon,         "SnsCon"},
    }
};


/* Controller Expansion 1 Setup Function Menu */
static const uiMenuDb_t uiMenuSetupExpan1 =
{
    uiSetupExpanDisplay,
    uiSetupExpanNext,
    uiSetupExpanPrev,
    {
        {uiSetupExpanDec,       "-"},
        {uiSetupExpanInc,       "+"},
        {NULL,                  NULL},
        {uiSetupExpanMAC,       "SET MAC ID"},
        {NULL,                  NULL},
        {uiSetupExpanBack,      "Back"},
    }
};

/* Controller Expansion 2 Setup Function Menu */
static const uiMenuDb_t uiMenuSetupExpan2 =
{
    uiSetupExpanDisplay,
    uiSetupExpanNext,
    uiSetupExpanPrev,
    {
        {uiSetupExpanDec,       "-"},
        {uiSetupExpanInc,       "+"},
        {NULL,                  NULL},
        {uiSetupExpanMAC,       "SET MAC ID"},
        {NULL,                  NULL},
        {uiSetupExpanBack,      "Back"},
    }
};

/* Controller Expansion 3 Setup Function Menu */
static const uiMenuDb_t uiMenuSetupExpan3 =
{
    uiSetupExpanDisplay,      
    uiSetupExpanNext,
    uiSetupExpanPrev,
    {
        {uiSetupExpanDec,       "-"},
        {uiSetupExpanInc,       "+"},
        {NULL,                  NULL},
        {uiSetupExpanMAC,       "SET MAC ID"},
        {NULL,                  NULL},
        {uiSetupExpanBack,      "Back"},
    }
};

/* Controller Expansion MAC ID Setup Function Menu */
static const uiMenuDb_t uiMenuSetupExpanMACMenu =
{
    uiSetupExpanMACDisplay,
    uiSetupExpanMACNext,
    uiSetupExpanMACPrev,
    {
        {uiSetupExpanMACDec,        "-"},
        {uiSetupExpanMACInc,        "+"},
        {NULL,                      NULL},
        {uiSetupExpanMacClear,      "Clear"},
        {uiSetupExpanMacAccept,     "Accept"},
        {uiSetupExpanMacCancel,     "Cancel"},
    }
};


/* Sensor Concentrator Setup Function Menu */
static const uiMenuDb_t uiMenuSetupSensorConMenu =
{
    uiSetupSensorConDisplay,
    uiSetupSensorConNext,
    uiSetupSensorConPrev,
    {
        {uiSetupSensorConAdd,       "Add"},
        {uiSetupSensorConDelete,    "Delete"},
        {uiSetupSensorConInc,       "+"},//{NULL,                      NULL},
        {uiSetupSensorConAccept,    "Accept"},
        {uiSetupSensorConDeny,      "Cancel"},
        {uiSetupExpanBack,          "Back"},
    }
};


/* Set Date/Time Function Menu */
static const uiMenuDb_t uiMenuDt =
{
    uiDtDisplay,
    uiDtNext,
    uiDtPrev,
    {
        {uiDtDec,               "-"},
        {uiDtInc,               "+"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Mode of Operation Function Menu */
static const uiMenuDb_t uiMenuOpMode =
{
    uiOpModeDisplay,
    uiOpModeNext,
    uiOpModePrev,
    {
        {uiOpModeDec,          "-"},
        {uiOpModeInc,          "+"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Irrigation Days Function Menu - summary */
static const uiMenuDb_t uiMenuIrrDaysSummary =
{
    uiIrrDaysSummaryDisplay,
    uiIrrDaysSummaryNext,
    uiIrrDaysSummaryPrev,
    {
        {uiIrrDaysSummaryToday, "\176TODAY"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Irrigation Days Function Menu - values */
static const uiMenuDb_t uiMenuIrrDays =
{
    uiIrrDaysDisplay,
    uiIrrDaysNext,
    uiIrrDaysPrev,
    {
        {uiIrrDaysDec,          "-"},
        {uiIrrDaysInc,          "+"},
        {uiIrrDaysOff,          "OFF"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Run Times Function Menu - summary */
static const uiMenuDb_t uiMenuRunTimeSummary =
{
    uiRunTimeSummaryDisplay,
    uiRunTimeSummaryNext,
    uiRunTimeSummaryPrev,
    {
        {uiRunTimeSummaryGotoA, "\176A"},
#if SYS_N_PROGRAMS > 1
        {uiRunTimeSummaryGotoB, "\176B"},
#else
        {NULL,                  NULL},
#endif
#if SYS_N_PROGRAMS > 2
        {uiRunTimeSummaryGotoC, "\176C"},
#else
        {NULL,                  NULL},
#endif
#if SYS_N_PROGRAMS > 3
        {uiRunTimeSummaryGotoD, "\176D"},
#else
        {NULL,                  NULL},
#endif
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Run Times Function Menu - values */
static const uiMenuDb_t uiMenuRunTime =
{
    uiRunTimeDisplay,
    uiRunTimeNext,
    uiRunTimePrev,
    {

          {uiRunTimeDec,          "-"},
          {uiRunTimeInc,          "+"},
          {uiRunTimeGroup2,       "13-24"},
          {uiRunTimeGroup3,       "25-36"},
          {uiRunTimeGroup4,       "37-48"},
          {uiRunTimeCopy,         "COPY"},
    }
};


/* Run Times Function Menu - copy */
static const uiMenuDb_t uiMenuRunTimeCopy =
{
    uiRunTimeCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiRunTimeCopyYes,      "YES"},
        {uiRunTimeCopyNo,       "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Application Rate Function Menu - summary */
static const uiMenuDb_t uiMenuAppRateSummary =
{
    uiAppRateSummaryDisplay,
    uiAppRateSummaryNext,
    uiAppRateSummaryPrev,
    {
        {uiAppRateSummaryGoto,  "\176EFF"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Application Rate Function Menu - values */
static const uiMenuDb_t uiMenuAppRate =
{
    uiAppRateDisplay,
    uiAppRateNext,
    uiAppRatePrev,
    {

          {uiAppRateDec,          "-"},
          {uiAppRateInc,          "+"},
          {NULL,                  NULL},
          {NULL,                  NULL},
          {NULL,                  NULL},
          {uiAppRateCopy,         "COPY"},
    }
};


/* Application Rate Function Menu - copy */
static const uiMenuDb_t uiMenuAppRateCopy =
{
    uiAppRateCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAppRateCopyYes,      "YES"},
        {uiAppRateCopyNo,       "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Application Efficiency - summary */
static const uiMenuDb_t uiMenuAppEffSummary =
{
    uiAppEffSummaryDisplay,
    uiAppEffSummaryNext,
    uiAppEffSummaryPrev,
    {
        {uiAppEffSummaryGoto,   "\176RATE"},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Application Efficiency - values */
static const uiMenuDb_t uiMenuAppEff =
{
    uiAppEffDisplay,
    uiAppEffNext,
    uiAppEffPrev,
    {

          {uiAppEffDec,           "-"},
          {uiAppEffInc,           "+"},
          {NULL,                  NULL},
          {NULL,                  NULL},
          {NULL,                  NULL},
          {uiAppEffCopy,          "COPY"},
    }
};


/* Application Efficiency - copy */
static const uiMenuDb_t uiMenuAppEffCopy =
{
    uiAppEffCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAppEffCopyYes,       "YES"},
        {uiAppEffCopyNo,        "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Soil Type Function Menu */
static const uiMenuDb_t uiMenuSoilType =
{
    uiSoilTypeDisplay,
    uiSoilTypeNext,
    uiSoilTypePrev,
    {

          {uiSoilTypeDec,         "-"},
          {uiSoilTypeInc,         "+"},
          {uiSoilTypeGroup2,      "13-24"},
          {uiSoilTypeGroup3,      "25-36"},
          {uiSoilTypeGroup4,      "37-48"},
          {uiSoilTypeCopy,        "COPY"},

    }
};


/* Soil Type Function Menu - copy */
static const uiMenuDb_t uiMenuSoilTypeCopy =
{
    uiSoilTypeCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiSoilTypeCopyYes,     "YES"},
        {uiSoilTypeCopyNo,      "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Slope Function Menu */
static const uiMenuDb_t uiMenuSlope =
{
    uiSlopeDisplay,
    uiSlopeNext,
    uiSlopePrev,
    {
          {uiSlopeDec,            "-"},
          {uiSlopeInc,            "+"},
          {uiSlopeGroup2,         "13-24"},
          {uiSlopeGroup3,         "25-36"},
          {uiSlopeGroup4,         "37-48"},
          {uiSlopeCopy,           "COPY"},
    }
};


/* Slope Function Menu - copy */
static const uiMenuDb_t uiMenuSlopeCopy =
{
    uiSlopeCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiSlopeCopyYes,        "YES"},
        {uiSlopeCopyNo,         "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Moisture Sensor Setpoints Function Menu */
static const uiMenuDb_t uiMenuMoist =
{
    uiMoistDisplay,
    uiMoistNext,
    uiMoistPrev,
    {
          {uiMoistDec,            "-"},
          {uiMoistInc,            "+"},
          {uiMoistGroup2,         "13-24"},
          {uiMoistGroup3,         "25-36"},
          {uiMoistGroup4,         "37-48"}, 
          //{NULL,                     NULL},         
          {uiFlowSetting,          "FLOW"},
    }
};


/* Moisture Sensor Setpoints Function Menu - copy */
static const uiMenuDb_t uiMenuMoistCopy =
{
    uiMoistCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiMoistCopyYes,        "YES"},
        {uiMoistCopyNo,         "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Moisture Sensor Values Menu */
static const uiMenuDb_t uiMenuMoistVal =
{
    uiMoistValDisplay,
    uiMoistValNext,
    uiMoistValPrev,
    {
        {uiMoistValGroup1,      "1-12"},
        {uiMoistValGroup2,      "13-24"},
        {uiMoistValGroup3,      "25-36"},
        {uiMoistValGroup4,      "37-48"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};
///////////////flow settings menu////////////
static const uiMenuDb_t uiMenuFlow =
{
    uiFlowDisplay,
    uiFlowNext,
    uiFlowPrev,
    {

          {uiFlowDec,          "-"},
          {uiFlowInc,          "+"},
          {uiFlowGroup2,       "13-24"},
          {uiFlowGroup3,       "25-36"},
          {uiFlowGroup4,       "37-48"},
          {uiFlowCopy,         "COPY"},
    }
};


static const uiMenuDb_t uiMenuFlowCopy =
{
    uiFlowCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiFlowCopyYes,        "YES"},
        {uiFlowCopyNo,          "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};



/* Groups Function Menu - summary */
static const uiMenuDb_t uiMenuGroupsSummary =
{
    uiGroupsSummaryDisplay,
    uiGroupsSummaryNext,
    uiGroupsSummaryPrev,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Groups Function Menu - values */
static const uiMenuDb_t uiMenuGroups =
{
    uiGroupsDisplay,
    uiGroupsNext,
    uiGroupsPrev,
    {
          {uiGroupsDec,           "-"},
          {uiGroupsInc,           "+"},
          {uiGroupsGroup2,        "13-24"},
          {uiGroupsGroup3,        "25-36"},
          {uiGroupsGroup4,        "37-48"},
          {NULL,                  NULL},
    }
};


/* Groups Function Menu - confirm removal of group leader */
static const uiMenuDb_t uiMenuGroupsUngroup =
{
    uiGroupsUngroupDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiGroupsUngroupYes,    "YES"},
        {uiGroupsUngroupNo,     "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Groups Function Menu - copy */
#if 0
static const uiMenuDb_t uiMenuGroupsCopy =
{
    uiGroupsCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiGroupsCopyYes,       "YES"},
        {uiGroupsCopyNo,        "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};
#endif

/* Climate Function Menu */
static const uiMenuDb_t uiMenuClimate =
{
    uiClimateDisplay,
    uiClimateNext,
    uiClimatePrev,
    {

          {uiClimateDec,          "-"},
          {uiClimateInc,          "+"},
          {uiClimateGroup2,       "13-24"},
          {uiClimateGroup3,       "25-36"},
          {uiClimateGroup4,       "37-48"},
          {uiClimateCopy,         "COPY"},
    }
};


/* Climate Function Menu - copy */
static const uiMenuDb_t uiMenuClimateCopy =
{
    uiClimateCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiClimateCopyYes,      "YES"},
        {uiClimateCopyNo,       "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Moisture Balance Values Menu */
static const uiMenuDb_t uiMenuMbVal =
{
    uiMbValDisplay,
    uiMbValNext,
    uiMbValPrev,
    {

          {uiMbValDec,            "-"},
          {uiMbValInc,            "+"},
          {uiMbValDecFast,        "--"},
          {uiMbValIncFast,        "++"},
          {uiMbValSetZero,        "ZERO"},
          {uiMbValCopy,           "COPY"},

    }
};


/* Moisture Balance Values Menu - copy */
static const uiMenuDb_t uiMenuMbValCopy =
{
    uiMbValCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiMbValCopyYes,        "YES"},
        {uiMbValCopyNo,         "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Plant Type Function Menu */
static const uiMenuDb_t uiMenuPlantType =
{
    uiPlantTypeDisplay,
    uiPlantTypeNext,
    uiPlantTypePrev,
    {
          {uiPlantTypeDec,        "-"},
          {uiPlantTypeInc,        "+"},
          {uiPlantTypeGroup2,     "13-24"},
          {uiPlantTypeGroup3,     "25-36"},
          {uiPlantTypeGroup4,     "37-48"},
          {uiPlantTypeCopy,       "COPY"},
    }
};


/* Plant Type Function Menu - copy */
static const uiMenuDb_t uiMenuPlantTypeCopy =
{
    uiPlantTypeCopyDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiPlantTypeCopyYes,    "YES"},
        {uiPlantTypeCopyNo,     "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Root Zone Working Water Storage (RZWWS) Values Menu */
static const uiMenuDb_t uiMenuRzwws =
{
    uiRzwwsDisplay,
    uiRzwwsNext,
    uiRzwwsPrev,
    {
        {uiRzwwsGroup1,         "1-12"},
        {uiRzwwsGroup2,         "13-24"},
        {uiRzwwsGroup3,         "25-36"},
        {uiRzwwsGroup4,         "37-48"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Advanced Function Menu - summary (reached via Function Key) */
static const uiMenuDb_t uiMenuAdvSummary =
{
    uiAdvSummaryDisplay,
    uiAdvSummaryNext,
    uiAdvSummaryPrev,
    {
        {uiUnlock,          "UNLOCK"},
        {uiLock,              "LOCK"},
        {uiAdvSummaryHidden0,   NULL},
        {uiAdvSummaryHidden0,   NULL},
        {uiAdvSummaryHidden0,   NULL},
        {uiAdvSummaryHidden1,   NULL},
    }
};


/* Advanced Function Menu - set factory defaults (reached from summary menu) */
static const uiMenuDb_t uiMenuAdvDefaults =
{
    uiAdvDefaultsDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAdvDefaultsYes,      "YES"},
        {uiAdvDefaultsNo,       "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Advanced Function Menu - product info screen 1 (follows summary menu) */
/* Note: User must press -Dbg1 followed by -Dbg2 to enter debug mode. */
static const uiMenuDb_t uiMenuAdvProduct1 =
{
    uiAdvProduct1Display,
    uiAdvProduct1Next,
    uiAdvProduct1Prev,
    {
        {uiAdvProductDbg0,      NULL},
        {uiAdvProductDbg2,      NULL},
        {uiAdvProductDbg0,      NULL},
        {uiAdvProductDbg0,      NULL},
        {uiAdvProductDbg1,      NULL},
        {uiAdvProductDbg0,      NULL},
    }
};


/* Advanced Function Menu - debug screen (reached from product info 1 menu) */
static const uiMenuDb_t uiMenuAdvProductDbg =
{
    uiAdvProductDbgDisplay,
    uiAdvProductDbgNext,
    uiAdvProductDbgPrev,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAdvProductDbgRadio,  "R-DBG"},
        {uiAdvProductDbgEvSave, "ESAVE"},
        {uiAdvProductDbgEvLog,  "EVIEW"},
        {uiAdvProductDbgReset,  "RESET"},
    }
};


/* Advanced Function Menu - debug event log screen (reached from debug menu) */
static const uiMenuDb_t uiMenuAdvEventLog =
{
    uiAdvEventLogDisplay,
    uiAdvEventLogNext,
    uiAdvEventLogPrev,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Advanced Function Menu - debug system reset (reached from debug menu) */
static const uiMenuDb_t uiMenuAdvReset =
{
    uiAdvResetDisplay,
    NULL,
    NULL,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAdvResetYes,         "YES"},
        {uiAdvResetNo,          "NO"},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Advanced Function Menu - product info screen 2 (follows info 1 menu) */
static const uiMenuDb_t uiMenuAdvProduct2 =
{
    uiAdvProduct2Display,
    uiAdvProduct2Next,
    uiAdvProduct2Prev,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
    }
};


/* Advanced Function Menu - radio status and config screen (follows info 2) */
static const uiMenuDb_t uiMenuAdvRadio =
{
    uiAdvRadioDisplay,
    uiAdvRadioNext,
    uiAdvRadioPrev,
    {
        {NULL,                  NULL},
        {uiAdvRadioHwReset,     "HW RST"},
        {NULL,                  NULL},
        {uiAdvRadioEdit,        "SET PAN"},
        {NULL,                  NULL},
        {uiAdvRadioReset,       "RESET"},
    }
};


/* Advanced Function Menu - radio status and config edit mode */
static const uiMenuDb_t uiMenuAdvRadioEdit =
{
    uiAdvRadioEditDisplay,
    uiAdvRadioEditNext,
    uiAdvRadioEditPrev,
    {
        {uiAdvRadioEditDec,     "-"},
        {uiAdvRadioEditInc,     "+"},
        {uiAdvRadioEditAny,     "ANY"},
        {uiAdvRadioEditOp,      "=OP"},
        {uiAdvRadioEditAccept,  "ACCEPT"},
        {uiAdvRadioEditCancel,  "CANCEL"},
    }
};


/* Advanced Function Menu - radio last msg and test screen */
static const uiMenuDb_t uiMenuAdvRadio2 =
{
    uiAdvRadio2Display,
    uiAdvRadio2Next,
    uiAdvRadio2Prev,
    {
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {NULL,                  NULL},
        {uiAdvRadio2Test,     "TEST"},
    }
};

/////// LOCK ////////////

static const uiMenuDb_t uiMenuLock =
{
    uiLockDisplay,
    uiLockDisplayNext,
    uiLockDisplayPrev,
    {
        {uiLockAdd,              "ADD"},
        {uiLockDel,              "DEL"},
        {NULL,                    NULL},
        {NULL,                    NULL},
        {NULL,                    NULL},
        {uiLockOK,              "LOCK"},
    }
};

static const uiMenuDb_t uiMenuLockOK =
{
    uiLockDisplayOK,
    NULL, 
    NULL,  
    {
        {NULL,              NULL},
        {NULL,              NULL},
        {NULL,              NULL},
        {NULL,              NULL},
        {NULL,              NULL},
        {NULL,              NULL},
    }
};

//////////// UNLOCK //////////
static const uiMenuDb_t uiMenuUnlock =
{
    uiUnlockDisplay,
    uiUnlockDisplayNext,
    uiUnlockDisplayPrev,
    {
        {uiUnlockAdd,              "ADD"},
        {uiUnlockDel,              "DEL"},
        {NULL,                      NULL},
        {NULL,                      NULL},
        {NULL,                      NULL},
        {uiUnlockOK,            "UNLOCK"},
    }
};

static const uiMenuDb_t uiMenuUnlockOK =
{
    uiUnlockDisplayOK,
    NULL,
    NULL,
    {
        {NULL,            NULL},
        {NULL,            NULL},
        {NULL,            NULL},
        {NULL,            NULL},
        {NULL,            NULL},
        {NULL,            NULL},
    }
};


/////////////////////////////////////////////////////

/******************************************************************************
 *
 * uiInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization routine
 *      to initialize the user interface subsystem.
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
void uiInit(void)
{
    /* Initialize the current menu state. */
    uiMenuSet(&uiMenuStatusIdle);
    uiScreenTimeout = UI_SCREEN_TIMEOUT;
    /* Set current function to Controller Status (home) menu. */
    drvLedFunc(UI_FUNC_HOME);
}


/******************************************************************************
 *
 * uiPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to
 *      manage the user interface logic.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine should be called frequently to provide a smooth user
 *      experience while interacting with front panel switches and display.
 *
 *****************************************************************************/
/* UI Poll Function */

void uiPoll(void)
{
    uint8_t newEvent;

    while ((newEvent = drvKeypadGet()) != 0)
    {
        uiLastEventTime = dtTickCount;
        switch (newEvent & DRV_KEYPAD_TYPE_MASK)
        {
            case DRV_KEYPAD_TYPE_FUNC:
                uiEventFunction(newEvent & DRV_KEYPAD_KEY_MASK, FALSE);
                break;
            case DRV_KEYPAD_TYPE_ALTFUNC:
                uiEventFunction(newEvent & DRV_KEYPAD_KEY_MASK, TRUE);
                break;
            case DRV_KEYPAD_TYPE_SOFT:
            case DRV_KEYPAD_TYPE_SOFT_SLOW:
            case DRV_KEYPAD_TYPE_SOFT_FAST:
                uiEventSoft(newEvent & DRV_KEYPAD_TYPE_MASK,
                            newEvent & DRV_KEYPAD_KEY_MASK);
                break;
            case DRV_KEYPAD_TYPE_NAV:
                uiEventDial(newEvent & DRV_KEYPAD_KEY_MASK);
                break;
            default:
                /* unknown event type - discard and ignore */
                break;
        }
    }

    /*
    **  Perform the following actions every second:
    **  - Refresh LCD screen display.
    **  - Check for UI inactivity timeout.
    */
    if ((uiLcdLastUpdateSec != dtSec) ||
        (uiLcdLastUpdateMin != dtMin))          /* for Win32 speed-up x60 */
    {
        /* Set flag to generate LCD refresh at the top of each second. */
        uiLcdRefreshTick = TRUE;

        /* Check for UI timeout. */
        if (dtElapsedSeconds(uiLastEventTime) > uiScreenTimeout)
        {
            /* Timeout on display.  Return to Controller Status screen. */
            uiStatusShow();
            /* Restore UI timeout to default value. */
            uiScreenTimeout = UI_SCREEN_TIMEOUT;
        }
    }

    /* Update LCD if either LCD refresh flag is set. */
    if (uiLcdRefreshReq || uiLcdRefreshTick)
    {
        /* Update the LCD screen display. */
        uiLcdUpdate();
    }
}


/******************************************************************************
 *
 * uiStatusShow
 *
 * PURPOSE
 *      This routine is called to force the system to show the main Controller
 *      Status screen.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void uiStatusShow(void)
{
    /* Only process if a main status screen is not already being displayed. */
    if ((uiCurMenu != &uiMenuStatusIdle) &&
        (uiCurMenu != &uiMenuStatusActive) &&
        (uiCurMenu != &uiMenuStatusPaused))
    {
        uiEventFunction(UI_FUNC_HOME, FALSE);
    }
}


/******************************************************************************
 *
 * uiEventFunction
 *
 * PURPOSE
 *      This routine is called to process function switch events.
 *
 * PARAMETERS
 *      position    IN  position of function button pressed (0-15)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiEventFunction(uint8_t position, bool_t altFunc)
{
    if (position == UI_FUNC_RFU2)
    {
        /*
        **  Toggle hardware clock speed-up feature for software test.
        **  Note: This switch position is not populated on production units.
        */
        uiRtcIsFast = !uiRtcIsFast;
        drvRtcAccel(uiRtcIsFast);
        drvLedDebug(uiRtcIsFast);
        return;
    }

    /* Initialize Menu State Variables */
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 0;
    uiPosition = position;

    /* Initialize Menu Temporary Variables */
    uiTemp8 = 0;
#ifdef RADIO_ZB
    uiTemp64 = 0;
#else
    uiTemp16 = 0;
    uiTemp162=0;
#endif


    /* if the user changed the time then send message to expansion units */
    if((timeChanged == TRUE )
          &(config.sys.unitType == UNIT_TYPE_MASTER)
          &(config.sys.numUnits >0))
    {
        timeChanged = FALSE;
        expansionBusSendCmd(RADIO_CMD_INIT_DATETIME, RADIO_EXP_SEND_ALL); 
    }
    
    
    /* Turn on the appropriate function LED. */
    drvLedFunc(position);

    /* Select the appropriate Function Menu */
    switch (position)
    {
        case UI_FUNC_HOME:                  /* func switch = CONTROLLER STATUS */
            /* Set Controller Status main menu. */
            uiMenuSet(&uiMenuStatusIdle);
            /* Initialize line 3 status message display cycle. */
            uiStatusReps = 0;
            uiStatusCycle = 0;
            break;

        case UI_FUNC_SETDT:                 /* func switch = SET DATE/TIME */
            uiMenuSet(&uiMenuDt);
            break;

        case UI_FUNC_SETUP:                 /* func switch = CONTROLLER SETUP */
            uiMenuSet(&uiMenuSetup);
            break;

        case UI_FUNC_OPMODE:                /* func switch = MODE OF OPERATION */
            uiMenuSet(&uiMenuOpMode);
            break;

        case UI_FUNC_IRRDAYS:               /* func switch = IRRIGATION DAYS */
            uiMenuSet(&uiMenuIrrDaysSummary);
            break;

        case UI_FUNC_RUNTIME:               /* func switch = RUN TIMES */
            uiMenuSet(&uiMenuRunTimeSummary);
            break;

        case UI_FUNC_APPRATE:               /* func switch = APPLICATION RATE */
            uiMenuSet(&uiMenuAppRateSummary);
            break;

        case UI_FUNC_SOILTYPE:              /* func switch = SOIL TYPE */
            uiMenuSet(&uiMenuSoilType);
            break;

        case UI_FUNC_SLOPE:                 /* func switch = SLOPE */
            uiMenuSet(&uiMenuSlope);
            break;

        case UI_FUNC_MOIST:                 /* func switch = SETPOINTS */
            if (altFunc)
            {
                /* Show moisture sensor values screen. */
                uiMenuSet(&uiMenuMoistVal);
            }
            else
            {
                /* Show moisture sensor setpoints screen */
                uiMenuSet(&uiMenuMoist);
            }
            break;

        case UI_FUNC_GROUPS:                /* func switch = GROUPS */
            uiMenuSet(&uiMenuGroupsSummary);
            break;

        case UI_FUNC_CLIMATE:               /* func switch = CLIMATE */
            if (altFunc)
            {
                /* Show MB values screen. */
                uiMenuSet(&uiMenuMbVal);
            }
            else
            {
                /* Show climate screen. */
                uiMenuSet(&uiMenuClimate);
            }
            break;

        case UI_FUNC_PLANTTYPES:            /* func switch = PLANT TYPES */
            if (altFunc)
            {
                /* Show RZWWS values screen. */
                uiMenuSet(&uiMenuRzwws);
            }
            else
            {
                /* Show plant types screen */
                uiMenuSet(&uiMenuPlantType);
            }
            break;

        case UI_FUNC_ADVANCED:              /* func switch = ADVANCED */
            uiMenuSet(&uiMenuAdvSummary);
            break;

        default:
            uiMenuSet(&uiMenuUnimplemented);
            break;
    }

    /* Request LCD screen refresh to display selected Function Menu. */
    uiLcdRefresh();
}


/******************************************************************************
 *
 * uiEventDial
 *
 * PURPOSE
 *      This routine is called to process navigation dial events.
 *
 * PARAMETERS
 *      direction   IN  direction of dial rotation (0=cw, 1=ccw)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiEventDial(uint8_t direction)
{
    /* Execute the appropriate Navigation Dial Menu Action routine. */
    switch (direction)
    {
        case 0:                         /* Clockwise Dial Rotation */
            if (uiCurMenu->pDialCw != NULL)
            {
                uiCurMenu->pDialCw();
            }
            break;
        case 1:                         /* Counter-Clockwise Dial Rotation */
            if (uiCurMenu->pDialCcw != NULL)
            {
                uiCurMenu->pDialCcw();
            }
            break;
        default:
            /* should never reach */
            break;
    }

    /* Request LCD screen refresh. */
    uiLcdRefresh();
}


/******************************************************************************
 *
 * uiEventSoft
 *
 * PURPOSE
 *      This routine is called to process soft key events.
 *
 * PARAMETERS
 *      eventType   IN  event type (press, press&hold slow, press&hold fast)
 *      eventKey    IN  soft-key number (0-5)
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiEventSoft(uint8_t eventType, uint8_t eventKey)
{
    /* Don't do anything if the key is masked or function pointer is null. */
    if ((uiKeyMask & (1 << eventKey)) == 0 &&
        uiCurMenu->softKey[eventKey].pAction != NULL)
    {
        /* Don't process if out-of-range. */
        if (eventKey < UI_N_SOFTKEYS)
        {
            /* Execute the Menu Action routine for this soft key event. */
            uiCurMenu->softKey[eventKey].pAction(eventType, eventKey);

            /* Request LCD screen refresh. */
            uiLcdRefresh();
        }
    }
}


/******************************************************************************
 *
 * uiMenuSet
 *
 * PURPOSE
 *      This routine is called to change the current menu.
 *
 * PARAMETERS
 *      menu    IN  specifies the desired menu
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiMenuSet(const uiMenuDb_t *menu)
{
    /* Set the current menu. */
    uiCurMenu = menu;
}


/******************************************************************************
 *
 * uiLcdRefresh
 *
 * PURPOSE
 *      This routine is called to request refresh of the LCD display on the
 *      next ui poll cycle.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void uiLcdRefresh(void)
{
    /* Set refresh request event flag. */
    uiLcdRefreshReq = TRUE;
}


/******************************************************************************
 *
 * uiLcdUpdate
 *
 * PURPOSE
 *      This routine is called to update the LCD display.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine initializes the LCD display buffer to all spaces and
 *      sets the soft-key mask to enable all soft-key positions; then it
 *      calls the current menu's display action routine to update the LCD
 *      buffer and soft-key mask as appropriate for the current menu.  After
 *      the display action routine returns, if the menu has soft-keys defined
 *      this routine generates the soft-key labels on line 4 of the display
 *      buffer according to the current value of the soft-key mask.  The
 *      completed LCD display buffer is then passed to the LCD hardware
 *      driver to be written to the LCD display.
 *
 *****************************************************************************/
static void uiLcdUpdate(void)
{
#ifdef WIN32
    uint8_t leds = 0;
#endif

    /* Exit if menu pointer is null. */
    if (uiCurMenu == NULL)
    {
        return;
    }

    /* Store current second & minute values. */
    uiLcdLastUpdateSec = dtSec;
    uiLcdLastUpdateMin = dtMin;

    /* Clear LCD display buffer to all space characters. */
    memset(uiLcdBuf, ' ', sizeof(uiLcdBuf));

    /*
    **  Initialize the soft key mask default to allow all soft keys.
    **  Note: The key mask may only be changed in menu display action routines.
    */
    uiKeyMask = UI_KM_NONE;

    /* Initialize the LCD cursor to be disabled. */
    uiLcdCursor = LCD_CURSOR_OFF;

    /* Call the menu display action routine. */
    uiCurMenu->pDisplay();

#if UI_N_SOFTKEYS != 6
#warning "Code assumes six soft-keys."
#endif
    if (uiCurMenu->softKey[0].pAction != NULL ||
        uiCurMenu->softKey[1].pAction != NULL ||
        uiCurMenu->softKey[2].pAction != NULL ||
        uiCurMenu->softKey[3].pAction != NULL ||
        uiCurMenu->softKey[4].pAction != NULL ||
        uiCurMenu->softKey[5].pAction != NULL)
    {
        /* Clear soft key label line. */
        memset(&uiLcdBuf[3 * 40], ' ', 40);

        /*
        **  Compute centered position for each soft key label and insert text.
        **  The layout was designed for up to five character labels and two
        **  blanks between each label.  (6*5 + 5*2) exactly fits in 40 columns!)
        **      "SOFT1  SOFT2  SOFT3  SOFT4  SOFT5  SOFT6"
        **  Longer soft key labels are allowed, but their use is discouraged
        **  because they consume the separating whitespace.
        **      "LONGER SOFT2 LONGEST SOFT4  SOFT5  SOFT6"
        */
        for (int i = 0; i < UI_N_SOFTKEYS; i++)
        {
            if (uiCurMenu->softKey[i].pLabel != NULL && (uiKeyMask & (1 << i)) == 0)
            {
                int len = (int)strlen(uiCurMenu->softKey[i].pLabel);
                int pos = (i * 7) + 2 - (len / 2);

                if (len > 40)
                {
                    /* Oops - label longer than display - skip it! */
                    continue;
                }

                if (pos < 0)
                {
                    /* Starts beyond left edge - change to start at edge. */
                    pos = 0;
                }
                else if (pos + len >= 40)
                {
                    /* Ends beyond right edge - change to end at edge. */
                    pos -= pos + len - 40;
                }
                memcpy(&uiLcdBuf[3 * 40 + pos], uiCurMenu->softKey[i].pLabel, len);
            }
        }

#ifdef WIN32
        /* Update soft key availability "leds" mask (for win32 prototype). */
        for (int i = 0; i < 6; i++)
        {
            if (uiCurMenu->softKey[i].pAction != NULL)
            {
                leds |= (1 << i);
            }
        }
        if (uiCurMenu->pDialCw != NULL || uiCurMenu->pDialCcw != NULL)
        {
            leds |= (1 << UI_N_SOFTKEYS);
        }
        leds &= ~uiKeyMask;
#endif
    }

    /* Replace any nulls with spaces. */
    for (int i = 0; i < sizeof(uiLcdBuf); i++)
    {
        if (uiLcdBuf[i] == '\0')
        {
            uiLcdBuf[i] = ' ';
        }
    }

    /* Write LCD buffer to hardware device driver. */
    drvLcdWrite(uiLcdBuf, uiLcdCursor);

    /* Clear LCD refresh flags. */
    uiLcdRefreshReq = FALSE;
    uiLcdRefreshTick = FALSE;

#ifdef WIN32
    /* Write soft key availability mask (for win32 prototype). */
    ledSet(leds);
#endif
}


/******************************************************************************
 *
 * uiFirstFieldFix
 *
 * PURPOSE
 *      This routine is called to fix the uiFirstField value for the specified
 *      number of display columns.
 *
 * PARAMETERS
 *      nColumns    IN  specifies the number of display columns
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiFirstFieldFix(uint8_t nColumns)
{
    /* fix uiFirstField if it is out of range */
    if (uiField < uiFirstField)
    {
        uiFirstField = (uiField / nColumns) * nColumns;
    }
    else if (uiField >= uiFirstField + (nColumns * 3))
    {
        uiFirstField = ((uiField / nColumns) * nColumns) - (nColumns * (3 - 1));
    }
}

/******************************************************************************
 *
 * uiFirstFieldFix2
 *
 * PURPOSE
 *      This routine is called to fix the uiFirstField value for the specified
 *      number of display columns.
 *
 * PARAMETERS
 *      nColumns    IN  specifies the number of display columns
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
static void uiFirstFieldFix2(uint8_t nColumns)
{
    /* fix uiFirstField if it is out of range */
    if (uiField < uiFirstField)
    {
        uiFirstField = (uiField / nColumns) * nColumns;
    }
    else if (uiField >= uiFirstField + (nColumns * 2))
    {
        uiFirstField = ((uiField / nColumns) * nColumns) - (nColumns * (2 - 1));
    }
}


/******************************************************************************
 *
 * uiFormatIrrMode
 *
 * PURPOSE
 *      This routine returns a string, identifying the current irrigation
 *      program and operating mode.  The irrigation mode string has a maximum
 *      size of 24 characters (not including the null character used for string
 *      termination).  Examples of strings generated by this routine are:
 *
 *          Program A, Runtime
 *          Program B, Sensor
 *          Program C, Weather/Pulse
 *
 * PARAMETERS
 *      buf     IN  buffer for return string (25 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the irrigation mode string.
 *
 *****************************************************************************/
static char *uiFormatIrrMode(char *buf)
{
    char *opMode;
    uint8_t program;

    switch (irrOpMode)
    {
        case CONFIG_OPMODE_RUNTIME:
            opMode = "Runtime";
            break;
        case CONFIG_OPMODE_SENSOR:
            opMode = "Sensor";
            break;
        case CONFIG_OPMODE_WEATHER:
            opMode = "Weather";
            break;
        default:
            /* should never reach */
            opMode = "?";
            break;
    }


    if((config.sys.unitType == UNIT_TYPE_MASTER) & (irrExpRunningProg != IRR_PGM_NONE))
    {
       program = irrExpRunningProg;
    } 
    else 
    {
        program = irrProgram;
    }
    
    sprintf(buf, "Program %c, %s%s",
        program + 'A',
        opMode,
        irrPulseMode == CONFIG_PULSEMODE_ON ? "/Pulse" : "");

    return buf;
}


/******************************************************************************
 *
 * uiFormatSystemMode
 *
 * PURPOSE
 *      This routine returns a formatted system mode string, describing the
 *      current WOIS operating status.  The system mode string is designed
 *      to fit on a 40 character LCD screen line.  Examples of system mode
 *      strings include the following.
 *
 *          Irrigation is turned OFF
 *          Irrigation is ON/AUTOMATIC
 *          TEST MODE  (1 minute/zone)
 *          MANUAL START  (Program A, Runtime)
 *          MANUAL START  (Program A, Sensor)
 *          AUTO STARTED  (Program A, Weather/Pulse)
 *          FORCE ON  (Program A, Runtime)
 *
 * PARAMETERS
 *      buf     IN  buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted system mode
 *      string.
 *
 *****************************************************************************/
char *uiFormatSystemMode(char *buf)
{
    char tmpbuf[41];

    switch (sysState)
    {
        case SYS_STATE_IDLE:
            if (!sysIsAuto)
            {
                sprintf(buf, "Irrigation is turned OFF");
            }
            else
            {
                sprintf(buf, "Irrigation is ON/AUTOMATIC");
            }
            break;

        case SYS_STATE_AUTORUN:
            sprintf(buf,
                    "AUTO STARTED  (%s)",
                    uiFormatIrrMode(tmpbuf));
            break;

        case SYS_STATE_MANUAL:
            sprintf(buf,
                    "MANUAL START  (%s)",
                    uiFormatIrrMode(tmpbuf));
            break;

        case SYS_STATE_FORCE:
            sprintf(buf,
                    "FORCE ON  (%s)",
                    uiFormatIrrMode(tmpbuf));
            break;

        case SYS_STATE_TEST:
            sprintf(buf,
                    "TEST MODE  (%d minute%s/zone)",
                    sysTestRunTime,
                    sysTestRunTime > 1 ? "s" : "");
            break;
        default:
            /* should never reach */
            sprintf(buf, "?");
            break;
    }

    return buf;
}


/******************************************************************************
 *
 * uiFormatSystemStatus
 *
 * PURPOSE
 *      This routine returns a formatted status string for display of current
 *      system status (displayed on Line 3 of Controller Status function menu).
 *
 * PARAMETERS
 *      buf     IN  buffer for return string (41 chars)
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted system status
 *      string.
 *
 * NOTES
 *      Some Line 3 status messages indicate System Error and System Fault
 *      conditions and typically use constant strings.  For example:
 *
 *          SYS_ERROR_DATETIME:     "Clock is not set. Please Set Date/Time."
 *          SYS_ERROR_NOPROGRAM:    "No runnable program is scheduled."
 *
 *      Other Line 3 status messages indicate irrigation program progress
 *      and typically include variable information that is formatted using
 *      current system and irrigation state information.  For example:
 *
 *          UI_SMSG_TOT_WATER:      "Total watering remaining 01:44:29."
 *          UI_SMSG_SENSOR:         "Moisture is 21%, target of 35%."
 *
 *****************************************************************************/
static char *uiFormatSystemStatus(char *buf)
{
    int count = 0;
    int i;
    char messageId[UI_SMSG_LIMIT];
    char tmpbuf[41];
    int32_t remainingSecs;
    uint16_t remainingPulses;
    int8_t moisture = 0;
    uint8_t failedMoistZone = 0;

    /*
    **  Build a list of all active status messsages to determine the
    **  total number of status messages available to cycle through.
    */

    /* List all active System Faults first. */
    for (i = 0; i < SYS_FAULT_LIMIT; i++)
    {
        if ((sysFaultFlags & (1 << i)) != 0)
        {
            /* Exclude moisture sensor fault check. */
            if (i != SYS_FAULT_MOIST)
            {
                messageId[count] = UI_SMSG_FAULT + i;
                count++;
            }
        }
    }

    /* If moisture sensor fault, list the first failed sensor. */
    failedMoistZone = moistFailureZoneGet();
    if (failedMoistZone != 0)
    {
            messageId[count] = UI_SMSG_SENSORFAIL;
            count++;
            
            switch(config.sys.unitType)
            {
                case UNIT_TYPE_EXPANSION_1:
                    failedMoistZone +=12;
                    break;
                case UNIT_TYPE_EXPANSION_2:
                    failedMoistZone +=24;
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    failedMoistZone +=36;
                    break;
            }
    }

    /* List all active System Errors. */
    for (i = 0; i < SYS_ERROR_LIMIT; i++)
    {
        if ((sysErrorFlags & (1 << i)) != 0)
        {
            messageId[count] = UI_SMSG_ERROR + i;
            count++;
        }
    }

    /* List offline radio status. */
    if (radioStatus == RADIO_STATUS_OFFLINE)
    {
        messageId[count] = UI_SMSG_RADIO_OFFLINE;
        count++;
    }


    /* List system date/time when automatic mode is enabled. */
    if (sysIsAuto)
    {
        messageId[count] = UI_SMSG_DATE_TIME;
        count++;
    }

    /* List automatic mode disabled exception, if system not idle. */
    if ((sysState != SYS_STATE_IDLE) && !sysIsAuto)
    {
        messageId[count] = UI_SMSG_AUTO_OFF;
        count++;
    }

    /* List irrigation progress messages, if system not idle. */
    switch (sysState)
    {
        case SYS_STATE_IDLE:
            break;
        case SYS_STATE_AUTORUN:
        case SYS_STATE_MANUAL:
            if ((irrOpMode == CONFIG_OPMODE_SENSOR) &&
                irrIsCurrentGroupLeader(irrCurZone))
            {
                messageId[count] = UI_SMSG_SENSOR;
                count++;
            }
            if ((irrPulseMode == CONFIG_PULSEMODE_ON) &&
                (irrState == IRR_STATE_WATERING) &&
                (irrCurZone != 0))
            {
                remainingSecs = irrRemainingZonePulseSecs(irrCurZone);
                remainingPulses = irrRemainingZonePulses(irrCurZone) - 1;
                /* Limit remaining pulses display to 3 digits. */
                remainingPulses = (remainingPulses > 999) ? 999 : remainingPulses;
                if ((remainingPulses > 0)&(expansionIrrState == IRR_STATE_IDLE))
                {
                    messageId[count] = UI_SMSG_PULSE;
                    count++;
                }
            }
            if ((irrState != IRR_STATE_SENSING)&(expansionIrrState == IRR_STATE_IDLE)
                & (irrState != IRR_STATE_IDLE))
            {
                messageId[count] = UI_SMSG_TOT_WATER;
                count++;
            }
            
            if ((irrState == IRR_STATE_WATERING) || (expansionIrrState == IRR_STATE_WATERING) )
            {
                messageId[count] = UI_SMSG_FLOW;
                count++;
            }
            
            break;
        case SYS_STATE_FORCE:
            if(expansionIrrState == IRR_STATE_IDLE)
            {              
                messageId[count] = UI_SMSG_TOT_WATER;
                count++;
            }
            if ((irrState == IRR_STATE_WATERING)||(expansionIrrState == IRR_STATE_WATERING))
            {              
                messageId[count] = UI_SMSG_FLOW;
                count++;
            }
            
            break;
        case SYS_STATE_TEST:
            if(expansionIrrState == IRR_STATE_IDLE)
            {    
                messageId[count] = UI_SMSG_TOT_WATER;
                count++;
            }
            
            if ((irrState == IRR_STATE_WATERING)||(expansionIrrState == IRR_STATE_WATERING))
            {    
                messageId[count] = UI_SMSG_FLOW;
                count++;
            }
            break;
        default:
            break;
    }

    /*
    **  Correct cycle index for the current status message if out of range
    **  due to overshoot/undershoot caused by nav dial actions or
    **  poll function timed auto cycle advancement.
    */
    if (uiStatusCycle >= count)
    {
        uiStatusCycle = 0;
    }
    else if (uiStatusCycle < 0)
    {
        uiStatusCycle = count - 1;
    }

    /* Generate the status message to display for the current message cycle. */
    if (count == 0)
    {
        sprintf(buf, "");
    }
    else if (messageId[uiStatusCycle] < UI_SMSG_ERROR_LIMIT)
    {
        sprintf(buf, "%.40s", uiSystemErrorMsg[messageId[uiStatusCycle] - UI_SMSG_ERROR]);
    }
    else if (messageId[uiStatusCycle] < UI_SMSG_FAULT_LIMIT)
    {
        if((messageId[uiStatusCycle] - UI_SMSG_FAULT) == SYS_FAULT_SNSCON)
        {
            sprintf(buf, "%.24s %08X%08X", uiSystemFaultMsg[messageId[uiStatusCycle] - UI_SMSG_FAULT],
                    (uint32_t)(config.sys.assocSensorCon[irrSnsConSolUnitIndex].macId >> 32),
                    (uint32_t)(config.sys.assocSensorCon[irrSnsConSolUnitIndex].macId & 0xFFFFFFFF));
        }
        else
        {
            sprintf(buf, "%.40s", uiSystemFaultMsg[messageId[uiStatusCycle] - UI_SMSG_FAULT]);
        }
    }
    else
    {
        switch (messageId[uiStatusCycle])
        {
            case UI_SMSG_DATE_TIME:
                dtFormatCurrentTimeDate(buf);
                break;

            case UI_SMSG_SENSOR:
                moisture = irrZoneMoisture(irrCurZone);
                if (moisture >= 0)
                {
                    sprintf(buf, "Moisture is %d%%, target of %d%%.",
                        moisture,
                        irrZone[irrCurZone - 1].maxMoist);
                }
                else
                {
                    sprintf(buf, "Moisture reading is not available.");
                }
                break;

            case UI_SMSG_SENSORFAIL:
                    sprintf(buf, "Moisture sensor failure in zone %d.",
                        failedMoistZone);
                break;

            case UI_SMSG_PULSE:
                printf(buf, "Pulse ends in %s, %d remaining.",
                dtFormatRunTimeSecs(tmpbuf, remainingSecs),
                remainingPulses);
                break;

            case UI_SMSG_TOT_WATER:
 
                remainingSecs = irrRemainingProgramSecs();
                sprintf(buf, "Total watering remaining %s.",
                dtFormatRunTimeSecs(tmpbuf, remainingSecs));
                break;
                
            case UI_SMSG_FLOW:
             
                if ( findFlow == 1 ) {
                    
                if ( expansionIrrState == IRR_STATE_WATERING )
                    sprintf(buf,
                            "Flow is %d GPM, Target of %d-%d GPM",
                            GPM,
                            config.zone[expansionIrrCurZone-1].minGPM,
                            config.zone[expansionIrrCurZone-1].maxGPM
                            ); 
                 else
                    sprintf(buf,
                            "Flow is %d GPM, Target of %d-%d GPM",
                            GPM,
                            config.zone[irrCurZone-1].minGPM,
                            config.zone[irrCurZone-1].maxGPM
                            ); 
                }
                break;    

            case UI_SMSG_AUTO_OFF:
                sprintf(buf, "Automatic irrigation is set to 'OFF'.");
                break;

            case UI_SMSG_RADIO_OFFLINE:
                sprintf(buf, "Radio is offline.");
                break;

            default:
                sprintf(buf, "???");
                break;
        }
    }

    return buf;
}


/******************************************************************************
 *
 * uiFormatRadioSignalBars
 *
 * PURPOSE
 *      This routine returns a formatted string for the radio signal strength
 *      bar-graph display representing the specified value.
 *
 * PARAMETERS
 *      radioStatusDb   IN  signal strength dBm value to format as bars
 *
 * RETURN VALUE
 *      This routine returns a string pointer to the formatted bar-graph
 *      display string.
 *
 *****************************************************************************/
static const char *uiFormatRadioSignalBars(uint8_t radioStatusDb)
{
    const char *pText = "";

    for (int i = 1;
         i < (sizeof(uiRadioSignalBars) / sizeof(uiRadioSignalBars[0]));
         i++)
    {
        if (radioStatusDb >= uiRadioSignalBars[i].threshold)
        {
            pText = uiRadioSignalBars[i].pText;
            break;
        }
    }

    return pText;
}



/******************************************************************************
 ******************************************************************************
 ******************************************************************************
 ***
 ***        MENU ACTION ROUTINES - GROUPED BY FUNCTION
 ***
 ******************************************************************************
 ******************************************************************************
 *****************************************************************************/



/******************************************************************************
 *
 *  UNIMPLEMENTED FUNCTION MENU
 *
 *  Note: This is the default menu template used when there is no valid menu
 *        defined for a function.  It was created during development as a
 *        temporary place-holder for new function key support.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiUnimplementedDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "WaterOptimizer");
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Function not implemented yet.");
}



/******************************************************************************
 *
 *  CONTROLLER STATUS FUNCTION MENU ACTION ROUTINES
 *
 *  Note: The Controller Status function is the default (home) function.
 *        This menu provides essential system status information as well as
 *        system state-appropriate soft-key functions for irrigation control.
 *        The WOIS front panel automatically returns to the Controller Status
 *        function main menu after 5 minutes of user inactivity.
 *        There are 3 menus defined to use the same main menu display action
 *        routine; the 3 main menu templates differ only in the set of soft
 *        keys they provide, depending on the system state (idle, watering or
 *        soaking, interupted).  Several subordinate menus are also used for
 *        soft-key commands such as Test, Manual Start, and confirmation of
 *        Resume requests.
 *
 *****************************************************************************/

/* Main Menu Display Action Routine */
static void uiStatusDisplay(void)
{
    char buf[41];
    //char buf11[41];
    uint8_t zoneNumber;
    

    /* Handle tick refresh event. */
    if (uiLcdRefreshTick)
    {
        /* Update line 3 status message reps; advance status message cycle. */
        if (++uiStatusReps >= UI_STATUS_N_REPS)
        {
            uiStatusReps = 0;
            ++uiStatusCycle;
        }
    }

    /* Write LCD line 1 (system mode) */
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        uiFormatSystemMode(buf));
    //sprintf(&uiLcdBuf[LCD_RC(0, 0)],
    //     "T:%3d,CT:%3d,CI:%d,SKIP:%d",
    //     dtTickCount,radioSnsConCheckinTime,scCheckedIn,TestSkipFlag);    

    /* Write LCD line 2 (irrigation status) */
    switch (sysState)
    {
        case SYS_STATE_IDLE:
            uiMenuSet(&uiMenuStatusIdle);
            uiKeyMask = UI_KM_KEY5 | UI_KM_KEY6;
            if (sysIsInhibited)
            {
                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "(Inhibited)");
            }
            break;

        case SYS_STATE_AUTORUN:
        case SYS_STATE_MANUAL:
        case SYS_STATE_FORCE:
        case SYS_STATE_TEST:
            uiMenuSet(&uiMenuStatusActive);
            if ((irrState == IRR_STATE_WATERING)|(expansionIrrState == IRR_STATE_WATERING))
            {
                uiKeyMask = UI_KM_NONE;
            }
            else
            {
                /* Mask off the SKIP key if not watering. */
                //uiKeyMask = UI_KM_KEY6;
            }
            if (sysIsInhibited)
            {
                uiMenuSet(&uiMenuStatusPaused);
                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "WATERING INTERRUPTED  (Inhibited)");
            }
            else if (sysIsPaused)
            {
                uiMenuSet(&uiMenuStatusPaused);
                if(irrCurZone != 0)
                {
                    if((config.zone[irrCurZone-1].sensorType == SNS_WIRELESS_MOIST) ||
                        ((irrSnsConSolUnitIndex!= IRR_SNS_SOL_NONE) && 
                         (config.zone[irrCurZone-1].sensorType != SNS_WIRELESS_MOIST))) 
                    {
                        //KAV
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WATERING ZONE %d PAUSED FOR SC %04X",irrCurZone,(uint16_t)(config.sys.assocSensorCon[config.zone[irrCurZone-1].snsConTableIndex].macId & 0xFFFF) );
                        break;
                    }
                }

                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "WATERING INTERRUPTED  (Paused)");

            }
            else if ((irrState == IRR_STATE_SOAKING)|(expansionIrrState == IRR_STATE_SOAKING))
            {
                if(expansionIrrState == IRR_STATE_SOAKING)
                {
                    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "SOAKING");
                }
                else
                {
                    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "SOAKING  (%s remaining)",
                        dtFormatRunTimeSecs(buf, irrRemainingSoakSecs())
                        );
                }
            }
            else if ((irrState == IRR_STATE_SENSING)|(expansionIrrState == IRR_STATE_SENSING))
            {
                uiKeyMask = UI_KM_ALL;
                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "SENSING . . .");
            }
            else
            {
                
                switch(config.sys.unitType)
                {
                  case UNIT_TYPE_MASTER:
                      zoneNumber = irrCurZone;
                      break;
                  case UNIT_TYPE_EXPANSION_1:
                      zoneNumber = 12+irrCurZone;
                      break;
                  case UNIT_TYPE_EXPANSION_2:
                      zoneNumber = 24+irrCurZone;
                      break;
                  case UNIT_TYPE_EXPANSION_3:
                      zoneNumber = 36+irrCurZone;
                      break;
                }

                if((expansionIrrState == IRR_STATE_WATERING))
                {                    
                    if(expansionIrrCurZone == 0)
                    {                    
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WAITING FOR ZONE TO START");
                    }
                    else 
                    {   
                        if( (config.sys.unitType == UNIT_TYPE_MASTER) ) //&& (findFlow == 1))
                        {
                            moistSample(flowIndex);
                            GPM = moistValueGet(flowIndex);
                            expansionBusSendCmd(RADIO_CMD_SEND_FLOW, RADIO_EXP_SEND_ALL); 
                        }
                        
                        
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WATERING EXP ZONE %d  ",
                             expansionIrrCurZone);                        
                                                   
                        
                        /*                         
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WATERING EXP ZONE %d : %d %d %s %d",
                            expansionIrrCurZone,
                            flowDelay,
                            findFlow,
                            "Flow:",
                            GPM
                            );      
                         */    
                    }
                }
                else
                {
                  
                    if(zoneNumber == 0)
                    {
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WAITING FOR ZONE TO START");
                    }
                    else 
                    {   
                        
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WATERING ZONE %d  (%s remaining)",
                            zoneNumber,
                            dtFormatRunTimeSecs(buf, irrRemainingZoneSecs(irrCurZone))
                            );
                             
                        /*    
                        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "sys:%d,irr:%d,WATERING ZONE %d  (%s)",
                            sysState,irrState,zoneNumber,
                            dtFormatRunTimeSecs(buf, irrRemainingZoneSecs(irrCurZone))
                            );
                         */
                                                        
                          /*
                          sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                            "WATERING ZONE %d (%s) %d %d %s %d",
                            zoneNumber,
                            dtFormatRunTimeSecs(buf, irrRemainingZoneSecs(irrCurZone)),
                            flowDelay,
                            findFlow,
                            "Flow:",
                            GPM
                            ); 
                            */
                    }
                }
            }                            
            break;

        default:
            /* never should reach */
            break;
    }
    
    if( lock==TRUE ) 
    {
        
        uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY2 | UI_KM_KEY3 | UI_KM_KEY4);
    }
    
    /* Write LCD line 3 (Date/Time or System Error/Fault Status). */
    sprintf(&uiLcdBuf[LCD_RC(2, 0)], "%s",
           uiFormatSystemStatus(buf));
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiStatusNext(void)
{
    /* Show the next line3 status message. */
    uiStatusReps = 0;
    uiStatusCycle++;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiStatusPrev(void)
{
    /* Show the previous line3 status message. */
    uiStatusReps = 0;
    uiStatusCycle--;
}


/* Soft Key Action Routine - OFF */
static void uiStatusOff(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Clear any pending auto programs. */
    irrAutoPgmPending = IRR_PGM_NONE;

    /* Stop any irrigation. */
    irrCmdStop();

    /* Set the system Off. */
    sysIsAuto = FALSE;
    
    /* send command to other units */
    expansionBusSendCmd(RADIO_CMD_IRR_OFF, RADIO_EXP_SEND_ALL); 
}


/* Soft Key Action Routine - AUTO */
static void uiStatusAuto(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Set the system state to Auto. */
    sysIsAuto = TRUE;
    
    /* send command to other units */
    expansionBusSendCmd(RADIO_CMD_IRR_AUTO, RADIO_EXP_SEND_ALL); 
}


/* Soft Key Action Routine - MANUAL */
static void uiStatusManual(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Select menu depending on inhibit state. */
    if (sysIsInhibited)
    {
        uiMenuSet(&uiMenuStatusResumeMan);
    }
    else
    {
        uiMenuSet(&uiMenuStatusManual);
        /* Initialize menu field index to select the first field. */
        uiField = 0;
    }
}


/* Soft Key Action Routine - TEST */
static void uiStatusTest(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (sysIsInhibited)
    {
        uiMenuSet(&uiMenuStatusResumeTest);
    }
    else
    {
        uiMenuSet(&uiMenuStatusTest);
    }
}


/* Soft Key Action Routine - STOP */
static void uiStatusStop(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Stop current irrigation operation. */
    irrCmdStop();
    uiMenuSet(&uiMenuStatusIdle);
    
    /* send command to other units */
    expansionBusSendCmd(RADIO_CMD_IRR_STOP, RADIO_EXP_SEND_ALL);
    if(config.sys.unitType != UNIT_TYPE_MASTER)
    {
        expansionBusSendCmd(RADIO_CMD_IRR_STOP, config.sys.masterMac);
    }

}


/* Soft Key Action Routine - RESUME */
static void uiStatusResume(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (sysIsInhibited)
    {
        uiMenuSet(&uiMenuStatusResume);
    }
    else
    {
        if (sysFaultOn(SYS_FAULT_24VAC))
        {
            /* Restart status cycle sequence to display fault message. */
            uiStatusCycle = 0;
            uiStatusReps = 0;
        }
        else
        {
            sysResume();
        }
    }
    
    /* send command to other units */
    expansionBusSendCmd(RADIO_CMD_IRR_RESUME, RADIO_EXP_SEND_ALL); 
}


/* Soft Key Action Routine - SKIP */
static void uiStatusSkip(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Call routine to skip watering to next zone */
    irrCmdSkip();
    
    /* send command to other units */
    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
          expansionBusSendCmd(RADIO_CMD_IRR_SKIP, RADIO_EXP_SEND_ALL);
    }
}


/*
**  CONTROLLER STATUS - TEST START MENU
*/


/* Display Action Routine */
static void uiStatusTestDisplay(void)
{
    uint16_t totalRT = 0;
    char buf[41];

    totalRT = (uint16_t)(sysTestRunTime * config.sys.numZones);
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Test Start");

    sprintf(&uiLcdBuf[LCD_RC(0, 18)],
            "(Watering Time:%6.6s)",
            dtFormatRunTime(buf, totalRT));


    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Time per Zone: %2d minute%s",
            sysTestRunTime,
            sysTestRunTime == 1 ? "" : "s");

    uiLcdCursor = LCD_RC(1, 16);
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiStatusTestInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (++sysTestRunTime > SYS_TEST_RUNTIME_MAX)
    {
        sysTestRunTime = 1;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiStatusTestDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (sysTestRunTime-- <= 1)
    {
        sysTestRunTime = SYS_TEST_RUNTIME_MAX;
    }
}


/* Soft Key Action Routine - START */
static void uiStatusTestStart(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Start the irrigation test. */
    irrCmdTest();
    uiStatusShow();
}


/* Soft Key Action Routine - TEST SKIP */
static void uiStatusTestSkip(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    TestSkipFlag = TRUE;
}


/*
**  CONTROLLER STATUS - MANUAL START MENU
**
**  Note: uiField is used to select the menu field.
*/


/* Display Action Routine */
static void uiStatusManualDisplay(void)
{
    uint16_t totalRT = 0;
    char buf[41];

    /* Don't allow Sensor mode selection if no sensors configured. */
    if ((sysManualOpMode == CONFIG_OPMODE_SENSOR) &&
        !moistSensorsConfigured())
    {
        sysManualOpMode = CONFIG_OPMODE_RUNTIME;
    }
    /* Don't allow Weather mode selection if no MB deficit for any zone. */
    if ((sysManualOpMode == CONFIG_OPMODE_WEATHER) &&
        !irrHaveMbDeficit())
    {
        sysManualOpMode = CONFIG_OPMODE_RUNTIME;
    }

    totalRT = (uint16_t)(irrProgramRuntime(sysManualProgram, sysManualOpMode) / 60);
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Manual Start");

    sprintf(&uiLcdBuf[LCD_RC(0, 18)],
            "(Watering Time:%6.6s)",
            dtFormatRunTime(buf, totalRT));

    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Program: %c",
            sysManualProgram + 'A');

    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "Mode:    %0.13s",
            uiOpModeNames[sysManualOpMode]);

    sprintf(&uiLcdBuf[LCD_RC(2, 26)],
            "%0.9s",
            (sysManualPulseMode != 0) ? "Pulse" : "Non-Pulse");

    switch (uiField)
    {
        case UI_MANUAL_FIELD_PROGRAM:
            uiLcdCursor = LCD_RC(1, 9);
            break;
        case UI_MANUAL_FIELD_OPMODE:
            uiLcdCursor = LCD_RC(2, 9);
            break;
        case UI_MANUAL_FIELD_PULSE:
            uiLcdCursor = LCD_RC(2, 26);
            break;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiStatusManualNext(void)
{
    if (++uiField >= UI_MANUAL_FIELD_LIMIT)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiStatusManualPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_MANUAL_FIELD_LIMIT - 1;
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiStatusManualInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_MANUAL_FIELD_PROGRAM:
            if (++sysManualProgram >= SYS_N_PROGRAMS)
            {
                sysManualProgram = 0;
            }
            break;
        case UI_MANUAL_FIELD_OPMODE:
            /* Increment mode, wrapping at end. */
            if (++sysManualOpMode >= CONFIG_OPMODE_LIMIT)
            {
                sysManualOpMode = 0;
            }
            /* Skip past Sensor mode if no sensors are configured. */
            if (sysManualOpMode == CONFIG_OPMODE_SENSOR &&
                !moistSensorsConfigured())
            {
                sysManualOpMode = CONFIG_OPMODE_WEATHER;
            }
            /* Skip past Weather mode if no moisture deficit on any zone. */
            if (sysManualOpMode == CONFIG_OPMODE_WEATHER &&
                !irrHaveMbDeficit())
            {
                sysManualOpMode = CONFIG_OPMODE_RUNTIME;
            }
            break;
        case UI_MANUAL_FIELD_PULSE:
            sysManualPulseMode = !sysManualPulseMode;
            break;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiStatusManualDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_MANUAL_FIELD_PROGRAM:
            if (sysManualProgram-- <= 0)
            {
                sysManualProgram = SYS_N_PROGRAMS - 1;
            }
            break;
        case UI_MANUAL_FIELD_OPMODE:
            /* Decrement mode, wrapping at beginning. */
            if (sysManualOpMode-- == 0)
            {
                sysManualOpMode = CONFIG_OPMODE_LIMIT - 1;
            }
            /* Skip past Weather mode if no moisture deficit on any zone. */
            if (sysManualOpMode == CONFIG_OPMODE_WEATHER &&
                !irrHaveMbDeficit())
            {
                sysManualOpMode = CONFIG_OPMODE_SENSOR;
            }
            /* Skip past Sensor mode if no sensors are configured. */
            if (sysManualOpMode == CONFIG_OPMODE_SENSOR &&
                !moistSensorsConfigured())
            {
                sysManualOpMode = CONFIG_OPMODE_RUNTIME;
            }
            break;
        case UI_MANUAL_FIELD_PULSE:
            sysManualPulseMode = !sysManualPulseMode;
            break;
    }
}


/* Soft Key Action Routine - START */
static void uiStatusManualStart(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuStatusIdle);
    /* Start the irrigation test. */
    irrCmdManualStart();
}


/* Soft Key Action Routine - CANCEL (shared by Man Start with Test menus). */
static void uiStatusCancel(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuStatusIdle);
}


/*
**  CONTROLLER STATUS - RESUME MENUS
**
**  Note: 3 menus are defined for Interrupted, Test and Manual Start cases.
**        These 3 menus only differ in the action routine called for the YES
**        soft key.
*/


/* Display Action Routine */
static void uiStatusResumeDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Watering is being inhibited by the");
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "municipality.  Override this control?");
}


/* Soft Key Action Routine - YES (from Interrupted Resume) */
static void uiStatusResumeOk(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    sysInhibitOff();
    uiMenuSet(&uiMenuStatusIdle);
}


/* Soft Key Action Routine - YES (from Test Start Resume) */
static void uiStatusResumeTestOk(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    sysInhibitOff();
    uiMenuSet(&uiMenuStatusTest);
}


/* Soft Key Action Routine - YES (from Manual Start Resume) */
static void uiStatusResumeManOk(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    sysInhibitOff();
    uiMenuSet(&uiMenuStatusManual);
    /* Initialize menu field index to select the first field. */
    uiField = 0;
}


/* Soft Key Action Routine - NO (cancel and return to main menu) */
static void uiStatusResumeCancel(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrState == IRR_STATE_IDLE)
    {
        uiMenuSet(&uiMenuStatusIdle);
    }
    else
    {
        uiMenuSet(&uiMenuStatusPaused);
    }
}



/******************************************************************************
 *
 *  SET DATE/TIME FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the menu field.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiDtDisplay(void)
{
    char buf[41];

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Date & Time: %0.23s",
            dtFormatCurrentDateTime(buf));
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Time Format: %0.23s",
            uiTimeFmtNames[config.sys.timeFmt]);
    //sprintf(&uiLcdBuf[LCD_RC(2, 0)],
    //      "dt:%d:%d",
    //    ii,uu);        
            

    if((config.sys.unitType != UNIT_TYPE_MASTER)) 
    {
        /* Mask off "-" "+" soft-key. */
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2);
    }
    
    if( lock==TRUE ) 
    {
        /* Mask off "-" "+" soft-key. */
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2);
    }

    switch (uiField)
    {
        case UI_DT_FIELD_YEAR:
            uiLcdCursor = LCD_RC(0, 16);
            break;
        case UI_DT_FIELD_MONTH:
            uiLcdCursor = LCD_RC(0, 18);
            break;
        case UI_DT_FIELD_MDAY:
            uiLcdCursor = LCD_RC(0, 23);
            break;
        case UI_DT_FIELD_HOUR:
            uiLcdCursor = LCD_RC(0, 30);
            break;
        case UI_DT_FIELD_MIN:
            uiLcdCursor = LCD_RC(0, 33);
            break;
        case UI_DT_FIELD_TIMEFMT:
            uiLcdCursor = LCD_RC(1, 13);
            break;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiDtNext(void)
{
    if (++uiField >= UI_DT_FIELD_LIMIT)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiDtPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_DT_FIELD_LIMIT - 1;
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiDtInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t maxMday;

    /*inform other units that user changed the time */
    timeChanged = TRUE;
    
    switch (uiField)
    {
        case UI_DT_FIELD_YEAR:
            if (++dtYear > DT_YEAR_MAX)
            {
                dtYear = DT_YEAR_MIN;
            }
            if (dtMon == 1 && (dtYear % 4) != 0 && dtMday > 28)
            {
                dtMday = 28;
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_MONTH:
            if (++dtMon > 11)
            {
                dtMon = 0;
            }
            maxMday = (dtMon == 1 && (dtYear % 4) != 0) ? 28 : dtDaysPerMonth(dtMon);
            if (dtMday > maxMday)
            {
                dtMday = maxMday;
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_MDAY:
            dtMday++;
            if (dtMday > dtDaysPerMonth(dtMon) ||
                (dtMday > 28 && dtMon == 1 && (dtYear % 4) != 0))
            {
                dtMday = 1;
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_HOUR:
            if (++dtHour > 23)
            {
                dtHour = 0;
            }
            dtSec = 0;
            /* Update time check logic. */
            dtPrevTime = (dtHour * 60) + dtMin;
            break;

        case UI_DT_FIELD_MIN:
            if (++dtMin > 59)
            {
                dtMin = 0;
            }
            dtSec = 0;
            /* Update time check logic. */
            dtPrevTime = (dtHour * 60) + dtMin;
            break;

        case UI_DT_FIELD_TIMEFMT:
            if (++config.sys.timeFmt >= CONFIG_TIMEFMT_LIMIT)
            {
                config.sys.timeFmt = 0;
            }
            break;
    }

    if (uiField != UI_DT_FIELD_TIMEFMT)
    {
        /* Clear date & time not set error. */
        sysErrorClear(SYS_ERROR_DATETIME);
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiDtDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t maxMday;

    /*inform other units that user changed the time */
    timeChanged = TRUE;
    
    switch (uiField)
    {
        case UI_DT_FIELD_YEAR:
            if (dtYear-- <= DT_YEAR_MIN)
            {
                dtYear = DT_YEAR_MAX;
            }
            if (dtMon == 1 && (dtYear % 4) != 0 && dtMday > 28)
            {
                dtMday = 28;
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_MONTH:
            if (dtMon-- == 0)
            {
                dtMon = 11;
            }
            maxMday = (dtMon == 1 && (dtYear % 4) != 0) ? 28 : dtDaysPerMonth(dtMon);
            if (dtMday > maxMday)
            {
                dtMday = maxMday;
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_MDAY:
            if (dtMday-- <= 1)
            {
                dtMday = (dtMon == 1 && (dtYear % 4) != 0) ? 28 : dtDaysPerMonth(dtMon);
            }
            dtWday = dtDayOfWeek(dtYear, dtMon, dtMday);
            break;

        case UI_DT_FIELD_HOUR:
            if (dtHour-- == 0)
            {
                dtHour = 23;
            }
            dtSec = 0;
            /* Update time check logic. */
            dtPrevTime = (dtHour * 60) + dtMin;
            break;

        case UI_DT_FIELD_MIN:
            if (dtMin-- == 0)
            {
                dtMin = 59;
            }
            dtSec = 0;
            /* Update time check logic. */
            dtPrevTime = (dtHour * 60) + dtMin;
            break;

        case UI_DT_FIELD_TIMEFMT:
            if (config.sys.timeFmt-- == 0)
            {
                config.sys.timeFmt = CONFIG_TIMEFMT_LIMIT - 1;
            }
            break;
    }

    if (uiField != UI_DT_FIELD_TIMEFMT)
    {
        /* Clear date & time not set error. */
        sysErrorClear(SYS_ERROR_DATETIME);
    }
}



/******************************************************************************
 *
 *  CONTROLLER SETUP FUNCTION MENU ACTIONS
 *
 *
 *  Note:  uiField is used to select the menu field.
 *         There is only one menu field at this time, but the menu is designed
 *         to support additional fields in the future.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiSetupDisplay(void)
{
    
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Total number of zones: %2d",
            (config.sys.numUnits+1)*12);
    
    if(config.sys.numUnits == 0)
    {
        config.sys.unitType = UNIT_TYPE_MASTER;
    }
    

   
    switch(config.sys.unitType)
    {        
      case UNIT_TYPE_MASTER:
          sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                  "Unit: 1-12");
          break;
      case UNIT_TYPE_EXPANSION_1:
          sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                  "Unit: 13-24");
          break; 
      case UNIT_TYPE_EXPANSION_2:
          sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                  "Unit: 25-36");
          break; 
       case UNIT_TYPE_EXPANSION_3:
          sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                  "Unit: 37-48");
          break;  
    }
 
 
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "Number zones connected to unit: %2d",
            config.sys.numZones);
          

       
    switch (uiField)
    {
        case UI_SETUP_FIELD_NZONES:
            if ((config.sys.numZones == 1)|(config.sys.unitType != UNIT_TYPE_MASTER))
            {
                /* Mask off "-" soft-key. */
                uiKeyMask |= UI_KM_KEY1;
            }
            if ((config.sys.numZones >= sysMaxZones) |(config.sys.unitType != UNIT_TYPE_MASTER))
            {
                /* Mask off "+" soft-key. */
                uiKeyMask |= UI_KM_KEY2;
            }
            uiLcdCursor = LCD_RC(2, 32);
            break;
        case UI_SETUP_FIELD_NUNITS:
            if ((config.sys.numUnits < 1)| (config.sys.unitType != UNIT_TYPE_MASTER))
            {
                /* Mask off "-" soft-key. */
                uiKeyMask |= UI_KM_KEY1;
            } 
            if ((config.sys.numUnits > SYS_N_UNITS )| (config.sys.unitType != UNIT_TYPE_MASTER))
            {
                /* Mask off "+" soft-key. */
                uiKeyMask |= UI_KM_KEY2;
            }
            
            uiLcdCursor = LCD_RC(0, 23);
            break;
        case UI_SETUP_FIELD_UNIT_TYPE:
            uiLcdCursor = LCD_RC(1, 7);
            break;
    }
    if((config.sys.numUnits == 0) 
                | (config.sys.unitType != UNIT_TYPE_MASTER))
    {
         /* Mask off "13-24", "25-36" and "37-48" soft-keys. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 |UI_KM_KEY5);
    } 
    else if(config.sys.numUnits == 1)
    {
         /* Mask off "25-36" and "37-48" soft-keys. */
         uiKeyMask |= (UI_KM_KEY4 |UI_KM_KEY5);
    } 
    else if(config.sys.numUnits == 2)
    {
         /* Mask off "37-48" soft-keys. */
         uiKeyMask |= UI_KM_KEY5;
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY3|UI_KM_KEY4|UI_KM_KEY5|UI_KM_KEY6);
    }
    
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "Number zones connected to unit: %2d",
            config.sys.numZones);
}



/* Display Action Routine for Expansion Config*/
static void uiSetupExpanDisplay(void)
{
    uint8_t numberZones;
    uint64_t macId;
    uint8_t expansionStatus=0;
    
    switch(uiTemp8)
    {       
      case 1:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Zones: 13-24");
          numberZones = config.sys.expNumZones1;
          macId = config.sys.expMac1;
          uiTemp64 = config.sys.expMac1;
          expansionStatus = radioStatusExpansion1;     
          break;
      case 2:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Zones: 25-36");
          numberZones = config.sys.expNumZones2;
          macId = config.sys.expMac2;
          uiTemp64 = config.sys.expMac2;
          expansionStatus = radioStatusExpansion2; 
          break;
      case 3:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Zones: 37-48");
          numberZones = config.sys.expNumZones3;
          macId = config.sys.expMac3;
          uiTemp64 = config.sys.expMac3;
          expansionStatus = radioStatusExpansion3; 
          break;
    }
    
    // display the cursor
    uiLcdCursor = LCD_RC(1, 17);
    
    if ((numberZones == 1))
    {
          /* Mask off "-" soft-key. */
           uiKeyMask |= UI_KM_KEY1;
    }
    if ((numberZones >= SYS_N_UNIT_ZONES))
    {
          /* Mask off "+" soft-key. */
          uiKeyMask |= UI_KM_KEY2;
    }
       
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Number of Zones: %2d",
            numberZones);        
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "MAC ID: %08X%08X",
            (uint32_t)(macId >> 32),
            (uint32_t)(macId & 0xFFFFFFFF));
            
    //need more control logic here
    if(expansionStatus == EXPANSION_CONNECTED)
    {      
        sprintf(&uiLcdBuf[LCD_RC(0, 18)],
              "Status: Connected");
    } 
    else 
    {
              sprintf(&uiLcdBuf[LCD_RC(0, 18)],
              "Status: Not Connected");
    }

}


/* Display Action Routine for Expansion 3 Config*/
static void uiSetupExpanMACDisplay(void)
{
    
    switch(uiTemp8)
    {       
      case 1:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Expansion 1: 13-24");
          
          break;
      case 2:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Expansion 2: 25-36");
          break;
      case 3:
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],
              "Expansion 3: 37-48");
          break;
    }

    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "MAC ID: %1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X%1X",       
        (uint32_t)((uiTemp64 & 0xF000000000000000ULL) >> 60),
        (uint32_t)((uiTemp64 & 0x0F00000000000000ULL) >> 56),
        (uint32_t)((uiTemp64 & 0x00F0000000000000ULL) >> 52),
        (uint32_t)((uiTemp64 & 0x000F000000000000ULL) >> 48),
        (uint32_t)((uiTemp64 & 0x0000F00000000000ULL) >> 44),
        (uint32_t)((uiTemp64 & 0x00000F0000000000ULL) >> 40),
        (uint32_t)((uiTemp64 & 0x000000F000000000ULL) >> 36),
        (uint32_t)((uiTemp64 & 0x0000000F00000000ULL) >> 32),
        (uint32_t)((uiTemp64 & 0x00000000F0000000ULL) >> 28),
        (uint32_t)((uiTemp64 & 0x000000000F000000ULL) >> 24),
        (uint32_t)((uiTemp64 & 0x0000000000F00000ULL) >> 20),
        (uint32_t)((uiTemp64 & 0x00000000000F0000ULL) >> 16),
        (uint32_t)((uiTemp64 & 0x000000000000F000ULL) >> 12),
        (uint32_t)((uiTemp64 & 0x0000000000000F00ULL) >> 8),
        (uint32_t)((uiTemp64 & 0x00000000000000F0ULL) >> 4),
        (uint32_t) (uiTemp64 & 0x000000000000000FULL)      );

     
     uiLcdCursor = LCD_RC(1, 8 + (uiField));

     if((config.sys.unitType != UNIT_TYPE_MASTER))
     {
          /* Mask off "-" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY3 | UI_KM_KEY4);
     }

}

/* Navigation Dial Action Routine - CW Rotation */
static void uiSetupNext(void)
{
    if (++uiField >= UI_SETUP_FIELD_LIMIT)
    {
        uiField = 0;
    }
}



/* Navigation Dial Action Routine - CCW Rotation */
static void uiSetupPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_SETUP_FIELD_LIMIT - 1;
    }
}

/* Navigation Dial Expansion Action Routine - CCW Rotation */
static void uiSetupExpanPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_SETUP_EXPAN_FIELD_LIMIT - 1;
    }
}

/* Navigation Dial Expansion Action Routine - CW Rotation */
static void uiSetupExpanNext(void)
{
    if (++uiField >= UI_SETUP_EXPAN_FIELD_LIMIT)
    {
        uiField = 0;
    }
}

/* Navigation Dial Expansion MAC Action Routine - CCW Rotation */
static void uiSetupExpanMACPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_SETUP_EXPAN_MAC_FIELD_LIMIT - 1;
    }
}

/* Navigation Dial Expansion MAC Action Routine - CW Rotation */
static void uiSetupExpanMACNext(void)
{
    if (++uiField >= UI_SETUP_EXPAN_MAC_FIELD_LIMIT)
    {
        uiField = 0;
    }
}

/* Soft Key Action Routine - Set MAC ID */
static void uiSetupExpanMAC(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupExpanMACMenu);
    uiField = 0;
    uiFirstField = 0;
}

/* Soft Key Action Routine - Config Expansion */
static void uiSetupExpanConfig(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    //expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, uint64_t macId);
}

/* Soft Key Expansion  Action Routine - Back */
static void uiSetupExpanBack(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetup);
    uiField = 0;
    uiFirstField = 0;
    uiTemp8 = 0;
}

/* Soft Key Expansion 3 MAC ID Action Routine - Clear */
static void uiSetupExpanMacClear(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    
    uiField = 0;
    uiFirstField = 0;
    switch(uiTemp8)
    { 
      case 1:
        config.sys.expMac1=0;
        uiMenuSet(&uiMenuSetupExpan1);
        break;
      case 2:
        config.sys.expMac2=0;
        uiMenuSet(&uiMenuSetupExpan2);
        break;
      case 3:
        config.sys.expMac3=0;
        uiMenuSet(&uiMenuSetupExpan3);
        break;
    }
    
}

/* Soft Key Expansion 3 MAC ID Action Routine - Accept */
static void uiSetupExpanMacAccept(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    
    uiField = 0;
    uiFirstField = 0;
    switch(uiTemp8)
    { 
      case 1:
        config.sys.expMac1=uiTemp64;
        uiMenuSet(&uiMenuSetupExpan1);
        break;
      case 2:
        config.sys.expMac2=uiTemp64;
        uiMenuSet(&uiMenuSetupExpan2);
        break;
      case 3:
        config.sys.expMac3=uiTemp64;
        uiMenuSet(&uiMenuSetupExpan3);
        break;
    }
    
}

/* Soft Key Expansion 3 MAC ID Action Routine - Cancel*/
static void uiSetupExpanMacCancel(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupExpan3);
    uiField = 0;
    uiFirstField = 0;
    
    switch(uiTemp8)
    { 
      case 1:
        uiMenuSet(&uiMenuSetupExpan1);
        break;
      case 2:
        uiMenuSet(&uiMenuSetupExpan2);
        break;
      case 3:
        uiMenuSet(&uiMenuSetupExpan3);
        break;
    }
}

/* Soft Key Action Routine - Expansion 1 setup ("13-24") */
static void uiSetupEx1(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupExpan1);
    uiField = 0;
    uiFirstField = 0;
    uiTemp8 = 1;
}

/* Soft Key Action Routine - Expansion 2 setup ("25-36") */
static void uiSetupEx2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupExpan2);
    uiField = 0;
    uiFirstField = 0;
    uiTemp8 = 2;
}


/* Soft Key Action Routine - Expansion 3 setup ("37-48") */
static void uiSetupEx3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupExpan3);
    uiField = 0;
    uiFirstField = 0;
    uiTemp8 = 3;
}  



/* Soft Key Action Routine - Increment Value ("+") */
static void uiSetupInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_SETUP_FIELD_NZONES:
            if (config.sys.numZones <  CONFIG_ZONE_MAX)
            {
                config.sys.numZones++;
                config.sys.masterNumZones= config.sys.numZones;
            }
            
            
            uiGroupsCheckAll();
            /* update sensor sample frequency */
            irrMoistConfigUpdate();
            break;
        case UI_SETUP_FIELD_NUNITS:
            if(config.sys.numUnits < (SYS_N_UNITS-1)) 
            {
                config.sys.numUnits++;
               /* if there is more than 1 unit then check to see if address is 0
                * if so then set MSB of mac to default 
                */
                if((config.sys.numUnits ==1))
                {     
                    sysFaultClear(SYS_FAULT_EXPAN_1);
                    if(config.sys.expMac1==0)
                    {
                        config.sys.expMac1 = 0x0013A20000000000;
                    }
                } 
                else if((config.sys.numUnits ==2))
                {     
                    sysFaultClear(SYS_FAULT_EXPAN_2);
                    if(config.sys.expMac2==0)
                    {
                        config.sys.expMac1 = 0x0013A20000000000;
                    }
                }
                else if((config.sys.numUnits ==3))
                {     
                    sysFaultClear(SYS_FAULT_EXPAN_3);
                    if(config.sys.expMac3==0)
                    {
                        config.sys.expMac1 = 0x0013A20000000000;
                    }
                }
                

                      
            }
            break;
        case UI_SETUP_FIELD_UNIT_TYPE:
            
              switch(config.sys.unitType)
              {
                case UNIT_TYPE_MASTER:
                    config.sys.masterMac=0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_1:
                    config.sys.expMac1 =0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_2:
                    config.sys.expMac2 =0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    config.sys.expMac3 =0x0013A20000000000; 
                    break;
              }
            
            config.sys.unitType += 1;

            if(config.sys.unitType > config.sys.numUnits)
            {
               config.sys.unitType = UNIT_TYPE_MASTER;
               sysFaultClear(SYS_FAULT_EXPAN_1);
               sysFaultClear(SYS_FAULT_EXPAN_2);
               sysFaultClear(SYS_FAULT_EXPAN_3);
            }
            
            if(config.sys.unitType == UNIT_TYPE_MASTER)
            {
                config.sys.masterMac = radioMacId;    
            }
       
            break;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiSetupDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_SETUP_FIELD_NZONES:
            if (config.sys.numZones > 1)
            {
                config.sys.numZones--;
                config.sys.masterNumZones= config.sys.numZones;
            }
            uiGroupsCheckAll();
            /* update sensor sample frequency */
            irrMoistConfigUpdate();
            break;
        case UI_SETUP_FIELD_NUNITS:
            if(config.sys.numUnits > 0) 
            {
                config.sys.numUnits--;
                
                switch(config.sys.numUnits)
                {
                  case 0:
                      sysFaultClear(SYS_FAULT_EXPAN_1);
                  case 1:
                      sysFaultClear(SYS_FAULT_EXPAN_2);
                  case 2:
                      sysFaultClear(SYS_FAULT_EXPAN_3);
                }
            }
            break;
        case UI_SETUP_FIELD_UNIT_TYPE:
            
            
            if(config.sys.unitType == UNIT_TYPE_MASTER)
            {
               config.sys.unitType = config.sys.numUnits;
            } 
            else 
            {
              
              sysFaultClear(SYS_FAULT_EXPAN_1);
              sysFaultClear(SYS_FAULT_EXPAN_2);
              sysFaultClear(SYS_FAULT_EXPAN_3);
              
              switch(config.sys.unitType)
              {
                case UNIT_TYPE_MASTER:
                    config.sys.masterMac=0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_1:
                    config.sys.expMac1 =0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_2:
                    config.sys.expMac2 =0x0013A20000000000; 
                    break;
                case UNIT_TYPE_EXPANSION_3:
                    config.sys.expMac3 =0x0013A20000000000; 
                    break;
              }
                    
              config.sys.unitType -= 1;
                         
              if(config.sys.unitType == UNIT_TYPE_MASTER)
              {
                config.sys.masterMac = radioMacId;               
              }
              
            }
            
            break;
    }
}   

/* Soft Key Expansion Action Routine - Increment Value ("+") */
static void uiSetupExpanInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t numZones;
    
    switch(uiTemp8)
    { 
      case 1:
           numZones = config.sys.expNumZones1;
           break;
      case 2:
           numZones = config.sys.expNumZones2;
           break;
      case 3:
           numZones = config.sys.expNumZones3;
           break;
    }
    
    switch (uiField)
    {
        case UI_SETUP_EXPAN_NZONES:
            if (numZones < SYS_N_UNIT_ZONES)
            {
                numZones++;
            }
            //uiGroupsCheckAll();
            /* update sensor sample frequency */
            //irrMoistConfigUpdate();
            break;
    }
    
    switch(uiTemp8)
    { 
      case 1:
           config.sys.expNumZones1 = numZones;
           break;
      case 2:
           config.sys.expNumZones2 = numZones;
           break;
      case 3:
           config.sys.expNumZones3 = numZones;
           break;
    }
}


/* Soft Key Expansion Action Routine - Decrement Value ("-") */
static void uiSetupExpanDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t numZones;
    
    switch(uiTemp8)
    { 
      case 1:
           numZones = config.sys.expNumZones1;
           break;
      case 2:
           numZones = config.sys.expNumZones2;
           break;
      case 3:
           numZones = config.sys.expNumZones3;
           break;
    }
    
    switch (uiField)
    {
        case UI_SETUP_EXPAN_NZONES:
            if (numZones > 1)
            {
                numZones--;
            }
            //uiGroupsCheckAll();
            /* update sensor sample frequency */
            //irrMoistConfigUpdate();
            break;
    }
    
    switch(uiTemp8)
    { 
      case 1:
           config.sys.expNumZones1 = numZones;
           break;
      case 2:
           config.sys.expNumZones2 = numZones;
           break;
      case 3:
           config.sys.expNumZones3 = numZones;
           break;
    }
}


/* Soft Key Expansion MAC ID Action Routine - Increment Value ("+") */
static void uiSetupExpanMACInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t nibble;
    uint8_t shift;
    uint64_t mask;

    shift = (15 - uiField) * 4;
    mask = 0x0FULL << shift;
    nibble = (uint8_t)((uiTemp64 & mask) >> shift);
    if (++nibble > 0x0F)
    {
        nibble = 0;
    }
    uiTemp64 = (uiTemp64 & ~mask) | ((uint64_t)nibble << shift);
}


/* Soft Key Expansion MAC ID Action Routine - Decrement Value ("-") */
static void uiSetupExpanMACDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t nibble;
    uint8_t shift;
    uint64_t mask;

    shift = (15 - uiField) * 4;
    mask = 0x0FULL << shift;
    nibble = (uint8_t)((uiTemp64 & mask) >> shift);
    if (--nibble > 0x0F)
    {
        nibble = 0x0F;
    }
    uiTemp64 = (uiTemp64 & ~mask) | ((uint64_t)nibble << shift);
}


/*
**  CONTROLLER SETUP - SENSOR CONCENTRATOR SETUP
**
**  Note: uiField is used to select the menu field.
*/


/* Display Action Routine */
static void uiSetupSensorConDisplay(void)
{
    uint8_t row;
    uint8_t col;
    uint8_t numSC = config.sys.numSensorCon;
    //uint8_t i;
    
    
    
    /*if unit isnt master then mask off all softkeys except for Back */
    if ((config.sys.unitType != UNIT_TYPE_MASTER))
    {
         /* Mask off  soft-key. */
         uiKeyMask |= UI_KM_KEY1 | UI_KM_KEY2 |UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5;
    }
    
    /* mask the Accept  softkeys if there isnt a new SC */
    if((newSensorConcenFound == FALSE))
    {
          /* Mask off soft-key. */
          uiKeyMask |= (UI_KM_KEY4);    
    }
    
    /* mask of "+" key if not on zones field */
    if((uiField2 == UI_GROUPS_FIELD_TYPE))
    {
          /* Mask off soft-key. */
          uiKeyMask |= (UI_KM_KEY3);
    }
    
    if((associateSCMode == TRUE))
    {
          /* Mask off Add and Delete keys if in add mode */
          uiKeyMask |= UI_KM_KEY1 | UI_KM_KEY2;
          
          if(newSensorConcenFound == TRUE)
          {
                sprintf(&uiLcdBuf[LCD_RC(0, 0)],"Found %08X%08X -- Press Accept", 
                            (uint32_t)(unassociatedSnsConMacId >> 32),
                            (uint32_t)(unassociatedSnsConMacId & 0xFFFFFFFF)); 
          }
          else
          {
                sprintf(&uiLcdBuf[LCD_RC(0, 0)],"Looking for Wireless Sensors to add...");
          }
    }
    else
    {
          /* Mask off Deny soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          
          sprintf(&uiLcdBuf[LCD_RC(0, 0)],"MAC ID");
          sprintf(&uiLcdBuf[LCD_RC(0, 19)],"Zones");
          sprintf(&uiLcdBuf[LCD_RC(0, 26)],"Battery");
          sprintf(&uiLcdBuf[LCD_RC(0, 35)],"Solar");
          
    }
    
    /* if in irrigate mode mask off delete key */
    /* user not allowed to delete if irrigating since changes wont take effect
    * in the irrigation program until stopped and restarted. */
    if((sysState != SYS_STATE_IDLE))
    {
        uiKeyMask |=  UI_KM_KEY2;    
    }
    
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix2(1);
    
    for (uint8_t r = 1, s = uiFirstField; 
         r < 3 && s < MAX_NUM_SC;
         r++, s++)
    {
        
        if((config.sys.assocSensorCon[s].macId == 0))
        {
            sprintf(&uiLcdBuf[LCD_RC(r, 0)],"                                        ");
        }
        else
        {
            sprintf(&uiLcdBuf[LCD_RC(r, 0)],
                    "%08X%08X",
                    (uint32_t)(config.sys.assocSensorCon[s].macId >> 32),
                    (uint32_t)(config.sys.assocSensorCon[s].macId & 0xFFFFFFFF));
            
            if(scDeleteList[s] == TRUE)
            {
                sprintf(&uiLcdBuf[LCD_RC(r, 16)],"--DELETE PENDING CHECKIN");   
            }
            else
            {
                if(config.sys.assocSensorCon[s].zoneRange == config.sys.unitType)
                {
                    sprintf(&uiLcdBuf[LCD_RC(r, 27)],"%d%%", (snsConInfo[s].battVoltage)); 
                    sprintf(&uiLcdBuf[LCD_RC(r, 34)],"%dmJ", (snsConInfo[s].chargeRate >>4));
                }
        
                switch(config.sys.assocSensorCon[s].zoneRange)
                {
                    case UNIT_TYPE_MASTER:
                        sprintf(&uiLcdBuf[LCD_RC(r, 19)],"1-12");
                        break;
                    case UNIT_TYPE_EXPANSION_1:
                        sprintf(&uiLcdBuf[LCD_RC(r, 19)],"13-24");
                        break;
                    case UNIT_TYPE_EXPANSION_2:
                        sprintf(&uiLcdBuf[LCD_RC(r, 19)],"25-36");
                        break;
                    case UNIT_TYPE_EXPANSION_3:
                        sprintf(&uiLcdBuf[LCD_RC(r, 19)],"37-48");
                        break;
                }
            }
        }
         
    }     
    
    row = (uiField - uiFirstField) / 1+1;
    
    if(row==3)
    {
        row=2;
    }
    
    switch (uiField2)
    {
        case UI_GROUPS_FIELD_TYPE:
        default:
            col = 0;
            break;
        case UI_GROUPS_FIELD_PARAM:
            col = 19;
    }

    uiLcdCursor = LCD_RC(row, col);
}

/* Soft Key Action Routine - Sensor Concentrator Setup */
static void uiSetupSnsCon(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSetupSensorConMenu);
    uiField = 0;
    uiFirstField = 0;
} 

/* Soft Key Sensor Concentrator Action Routine - Increment Value ("+") */
static void uiSetupSensorConInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t chanZone;
    
    /* change the zone settings back to defaults and no sensor for each zone 
    * associated with this sensor concentrator 
    */
    for(uint8_t k=0; k < SC_NUM_CHAN_UNIT ; k++)
    {
          chanZone = config.sys.assocSensorCon[uiField].channelZone[k];
          if(chanZone != SC_CHAN_NOT_ASSIGNED)
          {
               config.zone[chanZone].snsConChan = SC_NONE_SELECTED;
               config.zone[chanZone].snsConTableIndex = ZONE_SC_INDEX_NONE;
               config.zone[chanZone].sensorType = SNS_NONE;
               config.zone[chanZone].group =CONFIG_GROUP_NONE;
          }
          config.sys.assocSensorCon[uiField].channelZone[k]=SC_CHAN_NOT_ASSIGNED;
    }
          
    config.sys.assocSensorCon[uiField].zoneRange++;
    
    if(config.sys.assocSensorCon[uiField].zoneRange > config.sys.numUnits)
    {
        config.sys.assocSensorCon[uiField].zoneRange = UNIT_TYPE_MASTER;
    }
}

/* Navigation Dial Sensor Concentrator Action Routine - CCW Rotation */
static void uiSetupSensorConPrev(void)
{
    /*
     * This needs to deal with the conditional fields, skipping over sensor
     * parameter field(s) depending on sensor type.  Lots of fun...
     */ 
    
    if(config.sys.numSensorCon != 0)
    {
        if (uiField2 > 0)
        {
            /* was on non-first field of a multi-field zone - just back up one */
            uiField2--;
        }
        else
        {
            /* was on first field, so back up one zone (with wrap)... */
            if (uiField-- == 0)
            {
                uiField = MAX_NUM_SC - 1; //config.sys.numSensorCon-1;//UI_SETUP_SENSOR_CON_FIELD_LIMIT - 1;
            }
            /*else
            {
                uiField--;
            } */
            uiField2 = UI_GROUPS_FIELD_TYPE;

        }
    }

}

/* Navigation Dial Sensor Concentrator Action Routine - CW Rotation */
static void uiSetupSensorConNext(void)
{
    /*
     * This needs to deal with the conditional fields, skipping over sensor
     * parameter field(s) depending on sensor type.  Lots of fun...
     */

    if (uiField2 == UI_GROUPS_FIELD_PARAM)
    {
        /* was on last field for this zone - advance to next zone */
        uiField2 = UI_GROUPS_FIELD_TYPE;
        if (++uiField >= (MAX_NUM_SC -1))//(config.sys.numSensorCon-1))
        {
            uiField = 0;
        }
    }
    else
    {
        if(config.sys.numSensorCon != 0)
        {
            /* advance to next field for this zone */
            uiField2++;
        }
    }
}


/* Soft Key Sensor Concentrator Action Routine - Add Sensor Concetrator ("Add") */
static void uiSetupSensorConAdd(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    associateSCMode = TRUE;
}

/* Soft Key Sensor Concentrator Action Routine - Delete Sensor Concetrator ("Delete") */
static void uiSetupSensorConDelete(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    //radioRemoveSensorAssocHandler(config.sys.assocSensorCon[uiField].macId);
    if(config.sys.assocSensorCon[uiField].macId !=0)
    {
        
        //SC is flagged to be deleted and user pressed delete again
        // so delete immediately
        if(scDeleteList[uiField] == TRUE)
        {
            scDeleteIndex = uiField;
            expansionBusSendCmd(RADIO_CMD_SC_IS_REMOVED, config.sys.masterMac);
            
            radioRemoveSensorAssocList(config.sys.assocSensorCon[uiField].macId);
        }
        else
        {
            scDeleteList[uiField] = TRUE;
            scDeleteIndex = uiField;

            switch(config.sys.assocSensorCon[uiField].zoneRange)
            {
                case UNIT_TYPE_EXPANSION_1:
                     expansionBusSendCmd(RADIO_CMD_DELETE_SC, config.sys.expMac1);
                     break;
                case UNIT_TYPE_EXPANSION_2:
                     expansionBusSendCmd(RADIO_CMD_DELETE_SC, config.sys.expMac2);
                     break;
                case UNIT_TYPE_EXPANSION_3:
                     expansionBusSendCmd(RADIO_CMD_DELETE_SC, config.sys.expMac3);
                     break;
            }
                    
        }
        
        
    }
}

/* Soft Key Sensor Concentrator Action Routine - Accept Sensor Concetrator ("Accept") */
static void uiSetupSensorConAccept(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t sensorIndex;
    uint8_t i;
    
    
    sensorIndex = radioAddSensorAssocList(unassociatedSnsConMacId);
    config.sys.assocSensorCon[sensorIndex].zoneRange = UNIT_TYPE_MASTER;
    for(i=0; i<SC_NUM_CHAN_UNIT ;i++)
    {
        config.sys.assocSensorCon[sensorIndex].channelZone[i] = SC_CHAN_NOT_ASSIGNED;
    }
    associateSCMode = FALSE;
    newSensorConcenFound= FALSE;
    unassociatedSnsConMacId = 0;
}

/* Soft Key Sensor Concentrator Action Routine - Deny Sensor Concetrator ("Deny") */
static void uiSetupSensorConDeny(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    associateSCMode = FALSE;
    newSensorConcenFound= FALSE;
    unassociatedSnsConMacId = 0;
}


/******************************************************************************
 *
 *  MODE OF OPERATION FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the menu field.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiOpModeDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Operating Mode: %0.24s",
            uiOpModeNames[config.sys.opMode]);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Pulse Mode:     %0.24s",
            (config.sys.pulseMode != 0) ? "On" : "Off");

    switch (uiField)
    {
        case UI_OPMODE_FIELD_OPMODE:
            uiLcdCursor = LCD_RC(0, 16);
            break;
        case UI_OPMODE_FIELD_PULSE:
            uiLcdCursor = LCD_RC(1, 16);
            break;
    }
    
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" and "+" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY2);
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2);
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiOpModeNext(void)
{
    if (++uiField >= UI_OPMODE_FIELD_LIMIT)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiOpModePrev(void)
{
    if (uiField-- == 0)
    {
        uiField = UI_OPMODE_FIELD_LIMIT - 1;
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiOpModeInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_OPMODE_FIELD_OPMODE:
            if (++config.sys.opMode >= CONFIG_OPMODE_LIMIT)
            {
                config.sys.opMode = 0;
            }
            /* Update moisture sensor sampling frequency as appropriate. */
            irrMoistConfigUpdate();
            /* Clear stored ET data if not in Weather mode. */
            irrOpModeConfigChanged();
            break;

        case UI_OPMODE_FIELD_PULSE:
            config.sys.pulseMode = !config.sys.pulseMode;
            break;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiOpModeDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    switch (uiField)
    {
        case UI_OPMODE_FIELD_OPMODE:
            if (config.sys.opMode-- == 0)
            {
                config.sys.opMode = CONFIG_OPMODE_LIMIT - 1;
            }
            /* Update moisture sensor sampling frequency as appropriate. */
            irrMoistConfigUpdate();
            /* Clear stored ET data if not in Weather mode. */
            irrOpModeConfigChanged();
            break;

        case UI_OPMODE_FIELD_PULSE:
            config.sys.pulseMode = !config.sys.pulseMode;
            break;
    }
}



/******************************************************************************
 *
 *  IRRIGATION DAYS MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the program.
 *        uiField2 is used to select the day.
 *
 *        The navigation dial scrolls through program start times (uiField)
 *        then days (uiField2).  The summary screen is located prior to screen
 *        for Monday's start times.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiIrrDaysSummaryDisplay(void)
{
    uint16_t totalST[7];
    char *pBuf = &uiLcdBuf[LCD_RC(2, 2)];

    /* compute number of start times for each day */
    for (int d = 0; d < 7; d++)
    {
        totalST[d] = 0;
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            if (config.sched[d][p].startTime != htons(CONFIG_SCHED_START_DISABLED))
            {
                totalST[d]++;
            }
        }
    }

    sprintf(&uiLcdBuf[LCD_RC(0, 12)],
        "Irrigation Days");

    sprintf(&uiLcdBuf[LCD_RC(1, 3)],
            "MON  TUE  WED  THU  FRI  SAT  SUN");
    for (int d = 0; d < 7; d++)
    {
        pBuf += sprintf(pBuf, " %-3s ", (totalST[d] != 0) ? "On" : "Off");
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiIrrDaysSummaryNext(void)
{
    uiMenuSet(&uiMenuIrrDays);
    uiField = 0;
    uiField2 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiIrrDaysSummaryPrev(void)
{
    uiMenuSet(&uiMenuIrrDays);
    uiField = SYS_N_PROGRAMS - 1;
    uiField2 = 7 - 1;
}


/* Display Action Routine */
static void uiIrrDaysDisplay(void)
{
    char buf[41];

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Irrigation Start Times for %0.13s",
            uiDayFullNames[uiField2]);
#if SYS_N_PROGRAMS > 4
#error "The Irrigations Days screen does not support more than four programs."
#endif
    for (int i = 0; i < SYS_N_PROGRAMS; i++)
    {
        if (config.sched[uiField2][i].startTime != htons(CONFIG_SCHED_START_DISABLED))
        {
            sprintf(buf, "%0.7s",
                    dtFormatMinutes(buf,
                                    ntohs(config.sched[uiField2][i].startTime)));
        }
        else
        {
            strcpy(buf, "Off");
        }
        sprintf(&uiLcdBuf[LCD_RC(1, 0) + (i * 20)],
                "PGM %c: %0.7s  ",
                i + 'A',
                buf);
    }

    uint8_t row = (uiField / 2) + 1;
    uint8_t col = uiField % 2;
    uiLcdCursor = LCD_RC(row, (col * 20) + 7);
    if (config.sched[uiField2][uiField].startTime != htons(CONFIG_SCHED_START_DISABLED))
    {
        uiLcdCursor += 4;
    }
    
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "OFF" soft-key. */
          uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY3);
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY3);
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiIrrDaysNext(void)
{
    if (++uiField >= SYS_N_PROGRAMS)
    {
        uiField = 0;
        if (++uiField2 >= 7)
        {
            uiMenuSet(&uiMenuIrrDaysSummary);
        }
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiIrrDaysPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = SYS_N_PROGRAMS - 1;
        if (uiField2-- == 0)
        {
            uiMenuSet(&uiMenuIrrDaysSummary);
        }
    }
}


/* Soft Key Action Routine - TODAY */
static void uiIrrDaysSummaryToday(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuIrrDays);
    uiField = 0;
    uiField2 = (dtWday == 0) ? 6 : dtWday - 1;
}


#define SCHED_START_STEP        5

/* Soft Key Action Routine - Increment Value ("+") */
static void uiIrrDaysInc(uint8_t eventType, uint8_t /*eventKey*/)
{
    uint16_t startTime = ntohs(config.sched[uiField2][uiField].startTime);
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 4 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (startTime == CONFIG_SCHED_START_DISABLED)
        {
            /* increment to 00:00 */
            startTime = 0;
        }
        else if (startTime >= (CONFIG_SCHED_START_LIMIT - SCHED_START_STEP))
        {
            /* increment to disabled */
            startTime = CONFIG_SCHED_START_DISABLED;
        }
        else
        {
            startTime += SCHED_START_STEP;
        }
    }
    config.sched[uiField2][uiField].startTime = htons(startTime);
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiIrrDaysDec(uint8_t eventType, uint8_t /*eventKey*/)
{
    uint16_t startTime = ntohs(config.sched[uiField2][uiField].startTime);
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 4 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (startTime == CONFIG_SCHED_START_DISABLED)
        {
            /* decrement to 24:00-step */
            startTime = CONFIG_SCHED_START_LIMIT - SCHED_START_STEP;
        }
        else if (startTime == 0)
        {
            /* decrement to disabled */
            startTime = CONFIG_SCHED_START_DISABLED;
        }
        else if (startTime < SCHED_START_STEP)
        {
            /* decrement to 00:00 */
            startTime = 0;
        }
        else
        {
            startTime -= SCHED_START_STEP;
        }
    }
    config.sched[uiField2][uiField].startTime = htons(startTime);
}


/* Soft Key Action Routine - OFF */
static void uiIrrDaysOff(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    config.sched[uiField2][uiField].startTime = htons(CONFIG_SCHED_START_DISABLED);
}



/******************************************************************************
 *
 *  RUN TIMES FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiField2 is used to select the program.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through runtimes for zones (uiField) and
 *        then programs (uiField2).  The summary screen is located in the
 *        scroll sequence just prior to the screen for zone 1, program A
 *        runtime.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiRunTimeSummaryDisplay(void)
{
    uint16_t totalRT[SYS_N_PROGRAMS];
    char buf[41];

    
    /* compute run-time totals for each program */
    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {       
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            totalRT[p] = 0;
            for (int z = 0; z < ((config.sys.numUnits+1) *12); z++)
            {
                totalRT[p] += config.zone[z].runTime[p];
            }
        }
    } 
    else 
    {
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            totalRT[p] = 0;
            for (int z = 0; z < config.sys.numZones; z++)
            {
                totalRT[p] += config.zone[z].runTime[p];
            }
        }
    }

    sprintf(&uiLcdBuf[LCD_RC(0, 7)],
            "Run Times (Program Totals)");
#if SYS_N_PROGRAMS > 4
#warning "The Run Times summary screen is limited to four programs."
#endif
    for (int i = 0; i < 2; i++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(i + 1, 0)];

        for (int p = i * 2; p < (i * 2) + 2 && p < SYS_N_PROGRAMS; p++)
        {
            pBuf += sprintf(pBuf, "PGM %c: %6.6s       ",
                            p + 'A',
                            dtFormatRunTime(buf, totalRT[p]));
        }
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiRunTimeSummaryNext(void)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiRunTimeSummaryPrev(void)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = navEndZone - 1;
    uiFirstField = 0;
    uiField2 = SYS_N_PROGRAMS - 1;
}


/* Soft Key Action Routine - "->A" */
static void uiRunTimeSummaryGotoA(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 0;
}


#if SYS_N_PROGRAMS > 1
/* Soft Key Action Routine - "->B" */
static void uiRunTimeSummaryGotoB(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 1;
}
#endif


#if SYS_N_PROGRAMS > 2
/* Soft Key Action Routine - "->C" */
static void uiRunTimeSummaryGotoC(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 2;
}
#endif


#if SYS_N_PROGRAMS > 3
/* Soft Key Action Routine - "->D" */
static void uiRunTimeSummaryGotoD(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTime);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 3;
}
#endif


#if SYS_N_PROGRAMS > 4
#warning "Run Times summary screen soft-keys are limited to four programs."
#endif


/* Display Action Routine */
static void uiRunTimeDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;
    
    //mask out soft key short cuts based on number of units
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY2| UI_KM_KEY3| UI_KM_KEY4| UI_KM_KEY5 |UI_KM_KEY6);
          endingZone=12;
    }
    
    switch(config.sys.unitType)
    {
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum =12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum =24;
          break;
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum =36;
          break;
    }
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);

    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 10)];

        if (config.zone[z].runTime[uiField2] != 0)
        {
            pBuf += sprintf(pBuf, "%2d%c: %3.2d  ",
                            displayZoneNum + z + 1,
                            uiField2 + 'A',
                            config.zone[z].runTime[uiField2]);
        }
        else
        {
            pBuf += sprintf(pBuf, "%2d%c: Off  ",
                            displayZoneNum + z + 1,
                            uiField2 + 'A');
        }
    }


    uint8_t row = (uiField - uiFirstField) / 4;
    uint8_t col = (uiField - uiFirstField) % 4;
    uiLcdCursor = LCD_RC(row, (col * 10) + 7);
    if (config.zone[uiField].runTime[uiField2] == 0)
    {
        uiLcdCursor -= 2;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiRunTimeNext(void)
{
       
    if (++uiField >= navEndZone)
    {
        uiField = 0;
        if (++uiField2 >= SYS_N_PROGRAMS)
        {
            uiMenuSet(&uiMenuRunTimeSummary);
        }
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiRunTimePrev(void)
{
    if (uiField-- == 0)
    {
        uiField = navEndZone - 1;
        if (uiField2-- == 0)
        {
            uiMenuSet(&uiMenuRunTimeSummary);
        }
    }
}


/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiRunTimeGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=12;
    uiFirstField =12;
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiRunTimeGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=24;
    uiFirstField =24;
}


/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiRunTimeGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=36;
    uiFirstField =36;
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiRunTimeInc(uint8_t eventType, uint8_t /*eventKey*/)
{
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 5 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (++config.zone[uiField].runTime[uiField2] > CONFIG_RUNTIME_MAX)
        {
            /* wrap from maximum run-time of 4:00 to off (0:00) */
            config.zone[uiField].runTime[uiField2] = 0;
        }
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiRunTimeDec(uint8_t eventType, uint8_t /*eventKey*/)
{
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 5 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (config.zone[uiField].runTime[uiField2]-- == 0)
        {
            /* wrap from off (0:00) to maximum run-time of 4:00 */
            config.zone[uiField].runTime[uiField2] = CONFIG_RUNTIME_MAX;
        }
    }
}


/* Soft Key Action Routine - OFF */
static void uiRunTimeOff(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    config.zone[uiField].runTime[uiField2] = 0;
}


/* Soft Key Action Routine - COPY */
static void uiRunTimeCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTimeCopy);
}


/* Display Action Routine */
static void uiRunTimeCopyDisplay(void)
{
    if (config.zone[uiField].runTime[uiField2] != 0)
    {
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Copy zone #%d run time setting of %d",
                uiField + 1,
                config.zone[uiField].runTime[uiField2]);
    }
    else
    {
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Copy zone #%d run time setting of 'Off'",
                uiField + 1,
                config.zone[uiField].runTime[uiField2]);
    }
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "to all other zones in program %c?",
            uiField2 + 'A');
}


/* Soft Key Action Routine - YES */
static void uiRunTimeCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone application rate to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].runTime[uiField2] = config.zone[uiField].runTime[uiField2];
        }
    }
    uiMenuSet(&uiMenuRunTime);
}


/* Soft Key Action Routine - NO */
static void uiRunTimeCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuRunTime);
}



/******************************************************************************
 *
 *  APPLICATION RATE & APPLICATION EFFICIENCY FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiField2 is used to select the sub-field for inches or hundredths
 *          (for the original AppRate UI; it is unused for the new AppRate UI).
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through application rate sub-fields
 *        (uiField2) then zones (uiField).  There are two summary screens, one
 *        for application rate, one for application efficiency.  The summary
 *        screens are located in the scroll sequence just prior to their
 *        respective screens for zone 1 value configuration.
 *
 *****************************************************************************/

/* APPLICATION RATE SUMMARY SCREEN */


/* Display Action Routine */
static void uiAppRateSummaryDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 5)],
            "Application Rate (inches/hour)");
    
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY1);
    }        
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAppRateSummaryNext(void)
{
    uiMenuSet(&uiMenuAppRate);
    uiField = 0;
    uiFirstField = 0;
#ifndef UI_NEW_APP_RATE
    uiField2 = 0;
#endif
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAppRateSummaryPrev(void)
{
    uiMenuSet(&uiMenuAppEff);
    uiField = navEndZone- 1;
    uiFirstField = 0;
}


/* Soft Key Action Routine - "->EFF" */
static void uiAppRateSummaryGoto(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppEffSummary);
    uiField = 0;
    uiFirstField = 0;
}


/* APPLICATION RATE ZONE CONFIGURATION SCREEN */


#ifdef UI_NEW_APP_RATE


/* Display Action Routine */
static void uiAppRateDisplay(void)
{
    uint16_t appRate = ntohs(config.zone[uiField].appRate);

    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);

    for (int i = 0, z = uiFirstField;
         i < 12 && z < config.sys.numZones;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 10)];

        sprintf(pBuf, "%2d: %1d.%02d  ",
                z + 1,
                ntohs(config.zone[z].appRate) / 100,
                ntohs(config.zone[z].appRate) % 100);
    }

    if ((appRate <= CONFIG_APPRATE_MIN))
    {
        /* Mask off "-" soft-key. */
        uiKeyMask |= UI_KM_KEY1;
    }
    if ((appRate >= CONFIG_APPRATE_MAX))
    {
        /* Mask off "+" soft-key. */
        uiKeyMask |= UI_KM_KEY2;
    }

    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }


    uint8_t row = (uiField - uiFirstField) / 4;
    uint8_t col = (uiField - uiFirstField) % 4;
    uiLcdCursor = LCD_RC(row, (col * 10) + 7);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAppRateNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiMenuSet(&uiMenuAppEffSummary);
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAppRatePrev(void)
{
    if (uiField-- == 0)
    {
        uiMenuSet(&uiMenuAppRateSummary);
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiAppRateInc(uint8_t eventType, uint8_t /*eventKey*/)
{
    uint16_t appRate = ntohs(config.zone[uiField].appRate);
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 5 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (appRate < CONFIG_APPRATE_MAX)
        {
            appRate++;
        }
    }

    config.zone[uiField].appRate = htons(appRate);
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiAppRateDec(uint8_t eventType, uint8_t /*eventKey*/)
{
    uint16_t appRate = ntohs(config.zone[uiField].appRate);
    int repeatCount = (eventType == DRV_KEYPAD_TYPE_SOFT_FAST) ? 5 : 1;

    for (int i = 0; i < repeatCount; i++)
    {
        if (appRate > CONFIG_APPRATE_MIN)
        {
            appRate--;
        }
    }

    config.zone[uiField].appRate = htons(appRate);
}


#else /* !UI_NEW_APP_RATE */


/* Display Action Routine */
static void uiAppRateDisplay(void)
{
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);
    
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 10)];

        pBuf += sprintf(pBuf, "%2d: %1d.%02d  ",
                        displayZoneNum+z + 1,
                        ntohs(config.zone[z].appRate) / 100,
                        ntohs(config.zone[z].appRate) % 100);
    }

    uint8_t row = (uiField - uiFirstField) / 4;
    uint8_t col = (uiField - uiFirstField) % 4;
    uiLcdCursor = LCD_RC(row, (col * 10) + 4);
    if (uiField2 != 0)
    {
        uiLcdCursor += 3;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAppRateNext(void)
{
    if (uiField2 == 0)
    {
        uiField2 = 1;
    }
    else
    {
        uiField2 = 0;
        if (++uiField >= navEndZone)
        {
            uiMenuSet(&uiMenuAppEffSummary);
        }
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAppRatePrev(void)
{
    if (uiField2 > 0)
    {
        uiField2 = 0;
    }
    else
    {
        uiField2 = 1;
        if (uiField-- == 0)
        {
            uiMenuSet(&uiMenuAppRateSummary);
        }
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiAppRateInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint32_t appRateInt  = ntohs(config.zone[uiField].appRate) / 100;
    uint32_t appRateFrac = ntohs(config.zone[uiField].appRate) % 100;

    if (uiField2 == 0)
    {
        /* increment integer portion of application rate */
        if (++appRateInt >= 10)
        {
            if (appRateFrac == 0)
            {
                /* wrap from 9.00 to 1.00 */
                appRateInt = 1;
            }
            else
            {
                /* wrap from 9.xx to 0.xx */
                appRateInt = 0;
            }
        }
    }
    else
    {
        /* increment fractional portion of application rate */
        if (++appRateFrac >= 100)
        {
            if (appRateInt == 0)
            {
                /* wrap from 0.99 to 0.01 */
                appRateFrac = 1;
            }
            else
            {
                /* wrap from x.99 to x.00 */
                appRateFrac = 0;
            }
        }
    }

    config.zone[uiField].appRate = htons((uint16_t)((appRateInt * 100) + appRateFrac));
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiAppRateDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint32_t appRateInt  = ntohs(config.zone[uiField].appRate) / 100;
    uint32_t appRateFrac = ntohs(config.zone[uiField].appRate) % 100;

    if (uiField2 == 0)
    {
        /* decrement integer portion of application rate */
        if (appRateInt-- == 0)
        {
            /* wrap from 0.xx to 9.xx */
            appRateInt = 9;
        }
        if (appRateInt == 0 && appRateFrac == 0)
        {
            /* wrap from 1.00 to 9.00 */
            appRateInt = 9;
        }
    }
    else
    {
        /* decrement fractional portion of application rate */
        if (appRateFrac-- == 0)
        {
            /* wrap from x.00 to x.99 */
            appRateFrac = 99;
        }
        if (appRateInt == 0 && appRateFrac == 0)
        {
            /* wrap from 0.01 to 0.99 */
            appRateFrac = 99;
        }
    }

    config.zone[uiField].appRate = htons((uint16_t)((appRateInt * 100) + appRateFrac));
}


#endif /* !UI_NEW_APP_RATE */


/* Soft Key Action Routine - COPY */
static void uiAppRateCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppRateCopy);
}


/* Display Action Routine */
static void uiAppRateCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d application rate setting",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "of %1d.%02d in/hr to all other zones?",
            ntohs(config.zone[uiField].appRate) / 100,
            ntohs(config.zone[uiField].appRate) % 100);
}


/* Soft Key Action Routine - YES */
static void uiAppRateCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone application rate to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].appRate = config.zone[uiField].appRate;
        }
    }
    uiMenuSet(&uiMenuAppRate);
}


/* Soft Key Action Routine - NO */
static void uiAppRateCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppRate);
}


/* APPLICATION EFFICIENCY SUMMARY SCREEN */


/* Display Action Routine */
static void uiAppEffSummaryDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 7)],
            "Application Efficiency (%%)");
            
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 );
    }         
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAppEffSummaryNext(void)
{
    uiMenuSet(&uiMenuAppEff);
    uiField = 0;
    uiFirstField = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAppEffSummaryPrev(void)
{
    uiMenuSet(&uiMenuAppRate);
    uiField = navEndZone - 1;
    uiFirstField = 0;
#ifndef UI_NEW_APP_RATE
    uiField2 = 1;
#endif
}


/* Soft Key Action Routine - "->RATE" */
static void uiAppEffSummaryGoto(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppRateSummary);
    uiField = 0;
    uiFirstField = 0;
#ifndef UI_NEW_APP_RATE
    uiField2 = 0;
#endif
}


/* APPLICATION EFFICIENCY ZONE CONFIGURATION SCREEN */


/* Display Action Routine */
static void uiAppEffDisplay(void)
{
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 10)];

        pBuf += sprintf(pBuf, "%2d: %3d%%  ",
                        displayZoneNum+z + 1,
                        config.zone[z].appEff);
    }

    uint8_t row = (uiField - uiFirstField) / 4;
    uint8_t col = (uiField - uiFirstField) % 4;
    uiLcdCursor = LCD_RC(row, (col * 10) + 6);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAppEffNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiMenuSet(&uiMenuAppRateSummary);
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAppEffPrev(void)
{
    if (uiField-- == 0)
    {
        uiMenuSet(&uiMenuAppEffSummary);
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiAppEffInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (++config.zone[uiField].appEff > CONFIG_APPEFF_MAX)
    {
        /* Wrap from maximum value to minimum value. */
        config.zone[uiField].appEff = CONFIG_APPEFF_MIN;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiAppEffDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (config.zone[uiField].appEff-- <= CONFIG_APPEFF_MIN)
    {
        /* Wrap from minimum value to maximum value. */
        config.zone[uiField].appEff = CONFIG_APPEFF_MAX;
    }
}


/* Soft Key Action Routine - COPY */
static void uiAppEffCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppEffCopy);
}


/* Display Action Routine */
static void uiAppEffCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d application efficiency",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "setting of %d%% to all other zones?",
            config.zone[uiField].appEff);
}


/* Soft Key Action Routine - YES */
static void uiAppEffCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone application efficiency to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].appEff = config.zone[uiField].appEff;
        }
    }
    uiMenuSet(&uiMenuAppEff);
}


/* Soft Key Action Routine - NO */
static void uiAppEffCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAppEff);
}



/******************************************************************************
 *
 *  SOIL TYPE FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zones (uiField).
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiSoilTypeDisplay(void)
{
    uint8_t zi;             /* zone index */
    uint8_t i;              /* field index */
    uint8_t row;            /* selected row */
    uint8_t col;            /* selected column */
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    //mask out soft key short cuts based on number of units
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    
    /* Fix uiFirstField if it is out of range. */
    uiFirstFieldFix(2);

    /* Display zone soil types, in 2 columns (6 zones per screen). */
    for (i = 0, zi = uiFirstField;
         i < 6 && zi < endingZone;
         i++, zi++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 20)];

        pBuf += sprintf(pBuf, "%2d: %-16.16s",
                        displayZoneNum+zi + 1,
                        uiSoilTypeNames[config.zone[zi].soilType]);
    }

    row = (uiField - uiFirstField) / 2;
    col = (uiField - uiFirstField) % 2;
    uiLcdCursor = LCD_RC(row, (col * 20) + 4);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiSoilTypeNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiSoilTypePrev(void)
{
    if (uiField-- == 0)
    {
        uiField = navEndZone - 1;
    }
}


/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiSoilTypeGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =12;
    uiFirstField=12; 
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiSoilTypeGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =24;
    uiFirstField=24; 
}

/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiSoilTypeGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =36;
    uiFirstField=36; 
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiSoilTypeInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (++config.zone[uiField].soilType >= CONFIG_SOILTYPE_LIMIT)
    {
        config.zone[uiField].soilType = 0;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiSoilTypeDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (config.zone[uiField].soilType-- == 0)
    {
        config.zone[uiField].soilType = CONFIG_SOILTYPE_LIMIT - 1;
    }
}


/* Soft Key Action Routine - COPY */
static void uiSoilTypeCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSoilTypeCopy);
}


/* Display Action Routine */
static void uiSoilTypeCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d soil type setting of",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "'%-0.18s' to all other zones?",
            uiSoilTypeNames[config.zone[uiField].soilType]);
}


/* Soft Key Action Routine - YES */
static void uiSoilTypeCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone soil type to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].soilType = config.zone[uiField].soilType;
        }
    }
    uiMenuSet(&uiMenuSoilType);
}


/* Soft Key Action Routine - NO */
static void uiSoilTypeCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSoilType);
}



/******************************************************************************
 *
 *  SLOPE FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zones (uiField).
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiSlopeDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    // mask out short cut buttons based on number of units
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);

    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[i * 10 + 0];

        pBuf += sprintf(pBuf, "%2d: %-5.5s ",
                        displayZoneNum+z + 1,
                        uiSlopeNames[config.zone[z].slope]);
    }

    uint8_t row = (uiField - uiFirstField) / 4;
    uint8_t col = (uiField - uiFirstField) % 4;
    uiLcdCursor = LCD_RC(row, (col * 10) + 4);
    
     
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiSlopeNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiSlopePrev(void)
{
    if (uiField-- == 0)
    {
        uiField = navEndZone - 1;
    }
}



/* Soft Key Action Routine - Group2 display "13-24"*/
static void uiSlopeGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 12;
    uiFirstField = 12;

}

/* Soft Key Action Routine - Group3 display "25-36"*/
static void uiSlopeGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 24;
    uiFirstField = 24;

}

/* Soft Key Action Routine - Group4 display "37-48"*/
static void uiSlopeGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 36;
    uiFirstField = 36;

}

/* Soft Key Action Routine - Increment Value ("+") */
static void uiSlopeInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (++config.zone[uiField].slope >= CONFIG_SLOPE_LIMIT)
    {
        config.zone[uiField].slope = 0;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiSlopeDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (config.zone[uiField].slope-- == 0)
    {
        config.zone[uiField].slope = CONFIG_SLOPE_LIMIT - 1;
    }
}


/* Soft Key Action Routine - COPY */
static void uiSlopeCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSlopeCopy);
}


/* Display Action Routine */
static void uiSlopeCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d slope setting of",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "'%-0.5s' to all other zones?",
            uiSlopeNames[config.zone[uiField].slope]);
            
    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY2 |UI_KM_KEY6);
    }        
}


/* Soft Key Action Routine - YES */
static void uiSlopeCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone slope to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].slope = config.zone[uiField].slope;
        }
    }
    uiMenuSet(&uiMenuSlope);
}


/* Soft Key Action Routine - NO */
static void uiSlopeCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuSlope);
}


/******************************************************************************
 *
 *  MOISTURE SETPOINTS MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiField2 is used to select the sub-fields for min and max settings.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through min/max sub-fields (uiField2)
 *        then zones (uiField).
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiMoistDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3|UI_KM_KEY4|UI_KM_KEY5);//|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    //mask out soft key short cuts based on number of units
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    if ((uiField2 == 0 && config.zone[uiField].minMoist == CONFIG_MOIST_MIN) ||
        (uiField2 != 0 && config.zone[uiField].maxMoist <= (CONFIG_MOIST_MIN + 1)))
    {
        /* Mask off "-" soft-key. */
        uiKeyMask |= UI_KM_KEY1;
    }
    if ((uiField2 == 0 && config.zone[uiField].minMoist >= (CONFIG_MOIST_MAX - 1)) ||
        (uiField2 != 0 && config.zone[uiField].maxMoist >= CONFIG_MOIST_MAX) )
    {
        /* Mask off "+" soft-key. */
        uiKeyMask |= UI_KM_KEY2;
    }
    if ((config.zone[uiField].group != uiField))
    {
        /* Mask off "-", "+", and "COPY" soft-keys. */
        uiKeyMask |= UI_KM_KEY1 | UI_KM_KEY2;// | UI_KM_KEY6;
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2);
    }

    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(3);

    for (int r = 0, c = 0, z = uiFirstField;
         r < 3 && z < endingZone;
         z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(r, c * 13)];

        if ((config.zone[z].group == z) && 
            ((config.zone[z].sensorType == SNS_WIRED_MOIST) || (config.zone[z].sensorType == SNS_WIRELESS_MOIST) || (config.zone[z].sensorType == SNS_WIRELESS_VALVE))) 
        {
            pBuf += sprintf(pBuf, "%2d: %2.2d%%-%2.2d%%  ",
                            displayZoneNum+z + 1,
                            config.zone[z].minMoist,
                            config.zone[z].maxMoist);
        }
        else
        {
            pBuf += sprintf(pBuf, "%2d: --       ",
                            displayZoneNum+z + 1,
                            config.zone[z].minMoist,
                            config.zone[z].maxMoist);
        }

        if (++c > 2)
        {
            c = 0;
            r++;
        }
    }

    uint8_t row = (uiField - uiFirstField) / 3;
    uint8_t col = (uiField - uiFirstField) % 3;
    if (config.zone[uiField].group == uiField)
    {
        uiLcdCursor = LCD_RC(row, (col * 13) + 5);
        if (uiField2 != 0)
        {
            uiLcdCursor += 4;
        }
    }
    else
    {
        uiLcdCursor = LCD_RC(row, (col * 13) + 4);
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiMoistNext(void)
{
    if (uiField2 == 0 && config.zone[uiField].group == uiField &&
        (config.zone[uiField].sensorType == SNS_WIRED_MOIST 
        || config.zone[uiField].sensorType == SNS_WIRELESS_MOIST 
        || config.zone[uiField].sensorType == SNS_WIRELESS_VALVE ))
    {
        uiField2 = 1;
    }
    else
    {
        uiField2 = 0;
        if (++uiField >= navEndZone)
        {
            uiField = 0;
        }
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiMoistPrev(void)
{
    if (uiField2 > 0)
    {
        uiField2 = 0;
    }
    else
    {
        if (uiField-- == 0)
        {
            uiField = navEndZone - 1;
        }
        if ((config.zone[uiField].group == uiField)&&
            (config.zone[uiField].sensorType == SNS_WIRED_MOIST 
            || config.zone[uiField].sensorType == SNS_WIRELESS_MOIST
            || config.zone[uiField].sensorType == SNS_WIRELESS_VALVE))
        {
            uiField2 = 1;
        }
    }
}

/* Soft Key Action Routine - Jump to zone 13 ("13-24") */
static void uiMoistGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 12;
    uiField2 = 0;
    uiFirstField = 12;
}
/* Soft Key Action Routine - Jump to zone 25 ("25-36") */
static void uiMoistGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 24;
    uiField2 = 0;
    uiFirstField = 24;
}
/* Soft Key Action Routine - Jump to zone 37 ("37-48") */
static void uiMoistGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 36;
    uiField2 = 0;
    uiFirstField = 36;
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiMoistInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (uiField2 == 0)
    {
        if (++config.zone[uiField].minMoist >= config.zone[uiField].maxMoist)
        {
            config.zone[uiField].maxMoist = config.zone[uiField].minMoist + 1;
        }
    }
    else
    {
        ++config.zone[uiField].maxMoist;
    }
}





/* Soft Key Action Routine - Decrement Value ("-") */
static void uiMoistDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (uiField2 == 0)
    {
        --config.zone[uiField].minMoist;
    }
    else
    {
        if (--config.zone[uiField].maxMoist <= config.zone[uiField].minMoist)
        {
            config.zone[uiField].minMoist = config.zone[uiField].maxMoist - 1;
        }
    }
}


/* Soft Key Action Routine - COPY */
static void uiMoistCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuMoistCopy);
}


/* Display Action Routine */
static void uiMoistCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d min/max setpoints of",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "%d-%d%% to all other zones?",
            config.zone[uiField].minMoist,
            config.zone[uiField].maxMoist);
}


/* Soft Key Action Routine - YES */
static void uiMoistCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone moisture sensor setpoints to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].minMoist = config.zone[uiField].minMoist;
            config.zone[z].maxMoist = config.zone[uiField].maxMoist;
        }
    }
    uiMenuSet(&uiMenuMoist);
}


/* Soft Key Action Routine - NO */
static void uiMoistCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuMoist);
}



/******************************************************************************
 *
 *  MOISTURE SENSOR VALUES MENU ACTIONS
 *
 *  (Accessed via alt-function press of SETPOINTS function key.)
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zone pages, 12 zones at a
 *        time (uiField).  Each screen page displays up to 12 zones.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiMoistValDisplay(void)
{
    int8_t moisture;
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY6 );
    }
    
    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
        expansionBusSendCmd(RADIO_CMD_GET_MOIST, RADIO_EXP_SEND_ALL); 
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY3|UI_KM_KEY4 );
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY4 );
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 );
          break;
    }
    
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY2 | UI_KM_KEY3 | UI_KM_KEY4);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4);
          break;
    }

    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    /* Handle tick refresh event. */
    if (uiLcdRefreshTick)
    {
        /* Request new moisture samples (every 1-sec tick). */
        moistSampleAll();
    }

    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);

    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[i * 10 + 0];

        if (moistSensorConfigured(z + 1))
        {
            moisture = moistValueGet(z + 1);
            if (moisture >= 0)
            {
                switch(config.zone[z].sensorType)
                {
                    case SNS_FLOW:
                        pBuf += sprintf(pBuf, "%2d:%3dGPM ",
                                        displayZoneNum+z + 1,
                                        moisture);                
                        break;
                    case SNS_PRESSURE:
                        pBuf += sprintf(pBuf, "%2d:%3dPSI ",
                                        displayZoneNum+z + 1,
                                        moisture);
                        break;
                    case SNS_RAIN_GAUGE:
                        pBuf += sprintf(pBuf, "%2d:%3dIN  ",
                                        displayZoneNum+z + 1,
                                        moisture);
                        break;                
                    default:
                        pBuf += sprintf(pBuf, "%2d:%3d%%   ",
                                displayZoneNum+z + 1,
                                moisture);
                        //pBuf += sprintf(pBuf, "%d,%d,%d,",
                        //               displayZoneNum+z + 1,config.sys.unitType,moisture);        
                }
            }
            else    /* sensor failed */
            {
                pBuf += sprintf(pBuf, "%2d: FAIL",
                                displayZoneNum+z + 1);
                //pBuf += sprintf(pBuf, "%2d,%d,%d,",
                //                       displayZoneNum+z + 1,z,moisture);                
            }
        }
        else 
        {
            /* sensor not configured */
            pBuf += sprintf(pBuf, "%2d: --    ",
                            displayZoneNum+z + 1);
        }
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiMoistValNext(void)
{
    uiField += 12;
    if (uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiMoistValPrev(void)
{
    if (uiField == 0)
    {
        uiField = ((navEndZone - 1) / 12) * 12;
    }
    else
    {
        uiField -= 12;
    }
}

/* Soft Key Action Routine - Jump to zone 1 ("1-12") */
static void uiMoistValGroup1(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 0;
    uiFirstField = 0;
}

/* Soft Key Action Routine - Jump to zone 13 ("13-24") */
static void uiMoistValGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 12;
    uiFirstField = 12;
}
/* Soft Key Action Routine - Jump to zone 25 ("25-36") */
static void uiMoistValGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 24;
    uiFirstField = 24;
}
/* Soft Key Action Routine - Jump to zone 37 ("37-48") */
static void uiMoistValGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField = 36;
    uiFirstField = 36;
}

/******************************************************************************
 *
 *  SENSOR GROUPS FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiField2 is used to select between the sensor type/params fields.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through the sensor type/params (uiField2)
 *        then zones (uiField).  The summary screen is located prior to the
 *        first zone.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiGroupsSummaryDisplay(void)
{
    enum
    {
        GM_MULTIPLE_GROUPS,
        GM_SENSOR_PER_ZONE,
        GM_ONE_SENSOR_PER_SYSTEM,
    } groupMode = GM_MULTIPLE_GROUPS;   /* default is multiple group mode */
    int8_t masterZone = -1;
    int numGroups = 0;
    int numWithoutSensors = 0;
    int8_t numGenericSensors =0;
    uint8_t offset =0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;
    
    switch(config.sys.unitType)
    {
        case UNIT_TYPE_EXPANSION_1:
            offset =12;
            endingZone=config.sys.numZones;
            break;
        case UNIT_TYPE_EXPANSION_2:
            offset =24;
            endingZone=config.sys.numZones;
            break;
        case UNIT_TYPE_EXPANSION_3:
            offset =36;
            endingZone=config.sys.numZones;
            break;
    }

    if (config.sys.opMode == CONFIG_OPMODE_SENSOR)
    {
        sprintf(&uiLcdBuf[LCD_RC(0, 13)],
                "Sensor Groups");
    }
    else
    {
        sprintf(&uiLcdBuf[LCD_RC(0, 1)],
            "Sensor Groups (Sensor mode not enabled)");
    }

    /* Test for sensor per zone mode. */
    groupMode = GM_SENSOR_PER_ZONE;     /* set provisionally */
    for (int z = 0; z < endingZone; z++)
    {
        if (config.zone[z].group != (z+offset))
        {
            masterZone = config.zone[z].group;
            groupMode = GM_MULTIPLE_GROUPS;  /* set back to default */
            break;
        }
    }
    if (groupMode == GM_MULTIPLE_GROUPS)
    {
        /* Test for one sensor per system mode. */
        groupMode = GM_ONE_SENSOR_PER_SYSTEM;   /* set provisionally */
        for (int z = 0; z < endingZone; z++)
        {
            if (config.zone[z].group != masterZone)
            {
                groupMode = GM_MULTIPLE_GROUPS;  /* set back to default */
                break;
            }
        }
    }
    if (groupMode == GM_MULTIPLE_GROUPS)
    {
        /* Count the groups. */
        for (int z = 0; z < endingZone; z++)
        {
            if ((config.zone[z].group == z) &&
                ((config.zone[z].sensorType == SNS_WIRED_MOIST)   ||
                (config.zone[z].sensorType == SNS_WIRELESS_MOIST) ||
                (config.zone[z].sensorType == SNS_WIRELESS_VALVE)))
            {
                numGroups++;
            }
            else if ((config.zone[z].group == CONFIG_GROUP_NONE) &&
                     (config.zone[z].sensorType == SNS_NONE))
            {
                numWithoutSensors++;
            }
            else
            {
                numGenericSensors++;
            }
        }
    }

    switch (groupMode)
    {
        case GM_SENSOR_PER_ZONE:        /* sensor per zone mode */
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                    "Sensor per Zone configuration");
                sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                    "Each zone uses its own sensor");
           break;
        case GM_ONE_SENSOR_PER_SYSTEM:  /* one sensor per system mode */
            if (masterZone >= 0)
            {
                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                    "Sensor per System configuration");
                sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                    "System sensor is located in zone %d", masterZone + 1);
            }
            else
            {
                sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                        "No Sensors Configured");
            }
            break;
        case GM_MULTIPLE_GROUPS:        /* multiple group mode */
        default:
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "There %s %d sensor group%s",
                numGroups == 1 ? "is" : "are",
                numGroups,
                numGroups == 1 ? "" : "s");
            if (numWithoutSensors > 0)
            {
                sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                    "%d zone%s configured for no sensor",
                    numWithoutSensors,
                    numWithoutSensors == 1 ? " is" : "s are");
            }
                sprintf(&uiLcdBuf[LCD_RC(3, 0)],
                    "%d zone%s set to nonmoisture sensor",
                    numGenericSensors,
                    numGenericSensors == 1 ? " is" : "s are");
            break;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiGroupsSummaryNext(void)
{
       
    uiMenuSet(&uiMenuGroups);
    uiField = 0;
    uiFirstField = 0;
    uiField2 = 0;
    
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiGroupsSummaryPrev(void)
{
    uiMenuSet(&uiMenuGroups);
    uiField = navEndZone - 1;
    uiFirstField = 0;
    /* ...and position to the last field of this zone */
    if (config.zone[uiField].group == uiField)
    {
        /* wired sensor (group lead) - only one field */
        uiField2 = UI_GROUPS_FIELD_TYPE;
    }
    else
    {
        /* no sensor - position to second field */
        uiField2 = UI_GROUPS_FIELD_PARAM;
    }
}


/* Display Action Routine */
static void uiGroupsDisplay(void)
{
    uint8_t row;
    uint8_t col;
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;
    char * const WMoist   = "%2d: Wireless Val/Snsr";
    char * const WValve   = "%2d: Wireless Val Only";
    char *valve;
    char macStart;
    char chanStart;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    
    // mask out softkey short cuts based on number of units.
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    //don't allow user to edit groups while runing, just plain dangerous
    if((sysState != SYS_STATE_IDLE))
    {
      uiKeyMask |=  UI_KM_KEY2 | UI_KM_KEY1;
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2);
    }
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(1);

    for (int r = 0, z = uiFirstField;
         r < 3 && z < endingZone;
         r++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(r, 0)];
        
        //Reset values each row
        valve           = WMoist;
        macStart        = UI_WS_MAC_ADR_START;
        chanStart       = UI_WS_CHAN_STRT;

        if((config.zone[z].group == z) ||
           (config.zone[z].sensorType == SNS_WIRELESS_VALVE) ||
           (config.zone[z].sensorType == SNS_FLOW) ||
           (config.zone[z].sensorType == SNS_PRESSURE) ||
           (config.zone[z].sensorType == SNS_RAIN_GAUGE)) 
        {
            switch(config.zone[z].sensorType)
            {
                case SNS_WIRED_MOIST:
                    pBuf += sprintf(pBuf, "%2d: Wired Moist",
                            displayZoneNum + z + 1);
                    break;
                case SNS_WIRELESS_VALVE:
                    valve     = WValve;
                //    macStart  = UI_WV_MAC_ADR_START;
                //    chanStart = UI_WV_CHAN_STRT;
                case SNS_WIRELESS_MOIST:
                                
                    sprintf(&uiLcdBuf[LCD_RC(r, 0)], valve, displayZoneNum + z + 1);
                    
                    /*display mac id of sensor concentrator or notice that none are associated */
                    if( (config.zone[z].snsConTableIndex == ZONE_SC_INDEX_NONE) ||(config.sys.assocSensorCon[config.zone[z].snsConTableIndex].macId == 0) )
                    {
                        sprintf(&uiLcdBuf[LCD_RC(r, macStart)], "--------",
                            (uint32_t)(config.sys.assocSensorCon[config.zone[z].snsConTableIndex].macId & 0xFFFFFFFF));
                    }
                    else
                    {
                        sprintf(&uiLcdBuf[LCD_RC(r, macStart)], "%08X",
                            (uint32_t)(config.sys.assocSensorCon[config.zone[z].snsConTableIndex].macId & 0xFFFFFFFF));
                    }
                    
                    /*display assigned channel or notice that none are open */
                    switch(config.zone[z].snsConChan)
                    {
                        default:
                            sprintf(&uiLcdBuf[LCD_RC(r, chanStart)], "%c", config.zone[z].snsConChan + 'A');
                            break;
                        case SC_NONE_SELECTED:
                            sprintf(&uiLcdBuf[LCD_RC(r, chanStart)], "-");
                            break;
                        case SC_NONE_OPEN:
                            sprintf(&uiLcdBuf[LCD_RC(r, chanStart)], "NotAvail");
                            break;
                    }      
                    
                    if( (SNS_WIRELESS_VALVE == config.zone[z].sensorType) && (SC_NONE_OPEN != config.zone[z].snsConChan))
                    {
                      if (config.zone[z].group >= 0)
                      {
                          sprintf(&uiLcdBuf[LCD_RC(r, UI_WV_GRP_STRT)], "Grp %2d", config.zone[z].group + 1+displayZoneNum);
                      }
                      else
                      {
                          sprintf(&uiLcdBuf[LCD_RC(r, UI_WV_GRP_STRT)], "No Grp");
                      }
                    }
                    
                    break;
                case SNS_FLOW:
                   
                    sprintf(&uiLcdBuf[LCD_RC(r, 0)], "%2d: Flow  0-",
                            displayZoneNum + z + 1);
        
                    /* display upper range*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 12)], "%3d",
                            config.zone[z].maxMoist);
                    
                    /*display units*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 16)], "GPM");
                    
                    /* set off delay*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 23)], "OFF DELAY:");
                                        
                    if (flowDelay == 0)
                        sprintf(&uiLcdBuf[LCD_RC(r, 34)], "OFF");
                    else
                        sprintf(&uiLcdBuf[LCD_RC(r, 34)], "%dm%ds",flowDelay/60,flowDelay%60);
                    
                            
                    break;
                case SNS_PRESSURE:
                    
                    sprintf(&uiLcdBuf[LCD_RC(r, 0)], "%2d: Pressure  0-",
                            displayZoneNum + z + 1);
        
                    /* display upper range*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 16)], "%3d",
                            config.zone[z].maxMoist);
                    
                    /*display units*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 20)], "PSI");
                    break;
                case SNS_RAIN_GAUGE:
                    sprintf(&uiLcdBuf[LCD_RC(r, 0)], "%2d: Rain Gauge  0-",
                            displayZoneNum + z + 1);
        
                    /* display upper range*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 19)], "%3d",
                            config.zone[z].maxMoist);
                    
                    /*display units*/
                    sprintf(&uiLcdBuf[LCD_RC(r, 23)], "INCHES");
                    break;
            }
        }
        else if ((config.zone[z].group >= 0) && (SNS_WIRELESS_VALVE != config.zone[z].sensorType))
        {
             
            pBuf += sprintf(pBuf, "%2d: No Sensor     Grouped with Zone %2d",
                      displayZoneNum + z + 1,
                      config.zone[z].group + 1+displayZoneNum);
        }
        else
        {
            if(SNS_WIRELESS_VALVE != config.zone[z].sensorType)
            {
                //print the wireless valve group                           
                pBuf += sprintf(pBuf, "%2d: No Sensor     Not Grouped",
                            displayZoneNum + z + 1);
            }
        }
    }

    row = (uiField - uiFirstField) / 1;
    switch (uiField2)
    {
        case UI_GROUPS_FIELD_TYPE:
        default:
            col = 4;
            break;
        case UI_GROUPS_FIELD_PARAM:
                switch(config.zone[uiField].sensorType)
                {
                    case SNS_WIRELESS_MOIST:
                    case SNS_WIRELESS_VALVE:
                        col = UI_WS_MAC_ADR_START;
                        break;
                    case SNS_FLOW:
                        col = 12;
                        break;
                    case SNS_PRESSURE:
                        col = 16;
                        break;
                    case SNS_RAIN_GAUGE:
                        col = 18;
                        break;
                    default:
                        col = 18;
                }

            break;
        case UI_GROUPS_FIELD_PARAM_2:
            if (config.zone[uiField].sensorType == SNS_FLOW)
                col  = 33;
            else
                col = UI_WS_CHAN_STRT;
            break;
        case UI_GROUPS_FIELD_PARAM_3: //New grouping field for wireless valves
            col = UI_WV_GRP_STRT;
            break;
    }
    uiLcdCursor = LCD_RC(row, col);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiGroupsNext(void)
{
    /*
     * This needs to deal with the conditional fields, skipping over sensor
     * parameter field(s) depending on sensor type.  Lots of fun...
     */
     //uiField2; which field we are on (col)
     //uiField;  The field type of this zone (row)
       
   //  if ( uiField2 == (UI_GROUPS_FIELD_LIMIT -1) //wireless valve uses all fields
   //       || (((config.zone[uiField].group == uiField) || (uiField2 == (UI_GROUPS_FIELD_LIMIT - 1) )) && (config.zone[uiField].sensorType == SNS_WIRED_MOIST))
   //       || (uiField2 == (UI_GROUPS_FIELD_LIMIT - 2) && (config.zone[uiField].sensorType == SNS_WIRELESS_MOIST))
   //       || (uiField2 == (UI_GROUPS_FIELD_LIMIT - 3) && ( (config.zone[uiField].sensorType != SNS_WIRELESS_MOIST) 
   //          && (config.zone[uiField].sensorType != SNS_WIRELESS_VALVE)))
   //     )
        
if (   ((uiField2 == 0) && (config.zone[uiField].sensorType == SNS_WIRED_MOIST ))
    || ((uiField2 == 2) && (config.zone[uiField].sensorType == SNS_WIRELESS_MOIST ))
    || ((uiField2 == 3) && (config.zone[uiField].sensorType == SNS_WIRELESS_VALVE))
    || ((uiField2 == 2) && (config.zone[uiField].sensorType == SNS_FLOW))        
    || ((uiField2 == 1) && (config.zone[uiField].sensorType == SNS_PRESSURE))       
    || ((uiField2 == 1) && (config.zone[uiField].sensorType == SNS_NONE))
    || ((uiField2 == 1) && (config.zone[uiField].sensorType == SNS_RAIN_GAUGE)) )             
    {
          /* was on last field for this zone - advance to next zone */
          uiField2 = 0;
          if (++uiField >= navEndZone)
          {
              uiMenuSet(&uiMenuGroupsSummary);
          }
    }
    else
    {
        /* advance to next field for this zone */
        uiField2++;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiGroupsPrev(void)
{
    /*
     * This needs to deal with the conditional fields, skipping over sensor
     * parameter field(s) depending on sensor type.  Lots of fun...
     */

    if (uiField2 > 0)
    {
        /* was on non-first field of a multi-field zone - just back up one */
        uiField2--;
    }
    else
    {
        /* was on first field, so back up one zone (with wrap)... */
        if (uiField-- == 0)
        {
            uiMenuSet(&uiMenuGroupsSummary);
        }

        /* ...and position to the last field of this zone */
        if (config.zone[uiField].group == uiField)
        {
            /* wired sensor (group lead) - only one field */
            uiField2 = UI_GROUPS_FIELD_TYPE;
        }
        else
        {
            /* no sensor - position to second field */
            uiField2 = UI_GROUPS_FIELD_PARAM;
        }
    }
   
}

/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiGroupsGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
   uiField = 12; 
   uiFirstField =12;
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiGroupsGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
   uiField = 24; 
   uiFirstField =24;
}

/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiGroupsGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
   uiField = 36; 
   uiFirstField =36;
}

/* Soft Key Action Routine - Increment Value ("+") */
static void uiGroupsInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t uiCurGroup = config.zone[uiField].group;
    uint8_t minGroupZone = 0;
    uint8_t maxGroupZone = 11;
    uint8_t zoneRange;
    uint8_t searchCount =0;
    uint8_t tempZoneSnsChan;
    uint8_t numAttempts=0;
    int8_t snsConIndex = config.zone[uiField].snsConTableIndex;
    uint8_t chanIndex = config.zone[uiField].snsConChan;
    
    if((uiField >=12)&(uiField <=23))
    {
        minGroupZone = 12;
        maxGroupZone = 23;
    } 
    else if((uiField >=24)&(uiField <=35))
    {
        minGroupZone = 24;
        maxGroupZone = 35;
    }
    else if((uiField >=36)&(uiField <=47))
    {
        minGroupZone = 36;
        maxGroupZone = 47;
    }
          

    switch (uiField2)
    {
        //This is the sensor type field
        case UI_GROUPS_FIELD_TYPE:
            /*increment sensor type */
            config.zone[uiField].sensorType++;
            
            if(config.zone[uiField].sensorType >= SNS_TYPE_MAX)
            {
                config.zone[uiField].sensorType = SNS_NONE;
            }
            
            /* if previous type was wireless sensor then modify zone settings accordingly */
            if( ((config.zone[uiField].sensorType-1) ==SNS_WIRELESS_MOIST ) || ((config.zone[uiField].sensorType-1) ==SNS_WIRELESS_VALVE ) )
            {
                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]= SC_CHAN_NOT_ASSIGNED;
                config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE;
                config.zone[uiField].snsConChan = SC_NONE_SELECTED;
            }
            
            /* set min and max moisture values back to default*/
            config.zone[uiField].minMoist = CONFIG_MOIST_DEFAULT_MIN;
            config.zone[uiField].maxMoist = CONFIG_MOIST_DEFAULT_MAX;
            
            /* if new type is wireless then modify zone settings accordingly */
            switch(config.zone[uiField].sensorType )
            {
                case  SNS_WIRED_MOIST:
                case  SNS_WIRELESS_VALVE:          
                case  SNS_WIRELESS_MOIST:
                
                    /* Set moisture sensor sample frequency to active rate (1/hr). */
                     moistFreqSet(uiField+1, MOIST_FREQ_INACTIVE);
                     break;
                case SNS_FLOW:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_MAX_FLOW;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;
                case SNS_RAIN_GAUGE:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_DEFAULT_RAIN_GAUGE;;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;
                case SNS_PRESSURE:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_DEFAULT_PRESSURE;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;
            }             
            
 
             
            if ((config.zone[uiField].sensorType ==SNS_NONE) || (config.zone[uiField].sensorType == SNS_WIRELESS_VALVE) )
            {                 
                
                /* Set moisture sensor sample frequency to off */
                moistFreqSet(uiField+1, MOIST_FREQ_OFF);
                     
                /* group lead - change to None (and no group) */
                if (uiGroupsCountFollowers(uiField) > 0)
                {
                    /* about to delete a sensor that has followers - ask first */
                    uiMenuSet(&uiMenuGroupsUngroup);
                }
                else
                {
                    config.zone[uiField].group = CONFIG_GROUP_NONE;
                }
            }
            else
            {               
                /* change to group lead */
                config.zone[uiField].group = uiField;
            } 
            break;

        //This is the SC MAC address field
        case UI_GROUPS_FIELD_PARAM:
            switch(config.zone[uiField].sensorType)
            {
                case SNS_NONE:
                    do
                    {
                        /* increment to next zone */
                        config.zone[uiField].group++;
                        if (config.zone[uiField].group == uiField)
                        {
                            continue;
                        }
                        if (config.zone[uiField].group > maxGroupZone)
                        {
                            config.zone[uiField].group = CONFIG_GROUP_NONE;
                            break;
                        }
                        if (config.zone[uiField].group < minGroupZone)
                        {
                            config.zone[uiField].group = minGroupZone;
                    
                            if (config.zone[uiField].group == uiField)
                            {
                                continue;
                            }
                        }
                        /* exit if acceptable value (zone is a group leader) */
                        if (config.zone[config.zone[uiField].group].group == config.zone[uiField].group)
                        {
                            /* selected group is a group leader */
                            break;
                        }
                    } while (config.zone[uiField].group != uiCurGroup);
                    break;
                case SNS_WIRELESS_MOIST:
                case SNS_WIRELESS_VALVE:

                    if(config.sys.numSensorCon != 0)
                    {
                     
                        /*set which zone range need to look for based on current zone selected */
                        if(uiField < 12)
                        {
                            zoneRange = UNIT_TYPE_MASTER;
                        }
                        else if(uiField <24)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_1;
                        }
                        else if(uiField <36)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_2;
                        }
                        else if(uiField < 48)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_3;
                        }
                         
                        /* update the associated sensor concentrator information */
                        if((snsConIndex >= 0) && (chanIndex <SC_NUM_CHAN_UNIT))
                        {
                            config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                        }
                        
                        /*search for SC assigned to zone range */
                        do
                        {
                            config.zone[uiField].snsConTableIndex++;
                                                    
                            
                            //wrap back to beginning of sc list                        
                            if(config.zone[uiField].snsConTableIndex == config.sys.numSensorCon)
                            {
                                config.zone[uiField].snsConTableIndex = 0;
                            }
                            
                            //have we scanned entire list?
                            if(searchCount == config.sys.numSensorCon)
                            {
                                config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE; 
                                break;
                            }
                            searchCount++;
                        } while(config.sys.assocSensorCon[config.zone[uiField].snsConTableIndex].zoneRange != zoneRange);

                        /* update the associated sensor concentrator information */
                        if(config.zone[uiField].snsConTableIndex != ZONE_SC_INDEX_NONE)
                        {
                            //Clear Group value so we don't acidentally overwrite a valid group
                            //#warning "clear group value field"
                            config.zone[uiField].snsConChan = SC_NONE_SELECTED;
                            
                            //set group
                            //KAV
                            //config.sys.assocSensorCon[config.zone[uiField].snsConTableIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;//uiField;
                        }
                    }
                    break;
                 case SNS_FLOW:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist < SNS_MAX_FLOW)
                    {
                        config.zone[uiField].maxMoist++;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = config.zone[uiField].minMoist+1;
                    }
                    break; 
                 case SNS_PRESSURE:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist < SNS_MAX_PRESSURE)
                    {
                        config.zone[uiField].maxMoist++;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = config.zone[uiField].minMoist+1;
                    }
                    break;
                 case SNS_RAIN_GAUGE:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist < SNS_MAX_RAIN_GAUGE)
                    {
                        config.zone[uiField].maxMoist++;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = config.zone[uiField].minMoist+1;
                    }
                    break;                   
            }
            break;
        case UI_GROUPS_FIELD_PARAM_2:
            switch(config.zone[uiField].sensorType)
            {
                case SNS_WIRELESS_MOIST:
                case SNS_WIRELESS_VALVE:
                   
                    /*check to see if SC is still associated with these zones */
                    /*set which zone range need to look for based on current zone selected */
                    if(uiField < 12)
                    {
                        zoneRange = UNIT_TYPE_MASTER;
                    }
                    else if(uiField <24)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_1;
                    }
                    else if(uiField <36)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_2;
                    }
                    else if(uiField < 48)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_3;
                    }
                    
                    if(config.sys.assocSensorCon[snsConIndex].zoneRange != zoneRange)
                    {
                        config.zone[uiField].snsConChan = SC_NONE_SELECTED;
                        config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE;
                        break;
                    }
                    
                    /*if sensor concentrator is still valid for these zones proceed with incrementing channel*/    
                    if(snsConIndex != ZONE_SC_INDEX_NONE)
                    {

                        tempZoneSnsChan =config.zone[uiField].snsConChan;
                        numAttempts =0;
                        
                        /* search for open channel */
                        while(numAttempts < SC_NUM_CHAN_UNIT)
                        {
                            numAttempts++;
                            
                            /* increment channel index */
                            tempZoneSnsChan++;
                            if(tempZoneSnsChan >= SC_NONE_SELECTED)
                            {
                                tempZoneSnsChan = 0;
                            }
                            
                            /*if channel is available then assign it, if not keep looking */
                            if(config.sys.assocSensorCon[snsConIndex].channelZone[tempZoneSnsChan]==SC_CHAN_NOT_ASSIGNED)
                            {
                                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                                config.zone[uiField].snsConChan = tempZoneSnsChan;
                                break;
                            }
                            /*if the last one in the list is not available then set it to none open */
                            if(numAttempts ==SC_NUM_CHAN_UNIT)
                            {
                                config.zone[uiField].snsConChan = SC_NONE_OPEN;
                                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                            }
                        }
                    }
                    break;
                case    SNS_FLOW:
                    if (flowDelay == 300)
                        flowDelay = 0;
                    else 
                        flowDelay = flowDelay + 30;
                    break;
                default:
                    break;
            }
            /* update the associated sensor concentrator information */
            config.sys.assocSensorCon[snsConIndex].channelZone[config.zone[uiField].snsConChan]=uiField;
            break;   
        case UI_GROUPS_FIELD_PARAM_3:
            //select group for wireless valve only
            if(SNS_WIRELESS_VALVE == config.zone[uiField].sensorType)
            {
                do
                {
                    /* increment to next zone */
                    config.zone[uiField].group++;
                    if (config.zone[uiField].group == uiField)
                    {
                        continue;
                    }
                    if (config.zone[uiField].group > maxGroupZone)
                    {
                        config.zone[uiField].group = CONFIG_GROUP_NONE;
                        break;
                    }
                    if (config.zone[uiField].group < minGroupZone)
                    {
                        config.zone[uiField].group = minGroupZone;
                
                        if (config.zone[uiField].group == uiField)
                        {
                            continue;
                        }
                    }
                    /* exit if acceptable value (zone is a group leader) */
                    if (config.zone[config.zone[uiField].group].group == config.zone[uiField].group)
                    {
                        /* selected group is a group leader */
                        break;
                    }
                } while (config.zone[uiField].group != uiCurGroup);
             }
            break;

    }
    
    /* fix any broken group references */
    uiGroupsCheckZone(uiField);

    /* update sensor sample frequency */
    irrMoistConfigUpdate();
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiGroupsDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t uiCurGroup = config.zone[uiField].group;
    uint8_t minGroupZone = 0;
    uint8_t maxGroupZone = 11;
    uint8_t zoneRange;
    uint8_t searchCount =0;
    uint8_t tempZoneSnsChan;
    uint8_t numAttempts=0;
    int8_t snsConIndex = config.zone[uiField].snsConTableIndex;
    uint8_t chanIndex = config.zone[uiField].snsConChan;
    
    if((uiField >=12)&(uiField <=23))
    {
        minGroupZone = 12;
        maxGroupZone = 23;
    } 
    else if((uiField >=24)&(uiField <=35))
    {
        minGroupZone = 24;
        maxGroupZone = 35;
    }
    else if((uiField >=36)&(uiField <=47))
    {
        minGroupZone = 36;
        maxGroupZone = 47;
    }
    
    switch (uiField2)
    {
        case UI_GROUPS_FIELD_TYPE:

            
             if(config.zone[uiField].sensorType == SNS_NONE)
             {
                config.zone[uiField].sensorType = SNS_RAIN_GAUGE;;
             }
             else
             {
                /*decrement sensor type */
                config.zone[uiField].sensorType--;
             }
             
            /* if previous type was wireless sensor then modify zone settings accordingly */
            if( ((config.zone[uiField].sensorType+1) ==SNS_WIRELESS_MOIST ) || ((config.zone[uiField].sensorType+1) ==SNS_WIRELESS_VALVE ) )
            {
                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]= SC_CHAN_NOT_ASSIGNED;
                config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE;
                config.zone[uiField].snsConChan = SC_NONE_SELECTED;
            }
            
            /* set min and max moisture values back to default*/
            config.zone[uiField].minMoist = CONFIG_MOIST_DEFAULT_MIN;
            config.zone[uiField].maxMoist = CONFIG_MOIST_DEFAULT_MAX;
                 
            /* if new type is wireless then modify zone settings accordingly */
            switch(config.zone[uiField].sensorType )
            {
                case  SNS_WIRED_MOIST:
                case SNS_WIRELESS_MOIST:
                case SNS_WIRELESS_VALVE:
                     /* Set moisture sensor sample frequency to active rate (1/hr). */
                     moistFreqSet(uiField+1, MOIST_FREQ_INACTIVE);
                     break;
                case SNS_FLOW:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_MAX_FLOW;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;
                case SNS_RAIN_GAUGE:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_DEFAULT_RAIN_GAUGE;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;
                case SNS_PRESSURE:
                     /* if new type is another generic sensor then modify min and max values to set range */
                     config.zone[uiField].minMoist = 0;
                     config.zone[uiField].maxMoist = SNS_MAX_PRESSURE;
                     /* Set moisture sensor sample frequency to active rate (1/min). */
                     moistFreqSet(uiField+1, MOIST_FREQ_ACTIVE);
                     break;   
                
            }
            
             if ((config.zone[uiField].sensorType ==SNS_NONE) || (config.zone[uiField].sensorType == SNS_WIRELESS_VALVE) )
             {
                 /* Set moisture sensor sample frequency to off. */
                 moistFreqSet(uiField+1, MOIST_FREQ_OFF);
                 
                 /* group lead - change to None (and no group) */
                 if (uiGroupsCountFollowers(uiField) > 0)
                 {
                     /* about to delete a sensor that has followers - ask first */
                     uiMenuSet(&uiMenuGroupsUngroup);
                 }
                 else
                 {
                     config.zone[uiField].group = CONFIG_GROUP_NONE;
                 }
             }
             else
             {

                 
                 /* change to group lead */
                 config.zone[uiField].group = uiField;
             }           

             break;

        case UI_GROUPS_FIELD_PARAM:
            switch(config.zone[uiField].sensorType)
            {
                case SNS_NONE:
                    do
                    {
                        /* decrement to previous zone */
                        config.zone[uiField].group--;
                        if (config.zone[uiField].group == uiField)
                        {
                            continue;
                        }
                        if (config.zone[uiField].group == CONFIG_GROUP_NONE)
                        {
                            break;
                        }
                        if ((config.zone[uiField].group < minGroupZone)&
                            (minGroupZone !=0)&(config.zone[uiField].group>CONFIG_GROUP_NONE)) 
                        {
                            config.zone[uiField].group = CONFIG_GROUP_NONE;
                            break;
                        }
                        if (config.zone[uiField].group < CONFIG_GROUP_NONE)
                        {
                            config.zone[uiField].group = maxGroupZone;
                        }
                        /* exit if acceptable value (zone is a group leader) */
                        if (config.zone[config.zone[uiField].group].group == config.zone[uiField].group)
                        {
                            /* selected group is a group leader */
                            break;
                        }
                    } while (config.zone[uiField].group != uiCurGroup);
                    break;
                case SNS_WIRELESS_MOIST:
                case SNS_WIRELESS_VALVE:
                    
                    /*TO DO: need to have it increment to the next SC that is assigned to those zones */
                    if(config.sys.numSensorCon != 0)
                    {
                        /*set which zone range need to look for based on current zone selected */
                        if(uiField < 12)
                        {
                            zoneRange = UNIT_TYPE_MASTER;
                        }
                        else if(uiField <24)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_1;
                        }
                        else if(uiField <36)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_2;
                        }
                        else if(uiField < 48)
                        {
                            zoneRange = UNIT_TYPE_EXPANSION_3;
                        }
                        /* update the associated sensor concentrator information */
                        if((snsConIndex >= 0) && (chanIndex <SC_NUM_CHAN_UNIT)) 
                        {
                            config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                        }
                        
                        /*search for SC assigned to zone range */
                        do
                        {
                            if(config.zone[uiField].snsConTableIndex <= 0)
                            {
                                config.zone[uiField].snsConTableIndex = config.sys.numSensorCon-1;
                            }
                            else
                            {
                                config.zone[uiField].snsConTableIndex--;
                            }
                            
                            if(searchCount == config.sys.numSensorCon)
                            {
                                config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE; 
                                break;
                            }
                            
                            searchCount++;
                        } while(config.sys.assocSensorCon[config.zone[uiField].snsConTableIndex].zoneRange != zoneRange);
                    
                        /* update the associated sensor concentrator information */
                        if(config.zone[uiField].snsConTableIndex != ZONE_SC_INDEX_NONE)
                        {
                            //Clear Group value so we don't acidentally overwrite a valid group
                            //#warning "clear group value field"
                            config.zone[uiField].snsConChan = SC_NONE_SELECTED;
                            
                            //set group
                            //KAV
                            //config.sys.assocSensorCon[config.zone[uiField].snsConTableIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;//uiField;
                        }
                    }
                    break;
                 case SNS_FLOW:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist > (config.zone[uiField].minMoist+1))
                    {
                        config.zone[uiField].maxMoist--;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = SNS_MAX_FLOW;
                    }
                    break; 
                 case SNS_PRESSURE:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist > (config.zone[uiField].minMoist+1))
                    {
                        config.zone[uiField].maxMoist--;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = SNS_MAX_PRESSURE;
                    }
                    break; 
                 case SNS_RAIN_GAUGE:
                    /* increment max possible sensor reading */
                    if(config.zone[uiField].maxMoist > (config.zone[uiField].minMoist+1))
                    {
                        config.zone[uiField].maxMoist--;
                    }
                    else
                    {
                        config.zone[uiField].maxMoist = SNS_MAX_RAIN_GAUGE;;
                    }
                    break; 
            }
            break;
        case UI_GROUPS_FIELD_PARAM_2:
            switch(config.zone[uiField].sensorType)
            {
                case SNS_WIRELESS_MOIST:
                case SNS_WIRELESS_VALVE:
                    /*check to see if SC is still associated with these zones */
                    /*set which zone range need to look for based on current zone selected */
                    if(uiField < 12)
                    {
                        zoneRange = UNIT_TYPE_MASTER;
                    }
                    else if(uiField <24)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_1;
                    }
                    else if(uiField <36)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_2;
                    }
                    else if(uiField < 48)
                    {
                        zoneRange = UNIT_TYPE_EXPANSION_3;
                    }
                    
                    if(config.sys.assocSensorCon[snsConIndex].zoneRange != zoneRange)
                    {
                        config.zone[uiField].snsConChan = SC_NONE_SELECTED;
                        config.zone[uiField].snsConTableIndex = ZONE_SC_INDEX_NONE;
                        break;
                    }
                    
                    /*if sensor concentrator is still valid for these zones proceed with decrementing channel*/
                    if(snsConIndex != ZONE_SC_INDEX_NONE)
                    {
                        if(config.zone[uiField].snsConChan < SC_NUM_CHAN_UNIT)
                        {
                            tempZoneSnsChan =config.zone[uiField].snsConChan;
                        }
                        else
                        {
                            tempZoneSnsChan =0;
                        }
                        numAttempts =0;
                        
                        /* search for open channel */
                        while(numAttempts < SC_NUM_CHAN_UNIT)
                        {
                            numAttempts++;
                            
                            /* decrement channel index */
                            if(tempZoneSnsChan == 0)
                            {
                                tempZoneSnsChan = SC_NUM_CHAN_UNIT-1;
                            }
                            else
                            {
                                tempZoneSnsChan--;
                            }
                            
                            /*if channel is available then assign it, if not keep looking */
                            if(config.sys.assocSensorCon[snsConIndex].channelZone[tempZoneSnsChan]==SC_CHAN_NOT_ASSIGNED)
                            {
                                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                                config.zone[uiField].snsConChan = tempZoneSnsChan;
                                break;
                            }
                            /*if the last one in the list is not available then set it to none open */
                            if(numAttempts ==SC_NUM_CHAN_UNIT)
                            {
                                config.zone[uiField].snsConChan = SC_NONE_OPEN;
                                config.sys.assocSensorCon[snsConIndex].channelZone[chanIndex]=SC_CHAN_NOT_ASSIGNED;
                            }
                        }
                    }
                    break;
                case    SNS_FLOW:
                    if (flowDelay == 0)
                        flowDelay = 300;
                    else 
                        flowDelay = flowDelay - 30;
                    break;
                default:
                    break;
            }
            /* update the associated sensor concentrator information */
            config.sys.assocSensorCon[snsConIndex].channelZone[config.zone[uiField].snsConChan]=uiField;
            break;
        case UI_GROUPS_FIELD_PARAM_3:
            //select group for wireless valve only
            if(SNS_WIRELESS_VALVE == config.zone[uiField].sensorType)
            {
                do
                {
                    /* decrement to previous zone */
                    config.zone[uiField].group--;
                    if (config.zone[uiField].group == uiField)
                    {
                        continue;
                    }
                    if (config.zone[uiField].group == CONFIG_GROUP_NONE)
                    {
                        break;
                    }
                    if ((config.zone[uiField].group < minGroupZone)&
                        (minGroupZone !=0)&(config.zone[uiField].group>CONFIG_GROUP_NONE)) 
                    {
                        config.zone[uiField].group = CONFIG_GROUP_NONE;
                        break;
                    }
                    if (config.zone[uiField].group < CONFIG_GROUP_NONE)
                    {
                        config.zone[uiField].group = maxGroupZone;
                    }
                    /* exit if acceptable value (zone is a group leader) */
                    if (config.zone[config.zone[uiField].group].group == config.zone[uiField].group)
                    {
                        /* selected group is a group leader */
                        break;
                    }
                } while (config.zone[uiField].group != uiCurGroup);
             }
            break;
    }

    /* fix any broken group references */
    uiGroupsCheckZone(uiField);

    /* update sensor sample frequency */
    irrMoistConfigUpdate();
}


/* Display Action Routine */
static void uiGroupsUngroupDisplay(void)
{
    int followers = uiGroupsCountFollowers(uiField);

    /* sensor type is Wired (group lead) - set all zones to Wired? */
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "The zone %d sensor is used by %d other",
            uiField + 1, followers);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "zone%s.  Set this zone for 'No Sensor'",
            (followers == 1) ? "" : "s");
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "and dissolve this sensor group?");
}


/* Soft Key Action Routine - YES */
static void uiGroupsUngroupYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* set zone for 'No Sensor' (with no group association) */
    config.zone[uiField].group = CONFIG_GROUP_NONE;

    /* fix the broken group references */
    uiGroupsCheckZone(uiField);

    /* update sensor sample frequency */
    irrMoistConfigUpdate();

    uiMenuSet(&uiMenuGroups);
}


/* Soft Key Action Routine - NO */
static void uiGroupsUngroupNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuGroups);
}

#if 0
/* Soft Key Action Routine - COPY */
static void uiGroupsCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuGroupsCopy);
}


/* Display Action Routine */
static void uiGroupsCopyDisplay(void)
{
    if (config.zone[uiField].group == uiField)
    {
        /* sensor type is Wired (group lead) - set all zones to Wired? */
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Configure each zone to use its own");
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "moisture sensor?");
    }
    else
    {
        /* sensor type is None - set all zones to use same sensor? */
        if (config.zone[uiField].group >= 0)
        {
            sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                    "Configure all zones to use the moisture");
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                    "sensor for zone %d?",
                    config.zone[uiField].group + 1);
        }
        else
        {
            sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                    "Configure all zones for no moisture");
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                    "sensors?");
        }
    }
}


/* Soft Key Action Routine - YES */
static void uiGroupsCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (config.zone[uiField].group == uiField)
    {
        /* set "multiple" moisture group configuration */
        for (int z = 0; z < SYS_N_ZONES; z++)
        {
            config.zone[z].group = z;
        }
    }
    else
    {
        /* set "single" moisture group configuration */
        for (int z = 0; z < SYS_N_ZONES; z++)
        {
            if (z != uiField)
            {
                config.zone[z].group = config.zone[uiField].group;
            }
        }
    }

    /* update sensor sample frequency */
    irrMoistConfigUpdate();

    uiMenuSet(&uiMenuGroups);
}


/* Soft Key Action Routine - NO */
static void uiGroupsCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuGroups);
}
#endif

/* Utility Routine - Check a zone to fix broken group leader references. */
static void uiGroupsCheckZone(uint8_t zone)
{
    /* If this group isn't a group leader, insure it is not referenced. */
    if (config.zone[zone].group != zone)
    {
        for (int z = 0; z < config.sys.numZones; z++)
        {
            if (z != zone && config.zone[z].group == zone)
            {
                /* The current group zone is not a group leader. */
                /* Fix by disassociating from all zones. */
                config.zone[z].group = CONFIG_GROUP_NONE;
            }
        }
    }
}


/* Utility Routine - Check all zones and fix broken group leader references. */
static void uiGroupsCheckAll(void)
{
    /* Check all zones and fix any broken group references. */
    for (int z = 0; z < config.sys.numZones; z++)
    {
        if (config.zone[z].group >= config.sys.numZones)
        {
            /* A decrease in configured number of zones broke the group. */
            /* Fix by disassociating from all zones. */
            config.zone[z].group = CONFIG_GROUP_NONE;
        }
        else
        {
            /* Perform additional group reference checks. */
            uiGroupsCheckZone(z);
        }
    }
}


/* Utility Routine - Count number of followers a zone has. */
static int uiGroupsCountFollowers(uint8_t zone)
{
    int followers = 0;

    for (int z = 0; z < config.sys.numZones; z++)
    {
        if (z != zone && config.zone[z].group == zone)
        {
            followers++;
        }
    }

    return followers;
}



/******************************************************************************
 *
 *  CLIMATE FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zones (uiField).
 *
 *****************************************************************************/

/*
 *  The navigation dial scrolls through zones (uiField).
 */


/* Display Action Routine */
static void uiClimateDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    // mask out softkey short cuts based on number of units.
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(2);

    for (int i = 0, z = uiFirstField;
         i < 6 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(0, 0) + (i * 20)];

        pBuf += sprintf(pBuf, "%2d: %-9.9s       ",
                        displayZoneNum + z + 1,
                        uiClimateNames[config.zone[z].climate]);
    }

    uint8_t row = (uiField - uiFirstField) / 2;
    uint8_t col = (uiField - uiFirstField) % 2;
    uiLcdCursor = LCD_RC(row, (col * 20) + 4);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiClimateNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiClimatePrev(void)
{
    if (uiField-- == 0)
    {
        uiField = navEndZone - 1;
    }
}


/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiClimateGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
       uiField = 12;
       uiFirstField =12;
}

/* Soft Key Action Routine - Group 2 ("25-36") */
static void uiClimateGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
       uiField = 24;
       uiFirstField =24;
}

/* Soft Key Action Routine - Group 2 ("37-48") */
static void uiClimateGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
       uiField = 36;
       uiFirstField =36;
}

/* Soft Key Action Routine - Increment Value ("+") */
static void uiClimateInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (++config.zone[uiField].climate >= CONFIG_CLIMATE_LIMIT)
    {
        config.zone[uiField].climate = 0;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiClimateDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (config.zone[uiField].climate-- == 0)
    {
        config.zone[uiField].climate = CONFIG_CLIMATE_LIMIT - 1;
    }
}


/* Soft Key Action Routine - COPY */
static void uiClimateCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuClimateCopy);
}


/* Display Action Routine */
static void uiClimateCopyDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d climate setting of",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "'%s' to all other zones?",
            uiClimateNames[config.zone[uiField].climate]);
}


/* Soft Key Action Routine - YES */
static void uiClimateCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone climate to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].climate = config.zone[uiField].climate;
        }
    }
    uiMenuSet(&uiMenuClimate);
}


/* Soft Key Action Routine - NO */
static void uiClimateCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuClimate);
}



/******************************************************************************
 *
 *  MOISTURE BALANCE (MB) VALUES MENU ACTIONS
 *
 *  (Accessed via alt-function press of CLIMATE function key.)
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zones (uiField).
 *
 *****************************************************************************/

/*
 *  The navigation dial scrolls through zones (uiField).
 */

/* Utility Routine - Format string for signed MB value. */
static char *uiMbValFormat(char *pBuf, int mbVal)
{
    char *pTmp = pBuf;

    if (mbVal < 0)
    {
        *pTmp++ = '-';
        mbVal = -mbVal;
    }
    else if (mbVal > 0)
    {
        *pTmp++ = '+';
    }
    /* else, add no sign character for 0.00 */

    sprintf(pTmp, "%d.%02d",
            mbVal / 100,
            mbVal % 100);

    return pBuf;
}


/* Display Action Routine */
static void uiMbValDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    /* Insure that MB values are not above their respective RZWWS limits. */
    irrMbFix();

    if (irrMoistureBalance[uiField] <= IRR_MB_MIN)
    {
        uiKeyMask |= UI_KM_KEY1 | UI_KM_KEY3;
    }
    if (irrMoistureBalance[uiField] >= ntohs(config.zone[uiField].rzwws))
    {
        uiKeyMask |= UI_KM_KEY2 | UI_KM_KEY4;
    }


    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY1 | UI_KM_KEY2 |UI_KM_KEY6);
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY3|UI_KM_KEY4|UI_KM_KEY5|UI_KM_KEY6);
    }
    
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(3);

    for (int r = 0, c = 0, z = uiFirstField;
         r < 3 && z < endingZone;
         z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(r, c * 13)];
        char buf[41];

        sprintf(pBuf, "%2d: %6.6s   ",
                displayZoneNum + z + 1,
                uiMbValFormat(buf, irrMoistureBalance[z]));

        if (++c > 2)
        {
            c = 0;
            r++;
        }
    }

    uint8_t row = (uiField - uiFirstField) / 3;
    uint8_t col = (uiField - uiFirstField) % 3;
    uiLcdCursor = LCD_RC(row, (col * 13) + 9);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiMbValNext(void)
{
    if (++uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiMbValPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = navEndZone - 1;
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiMbValInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrMoistureBalance[uiField] < ntohs(config.zone[uiField].rzwws))
    {
        irrMoistureBalance[uiField]++;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiMbValDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrMoistureBalance[uiField] > IRR_MB_MIN)
    {
        irrMoistureBalance[uiField]--;
    }
}


/* Soft Key Action Routine - Fast Increment Value ("++") */
static void uiMbValIncFast(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrMoistureBalance[uiField] < ntohs(config.zone[uiField].rzwws))
    {
        if (irrMoistureBalance[uiField] <
            (ntohs(config.zone[uiField].rzwws) - UI_MBVAL_FAST_DELTA))
        {
            irrMoistureBalance[uiField] += UI_MBVAL_FAST_DELTA;
        }
        else
        {
            irrMoistureBalance[uiField] = ntohs(config.zone[uiField].rzwws);
        }
    }
}


/* Soft Key Action Routine - Fast Decrement Value ("--") */
static void uiMbValDecFast(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrMoistureBalance[uiField] > IRR_MB_MIN)
    {
        if (irrMoistureBalance[uiField] > (IRR_MB_MIN + UI_MBVAL_FAST_DELTA))
        {
            irrMoistureBalance[uiField] -= UI_MBVAL_FAST_DELTA;
        }
        else
        {
            irrMoistureBalance[uiField] = IRR_MB_MIN;
        }
    }
}


/* Soft Key Action Routine - ZERO */
static void uiMbValSetZero(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    irrMoistureBalance[uiField] = 0;
}


/* Soft Key Action Routine - COPY */
static void uiMbValCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuMbValCopy);
}


/* Display Action Routine */
static void uiMbValCopyDisplay(void)
{
    if (irrMoistureBalance[uiField] >= ntohs(config.zone[uiField].rzwws))
    {
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Zone #%d moisture balance is at its",
                uiField + 1);
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "maximum value.  Set all other zones to");
        sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                "their respective maximum values?");
    }
    else
    {
        char buf[41];

        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Copy zone #%d moisture balance of",
                uiField + 1);
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "%-0.6s inches to all other zones?",
                uiMbValFormat(buf, irrMoistureBalance[uiField]));
    }
}


/* Soft Key Action Routine - YES */
static void uiMbValCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (irrMoistureBalance[uiField] >= ntohs(config.zone[uiField].rzwws))
    {
        /* set zone moisture balance for each zone to its RZWWS (max) value */
        for (int z = 0; z < SYS_N_ZONES; z++)
        {
            if (z != uiField)
            {
                irrMoistureBalance[z] = ntohs(config.zone[z].rzwws);
            }
        }
    }
    else
    {
        /* copy current zone moisture balance to all other zones */
        for (int z = 0; z < SYS_N_ZONES; z++)
        {
            if (z != uiField)
            {
                if (irrMoistureBalance[uiField] <= ntohs(config.zone[z].rzwws))
                {
                    /* current zone MB within target zone allowed range - copy */
                    irrMoistureBalance[z] = irrMoistureBalance[uiField];
                }
                else
                {
                    /* out of target's range - set target zone MB to its max */
                    irrMoistureBalance[z] = ntohs(config.zone[z].rzwws);
                }
            }
        }
    }

    uiMenuSet(&uiMenuMbVal);
}


/* Soft Key Action Routine - NO */
static void uiMbValCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuMbVal);
}



/******************************************************************************
 *
 *  PLANT TYPES FUNCTION MENU ACTIONS
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiField2 is used to select the sub-fields for plant species, density,
 *            and drought-tolerance (DT).
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through plant type sub-fields for
 *        species/density/DT (uiField2) then through zones (uiField).
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiPlantTypeDisplay(void)
{
    uint8_t type;
    uint8_t density;
    uint8_t dt;
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    // mask out softkey short cuts based on number of units.
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }

    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY1|UI_KM_KEY2|UI_KM_KEY6);
    }
    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(1);

    for (int r = 0, z = uiFirstField;
         r < 3 && z < endingZone;
         z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(r, 0)];
        configZonePlantTypeGet(z, &type, &density, &dt);

        pBuf += sprintf(pBuf, "%2d: %-12.12s  ",
                        displayZoneNum + z + 1,
                        uiPlantTypeNames[type]);
        if (type != CONFIG_PLANTTYPE_FESCUE &&
            type != CONFIG_PLANTTYPE_BERMUDA)
        {
            pBuf += sprintf(pBuf, "%-12.12s   %-7.7s",
                            uiPlantTypeDensities[density],
                            uiPlantTypeDroughtTolerances[dt]);
        }
        else
        {
            pBuf += sprintf(pBuf, "--             --");
        }

        r++;
    }

    uint8_t row = (uiField - uiFirstField) / 1;
    uint8_t col;
    switch (uiField2)
    {
        case UI_PT_FIELD_TYPE:
        default:
            col = 4;
            break;
        case UI_PT_FIELD_DENSITY:
            col = 18;
            break;
        case UI_PT_FIELD_DT:
            col = 33;
            break;
    }
    uiLcdCursor = LCD_RC(row, col);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiPlantTypeNext(void)
{
    uint8_t type;

    /*
     * This needs to deal with the conditional fields, skipping over density
     * and DT if the current plant type is a turf.  Lots of fun...
     */

    type = CONFIG_PT_SPECIES(config.zone[uiField].plantType);

    if (uiField2 == (UI_PT_FIELD_LIMIT - 1) ||
        (type == CONFIG_PLANTTYPE_FESCUE ||
         type == CONFIG_PLANTTYPE_BERMUDA))
    {
        /* was on last field for this zone - advance to next zone */
        uiField2 = 0;
        if (++uiField >= navEndZone)
        {
            uiField = 0;
        }
    }
    else
    {
        /* advance to next field for this (non-turf) zone */
        uiField2++;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiPlantTypePrev(void)
{
    uint8_t type;

    /*
     * This needs to deal with the conditional fields, skipping over density
     * and DT if the current plant type is a turf.  More fun...
     */

    if (uiField2 > 0)
    {
        /* was on non-first field of a multi-field zone - just back up one */
        uiField2--;
    }
    else
    {
        /* was on first field, so back up one zone (with wrap)... */
        if (uiField-- == 0)
        {
            uiField = navEndZone - 1;
        }
        type = CONFIG_PT_SPECIES(config.zone[uiField].plantType);

        /* ...and position to the last field of this zone */
        if (type == CONFIG_PLANTTYPE_FESCUE ||
            type == CONFIG_PLANTTYPE_BERMUDA)
        {
            /* turf zone - only one field */
            uiField2 = 0;
        }
        else
        {
            /* non-turf zone - position to last field */
            uiField2 = UI_PT_FIELD_LIMIT - 1;
        }
    }
}

/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiPlantTypeGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =12;
    uiFirstField =12;
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiPlantTypeGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =24;
    uiFirstField =24;
}

/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiPlantTypeGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =36;
    uiFirstField =36;
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiPlantTypeInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t type;
    uint8_t density;
    uint8_t dt;

    configZonePlantTypeGet(uiField, &type, &density, &dt);

    switch (uiField2)
    {
        case UI_PT_FIELD_TYPE:
            if (++type >= CONFIG_PLANTTYPE_LIMIT)
            {
                type = 0;
            }
            break;
        case UI_PT_FIELD_DENSITY:
            if (++density >= CONFIG_PLANTDENSITY_LIMIT)
            {
                density = 0;
            }
            break;
        case UI_PT_FIELD_DT:
            if (++dt >= CONFIG_PLANTDT_LIMIT)
            {
                dt = 0;
            }
            break;
    }

    configZonePlantTypeSet(uiField, type, density, dt);
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiPlantTypeDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t type;
    uint8_t density;
    uint8_t dt;

    configZonePlantTypeGet(uiField, &type, &density, &dt);

    switch (uiField2)
    {
        case UI_PT_FIELD_TYPE:
            if (type-- == 0)
            {
                type = CONFIG_PLANTTYPE_LIMIT - 1;
            }
            break;
        case UI_PT_FIELD_DENSITY:
            if (density-- == 0)
            {
                density = CONFIG_PLANTDENSITY_LIMIT - 1;
            }
            break;
        case UI_PT_FIELD_DT:
            if (dt-- == 0)
            {
                dt = CONFIG_PLANTDT_LIMIT - 1;
            }
            break;
    }

    configZonePlantTypeSet(uiField, type, density, dt);
}


/* Soft Key Action Routine - COPY */
static void uiPlantTypeCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuPlantTypeCopy);
}


/* Display Action Routine */
static void uiPlantTypeCopyDisplay(void)
{
    uint8_t type;
    uint8_t density;
    uint8_t dt;

    configZonePlantTypeGet(uiField, &type, &density, &dt);

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d plant/turf type setting of",
            uiField + 1);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "'%s / %s / %s'",
            uiPlantTypeNames[type],
            uiPlantTypeDensities[density],
            uiPlantTypeDroughtTolerances[dt]);
    sprintf(&uiLcdBuf[LCD_RC(2, 0)], "to all other zones?");
}


/* Soft Key Action Routine - YES */
static void uiPlantTypeCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone plant type to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            uint8_t type;
            uint8_t density;
            uint8_t dt;

            /* use Get & Set routines so that RZWWS gets set as well */
            configZonePlantTypeGet(uiField, &type, &density, &dt);
            configZonePlantTypeSet(z, type, density, dt);
        }
    }
    uiMenuSet(&uiMenuPlantType);
}


/* Soft Key Action Routine - NO */
static void uiPlantTypeCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuPlantType);
}



/******************************************************************************
 *
 *  ROOT ZONE WORKING WATER STORAGE (RZWWS) VALUES MENU ACTIONS
 *
 *  (Accessed via alt-function press of PLANT TYPES function key.)
 *
 *
 *  Note: uiField is used to select the zone.
 *        uiFirstField specifies the first zone to be displayed on LCD line 1.
 *
 *        The navigation dial scrolls through zone pages, 12 zones at a
 *        time (uiField).  Each screen page displays up to 12 zones.
 *
 *****************************************************************************/

/* Display Action Routine */
static void uiRzwwsDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5|UI_KM_KEY6 );
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    // mask out softkey short cuts based on number of units.
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY2 | UI_KM_KEY3 | UI_KM_KEY4);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4);
          break;
    }

    
    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(4);

    for (int i = 0, z = uiFirstField;
         i < 12 && z < endingZone;
         i++, z++)
    {
        char *pBuf = &uiLcdBuf[i * 10 + 0];
        int rzwwsInches = ntohs(config.zone[z].rzwws) / 100;
        int rzwwsHundredths = ntohs(config.zone[z].rzwws) % 100;

        sprintf(pBuf, "%2d:%2d.%02d  ",
                displayZoneNum + z + 1,
                rzwwsInches,
                rzwwsHundredths);
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiRzwwsNext(void)
{
    uiField += 12;
    if (uiField >= navEndZone)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiRzwwsPrev(void)
{
    if (uiField == 0)
    {
        uiField = ((navEndZone - 1) / 12) * 12;
    }
    else
    {
        uiField -= 12;
    }
}

/* Soft Key Action Routine - Group 1 ("1-12") */
static void uiRzwwsGroup1(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =0;
    uiFirstField =0;
}

/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiRzwwsGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =12;
    uiFirstField =12;
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiRzwwsGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =24;
    uiFirstField =24;
}

/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiRzwwsGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField =36;
    uiFirstField =36;
}

/******************************************************************************
 *
 *  ADVANCED FUNCTION MENU ACTIONS
 *
 *
 *  Note: The navigation dial scrolls through advanced screens and fields.
 *        The summary page is located prior to the first field.
 *
 *****************************************************************************/


/*
**  ADVANCED FUNCTION SUMMARY SCREEN
**
**  Note: UiTemp8 is used to count the number of soft-key presses to enter
**        the hidden menu for Configuration Reset.
*/

/* Display Action Routine */
static void uiAdvSummaryDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 4)],
        "Advanced Configuration Settings");
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "(This mode is intended for installers.)");
    //sprintf(&uiLcdBuf[LCD_RC(0, 0)],
    //    "Assoc  : Recv:%d,  SendAck:%d",
    //     assocflag,assocack);
    //sprintf(&uiLcdBuf[LCD_RC(1, 0)],
    //    "Status : Recv:%d,  SendAck:%d",
    //     statusflag,statusack);       
          
    if (lock == FALSE) 
        uiKeyMask |= UI_KM_KEY1;
    else 
        uiKeyMask |= UI_KM_KEY2;
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvSummaryNext(void)
{
    uiMenuSet(&uiMenuAdvProduct1);
    uiTemp8 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvSummaryPrev(void)
{
    uiMenuSet(&uiMenuAdvRadio2);
    uiTemp8 = 0;
}


/*
**  Soft Key Action Routine - (unlabeled key)
**  The Hidden0 buttons reset the button-press counter
**  to prevent continuous random soft-key presses from
**  invoking the hidden Factory Reset option.
*/
static void uiAdvSummaryHidden0(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiTemp8 = 0;
    //assocflag = 0;
    //assocack = 0;
    //statusflag = 0;
    //statusack = 0;
}


/*
**  Soft Key Action Routine - (unlabeled key)
**  The Hidden1 button must be pressed 5 times to get to
**  the hidden Factory Reset option (Yes/No) screen.
*/
static void uiAdvSummaryHidden1(uint8_t eventType, uint8_t /*eventKey*/)
{
    /* only accept discrete soft-key presses, not auto-repeat */
    if (eventType == DRV_KEYPAD_TYPE_SOFT)
    {
        if (++uiTemp8 >= 5)
        {
            /* Go to factory defaults menu. */
            uiMenuSet(&uiMenuAdvDefaults);
            uiTemp8 = 0;
        }
    }
}


/*
**  ADVANCED FUNCTION CONFIGURATION RESET MENU
*/

/* Display Action Routine */
static void uiAdvDefaultsDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "Reset all configuration data to");
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "factory defaults?");
}


/* Soft Key Action Routine - YES */
static void uiAdvDefaultsYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    dtDebug("Command to RESET to factory defaults.\n");

    /* Clear EEPROM to factory default. */
    configFactoryDefaultInit();

    /* Reset RF Module configuration. */
    radioConfigReset();

    uiMenuSet(&uiMenuAdvSummary);
}


/* Soft Key Action Routine - NO */
static void uiAdvDefaultsNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAdvSummary);
}


/*
**  ADVANCED FUNCTION PRODUCT INFORMATION SCREEN #1
**
**  Note: UiTemp8 is used to encode access to the software debug
**        menu screens.
*/

/* Display Action Routine */
static void uiAdvProduct1Display(void)
{
    char buf[41];

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "Serial Number:    %c%c%c%c%c%c%c%c",
        configSerialNumber[0],
        configSerialNumber[1],
        configSerialNumber[2],
        configSerialNumber[3],
        configSerialNumber[4],
        configSerialNumber[5],
        configSerialNumber[6],
        configSerialNumber[7]);
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "Firmware Version: %-20.20s",sysFormatFirmwareVer(buf));//RANGE"); //
       
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
        "System Up Time:   %-20.20s",
        dtFormatUpTime(buf));
     
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvProduct1Next(void)
{
    uiMenuSet(&uiMenuAdvProduct2);
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvProduct1Prev(void)
{
    uiMenuSet(&uiMenuAdvSummary);
}


/*  Soft Key Action Routine - (unlabeled key) */
static void uiAdvProductDbg1(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* First step of entering debug mode. */
    uiTemp8 = 99;
}


/*  Soft Key Action Routine - (unlabeled key) */
static void uiAdvProductDbg2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (uiTemp8 == 99)
    {
        /* Second step of entering debug mode. */
        uiMenuSet(&uiMenuAdvProductDbg);
        /* Set screen timeout to 24 hours. */
        uiScreenTimeout = DT_SECS_24_HOURS;
    }
    uiTemp8 = 0;
}


/*  Soft Key Action Routine - (unlabeled key) */
static void uiAdvProductDbg0(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Re-initialize debug screen-access hidden key sequence. */
    uiTemp8 = 0;
    /* Restore standard screen timeout. */
    uiScreenTimeout = UI_SCREEN_TIMEOUT;
}


/*
**  ADVANCED FUNCTION DEBUG MENU
**
**  Note: uiField is used to control which zone's debug information to display.
*/

/* Display Action Routine */
static void uiAdvProductDbgDisplay(void)
{
    char tmpbuf1[50];
    char tmpbuf2[50];

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "DEBUG %s  Cur Zone: %d",
        radioDebug ? "R" : " ",
        irrCurZone);
    sprintf(&uiLcdBuf[LCD_RC(0, 24)],
        "Reset Reason: %02X", drvSysResetReason());

    if (config.sys.opMode == CONFIG_OPMODE_WEATHER)
    {
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "Zone %2d  %s  MB:%5d  RZWWS:%4d",
            uiField + 1,
            dtFormatRunTimeSecs(tmpbuf1, irrRemainingZoneSecs(uiField + 1)),
            irrMoistureBalance[uiField],
            ntohs(config.zone[uiField].rzwws));
    }
    else
    {
        if ((irrState != IRR_STATE_IDLE) && irrIsCurrentGroupLeader(uiField + 1))
        {
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "Zone %2d  %s  Sensor: %02d%% (%02d%%)",
                uiField + 1,
                dtFormatRunTimeSecs(tmpbuf1, irrRemainingZoneSecs(uiField + 1)),
                moistValueGet(uiField + 1),
                moistThresholdGet(uiField + 1));
        }
        else if ((irrState == IRR_STATE_IDLE) && irrIsConfigGroupLeader(uiField + 1))
        {
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "Zone %2d  %s  Sensor: %02d%%",
                uiField + 1,
                dtFormatRunTimeSecs(tmpbuf1, irrRemainingZoneSecs(uiField + 1)),
                moistValueGet(uiField + 1));
        }
        else
        {
            sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "Zone %2d  %s  Sensor Group: %02d",
                uiField + 1,
                dtFormatRunTimeSecs(tmpbuf1, irrRemainingZoneSecs(uiField + 1)),
                irrState == IRR_STATE_IDLE ?
                    config.zone[uiField].group + 1 :
                    irrZone[uiField].group + 1);
        }
    }

    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
        "Pulse:   %s  Soak:   %s",
        dtFormatRunTimeSecs(tmpbuf1, irrRemainingZonePulseSecs(uiField + 1)),
        dtFormatRunTimeSecs(tmpbuf2, irrRemainingZoneSoakSecs(uiField + 1)));
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvProductDbgNext(void)
{
    if (++uiField >= config.sys.numZones)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvProductDbgPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = config.sys.numZones - 1;
    }
}


/* Soft Key Action Routine - R-DBG */
static void uiAdvProductDbgRadio(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    radioDebug = !radioDebug;
}


/* Soft Key Action Routine - ESAVE */
static void uiAdvProductDbgEvSave(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Save snapshot of event log. */
    configEventLogSave();
    /*
    **  Note: This routine is designed to save an event log snapshot in
    **        non-volatile memory and then exit the debug screen.
    **        (A non-technical person in the field could be guided in this
    **        process so that the event log could later be reviewed by an
    **        engineer.)
    */
    /* Restore standard screen timeout. */
    uiScreenTimeout = UI_SCREEN_TIMEOUT;
    /* Return to non-debug screen. */
    uiMenuSet(&uiMenuAdvProduct1);
    uiTemp8 = 0;
}


/* Soft Key Action Routine - EVIEW */
static void uiAdvProductDbgEvLog(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAdvEventLog);
    uiField = SYS_EVENT_LIMIT;
}


/* Soft Key Action Routine - RESET */
static void uiAdvProductDbgReset(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Go to Reset menu. */
    uiMenuSet(&uiMenuAdvReset);
}


/*
**  ADVANCED FUNCTION DEBUG EVENT LOG MENU
**
**  Note: uiField is used to specify the event log entry for scrolling.
*/

/* Display Action Routine */
static void uiAdvEventLogDisplay(void)
{
    uint8_t i;
    sysEventLogMaint_t mn;
    sysEventLogEntry_t eventBuf[4];
    uint32_t startOffset;

    if (uiField >= SYS_EVENT_LIMIT)
    {
        configEventLogRead(woffsetof(sysEventLog_t, mn),
                           &mn,
                           sizeof(sysEventLogMaint_t));
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "System Event Log");
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "Saved on %02d/%02d/%02d at %02d:%02d:%02d",
                mn.month,
                mn.day,
                mn.year,
                mn.hour,
                mn.min,
                mn.sec);
        sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                "Next Entry = %d",
                mn.next);
        return;
    }

    startOffset = woffsetof(sysEventLog_t, ev) + uiField * sizeof(sysEventLogEntry_t);

    configEventLogRead(startOffset,
                       &eventBuf,
                       sizeof(sysEventLogEntry_t) * 4);


    for (i = 0; (i < 4) && ((uiField + i) < SYS_EVENT_LIMIT); i++)
    {
        sprintf(&uiLcdBuf[LCD_RC(i, 0)],
            "%02d  %010d  %04X %04X",
            uiField + i,
            ntohl(eventBuf[i].time),
            ntohs(eventBuf[i].type),
            ntohs(eventBuf[i].data));
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvEventLogNext(void)
{
    if (++uiField > SYS_EVENT_LIMIT)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvEventLogPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = SYS_EVENT_LIMIT;
    }
}


/*
**  ADVANCED FUNCTION DEBUG - SYSTEM RESET MENU
*/

/* Display Action Routine */
static void uiAdvResetDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "Reset system, clearing all current");
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "state information?");
}


/* Soft Key Action Routine - YES */
static void uiAdvResetYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Force a system reset (no effect in WIN32). */
    dtDebug("Command to RESET SYSTEM.\n");
    sysResetRequest = TRUE;
}


/* Soft Key Action Routine - NO */
static void uiAdvResetNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAdvProductDbg);
}


/*
**  ADVANCED FUNCTION PRODUCT INFORMATION SCREEN #2
*/

/* Display Action Routine */
static void uiAdvProduct2Display(void)
{
    configManuf_t manuf;        /* Manufacturing Data Image */

    /* Read manufacturing data from EEPROM. */
    configManufRead(&manuf);

    uint16_t systemRevision = ntohs(manuf.systemRevision);
    uint16_t boardRevision = ntohs(manuf.boardRevision);

    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "System Revision: %c.%-3d   [%08X]",
        (systemRevision >> 8) + 'A',
        systemRevision & 0x00FF,
        ntohl(manuf.systemType));
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "Board Revision:  %c.%-3d   [%08X]",
        (boardRevision >> 8) + 'A',
        boardRevision & 0x00FF,
        ntohl(manuf.boardType));
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
        "Radio Firmware:  %04X    Hardware: %04X",
        radioStatusVr,
        radioStatusHv);
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvProduct2Next(void)
{
    uiMenuSet(&uiMenuAdvRadio);
    uiTemp8 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvProduct2Prev(void)
{
    uiMenuSet(&uiMenuAdvProduct1);
    uiTemp8 = 0;
}


/*
**  ADVANCED FUNCTION RADIO MENU SCREEN 1 - STATUS AND PAN ID
**
**  Note: UiTemp8 is used to delay status display transition from "SCANNING".
*/

/* Display Action Routine */
static void uiAdvRadioDisplay(void)
{
    uint8_t status;

    status = radioStatus;

    /* Handle tick refresh event. */
    if (uiLcdRefreshTick)
    {
        if (status == RADIO_STATUS_SCANNING)
        {
            /* Load Scanning status delay countdown register. */
            uiTemp8 = 5;
        }
        else if (status == RADIO_STATUS_OFFLINE)
        {
            if (uiTemp8 > 0)
            {
                /* Continue to display Scanning status until countdown == 0. */
                status = RADIO_STATUS_SCANNING;
                uiTemp8--;
            }
        }
        else
        {
            uiTemp8 = 0;
        }
    }


    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
          /* Mask off "-" "+" "copy" soft-key. */
          uiKeyMask |= (UI_KM_KEY4);
    }
    
    if( lock==TRUE ) 
    {
        uiKeyMask |= (UI_KM_KEY2|UI_KM_KEY4|UI_KM_KEY6);
    }
    
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "Radio Status: %0.12s",
        uiRadioStatus[status]);
    
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "PAN ID: %04X",
        ntohs(config.sys.radioPanId));

    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
        "MAC ID: %08X%08X",
        (uint32_t)(radioMacId >> 32),
        (uint32_t)(radioMacId & 0xFFFFFFFF));
}



/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvRadioNext(void)
{
    uiMenuSet(&uiMenuAdvRadio2);
    uiTemp8 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvRadioPrev(void)
{
    uiMenuSet(&uiMenuAdvProduct2);
}


/*  Soft Key Action Routine - RESET */
static void uiAdvRadioReset(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    dtDebug("Command to RESET Radio\n");
    radioReset();
}


/*  Soft Key Action Routine - HW RST */
static void uiAdvRadioHwReset(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    dtDebug("Command to HW RST Radio\n");
    radioResetHw();
}


/*  Soft Key Action Routine - SET PAN */
static void uiAdvRadioEdit(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAdvRadioEdit);
    uiField = 0;
    uiTemp16 = htons(config.sys.radioPanId);
}


/*
**  ADVANCED FUNCTION RADIO MENU SCREEN 1 - EDIT PAN ID
**
**  Note: uiField is used to select the PAN ID hexadecimal digit field.
*/


/* Display Action Routine */
static void uiAdvRadioEditDisplay(void)
{
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
        "Radio Status: %0.12s",
        uiRadioStatus[radioStatus]);


    /* Mask off =OP soft-key if radio is not online. */
    uiKeyMask = UI_KM_KEY4;


    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
        "PAN ID: %1X %1X %1X %1X",
        uiTemp16 >> 12,
        (uiTemp16 & 0x0F00) >> 8,
        (uiTemp16 & 0x00F0) >> 4,
        uiTemp16 & 0x000F);


    switch (uiField)
    {
        case 0:
            uiLcdCursor = LCD_RC(1, 8);
            break;
        case 1:
            uiLcdCursor = LCD_RC(1, 10);
            break;
        case 2:
            uiLcdCursor = LCD_RC(1, 12);
            break;
        case 3:
            uiLcdCursor = LCD_RC(1, 14);
            break;
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvRadioEditNext(void)
{
    if (++uiField >= 4)
    {
        uiField = 0;
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvRadioEditPrev(void)
{
    if (uiField-- == 0)
    {
        uiField = 4 - 1;
    }
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiAdvRadioEditInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t nibble;

    switch (uiField)
    {
        case 0:
            /* high nibble range is 0-3,F */
            nibble = uiTemp16 >> 12;
            if (nibble == 0x0F)
            {
                nibble = 0;
            }
            else if (++nibble > 3)
            {
                nibble = 0x0F;
            }
            uiTemp16 = (uiTemp16 & 0x0FFF) | (nibble << 12);
            break;
        case 1:
            nibble = (uiTemp16 & 0x0F00) >> 8;
            if (++nibble > 0x0F)
            {
                nibble = 0;
            }
            uiTemp16 = (uiTemp16 & 0xF0FF) | (nibble << 8);
            break;
        case 2:
            nibble = (uiTemp16 & 0x00F0) >> 4;
            if (++nibble > 0x0F)
            {
                nibble = 0;
            }
            uiTemp16 = (uiTemp16 & 0xFF0F) | (nibble << 4);
            break;
        case 3:
            nibble = uiTemp16 & 0x000F;
            if (++nibble > 0x0F)
            {
                nibble = 0;
            }
            uiTemp16 = (uiTemp16 & 0xFFF0) | nibble;
            break;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiAdvRadioEditDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uint8_t nibble;

    /* PAN ID valid range is 0x0000-0x3FFF, and 0xFFFF. */
    switch (uiField)
    {
        case 0:
            /* Restrict PAN ID high nibble to 0-3,F */
            nibble = uiTemp16 >> 12;
            if (nibble == 0x0F)
            {
                nibble = 0x03;
            }
            else if (--nibble > 3)
            {
                nibble = 0x0F;
            }
            uiTemp16 = (uiTemp16 & 0x0FFF) | (nibble << 12);
            break;
        case 1:
            nibble = (uiTemp16 & 0x0F00) >> 8;
            if (--nibble > 0x0F)
            {
                nibble = 0x0F;
            }
            uiTemp16 = (uiTemp16 & 0xF0FF) | (nibble << 8);
            break;
        case 2:
            nibble = (uiTemp16 & 0x00F0) >> 4;
            if (--nibble > 0x0F)
            {
                nibble = 0x0F;
            }
            uiTemp16 = (uiTemp16 & 0xFF0F) | (nibble << 4);
            break;
        case 3:
            nibble = uiTemp16 & 0x000F;
            if (--nibble > 0x0F)
            {
                nibble = 0x0F;
            }
            uiTemp16 = (uiTemp16 & 0xFFF0) | nibble;
            break;
    }
}


/*  Soft Key Action Routine - ANY */
static void uiAdvRadioEditAny(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiTemp16 = CONFIG_PAN_ID_ANY;
}


/*  Soft Key Action Routine - "=OP" */
static void uiAdvRadioEditOp(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiTemp16 = radioStatusOp;
    uiField = 0;
}


/*  Soft Key Action Routine - ACCEPT */
static void uiAdvRadioEditAccept(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    config.sys.radioPanId = htons(uiTemp16);
    radioPanIdSet();
    uiMenuSet(&uiMenuAdvRadio);
    uiTemp8 = 0;
}



/*  Soft Key Action Routine - CANCEL */
static void uiAdvRadioEditCancel(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuAdvRadio);
    uiTemp8 = 0;
}


/*
**  ADVANCED FUNCTION RADIO MENU SCREEN 2 - LAST MESSAGE/SIGNAL STRENGTH/TEST
**
**  Note: UiTemp8 is used for counting seconds until test timeout.
*/

/* Display Action Routine */
static void uiAdvRadio2Display(void)
{
    const char *pTestDisplay;       /* Testing display string or NULL */
    /*
    **  Note: The pTestDisplay variable is set to NULL for normal radio
    **  status display.  When this variable is NOT NULL it points to a
    **  string for display on LCD line 1 during loopback testing.
    */
    pTestDisplay = NULL;

    /* Check if loopback testing is in progress. */
    if (radioLbState != RADIO_LBS_IDLE)
    {
        /* Set Testing mode display string. */
        pTestDisplay = "Testing...";
    }

    /* Mask off TEST soft-key if radio is not online, or testing. */
    if ((radioStatus != RADIO_STATUS_ONLINE) ||
        (radioLbState != RADIO_LBS_IDLE))
    {
        uiKeyMask = UI_KM_KEY6;
    }
    
    if (lock == TRUE)
       uiKeyMask = UI_KM_KEY6;

    /* Handle tick refresh event for loopback testing. */
    if (uiLcdRefreshTick)
    {
        switch (radioLbState)
        {
            case RADIO_LBS_IDLE:        /* no loopback test */
                break;
            case RADIO_LBS_START:
            case RADIO_LBS_TESTING:
                if (uiTemp8 > 0)
                {
                    /* Decrement the test timeout counter. */
                    uiTemp8--;
                }
                else
                {
                    /* Indicate Test timeout waiting for a response. */
                    pTestDisplay = "No Response";
                    /* Cancel the test. */
                    radioLoopbackTestCancel();
                }
                break;
            case RADIO_LBS_SUCCESS:
                if ((uiTemp8 > 0) && (radioStatusDb == 0))
                {
                    /* Wait for RSSI value; decrement test timeout counter. */
                    uiTemp8--;
                }
                else
                {
                    /* Cancel the test. */
                    radioLoopbackTestCancel();
                }
                break;
            case RADIO_LBS_FAIL:        /* test returned failure status */
            default:                    /* unknown status - treat as failure */
                /* Indicate Test failed.  */
                pTestDisplay = "Test Failed";
                /* Cancel the test. */
                radioLoopbackTestCancel();
                break;
        }
    }

    /* Check if Testing status display mode. */
    if (pTestDisplay != NULL)
    {
        /* Generate LCD line 1 - Loopback Testing status message. */
        sprintf(&uiLcdBuf[LCD_RC(0, 0)], "%s", pTestDisplay);
    }
    else
    {
        /* Generate LCD line 1 - Last Message Received Signal Strength. */
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
                "Radio Signal:");

        if (radioStatusDb != 0)
        {
            /*sprintf(&uiLcdBuf[LCD_RC(0, 14)],
                    "%ddBm",
                    0 - radioStatusDb);
            sprintf(&uiLcdBuf[LCD_RC(0, 35)],
                    "%-5.5s",
                    uiFormatRadioSignalBars(radioStatusDb));   */
            if(radioStatusDb < 85)
            {
                sprintf(&uiLcdBuf[LCD_RC(0, 14)],
                    "OK");
            } 
            else
            {
                sprintf(&uiLcdBuf[LCD_RC(0, 14)],
                    "LOW   %ddBm",
                    0 - radioStatusDb);
            }
        }

        /* Generate LCD line 2 - Last Message Received Description. */
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
                "Last Message: %0.25s",
                radioLastMsgDesc);

        /* Generate LCD line 3 - Last Message Received Date/Time. */
        sprintf(&uiLcdBuf[LCD_RC(2, 0)],
                "Receive Date: %0.20s",
                radioLastMsgDate);
    }
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiAdvRadio2Next(void)
{
    uiMenuSet(&uiMenuAdvSummary);
    uiTemp8 = 0;
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiAdvRadio2Prev(void)
{
    uiMenuSet(&uiMenuAdvRadio);
    uiTemp8 = 0;
}


/*  Soft Key Action Routine - TEST */
static void uiAdvRadio2Test(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* Set test to timeout after 10 seconds. */
    uiTemp8 = 10;
    /* Clear last radio message information, including last RSSI value. */
    radioLastMsgDesc[0] = '\0';
    radioLastMsgDate[0] = '\0';
    radioStatusDb = 0;
    /* Start loopback test. */
    radioLoopbackTestStart();
}

/////////////////////////////////////////
//////////////////////////////////////
/////////////////////////////////////FLOW settings
/////////////////////////////////Display Action Routine */
static void uiFlowSetting(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
  
  uiMenuSet(&uiMenuFlow); 

}


static void uiFlowDisplay(void)
{
    uint8_t displayZoneNum=0;
    uint8_t endingZone=(config.sys.numUnits+1) *12;

    if((config.sys.unitType != UNIT_TYPE_MASTER))
    {
        endingZone =12;
        uiKeyMask |= (UI_KM_KEY1 |UI_KM_KEY2|UI_KM_KEY3|UI_KM_KEY4|UI_KM_KEY5|UI_KM_KEY6);
    }
    
    switch(config.sys.unitType){
      case UNIT_TYPE_EXPANSION_1:
          displayZoneNum=12;
          break;
      case UNIT_TYPE_EXPANSION_2:
          displayZoneNum=24;
          break;     
      case UNIT_TYPE_EXPANSION_3:
          displayZoneNum=36;
          break;
    }
    
    //mask out soft key short cuts based on number of units
    switch(config.sys.numUnits)
    {
      case 0:
          /* Mask off "13-24, 25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY3 | UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 1:
          /* Mask off "25-36, 37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY4 | UI_KM_KEY5);
          break;
      case 2:
          /* Mask off "37-48" soft-key. */
          uiKeyMask |= (UI_KM_KEY5);
          break;
    }
    
    if ((uiField2 == 0 && config.zone[uiField].minGPM == 0) ||
        (uiField2 != 0 && config.zone[uiField].maxGPM <= 1) )
    {
        /* Mask off "-" soft-key. */
        uiKeyMask |= UI_KM_KEY1;
    }
    if ((uiField2 == 0 && config.zone[uiField].minGPM >= (99 - 1)) ||
        (uiField2 != 0 && config.zone[uiField].maxGPM >= 99) )
    {
        /* Mask off "+" soft-key. */
        uiKeyMask |= UI_KM_KEY2;
    }
    if (lock == TRUE )
    {
        /* Mask off "-", "+", and "COPY" soft-keys. */
        uiKeyMask |= UI_KM_KEY1 | UI_KM_KEY2 | UI_KM_KEY6;
    }

    /* fix uiFirstField if it is out of range */
    uiFirstFieldFix(3);

    for (int r = 0, c = 0, z = uiFirstField;
         r < 3 && z < endingZone;
         z++)
    {
        char *pBuf = &uiLcdBuf[LCD_RC(r, c * 13)];        
            pBuf += sprintf(pBuf, "%2d: %2d-%2d",
                            displayZoneNum+z + 1,
                            config.zone[z].minGPM,
                            config.zone[z].maxGPM);
      
      

        if (++c > 2)
        {
            c = 0;
            r++;
        }
    }

    uint8_t row = (uiField - uiFirstField) / 3;
    uint8_t col = (uiField - uiFirstField) % 3;
    uiLcdCursor = LCD_RC(row, (col * 13) + 5);
    if (uiField2 != 0)
      {
         uiLcdCursor += 3;
      }
    
}


/* Navigation Dial Action Routine - CW Rotation */
static void uiFlowNext(void)
{
       
    if (uiField2 == 0 )//&& config.zone[uiField].group == uiField)
    {
        uiField2 = 1;
    }
    else
    {
        uiField2 = 0;
        if (++uiField >= navEndZone)
        {
            uiField = 0;
        }
    }
}


/* Navigation Dial Action Routine - CCW Rotation */
static void uiFlowPrev(void)
{
    if (uiField2 > 0)
    {
        uiField2 = 0;
    }
    else
    {
        uiField2 = 1;
        if (uiField-- == 0)
        {
            uiField = navEndZone - 1;
        }
                
    }
}


/* Soft Key Action Routine - Group 2 ("13-24") */
static void uiFlowGroup2(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=12;
    uiField2 = 0;
    uiFirstField =12;
}

/* Soft Key Action Routine - Group 3 ("25-36") */
static void uiFlowGroup3(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=24;
    uiField2 = 0;
    uiFirstField =24;
}


/* Soft Key Action Routine - Group 4 ("37-48") */
static void uiFlowGroup4(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiField=36;
    uiField2 = 0;
    uiFirstField =36;
}


/* Soft Key Action Routine - Increment Value ("+") */
static void uiFlowInc(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (uiField2 == 0)
    {
        if (++config.zone[uiField].minGPM >= config.zone[uiField].maxGPM)
        {
            config.zone[uiField].maxGPM = config.zone[uiField].minGPM + 1;
        }
    }
    else
    {
        ++config.zone[uiField].maxGPM;
    }
}


/* Soft Key Action Routine - Decrement Value ("-") */
static void uiFlowDec(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    if (uiField2 == 0)
    {
        --config.zone[uiField].minGPM;
    }
    else
    {
        if (--config.zone[uiField].maxGPM <= config.zone[uiField].minGPM)
        {
            config.zone[uiField].minGPM = config.zone[uiField].maxGPM - 1;
        }
    }
}



/* Soft Key Action Routine - COPY */
static void uiFlowCopy(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuFlowCopy);
}


/* Display Action Routine */
static void uiFlowCopyDisplay(void)
{
    
    sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "Copy zone #%d Min/Max Flow setting of ",
            uiField + 1);
    
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "'%d-%d' to all other zones ?",
            config.zone[uiField].minGPM,
            config.zone[uiField].maxGPM);
}


/* Soft Key Action Routine - YES */
static void uiFlowCopyYes(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    /* copy current zone setting to all other zones */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        if (z != uiField)
        {
            config.zone[z].maxGPM = config.zone[uiField].maxGPM;
            config.zone[z].minGPM = config.zone[uiField].minGPM;
        }
    }
    uiMenuSet(&uiMenuFlow);
}


/* Soft Key Action Routine - NO */
static void uiFlowCopyNo(uint8_t /*eventType*/, uint8_t /*eventKey*/)
{
    uiMenuSet(&uiMenuFlow);
}

///////////////////////////////////////////////////
/////////////////////////////////////////////////////
///////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////// LOCK SCREEN ///////////////////////////////
static void uiLock(uint8_t /*eventType*/, uint8_t /*eventKey*/){
   
   uiMenuSet(&uiMenuLock);
   uiField = 55;
   pinIndex = 0;
   pd = 0;
   pin[0] = '\0';
   pin[1] = '\0';
   pin[2] = '\0';
   pin[3] = '\0';
    
   
}

static void uiLockDisplay( void ){

    sprintf(&uiLcdBuf[LCD_RC(0, 2)],
            "Please Input 4 Digit to Lock Screen:");    
    sprintf(&uiLcdBuf[LCD_RC(1, 15)],
            "0123456789");
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "PIN:%4s",
             pin);         
    
    pd = uiField - 7;
    uiLcdCursor = LCD_RC(0, uiField);            

   
}

static void uiLockDisplayNext( void ){
     if ( ++uiField >= 65)
    uiField = 55;   
}

static void uiLockDisplayPrev( void ){
    if ( uiField-- == 55)
    uiField = 64;   
}

static void uiLockAdd(uint8_t /*eventType*/, uint8_t /*eventKey*/){
    if ( pinIndex >= 4 ) pinIndex = 3;
    pin[pinIndex++] = pd;
    pin[4] = '\0';
}

static void uiLockDel(uint8_t /*eventType*/, uint8_t /*eventKey*/){
     if ( pinIndex-- ==0 ) pinIndex =0 ;
     pin[pinIndex] = '\0';
}

static void uiLockOK(uint8_t /*eventType*/, uint8_t /*eventKey*/){
    uiMenuSet(&uiMenuLockOK);
}

static void uiLockDisplayOK( void ){

    if (pinIndex == 4) {        
        lock = TRUE; 
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "LOCKED, YOUR PIN : %4s", pin);
    } else {
        sprintf(&uiLcdBuf[LCD_RC(0, 0)],
            "PIN INCORRECT,Please input PIN again...");
        sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "YOUR INPUT PIN : %4s",pin);    
    }
                
}
////////////////////////////////////////
////////////////unlock //////////////////
//////////////////////////////////////////
static void uiUnlock(uint8_t /*eventType*/, uint8_t /*eventKey*/){
   uiMenuSet(&uiMenuUnlock);
   uiField = 55;
   unpinIndex = 0;
   pd = 0;
   unpin[0] = '\0';
   unpin[1] = '\0';
   unpin[2] = '\0';
   unpin[3] = '\0';
}

static void uiUnlockDisplay( void ){

    sprintf(&uiLcdBuf[LCD_RC(0, 1)],
            "Please Input 4 Digit to Unlock Screen:");    
    sprintf(&uiLcdBuf[LCD_RC(1, 15)],
            "0123456789");
    sprintf(&uiLcdBuf[LCD_RC(2, 0)],
            "PIN:%4s",
             unpin);         
    
    pd = uiField - 7;
    uiLcdCursor = LCD_RC(0, uiField);   
              
}

static void uiUnlockDisplayNext( void ){
     if ( ++uiField >= 65)
        uiField = 55;     
}

static void uiUnlockDisplayPrev( void ){
     if ( uiField-- == 55)
        uiField = 64;    
}


static void uiUnlockAdd(uint8_t /*eventType*/, uint8_t /*eventKey*/){    
     if ( unpinIndex >= 4 ) unpinIndex = 3;
     unpin[unpinIndex++] = pd;
     unpin[4] = '\0';
}

static void uiUnlockDel(uint8_t /*eventType*/, uint8_t /*eventKey*/){
     if ( unpinIndex-- ==0 ) unpinIndex =0 ;
     unpin[unpinIndex] = '\0';
}

static void uiUnlockOK(uint8_t /*eventType*/, uint8_t /*eventKey*/){
    uiMenuSet(&uiMenuUnlockOK);
}


static void uiUnlockDisplayOK( void ){ 
    if (( (pin[0] == unpin[0]) && (pin[1] == unpin[1]) &&
          (pin[2] == unpin[2]) && (pin[3] == unpin[3]) ) ||
       ( (unpin[0] == 52) && (unpin[1] == 57) && (unpin[2] == 50) && (unpin[3] == 49) ) )
    {        
    lock = FALSE; 
    sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "UNLOCKED... PIN : %s...",unpin);
    } else {
    
     sprintf(&uiLcdBuf[LCD_RC(1, 0)],
            "PIN INCORRECT, Please Input PIN Again...");       
    }
}



