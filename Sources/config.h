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
 * Module       : config.h
 * Description  : This file defines the configuration manager interfaces.
 *
 *****************************************************************************/

#ifndef __config_H
#define __config_H

/* MODULE config */

#include "debug.h"
#include "global.h"
#include "system.h"
#include "radio.h"
#include "ui.h"



/******************************************************************************
 *
 *  CONFIGURATION MANAGER STATE AND IMPLEMENTATION
 *
 *****************************************************************************/

/* Configuration State */
#define CONFIG_STATE_RESTART    0       /* Restarting */
#define CONFIG_STATE_CLEAN      1       /* Data Store is in-sync */
#define CONFIG_STATE_DIRTY      2       /* Data Store is out-of-sync */
#define CONFIG_STATE_WRITING1   3       /* Updating EEPROM data copy 1 */
#define CONFIG_STATE_WRITING2   4       /* Updating EEPROM data copy 2 */
extern uint8_t configState;             /* Configuration Manager State */

/* Configuration Image Status */
#define CONFIG_IMAGE_CORRUPT    0       /* Image is Corrupted */
#define CONFIG_IMAGE_VALID      1       /* Image is Valid */
#define CONFIG_IMAGE_OLD        2       /* Image is Valid but Out-of-Sync */


#define CONFIG_LAZY_WRITE_SECS  5       /* Dirty config secs before Write */
#define CONFIG_MB_FIX_SECS      10      /* Dirty config secs before MB fix */


#ifdef RADIO_ZB
#define CONFIG_VERSION          4       /* Configuration Data Format Version */
#else
#define CONFIG_VERSION          1       /* Configuration Data Format Version */
//#error "New irrigation algorithm not supported for ZNET 2.5 radio version."
#endif

#define CONFIG_MANUF            0x0000  /* EEPROM offset manuf image */
#define CONFIG_IMAGE1           0x800//0x0400  /* EEPROM offset to config image 1 */
#define CONFIG_IMAGE2           0x1000//0x0800  /* EEPROM offset to config image 2 */
#define CONFIG_IMAGE_BUFFER     0x1800//0x0C00  /* EEPROM offset to config download */
#define CONFIG_IMAGE_SNAPSHOT   0x2000//0x1000  /* EEPROM offset to config snapshot */
#define CONFIG_EVENT_LOG        0x2800//0x1400  /* EEPROM offset to saved event log */
#define FLOW_SNS_DATA           0x2C00          /* EEPROM offset to save 1 days worth of flow sensor data */
#define LEVEL_SNS_DATA          0x5600          /* EEPROM offset to save 1 days worth of level sensor data */


#define CONFIG_BLOCK_SIZE       64      /* EEPROM sector size */
#define CONFIG_EEPROM_SIZE      32768   /* EEPROM size (assuming only 32K) */
#define FLOW_DATA_SIZE          1440
#define LEVEL_DATA_SIZE         1440


/******************************************************************************
 *
 *  SYSTEM CONFIGURATION DATA
 *
 *****************************************************************************/

/* Number of Zones */
#define CONFIG_ZONE_MAX         12      /* maximum zones in shipping product */



/* Unit Type */
#define UNIT_TYPE_MASTER        0
#define UNIT_TYPE_EXPANSION_1   1
#define UNIT_TYPE_EXPANSION_2   2
#define UNIT_TYPE_EXPANSION_3   3

/* Operating mode */
#define CONFIG_OPMODE_RUNTIME   0       /* run-time-based */
#define CONFIG_OPMODE_SENSOR    1       /* sensor-based (moisture) */
#define CONFIG_OPMODE_WEATHER   2       /* weather-based (ET) */
#define CONFIG_OPMODE_LIMIT     3

/* Pulse mode */
#define CONFIG_PULSEMODE_OFF    0       /* pulse mode off */
#define CONFIG_PULSEMODE_ON     1       /* pulse mode on */
#define CONFIG_PULSEMODE_LIMIT  2

/* Date & Time Format */
#define CONFIG_TIMEFMT_12HR     0       /* 12-hour format ("11:59pm") */
#define CONFIG_TIMEFMT_24HR_US  1       /* 24-hour US format ("23:59") */
#define CONFIG_TIMEFMT_24HR_CA  2       /* 24-hour CA format ("23h59m") */
#define CONFIG_TIMEFMT_LIMIT    3

/* Radio PAN ID */
#ifdef RADIO_ZB
#define CONFIG_PAN_ID_ANY       0
#else
#define CONFIG_PAN_ID_LIMIT     0x4000
#define CONFIG_PAN_ID_ANY       0xFFFF
#endif

/*
** SENSOR CONCENTRATOR STRUCT
*/
typedef struct
{
    uint64_t macId;             /* mac id of sensor concentrator */
    uint8_t zoneRange;          /* zone range */
    uint8_t channelZone[4];     /* shows which zones are assigned to which channels of the SC */
} snsConConfig_t;

#define MAX_NUM_SC              12      /*Number of Sensor Concentrators */

/* sensor concentrator sleep time in seconds  when not irrigating*/
#ifdef DEBUG_TIMINGS_ENABLED
    #pragma message("DEBUG!!! RADIO_SNSCON_FAIL_SECS @ 120s")
    #define SC_SLEEP_TIME 		      20
    #define SC_SLEEP_TIME_IRR         10      /* sensor concentrator sleep time in seconds when irrigating*/
#else
    #define SC_SLEEP_TIME 		      180
    #define SC_SLEEP_TIME_IRR         30
#endif

#define SC_HIBERNATE_TEMP_C	    4	    /* sensor concentrator hiberate temperature is degree C */
#define SC_HIBERNATE_THRES_TIME	86400 /* sensor concentrator hiberate threshold time in msec (24hours)*/
#define SC_NUM_CHAN_UNIT        4       /* number of channels per sensor concentrator */
#define SC_NONE_SELECTED        4
#define SC_NONE_OPEN            5

/* Sensor Concentrator Valve Zone States */
#define SC_ALL_CHAN_OFF     0x00
#define SC_CHAN_1_ON        0x01
#define SC_CHAN_2_ON        0x02
#define SC_CHAN_3_ON        0x04
#define SC_CHAN_4_ON        0x08

/* Sensor Concentrator Channel Assignment */
#define SC_CHAN_NOT_ASSIGNED    48      /* denotes that channel of sensor concentrator isnt assigned to a zone */
#define IRR_SNS_SOL_NONE -1

/* Sensor Concentrator Index to List */
#define ZONE_SC_INDEX_NONE -1   /* no sensor concentrator is picked for index yet */

/*
**  SYSTEM CONFIG STRUCT
*/

typedef struct
{
    uint16_t version;                       /* Config Data Store Version */
    uint16_t checkSum;                      /* Config Data Image Checksum */
    uint8_t numZones;                       /* Number of zones visible for system*/
    uint8_t opMode;                         /* Irrig Operating Mode */
    uint8_t pulseMode;                      /* Pulse Mode  0=standard, 1=pulse */
    uint8_t timeFmt;                        /* Date & Time format */
    uint16_t radioPanId;                    /* Config Radio PAN ID */
    uint8_t unitType;                       /* unit type of master or expansion */
                                            /*  0 for master, 
                                             * 1-3 for expansion units */
    uint8_t numUnits;                       /* number of units in system */
    uint64_t masterMac;                     /* master mac id */
    uint64_t expMac1;                       /* expansion 1 mac id */
    uint64_t expMac2;                       /* expansion 2 mac id */
    uint64_t expMac3;                       /* expansion 3 mac id */
    uint8_t masterNumZones;                 /* master number of zones */
    uint8_t expNumZones1;                   /* expansion 1 number of zones */
    uint8_t expNumZones2;                   /* expansion 2 number of zones */
    uint8_t expNumZones3;                   /* expansion 3 number of zones */
    uint8_t numSensorCon;                   /* number of sensor concentrators associated */
    snsConConfig_t assocSensorCon[MAX_NUM_SC];     /* array to hold the MAC IDs of sensor    *
                                                    * concentrators that are associated with *
                                                    * unit */                                        
    uint8_t pad[6];                         /* System Pad Bytes - Reserved */

} configSys_t;



/******************************************************************************
 *
 *  PER-ZONE CONFIGURATION DATA
 *
 *****************************************************************************/

/* Run Time */
#define CONFIG_RUNTIME_MAX              (4 * 60)    /* 4 hours (in minutes) */
#define CONFIG_RUNTIME_MAX_SECS         (CONFIG_RUNTIME_MAX * 60) /* seconds */

/* Application Rate */
#define CONFIG_APPRATE_MIN              1           /* 0.01 inches/hour */
#define CONFIG_APPRATE_MAX              999         /* 9.99 inches/hour */

/* Soil Type */
#define CONFIG_SOILTYPE_CLAY            0   /* soil type of clay */
#define CONFIG_SOILTYPE_SILTY_CLAY      1   /* soil type of silty clay */
#define CONFIG_SOILTYPE_CLAY_LOAM       2   /* soil type of clay/loam */
#define CONFIG_SOILTYPE_LOAM            3   /* soil type of loam */
#define CONFIG_SOILTYPE_SANDY_LOAM      4   /* soil type of sandy loam */
#define CONFIG_SOILTYPE_LOAMY_SAND      5   /* soil type of loamy sand */
#define CONFIG_SOILTYPE_SAND            6   /* soil type of sand */
#define CONFIG_SOILTYPE_LIMIT           7

/* Slope */
#define CONFIG_SLOPE_0_3                0   /* slope of 0-3% */
#define CONFIG_SLOPE_4_6                1   /* slope of 4-6% */
#define CONFIG_SLOPE_7_12               2   /* slope of 7-12% */
#define CONFIG_SLOPE_13_100             3   /* slope of >12% */
#define CONFIG_SLOPE_LIMIT              4   /* slope of >12% */

/* Moisture Sensor Setpoints */
/* Note: a MINMOIST setting must always be less than the MAXMOIST setting. */
#define CONFIG_MOIST_MIN                0//1
#define CONFIG_MOIST_MAX                SYS_MOISTURE_MAX
#define CONFIG_MOIST_DEFAULT_MIN        20
#define CONFIG_MOIST_DEFAULT_MAX        30

/* Sensor Group */
#define CONFIG_GROUP_NONE               -1  /* No sensor; no group. */

/* Climate */
#define CONFIG_CLIMATE_FULL_SUN         0   /* climate type of full sun */
#define CONFIG_CLIMATE_50_SHADE         1   /* climate type of 50% shade */
#define CONFIG_CLIMATE_75_SHADE         2   /* climate type of 75% shade */
#define CONFIG_CLIMATE_LIMIT            3

/* Plant Type Field Macros */
#define CONFIG_PT_SPECIES(x)            (x & 0x0F)
#define CONFIG_PT_DENSITY(x)            ((x & 0x30) >> 4)
#define CONFIG_PT_DT(x)                 ((x & 0xC0) >> 6)

/* Plant Type - Species */
#define CONFIG_PLANTTYPE_TREES          0   /* plant type of trees */
#define CONFIG_PLANTTYPE_SHRUBS         1   /* plant type of shrubs */
#define CONFIG_PLANTTYPE_GROUNDCOVER    2   /* plant type of ground cover */
#define CONFIG_PLANTTYPE_MIXTURE        3   /* plant type of mixture */
#define CONFIG_PLANTTYPE_FESCUE         4   /* plant type of fescue turf */
#define CONFIG_PLANTTYPE_BERMUDA        5   /* plant type of bermuda turf */
#define CONFIG_PLANTTYPE_LIMIT          6

/* Plant Type - Density */
#define CONFIG_PLANTDENSITY_SPARSE      0   /* plant density of sparse */
#define CONFIG_PLANTDENSITY_AVG         1   /* plant density of average */
#define CONFIG_PLANTDENSITY_HIGH        2   /* plant density of high */
#define CONFIG_PLANTDENSITY_LIMIT       3

/* Plant Type - Drought Tolerance */
#define CONFIG_PLANTDT_LOW              0   /* plant drought tolerance low */
#define CONFIG_PLANTDT_AVG              1   /* plant drought tolerance avg */
#define CONFIG_PLANTDT_HIGH             2   /* plant drought tolerance high */
#define CONFIG_PLANTDT_LIMIT            3

/* Application Efficiency */
#define CONFIG_APPEFF_MIN               1   /* min application efficiency */
#define CONFIG_APPEFF_MAX               100 /* max application efficiency */

/* RZWWS (Root Zone Working Water Storage), in units of 0.01 inch */
#define CONFIG_RZWWS_MIN                1       /* min RZWWS (0.01 inches) */
#define CONFIG_RZWWS_MAX                1000    /* max RZWWS (10.00 inches) */



/*Sensor Types */
#define SNS_NONE            0
#define SNS_WIRED_MOIST     1
#define SNS_WIRELESS_MOIST  2
#define SNS_WIRELESS_VALVE  3
#define SNS_FLOW            4
#define SNS_PRESSURE        5
#define SNS_RAIN_GAUGE      6
#define SNS_TYPE_MAX        7


/*generic sensor max values */
#define SNS_MAX_FLOW        100
#define SNS_MAX_PRESSURE    250
#define SNS_MAX_RAIN_GAUGE      100
/*generic sensor default values */
#define SNS_DEFAULT_PRESSURE    150
#define SNS_DEFAULT_RAIN_GAUGE  5


/*
**  PER-ZONE CONFIG STRUCT
*/

typedef struct
{
    uint8_t runTime[SYS_N_PROGRAMS];    /* per-program run times (0-240 min) */
    uint16_t appRate;                   /* applicaton rate, in 0.1 in/hr */
    uint8_t soilType;                   /* soil type */
    uint8_t slope;                      /* slope of terrain */
    uint8_t minMoist;                   /* minimum moisture threshold (0-98), for other sensors this is the min sensor value */
    uint8_t maxMoist;                   /* maximum moisture threshold (1-99), for other sensors this is the max sensor value */
    uint8_t climate;                    /* zone climate */
    uint8_t plantType;                  /* plant type, density, dt */
    int8_t group;                       /* sensor group leader, */
    uint8_t appEff;                     /* application efficiency (1-100%) */
    uint16_t rzwws;                     /* RZWWS, in 0.01 inches */
    uint8_t sensorType;                 /* sensor type */
    int8_t snsConTableIndex;            /* mac id of the wireless sensor concentrator */
    uint8_t snsConChan;                 /* channel of sensor concentrator zone assigned to 0 means no channel and 1-4 are channels.*/
    uint8_t minGPM;                     /* minimum water flow to shut off valves, gallon per min */
    uint8_t maxGPM;                     /* maximum water flow to shut off valves, gallon per min */

} configZone_t;



/******************************************************************************
 *
 *  PER-DAY IRRIGATION START TIMES
 *
 *****************************************************************************/

#define CONFIG_SCHED_START_LIMIT        (24 * 60)
#define CONFIG_SCHED_START_DISABLED     (24 * 60)   /* disabled */

#define CONFIG_SCHED_DAY_MON            0       /* Monday */
#define CONFIG_SCHED_DAY_TUE            1       /* Tuesday */
#define CONFIG_SCHED_DAY_WED            2       /* Wednesday */
#define CONFIG_SCHED_DAY_THU            3       /* Thursday */
#define CONFIG_SCHED_DAY_FRI            4       /* Friday */
#define CONFIG_SCHED_DAY_SAT            5       /* Saturday */
#define CONFIG_SCHED_DAY_SUN            6       /* Sunday */
#define CONFIG_SCHED_DAY_LIMIT          7


/*
**  PER-DAY/PER-PROGRAM IRRIGATION START TIME CONFIG STRUCT
*/

typedef struct
{
    uint16_t startTime;                 /* number of minutes after midnight */
} configSched_t;



/******************************************************************************
 *
 *  MANUFACTURING DATA
 *
 *****************************************************************************/

#define CONFIG_SN_SIZE                  8       /* WOIS Serial Number size */

typedef struct
{
    uint8_t serialNumber[CONFIG_SN_SIZE];   /* WOIS Serial Number (ASCII) */
    uint32_t systemType;        /* system type/configuration (binary) */
    uint32_t boardType;         /* board type/configuration (binary) */
    uint16_t systemRevision;    /* system major & minor revision (binary) */
    uint16_t boardRevision;     /* board major & minor revision (binary) */
    uint8_t pad[42];            /* pad out structure to total 64 bytes */
    uint16_t checkSum;          /* Manufacturing Data Image Checksum */
} configManuf_t;

#define CONFIG_MANUF_SIZE       sizeof(configManuf_t)   /* 64 bytes */


/*
**  Working copy of WOIS Serial Number kept in Global Memory
*/
extern uint8_t configSerialNumber[CONFIG_SN_SIZE];



/******************************************************************************
 *
 *  CONFIGURATION DATA IMAGE
 *
 *****************************************************************************/

typedef struct
{
    configSys_t sys;                        /* Header and System Config */
    configZone_t zone[SYS_N_ZONES];         /* Config for 48 Zones */
    configSched_t sched[7][SYS_N_PROGRAMS]; /* 7-day (0=Mon), 4-Pgm Schedule */
} configImage_t;
extern configImage_t config;

#define CONFIG_IMAGE_SIZE       sizeof(configImage_t)



/******************************************************************************
 *
 *  CONFIG FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void configInit(void);
void configPoll(void);
void configRestart(void);
void configImageSyncNeeded(void);
void configDefaultLoad(void);
bool_t configMemoryChecksumIsValid(uint16_t *pChecksum);
void configMemoryChecksumUpdate(void);
uint16_t configMemoryChecksumCalc(void);
void configWriteNextBlock(uint32_t imageOffset);
bool_t configImageChecksumIsValid(uint32_t imageOffset, uint16_t *checksum);
int16_t configImageContentValidate(uint32_t imageOffset);
bool_t configManufInit(void);
void configManufRead(configManuf_t *pManufImage);
void configImageVerify(void);
void configZonePlantTypeGet(uint8_t zi, uint8_t *pType, uint8_t *pDensity, uint8_t *pDt);
void configZonePlantTypeSet(uint8_t zi, uint8_t type, uint8_t density, uint8_t dt);
void configFactoryDefaultInit(void);
void configSnapshotSave(void);
void configSnapshotRead(uint32_t offset, void *pBuf, uint32_t len);
int16_t configBufferLoad(void);
void configBufferClear(void);
void configBufferWrite(const void *pBuf, uint32_t offset, uint32_t len);
void configEventLogSave(void);
void configEventLogRead(uint32_t offset, void *pBuf, uint32_t len);
uint16_t configRzwwsDefault(uint8_t plantType);
uint16_t configMemorySnapShotChecksumCalc(uint8_t *pData);

/* END config */

#endif
