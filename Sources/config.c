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
 * Module       : config.c
 * Description  : This file implements the configuration manager.
 *
 *****************************************************************************/

/* Used for building in Windows environment. */
#include "stdafx.h"

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "radio.h"
#include "datetime.h"
#include "irrigation.h"
#include "crc.h"
#include "ui.h"
#include "drvEeprom.h"
#include "drvRtc.h"
#include "ui.h"


/* Config Events */
#define CONFIG_EVENT_CORRUPT    SYS_EVENT_CONFIG + 1    /* Cfg Image corrupt */
#define CONFIG_EVENT_SET_DFLT   SYS_EVENT_CONFIG + 2    /* Set default cfg */
#define CONFIG_EVENT_EVT_SAVED  SYS_EVENT_CONFIG + 3    /* Event trace saved */
#define CONFIG_EVENT_DL_APPLY   SYS_EVENT_CONFIG + 4    /* Cfg Dld applied */
#define CONFIG_EVENT_DL_APP_F   SYS_EVENT_CONFIG + 5    /* Cfg Dld apply fail*/



/******************************************************************************
 *
 *  GLOBAL VARIABLES
 *
 *****************************************************************************/
extern uint8_t navEndZone;


/*
**  WOIS Serial Number (Working/Cache copy from Manuf Image)
*/
uint8_t configSerialNumber[CONFIG_SN_SIZE];     /* WOIS Serial Number */

/*
**  Read-Write System Configuration Data (Working/Cache copy of Config Image)
*/
configImage_t config;                           /* Config Image in RAM */

/*
**  Configuration State Data
*/
static uint32_t configDirtyTime;                /* dirty-detect tick count */
static uint32_t configNextWriteOffset;          /* next EEPROM write offset */
static uint8_t configImageStatus1;              /* Config Image 1 Status */
static uint8_t configImageStatus2;              /* Config Image 2 Status */
uint8_t configState = CONFIG_STATE_RESTART;     /* Config Manager State */
static bool_t configRzwwsChanged;               /* RZWWS changed flag */

/*
**  Default RZWWS values based on the plant type
*/
static const uint16_t configPlantTypeRzwws[] =
{
    510,        /* Trees */
    200,        /* Shrubs */
    90,         /* Ground Cover */
    225,        /* Mixture */
    85,         /* Fescue Turf */
    55,         /* Bermuda Turf */
};


/******************************************************************************
 *
 * configInit
 *
 * PURPOSE
 *      This routine is called by the system's initialization routine
 *      to initialize the configuration manager and load the configuration
 *      data store image into RAM.
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
void configInit(void)
{
    /* Verify manufacturing data and load the WOIS serial number. */
    if (!configManufInit())
    {
        /* Generate a system fault for invalid manufacturing data */
        sysFaultSet(SYS_FAULT_MANUFDATA);
    }

    /* Verify configuration image copies stored in EEPROM. */
    configImageVerify();

    /* Load configuration image working copy into RAM. */
    if (configImageStatus1 != CONFIG_IMAGE_CORRUPT)
    {
        /* Load configuration image 1 into memory */
        drvEepromRead(CONFIG_IMAGE1, &config, CONFIG_IMAGE_SIZE);
    }
    else if (configImageStatus2 != CONFIG_IMAGE_CORRUPT)
    {
        /* Load configuration image 2 into memory */
        drvEepromRead(CONFIG_IMAGE2, &config, CONFIG_IMAGE_SIZE);
    }
    else
    {
        /* Load factory default configuration image into RAM. */
        configDefaultLoad();
    }

    /* Set configuration state to Clean. */
    configState = CONFIG_STATE_CLEAN;

    /* Clear plant type changed flag. */
    configRzwwsChanged = FALSE;
}


/******************************************************************************
 *
 * configPoll
 *
 * PURPOSE
 *      This routine is called by the system's main polling loop to check for
 *      modifications to configuration settings and maintain the non-volatile
 *      configuration image copies stored in EEPROM.
 *
 * CALLING SEQUENCE
 *      configPoll();
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine keeps the two copies of non-volatile configuration image
 *      in the EEPROM up-to-date and in sync with each other.
 *
 *      The Configuration Manager uses a finite state machine to control
 *      the sequence of configuration data store management operations.
 *
 *      Writing data to EEPROM is a time-consuming process and it is not
 *      possible to write a complete configuration image in a single polling
 *      cycle without severely degrading the user interface response-time.
 *      Accordingly, the Configuration Manager is designed to performs its
 *      synchronization duties, writing a block at a time, as a background
 *      task.
 *
 *      The Configuration Manager also delays the start of re-synchronization
 *      EEPROM writes via lazy-write logic (using a "dirty" timer).
 *      The lazy-write logic coupled with restarting the dirty timer
 *      on each detected change helps to reduce the number of EEPROM
 *      write operations from what would otherwise result as the user
 *      cycles through multiple field values while modifying configuration
 *      settings using the WOIS front panel switches.
 *
 *****************************************************************************/
void configPoll(void)
{
    bool_t isModified = FALSE;
    uint16_t newChecksum;

    /* set navigation zones based on unit type */
    if(config.sys.unitType == UNIT_TYPE_MASTER)
    {
       navEndZone = (config.sys.numUnits+1) * 12;
    }
    else
    {
       navEndZone = 12;
    }
    
    /* Check for modification of configuration image RAM working copy. */
    isModified = !configMemoryChecksumIsValid(&newChecksum);
    if (isModified)
    {
        /* Update configuration image RAM working copy checksum. */
        config.sys.checkSum = htons(newChecksum);

        /* Schedule re-write of both EEPROM configuration images. */
        configImageSyncNeeded();
        

    }

    /* Check if any Plant Type configuration changes were made. */
    if (configRzwwsChanged)
    {
        /* Hold off MB fix to reduce artifacts from intermediate values. */
        if (dtElapsedSeconds(configDirtyTime) > CONFIG_MB_FIX_SECS)
        {
            /* Insure MB values are within configured RZWWS limits. */
            irrMbFix();
            /* Clear plant type changed flag. */
            configRzwwsChanged = FALSE;
        }
    }

    switch (configState)
    {
        case CONFIG_STATE_RESTART:
            /* Check images stored in non-volatile memory for corruption. */
            configImageVerify();
            configState = CONFIG_STATE_CLEAN;
            break;

        case CONFIG_STATE_CLEAN:
            /* Check for corrupt images needing rewrite. */
            if ((configImageStatus1 != CONFIG_IMAGE_VALID) ||
                (configImageStatus2 != CONFIG_IMAGE_VALID))
            {
                /* Change state to Dirty. */
                configState = CONFIG_STATE_DIRTY;
            }
            break;

        case CONFIG_STATE_DIRTY:
            /* First priority is fixing any corrupt configuration images. */
            if (configImageStatus1 == CONFIG_IMAGE_CORRUPT)
            {
                /* Begin corruption-fix re-sync for image 1. */
                configState = CONFIG_STATE_WRITING1;
                configNextWriteOffset = 0;
            }
            else if (configImageStatus2 == CONFIG_IMAGE_CORRUPT)
            {
                /* Begin corruption-fix re-sync for image 2. */
                configState = CONFIG_STATE_WRITING2;
                configNextWriteOffset = 0;
            }
            else if (dtElapsedSeconds(configDirtyTime) > CONFIG_LAZY_WRITE_SECS)
            {
                /* Begin lazy write re-sync, starting with image 1. */
                configState = CONFIG_STATE_WRITING1;
                configNextWriteOffset = 0;
            }
            break;

        case CONFIG_STATE_WRITING1:
            if (isModified)
            {
                /* Another change was made, so start over. */
                configNextWriteOffset = 0;
            }
            if (configNextWriteOffset < CONFIG_IMAGE_SIZE)
            {
                configWriteNextBlock(CONFIG_IMAGE1);
            }
            else
            {
                /* Mark image 1 as now being in-sync. */
                configImageStatus1 = CONFIG_IMAGE_VALID;
                /* Check if image 2 needs updating. */
                if (configImageStatus2 == CONFIG_IMAGE_VALID)
                {
                    /* All images are in-sync, set state to Clean. */
                    configState = CONFIG_STATE_CLEAN;
                    
                    /* if master then send new config to expansion units */
                    if(config.sys.unitType == UNIT_TYPE_MASTER) 
                    {
                       expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, RADIO_EXP_SEND_ALL);     
                    }
                }
                else
                {
                    /* Update image 2. */
                    configState = CONFIG_STATE_WRITING2;
                    configNextWriteOffset = 0;
                }
            }
            break;

        case CONFIG_STATE_WRITING2:
            if (isModified)
            {
                /* Another change was made, so start over. */
                configNextWriteOffset = 0;
            }
            if (configNextWriteOffset < CONFIG_IMAGE_SIZE)
            {
                configWriteNextBlock(CONFIG_IMAGE2);
            }
            else
            {
                /* Mark image 2 as now being in-sync. */
                configImageStatus2 = CONFIG_IMAGE_VALID;
                /* Check if image 1 needs updating. */
                if (configImageStatus1 == CONFIG_IMAGE_VALID)
                {
                    /* All images are in-sync, set state to Clean. */
                    configState = CONFIG_STATE_CLEAN;
                    
                    /* if master then send new config to expansion units */
                    if(config.sys.unitType == UNIT_TYPE_MASTER) 
                    {
                       expansionBusSendCmd(RADIO_CMD_CFG_PUT_START, RADIO_EXP_SEND_ALL);     
                    }
                }
                else
                {
                    /* Update image 1. */
                    configState = CONFIG_STATE_WRITING1;
                    configNextWriteOffset = 0;
                }
            }
            break;

        default:
            /* Invalid state - should never occur. */
            break;
    }
}


/******************************************************************************
 *
 * configRestart
 *
 * PURPOSE
 *      This routine is called to restart the configuration manager upon return
 *      from a low-power shutdown.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      After returning from a low-power shutdown, the configuration image
 *      working copy in RAM is assumed the most up-to-date copy.  The EEPROM
 *      configuration image copies will be inspected for corruption, and
 *      rewritten if necessary.  Any EEPROM configuration copies that are not
 *      in-sync with the RAM working copy will be rewritten as well.
 *      (An EEPROM configuration image copy found to be corrupted shall be
 *      given first priority for update.)
 *
 *****************************************************************************/
void configRestart(void)
{
    configState = CONFIG_STATE_RESTART;
}


/******************************************************************************
 *
 * configImageSyncNeeded
 *
 * PURPOSE
 *      This routine is called to mark EEPROM configuration image copies
 *      as being out-of-sync with the working configuration image in RAM
 *      and to facilitate scheduling of EEPROM configuration image updates.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine marks the EEPROM configuration image copies as being
 *      out-of-sync with the working configuration image in RAM.
 *      If an EEPROM configuration image is flagged as being corrupt, this
 *      routine does not modify its status; corrupt is a more urgent status
 *      than out-of-sync.
 *
 *      This routine restarts the "Dirty" timer used for "lazy write" updates
 *      to hold off EEPROM update until there is a lull in configuration
 *      change activity.  This helps to reduce the number of EEPROM writes
 *      that might otherwise occur as a result of the user transitioning
 *      through intermediate field values.
 *
 *****************************************************************************/
void configImageSyncNeeded(void)
{
    /* Restart the dirty timer. */
    configDirtyTime = dtTickCount;

    /* Mark all EEPROM images for re-sync (corrupt images already marked). */
    if (configImageStatus1 != CONFIG_IMAGE_CORRUPT)
    {
        configImageStatus1 = CONFIG_IMAGE_OLD;
    }
    if (configImageStatus2 != CONFIG_IMAGE_CORRUPT)
    {
        configImageStatus2 = CONFIG_IMAGE_OLD;
    }
}


/******************************************************************************
 *
 * configDefaultLoad
 *
 * PURPOSE
 *      This routine loads the "factory default" configuration into the
 *      configuration image memory working/cache data store.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The default configuation is the same for each zone, and represents
 *      typical (or average) settings.
 *
 *****************************************************************************/
void configDefaultLoad(void)
{
    /* Initialize factory default system configuration. */
    config.sys.version      = htons(CONFIG_VERSION);
    config.sys.numZones     = CONFIG_ZONE_MAX;
    config.sys.opMode       = CONFIG_OPMODE_RUNTIME;
    config.sys.pulseMode    = CONFIG_PULSEMODE_OFF;
    config.sys.timeFmt      = CONFIG_TIMEFMT_12HR;
#ifdef RADIO_ZB1
    config.sys.radioPanId   = htonll(CONFIG_PAN_ID_ANY);
#else
    config.sys.radioPanId   = htons(PAN_SYS_DEFAULT);
    memset(config.sys.pad, 0, sizeof(config.sys.pad));
#endif

    config.sys.unitType = UNIT_TYPE_MASTER;
    config.sys.numUnits = 0;
    config.sys.masterMac = 0x0013A20000000000;
    config.sys.expMac1 = 0x0013A20000000000;
    config.sys.expMac2 = 0x0013A20000000000;
    config.sys.expMac3 = 0x0013A20000000000;
    config.sys.masterNumZones = CONFIG_ZONE_MAX;
    config.sys.expNumZones1 = CONFIG_ZONE_MAX;
    config.sys.expNumZones2 = CONFIG_ZONE_MAX;
    config.sys.expNumZones3 = CONFIG_ZONE_MAX;
    config.sys.numSensorCon = 0;
    
    /*Initialize factory default sensor concentrator configuration */
    for(int sc = 0; sc < MAX_NUM_SC; sc++)
    {
        for(int ch =0; ch < SC_NUM_CHAN_UNIT;ch++)
        {
            config.sys.assocSensorCon[sc].channelZone[ch] = SC_CHAN_NOT_ASSIGNED;
        }
        config.sys.assocSensorCon[sc].macId = 0;
        config.sys.assocSensorCon[sc].zoneRange =0;
    }
    
    /* Initialize factory default zone configuration for all zones. */
    for (int z = 0; z < SYS_N_ZONES; z++)
    {
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            config.zone[z].runTime[p]  = 0;
        }
        config.zone[z].appRate   = htons(150);
        config.zone[z].soilType  = CONFIG_SOILTYPE_LOAM;
        config.zone[z].slope     = CONFIG_SLOPE_0_3;
        config.zone[z].minMoist  = CONFIG_MOIST_DEFAULT_MIN;
        config.zone[z].maxMoist  = CONFIG_MOIST_DEFAULT_MAX;
        config.zone[z].minGPM    = 0;
        config.zone[z].maxGPM    = 99;        
        config.zone[z].climate   = CONFIG_CLIMATE_FULL_SUN;
        config.zone[z].group     = CONFIG_GROUP_NONE;
        configZonePlantTypeSet(z, CONFIG_PLANTTYPE_BERMUDA,
                                  CONFIG_PLANTDENSITY_AVG,
                                  CONFIG_PLANTDT_AVG);
        config.zone[z].appEff   = 100;
        /* config.zone[z].rzwws is set by configZonePlantTypeSet() */
        config.zone[z].sensorType = SNS_NONE;
        config.zone[z].snsConTableIndex = ZONE_SC_INDEX_NONE;           
        config.zone[z].snsConChan = SC_NONE_SELECTED;               
        
    }

    /* Initialize factory default irrigation schedule configuration. */
    for (int d = 0; d < CONFIG_SCHED_DAY_LIMIT; d++)
    {
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            config.sched[d][p].startTime = htons(CONFIG_SCHED_START_DISABLED);
        }
    }

    /* Update the configuration image checksum. */
    configMemoryChecksumUpdate();

    /* Trace set default configuration event. */
    sysEvent(CONFIG_EVENT_SET_DFLT, 0);
}


/******************************************************************************
 *
 * configManufInit
 *
 * PURPOSE
 *      This routine reads the manufacturing data from EEPROM and verifies
 *      that it is not corrupted.  This routine also copies the WOIS serial
 *      number into global memory.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns TRUE if the manufacturing data image is valid;
 *      otherwise FALSE is returned.
 *
 * NOTES
 *      If the manufacturing data image is not valid, this routine has no way
 *      of "fixing" the corrupted EEPROM data which can only be written at
 *      the factory.  This routine loads a default WOIS serial number to
 *      allow the WOIS to operate so it can communicate the fault condition
 *      via front panel and radio.  The default serial number consists of all
 *      ASCII space characters (0x20).
 *
 *****************************************************************************/
bool_t configManufInit(void)
{
    configManuf_t manufImage;      /* Manufacturing Data Image */
    uint16_t checksum;
    bool_t isValid;

    /* Read manufacturing data from EEPROM. */
    configManufRead(&manufImage);

    /* Compute CRC for all manufacturing data stored up to checksum. */
    checksum = crc16(&manufImage, CONFIG_MANUF_SIZE - 2);

    isValid = (checksum == ntohs(manufImage.checkSum));

    if (isValid)
    {
        /* Copy WOIS serial number to global memory. */
        memcpy(configSerialNumber, manufImage.serialNumber, CONFIG_SN_SIZE);
    }
    else
    {
        /* Load default serial number - all spaces. */
        memset(configSerialNumber, ' ', CONFIG_SN_SIZE);
    }

    return isValid;
}


/******************************************************************************
 *
 * configManufRead
 *
 * PURPOSE
 *      This routine reads the manufacturing data image from EEPROM into the
 *      caller's buffer.
 *
 * PARAMETERS
 *      pManufImage OUT     pointer to manufacturing data buffer
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      The caller's buffer must be sized to accommodate the entire "raw"
 *      manufacturing data image (64 bytes).
 *
 *****************************************************************************/
void configManufRead(configManuf_t *pManufImage)
{
    /* Read manufacturing data from EEPROM. */
    drvEepromRead(CONFIG_MANUF, pManufImage, CONFIG_MANUF_SIZE);
}


/******************************************************************************
 *
 * configMemoryChecksumIsValid
 *
 * PURPOSE
 *      This routine determines if the configuration memory checksum is valid.
 *
 * PARAMETERS
 *      pChecksum   OUT     the current checksum
 *
 * RETURN VALUE
 *      This routine returns TRUE if the configuration memory checksum is
 *      valid.
 *
 *  NOTES
 *      This routine enables the Configuration Manager to detect when
 *      changes are made to the configuration image cached in RAM.
 *      When the checksum is invalid, the configuration is considered
 *      "dirty" or out-of-sync with the image copies stored in EEPROM.
 *
 *****************************************************************************/
bool_t configMemoryChecksumIsValid(uint16_t *pChecksum)
{
    *pChecksum = configMemoryChecksumCalc();

    return (*pChecksum == ntohs(config.sys.checkSum));
}


/******************************************************************************
 *
 * configMemoryChecksumUpdate
 *
 * PURPOSE
 *      This routine updates the configuration memory checksum.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine fixes the checksum of the configuration image cached in
 *      RAM (makes it "clean").
 *
 *****************************************************************************/
void configMemoryChecksumUpdate(void)
{
    config.sys.checkSum = htons(configMemoryChecksumCalc());
}


/******************************************************************************
 *
 * configMemoryChecksumCalc
 *
 * PURPOSE
 *      This routine calculates the checksum for the configuration image in
 *      global memory.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      The routine returns the configuration memory checksum.
 *
 * NOTES
 *
 *****************************************************************************/
uint16_t configMemoryChecksumCalc(void)
{
    uint8_t *pData = (uint8_t *)&config;
    uint16_t checksum;

    /* Compute CRC for all config data stored after version and checksum. */
    checksum = crc16(pData + 4, CONFIG_IMAGE_SIZE - 4);

    return checksum;
}

/******************************************************************************
 *
 * configMemorySnapShotChecksumCalc
 *
 * PURPOSE
 *      This routine calculates the checksum for the configuration image in
 *      global memory.
 *
 * PARAMETERS
 *      pData   IN    pointer to the snapshot configuration in memory
 *
 * RETURN VALUE
 *      The routine returns the configuration memory checksum.
 *
 * NOTES
 *
 *****************************************************************************/
uint16_t configMemorySnapShotChecksumCalc(uint8_t *pData)
{
    uint16_t checksum;

    /* Compute CRC for all config data stored after version and checksum. */
    checksum = crc16(pData + 4, CONFIG_IMAGE_SIZE - 4);

    return checksum;
}


/******************************************************************************
 *
 * configWriteNextBlock
 *
 * PURPOSE
 *      This routine is called by the configuration manager state machine
 *      to write the next block of configuration data to EEPROM.
 *
 * PARAMETERS
 *      imageOffset IN  offset to start of configuration image in EEPROM
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      Writing a config image to EEPROM is a slow process.  The EEPROM is
 *      organized into 64-byte segments and there is a 5 ms delay for each
 *      segment write to complete before the next segment can be written.
 *      While the EEPROM driver permits writing large buffers that span
 *      segment boudaries, it can be inefficient to write buffers across
 *      a segment boundary.  The Configuration Manager state machine is
 *      designed to write configuration images to EEPROM one segment at
 *      a time.  This routine keeps track of its progress using the
 *      configNextWriteOffset global variable.
 *
 *****************************************************************************/
void configWriteNextBlock(uint32_t imageOffset)
{
    uint8_t *pData = (uint8_t *)&config;
    uint32_t nBytes;

    nBytes = CONFIG_IMAGE_SIZE - configNextWriteOffset;
    if (nBytes > CONFIG_BLOCK_SIZE)
    {
        nBytes = CONFIG_BLOCK_SIZE;
    }

    drvEepromWrite(&pData[configNextWriteOffset],
                   imageOffset + configNextWriteOffset,
                   nBytes);

    configNextWriteOffset += nBytes;
}


/******************************************************************************
 *
 * configImageChecksumIsValid
 *
 * PURPOSE
 *      This routine verifies the integrity of a config image in EEPROM.
 *      The verification involves calculating the checksum of the config
 *      image as it exists in EEPROM and then verifying that the header
 *      contains the correct config version and checksum.
 *
 * PARAMETERS
 *      imageOffset IN  offset to start of config image in EEPROM
 *      checksum    OUT checksum value contained in image
 *
 * RETURN VALUE
 *      This routine returns TRUE if the configuration image has a valid
 *      checksum.
 *
 * NOTES
 *      Each config image has a 4-byte header.  The first 2 header bytes are
 *      the config version; the second 2 bytes are the checksum.  The checksum
 *      is calculated over the remaining bytes (offset 4-n) of the config
 *      image.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
bool_t configImageChecksumIsValid(uint32_t imageOffset, uint16_t *checksum)
{
    uint16_t crc = CRC_INIT_VALUE;      /* CRC accumulator */
    uint16_t imageVersion;              /* version number contained in image */
    uint16_t imageCrc;                  /* CRC value contained in image */
    uint32_t nextIndex;                 /* next offset within image to read */
    uint32_t nBytes;                    /* next number of bytes to read */
    uint8_t data[CONFIG_BLOCK_SIZE];    /* read buffer */
    uint32_t i;                         /* index */

    /*
    **  Read version and checksum fields from the configuration image header.
    **  Note: Config version (2 bytes) is stored at image offset 0.
    **        Config checksum (2 bytes) is stored at image offset 2.
    */
    drvEepromRead(imageOffset, &imageVersion, sizeof(imageVersion));
    drvEepromRead(imageOffset + 2, &imageCrc, sizeof(imageCrc));
    /* Insure correct byte order. */
    imageVersion = ntohs(imageVersion);
    imageCrc = ntohs(imageCrc);
    /* Store CRC checksum value from image in caller's designated location. */
    *checksum = imageCrc;

    /*
    **  Read remaining configuration image, a block at a time, starting after
    **  the 4-byte version and checksum header, to calculate the actual CRC
    **  for the configuration image data.
    **  Note: The configuration image CRC does not cover the 4-byte header.
    */
    nextIndex = 4;
    while (nextIndex < CONFIG_IMAGE_SIZE)
    {
        nBytes = CONFIG_IMAGE_SIZE - nextIndex;
        if (nBytes > CONFIG_BLOCK_SIZE)
        {
            nBytes = CONFIG_BLOCK_SIZE;
        }

        drvEepromRead(imageOffset + nextIndex, data, nBytes);

        /* Compute CRC for data block, updating the CRC accumulator. */
        for (i = 0; i < nBytes; i++)
        {
            crcByte(data[i], &crc);
        }

        nextIndex += nBytes;

        /* Make sure watchdog doesn't timeout during this long operation. */
        sysExecutionExtend();
    }

    /*
    **  Verify that the configuration version of the image is correct and
    **  compare the calculated CRC value for the configuration image with
    **  the CRC value contained in the image header.  The return value is
    **  TRUE if both of these values are correct.
    */
    return (imageVersion == CONFIG_VERSION) && (imageCrc == crc);
}


/******************************************************************************
 *
 * configImageContentValidate
 *
 * PURPOSE
 *      This routine verifies the content and integrity of a config image in
 *      EEPROM.
 *
 * PARAMETERS
 *      imageOffset IN  offset to start of config image in EEPROM
 *
 * RETURN VALUE
 *      This routine returns -1 if the configuration image has a valid
 *      checksum and valid contents; otherwise, the byte-offset of the first
 *      encountered invalid field is returned.
 *
 * NOTES
 *      Errors cause the byte-offset of the first-encountered invalid field to
 *      be returned, however there is no guarantee that all fields before
 *      the error offset are valid.  Fields are not necessarily validated
 *      in strict physical order; thus it can't be implied that no errors exist
 *      in the physically preceding content.  Some earlier-occuring fields are
 *      re-validated after later fields have already been validated.
 *
 *      Because the size of a configuration image is quite large (840 bytes for
 *      Configuration Version 1, and likely to grow larger in future versions),
 *      this routine reads the config image in small blocks that can easily fit
 *      on the stack.  A block is read into an automatic variable defined as
 *      a union of 3 structures: the system config struct, the per-zone config
 *      struct, and the schedule config struct.  In Configuration Version 1,
 *      the largest block is the schedule config struct at 56 bytes; the
 *      smallest block  is 16 bytes.  The zone config struct is iteratively
 *      read for each zone.  Certain configuration parameters are copied to
 *      the stack for use in later additional validity checking.
 *
 *      The System Manager sysExecutionExtend function is regularly called
 *      during this long process to prevent watchdog reset and manage any
 *      power failures.
 *
 *****************************************************************************/
int16_t configImageContentValidate(uint32_t imageOffset)
{
    int i;                      /* generic index */
    uint8_t day;                /* day index */
    uint8_t zi;                 /* zone index */
    uint8_t numZones;           /* configured number of zones */
    int8_t group[SYS_N_ZONES];  /* zone group array */
    uint32_t nextIndex = 0;     /* next offset within image to read */
    size_t offset;              /* error offset if invalid field is found */
    uint16_t checksum;          /* configuration image checksum */
    char debugBuf[8];           /* DEBUG BUFFER */
    union                       /* buffer for EEPROM config data reads */
    {                           /* -> organized as union of structures: */
        configSys_t sys;                            /* System Config */
        configZone_t zone;                          /* Zone Config */
        configSched_t sched[7][SYS_N_PROGRAMS];     /* Schedule Config */
    } data;

    /* Read image header and system configuration data. */
    drvEepromRead(imageOffset + nextIndex, &data, sizeof(configSys_t));
    /* Verify version. */
    if (ntohs(data.sys.version) != CONFIG_VERSION)
    {
        offset = woffsetof(configImage_t, sys.version);
        goto error_exit;
    }
    /* Verify checksum. */
    if (!configImageChecksumIsValid(imageOffset, &checksum))
    {
        offset = woffsetof(configImage_t, sys.checkSum);
        goto error_exit;
    }
    /* Verify number of zones. */
    numZones = data.sys.numZones;
    if (numZones > sysMaxZones)
    {
        offset = woffsetof(configImage_t, sys.numZones);
        goto error_exit;
    }
    /* Verify operating mode. */
    if (data.sys.opMode >= CONFIG_OPMODE_LIMIT)
    {
        offset = woffsetof(configImage_t, sys.opMode);
        goto error_exit;
    }
    /* Verify pulse mode. */
    if (data.sys.pulseMode >= CONFIG_PULSEMODE_LIMIT)
    {
        offset = woffsetof(configImage_t, sys.pulseMode);
        goto error_exit;
    }
    /* Verify time format. */
    if (data.sys.timeFmt >= CONFIG_TIMEFMT_LIMIT)
    {
        offset = woffsetof(configImage_t, sys.timeFmt);
        goto error_exit;
    }
#ifdef RADIO_ZB
    /* Verify radio PAN ID. */

    /* NOTE:  All possible 64-bit PAN ID values from 0 - 0xFFFFFFFFFFFFFFFF are valid. */

#else
    /* Verify radio PAN ID. */
    if ((ntohs(data.sys.radioPanId) >= CONFIG_PAN_ID_LIMIT) &&
        (ntohs(data.sys.radioPanId) != CONFIG_PAN_ID_ANY))
    {
        offset = woffsetof(configImage_t, sys.radioPanId);
        goto error_exit;
    }
    /* Verify pad bytes are zeros. */
    for (i = 0; i < 6; i++)
    {
        if (data.sys.pad[i] != 0)
        {
            offset = woffsetof(configImage_t, sys.pad[i]);
            goto error_exit;
        }
    }
#endif

    /* Increment next byte index past end of system configuration data. */
    nextIndex += sizeof(configSys_t);

    /*
    **  Read and verify zone configuration data, iterating through each zone.
    */
    for (zi = 0; zi < SYS_N_ZONES; zi++)
    {
        drvEepromRead(imageOffset + nextIndex, &data, sizeof(configZone_t));
        /* Increment next byte index past end of current zone. */
        nextIndex += sizeof(configZone_t);
        /* Verify zone run time. */
        for (i = 0; i < SYS_N_PROGRAMS; i++)
        {
            if (data.zone.runTime[i] > CONFIG_RUNTIME_MAX)
            {
                offset = woffsetof(configImage_t, zone[zi].runTime[i]);
                goto error_exit;
            }
        }
        /* Verify zone application rate. */
        if ((ntohs(data.zone.appRate) == 0) ||
            (ntohs(data.zone.appRate) > CONFIG_APPRATE_MAX))
        {
            offset = woffsetof(configImage_t, zone[zi].appRate);
            goto error_exit;
        }
        /* Verify zone soil type. */
        if (data.zone.soilType >= CONFIG_SOILTYPE_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].soilType);
            goto error_exit;
        }
        /* Verify zone slope. */
        if (data.zone.slope >= CONFIG_SLOPE_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].slope);
            goto error_exit;
        }
        /* Verify zone minimum moisture. */
        if ((data.zone.minMoist < CONFIG_MOIST_MIN) ||
            (data.zone.minMoist > (CONFIG_MOIST_MAX - 1)))
        {
            offset = woffsetof(configImage_t, zone[zi].minMoist);
            goto error_exit;
        }
        /* Verify zone maximum moisture. */
        if (((data.zone.sensorType == SNS_WIRED_MOIST) ||(data.zone.sensorType == SNS_WIRELESS_MOIST)) &&
            ((data.zone.maxMoist < (CONFIG_MOIST_MIN + 1)) ||(data.zone.maxMoist > CONFIG_MOIST_MAX)))
        {
            offset = woffsetof(configImage_t, zone[zi].maxMoist);
            goto error_exit;
        }
        /* Verify zone moisture min-max relationship. */
        if (data.zone.minMoist >= data.zone.maxMoist)
        {
            offset = woffsetof(configImage_t, zone[zi].minMoist);
            goto error_exit;
        }
        /* Verify zone climate. */
        if (data.zone.climate >= CONFIG_CLIMATE_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].climate);
            goto error_exit;
        }
        /* Verify zone plant type. */
        if (CONFIG_PT_SPECIES(data.zone.plantType) >= CONFIG_PLANTTYPE_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].plantType);
            goto error_exit;
        }
        if (CONFIG_PT_DENSITY(data.zone.plantType) >= CONFIG_PLANTDENSITY_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].plantType);
            goto error_exit;
        }
        if (CONFIG_PT_DT(data.zone.plantType) >= CONFIG_PLANTDT_LIMIT)
        {
            offset = woffsetof(configImage_t, zone[zi].plantType);
            goto error_exit;
        }
        /* Verify zone sensor group. */
        if ((data.zone.group != CONFIG_GROUP_NONE) &&
            ((data.zone.group < 0) || (data.zone.group >= SYS_N_ZONES)))
        {
            offset = woffsetof(configImage_t, zone[zi].group);
            goto error_exit;
        }
        /* Save zone group setting for final group consistency check. */
        group[zi] = data.zone.group;

        /* Verify zone application rate efficiency. */
        if ((data.zone.appEff < CONFIG_APPEFF_MIN) ||
            (data.zone.appEff > CONFIG_APPEFF_MAX))
        {
            offset = woffsetof(configImage_t, zone[zi].appEff);
            goto error_exit;
        }

        /* Verify zone RZWWS. */
        if ((ntohs(data.zone.rzwws) < CONFIG_RZWWS_MIN) ||
            (ntohs(data.zone.rzwws) > CONFIG_RZWWS_MAX))
        {
            offset = woffsetof(configImage_t, zone[zi].rzwws);
            goto error_exit;
        }

        /*
        **  Call sysExecutionExtend at the end of each loop to prevent
        **  watchdog reset during this relatively long operation.
        **  (This routine iterates through each of 48 zones, requiring
        **  48 individual EEPROM read operations to be performed.)
        */
        sysExecutionExtend();
    }

    /* Perform final zone sensor group consistency check. */
    for (zi = 0; zi < numZones; zi++)
    {
        if ((group[zi] == CONFIG_GROUP_NONE) ||
            (group[zi] == zi))
        {
            /* No Sensor, Not Grouped OR Wired Sensor. */
            continue;
        }
        if ((group[zi] < numZones) &&
            (group[group[zi]] == group[zi]))
        {
            /* No Sensor, Grouped with Wired Sensor of another zone. */
            continue;
        }
        offset = woffsetof(configImage_t, zone[zi].group);
        goto error_exit;
    }

    /* Read and verify irrigation schedule configuration. */
    drvEepromRead(imageOffset + nextIndex, &data, sizeof(data.sched));
    for (day = 0; day < CONFIG_SCHED_DAY_LIMIT; day++)
    {
        for (i = 0; i < SYS_N_PROGRAMS; i++)
        {
            if ((ntohs(data.sched[day][i].startTime) != CONFIG_SCHED_START_DISABLED) &&
                (ntohs(data.sched[day][i].startTime) >= CONFIG_SCHED_START_LIMIT))
            {
                offset = woffsetof(configImage_t, sched[day][i].startTime);
                goto error_exit;
            }
        }
    }
    /* Configuration content is valid. */
    return -1;

error_exit:

    debugWrite("configImageContentValidate failed at offset 0x");
    sprintf(debugBuf, "%04X\n", offset);
    debugWrite(debugBuf);

    return (int16_t)offset;
}


/******************************************************************************
 *
 * configImageVerify
 *
 * PURPOSE
 *      This routine verifies the checksum of all configuration image copies
 *      in EEPROM and updates the Image Status global variables.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine updates the values of the following Image Status global
 *      variables:
 *          configImageStatus1 - status of EEPROM config image #1
 *          configImageStatus2 - status of EEPROM config image #2
 *
 *****************************************************************************/
void configImageVerify(void)
{
    uint16_t checksum;

    /* Verify configuration image 1. */
    if (configImageChecksumIsValid(CONFIG_IMAGE1, &checksum))
    {
        if (checksum == ntohs(config.sys.checkSum))
        {
            /* Configuration is valid and matches the copy in RAM. */
            configImageStatus1 = CONFIG_IMAGE_VALID;
        }
        else
        {
            /* Configuration is valid but out of date. */
            configImageStatus1 = CONFIG_IMAGE_OLD;
        }
    }
    else
    {
        configImageStatus1 = CONFIG_IMAGE_CORRUPT;

        /* Trace corrupt image1 detected event. */
        sysEvent(CONFIG_EVENT_CORRUPT, 1);
    }

    /* Verify configuration image 2. */
    if (configImageChecksumIsValid(CONFIG_IMAGE2, &checksum))
    {
        if (checksum == ntohs(config.sys.checkSum))
        {
            configImageStatus2 = CONFIG_IMAGE_VALID;
        }
        else
        {
            configImageStatus2 = CONFIG_IMAGE_OLD;
        }
    }
    else
    {
        configImageStatus2 = CONFIG_IMAGE_CORRUPT;

        /* Trace corrupt image2 detected event. */
        sysEvent(CONFIG_EVENT_CORRUPT, 2);
    }
}


/******************************************************************************
 *
 * configZonePlantTypeGet
 *
 * PURPOSE
 *      This routine returns 3 bytes of plant type information for a zone.
 *      The plant type, density, and drought tolerance are stored in a single
 *      byte within the configuration data store, and this routine splits up
 *      the bit-fields into separate bytes for ease of use.
 *
 * PARAMETERS
 *      zi          IN      zone index (0-47)
 *      pType       OUT     plant type (i.e., the plant species type)
 *      pDensity    OUT     plant density
 *      pDt         OUT     plant drought tolerance
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void configZonePlantTypeGet(uint8_t zi,
                            uint8_t *pType,
                            uint8_t *pDensity,
                            uint8_t *pDt)
{
    if (zi < SYS_N_ZONES)
    {
        *pType = CONFIG_PT_SPECIES(config.zone[zi].plantType);
        *pDensity = CONFIG_PT_DENSITY(config.zone[zi].plantType);
        *pDt = CONFIG_PT_DT(config.zone[zi].plantType);
    }
}


/******************************************************************************
 *
 * configZonePlantTypeSet
 *
 * PURPOSE
 *      This routine takes 3 bytes of plant type information for a zone and
 *      combines this data into bit-fields of a single byte for plant type
 *      in the configuration data store.  It also sets RZWWS based on the plant
 *      type.
 *
 * PARAMETERS
 *      zi          IN      zone index (0-47)
 *      type        IN      plant type (i.e., the plant species type)
 *      density     IN      plant density
 *      dt          IN      plant drought tolerance
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void configZonePlantTypeSet(uint8_t zi,
                            uint8_t type,
                            uint8_t density,
                            uint8_t dt)
{
    if (zi < SYS_N_ZONES)
    {
        config.zone[zi].plantType = type | (density << 4) | (dt << 6);
        config.zone[zi].rzwws = htons(configRzwwsDefault(type));
        /* Set RZWWS changed flag to indicate need for MB range check. */
        configRzwwsChanged = TRUE;
    }
}


/******************************************************************************
 *
 * configFactoryDefaultInit
 *
 * PURPOSE
 *      This routine initializes the configuration data store to the factory
 *      default setting.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine "erases" all of the EEPROM, except for manufacturing
 *      data stored in the first sector.
 *      This routine can take several seconds to complete.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
void configFactoryDefaultInit(void)
{
    uint8_t data[CONFIG_BLOCK_SIZE];
    int i;

    /* Set all memory after manufacturing data to 0xFF. */
    memset(data, 0xFF, sizeof(data));
    for (i = CONFIG_MANUF + CONFIG_BLOCK_SIZE;
         i < CONFIG_EEPROM_SIZE;
         i += CONFIG_BLOCK_SIZE)
    {
        drvEepromWrite(data, i, sizeof(data));
        /* Make sure watchdog doesn't timeout during this long operation. */
        sysExecutionExtend();
    }

    /* Load factory defaults. */
    configDefaultLoad();

    configState = CONFIG_STATE_CLEAN;
}


/******************************************************************************
 *
 * configSnapshotSave
 *
 * PURPOSE
 *      This routine saves a copy of the current configuration to the snapshot
 *      image buffer location in EEPROM.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine writes the entire configuration to EEPROM before it
 *      returns.  This operation can take up to 150 ms to complete.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
void configSnapshotSave(void)
{
    int i;
    uint32_t nBytes;
    uint8_t *pData = (uint8_t *)&config;
    uint16_t oldCheckSum;
    uint16_t newCheckSum;

    /* Save RAM checksum. */
    oldCheckSum = config.sys.checkSum;

    /* Insure checksum in RAM is valid. */
    if (!configMemoryChecksumIsValid(&newCheckSum))
    {
        /* Update the RAM checksum. */
        config.sys.checkSum = htons(newCheckSum);
    }

    i = 0;
    while (i < CONFIG_IMAGE_SIZE)
    {
        nBytes = CONFIG_IMAGE_SIZE - i;
        if (nBytes > CONFIG_BLOCK_SIZE)
        {
            nBytes = CONFIG_BLOCK_SIZE;
        }
        drvEepromWrite(pData + i, CONFIG_IMAGE_SNAPSHOT + i, nBytes);
        i += nBytes;
        /* Make sure watchdog doesn't timeout during this long operation. */
        sysExecutionExtend();
    }

    /* Restore original RAM checksum. */
    config.sys.checkSum = oldCheckSum;
}


/******************************************************************************
 *
 * configSnapshotRead
 *
 * PURPOSE
 *      This routine reads data from the configuration snapshot buffer area
 *      of EEPROM.
 *
 * PARAMETERS
 *      offset  IN  offset in snapshot buffer to start reading
 *      pBuf    OUT pointer to read data destination
 *      len     IN  length of data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void configSnapshotRead(uint32_t offset, void *pBuf, uint32_t len)
{
    drvEepromRead(CONFIG_IMAGE_SNAPSHOT + offset, pBuf, len);
}


/******************************************************************************
 *
 * configBufferLoad
 *
 * PURPOSE
 *      This routine verifies the configuration image contained in the
 *      configuration download buffer and, if the version and checksum are
 *      valid, copies the image to RAM, making it the new configuration.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      This routine returns -1 if the configuration download buffer
 *      contains a valid configuration image, otherwise it returns the
 *      byte-offset of the first invalid field detected.
 *
 * NOTES
 *      The stored configuration data images will be automatically updated
 *      to match the new version in RAM.
 *      The user interface is sent back to its "home" Control & Status screen.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
int16_t configBufferLoad(void)
{
    uint32_t nBytes;                    /* next number of bytes to read */
    uint32_t i;                         /* index */
    int16_t result;                     /* return value, (-1) = valid image*/
    uint8_t *pData = (uint8_t *)&config;
#ifdef RADIO_ZB1
    uint64_t oldRadioPanId = config.sys.radioPanId;
#else
    uint16_t oldRadioPanId = config.sys.radioPanId;
#endif
    uint16_t totalRT[SYS_N_PROGRAMS];  /* array to hold run times */
    uint8_t endZone=0;                 /* zone to calculate runtime up to */
    uint8_t day=0;                     /* day of the week in the run time schedule */
    uint8_t prog =0;                   /* program in the runtime schedule */
    uint8_t deadTime =0;               /* the amount of deadtime added inbetween units
                                        * stopping and the next one starting */
    
    /* Verify configuration data values are within acceptable bounds. */
    result = configImageContentValidate(CONFIG_IMAGE_BUFFER);

    if (result == -1)
    {
        i = 0;
        while (i < CONFIG_IMAGE_SIZE)
        {
            nBytes = CONFIG_IMAGE_SIZE - i;
            if (nBytes > CONFIG_BLOCK_SIZE)
            {
                nBytes = CONFIG_BLOCK_SIZE;
            }
            drvEepromRead(CONFIG_IMAGE_BUFFER + i, pData + i, nBytes);
            i += nBytes;
            /* Make sure watchdog doesn't timeout during long operation. */
            sysExecutionExtend();
        }


        switch(config.sys.unitType)
        {
            case UNIT_TYPE_EXPANSION_1:
                  config.sys.numZones = config.sys.expNumZones1;
                  endZone = 12;
                  deadTime = DEAD_TIME;
                  break;
            case UNIT_TYPE_EXPANSION_2:
                  config.sys.numZones = config.sys.expNumZones2;
                  endZone = 24;
                  deadTime = DEAD_TIME*2;
                  break;
            case UNIT_TYPE_EXPANSION_3:
                  config.sys.numZones = config.sys.expNumZones3;
                  endZone = 36;
                  deadTime = DEAD_TIME*3;
                  break;
        }
        
        /* compute run-time totals for each program */
        for (int p = 0; p < SYS_N_PROGRAMS; p++)
        {
            totalRT[p] = deadTime;
            for (int z = 0; z < endZone; z++)
            {
                totalRT[p] += config.zone[z].runTime[p];
            }
         }
        
        
        /*modify the configuration based on unit type before loading into EEPROM*/
        switch(config.sys.unitType)
        {
            case UNIT_TYPE_EXPANSION_1:
                /*move the zone 13-24 over to 1-12 for expansion 1 */
                memcpy(&config.zone[0], &config.zone[12], (12*sizeof(configZone_t)));
      
                /*modify runtimes to account for the time for zones 1-12 to run*/
                for(day = 0; day<8;day++)
                {
                    for(prog=0; prog <4;prog++)
                    {    
                        if(config.sched[day][prog].startTime != htons(CONFIG_SCHED_START_DISABLED))
                        {
                            config.sched[day][prog].startTime += (totalRT[prog]);
                        }
                    }
                } 
       
                
       
                /*change the zone group number accordingly */
                for(i=0; i<12;i++)
                {
                    if(config.zone[i].group != CONFIG_GROUP_NONE)
                    {
                      config.zone[i].group -=12;
                    }
                }
                
                /* recompute the CRC checksum */
                config.sys.checkSum=configMemorySnapShotChecksumCalc((uint8_t *)&config);
                break;
            case UNIT_TYPE_EXPANSION_2:
                /*move the zone 25-36 over to 1-12 for expansion 1 */
                memcpy(&config.zone[0], &config.zone[24], (12*sizeof(configZone_t)));
      
                /*modify runtimes to account for the time for zones 1-12 to run*/
                for(day = 0; day<8;day++)
                {
                    for(prog=0; prog <4;prog++)
                    {    
                        if(config.sched[day][prog].startTime != htons(CONFIG_SCHED_START_DISABLED))
                        {
                            config.sched[day][prog].startTime += (totalRT[prog]);
                        }
                    }
                } 
       
                /*change the zone group number accordingly */
                for(i=0; i<12;i++)
                {
                    if(config.zone[i].group != CONFIG_GROUP_NONE)
                    {
                      config.zone[i].group -=24;
                    }
                }
                
                /* recompute the CRC checksum */
                config.sys.checkSum=configMemorySnapShotChecksumCalc((uint8_t *)&config);
                break;
            case UNIT_TYPE_EXPANSION_3:
                /*move the zone 37-48 over to 1-12 for expansion 1 */
                memcpy(&config.zone[0], &config.zone[36], (12*sizeof(configZone_t)));
      
                /*modify runtimes to account for the time for zones 1-12 to run*/
                for(day = 0; day<8;day++)
                {
                    for(prog=0; prog <4;prog++)
                    {    
                        if(config.sched[day][prog].startTime != htons(CONFIG_SCHED_START_DISABLED))
                        {
                            config.sched[day][prog].startTime += (totalRT[prog]);
                        }
                    }
                } 
       
                /*change the zone group number accordingly */
                for(i=0; i<12;i++)
                {
                    if(config.zone[i].group != CONFIG_GROUP_NONE)
                    {
                      config.zone[i].group -=36;
                    }
                }
                
                /* recompute the CRC checksum */
                config.sys.checkSum=configMemorySnapShotChecksumCalc((uint8_t *)&config);
                break;
        }
        
        /* Indicate the need to re-sync EEPROM configuration image copies. */
        configImageSyncNeeded();
        /* Update sensor sample frequency for new configuration. */
        irrMoistConfigUpdate();
        /* Update the Radio PAN ID if it has changed. */
        if (config.sys.radioPanId != oldRadioPanId)
        {
            radioPanIdSet();
        }
        /* Insure MB values are not above their respective RZWWS limits. */
        irrMbFix();
        /* Send UI to the main Controller Status screen. */
        uiStatusShow();
        /* Trace configuration download applied event. */
        sysEvent(CONFIG_EVENT_DL_APPLY, 0);
    }
    else
    {
        /* Trace configuration download apply failed event. */
        sysEvent(CONFIG_EVENT_DL_APP_F, result);
    }
    return result;
}


/******************************************************************************
 *
 * configBufferClear
 *
 * PURPOSE
 *      This routine clears the configuration download buffer area of EEPROM.
 *      The area is cleared by writing 0xFF characters throughout all EEPROM
 *      blocks reserved for download buffering (up to the next block boundary).
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine completes all necessary EEPROM writes before it
 *      returns.  This operation can take up to 150 ms to complete.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
void configBufferClear(void)
{
    uint8_t data[CONFIG_BLOCK_SIZE];
    int i;

    /* Set configuration download buffer all 0xFF bytes. */
    memset(data, 0xFF, sizeof(data));
    for (i = CONFIG_IMAGE_BUFFER;
        i < (CONFIG_IMAGE_SIZE + CONFIG_BLOCK_SIZE);
        i += CONFIG_BLOCK_SIZE)
    {
        drvEepromWrite(data, i, sizeof(data));
        /* Make sure watchdog doesn't timeout during this long operation. */
        sysExecutionExtend();
    }
}


/******************************************************************************
 *
 * configBufferWrite
 *
 * PURPOSE
 *      This routine writes data to the configuration download buffer area
 *      of EEPROM.
 *
 * PARAMETERS
 *      pBuf    IN  pointer to write data source
 *      offset  IN  offset in download buffer to start writing
 *      len     IN  length of data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void configBufferWrite(const void *pBuf, uint32_t offset, uint32_t len)
{
    drvEepromWrite(pBuf, CONFIG_IMAGE_BUFFER + offset, len);
}


/******************************************************************************
 *
 * configEventLogSave
 *
 * PURPOSE
 *      This routine saves a copy of the current system event log to the Event
 *      Log snapshot buffer location in EEPROM.
 *
 * PARAMETERS
 *      None.
 *
 * RETURN VALUE
 *      None.
 *
 * NOTES
 *      This routine writes the entire system event log to EEPROM before it
 *      returns.  This operation can take up to 150 ms to complete.
 *      The system is regularly notified during this long process to prevent
 *      watchdog reset and manage any power failures.
 *
 *****************************************************************************/
void configEventLogSave(void)
{
    int i;
    uint32_t nBytes;
    uint8_t *pData = (uint8_t *)&sysEventLog;
    uint32_t curCountMs = drvMSGet();

    /* Save current date/time in event buffer. */
    sysEventLog.mn.year = dtYear - 2000;
    sysEventLog.mn.month = dtMon + 1;
    sysEventLog.mn.day = dtMday;
    sysEventLog.mn.hour = dtHour;
    sysEventLog.mn.min = dtMin;
    sysEventLog.mn.sec = dtSec;
    /* Trace event buffer saved event. */
    sysEvent(CONFIG_EVENT_EVT_SAVED, (uint16_t)(curCountMs - dtTickCountMs));

    i = 0;
    while (i < SYS_EVENT_LOG_SIZE)
    {
        nBytes = SYS_EVENT_LOG_SIZE - i;
        if (nBytes > CONFIG_BLOCK_SIZE)
        {
            nBytes = CONFIG_BLOCK_SIZE;
        }
        drvEepromWrite(pData + i, CONFIG_EVENT_LOG + i, nBytes);
        i += nBytes;
        /* Make sure watchdog doesn't timeout during this long operation. */
        sysExecutionExtend();
    }
}


/******************************************************************************
 *
 * configEventLogRead
 *
 * PURPOSE
 *      This routine reads data from the configuration event log save area
 *      of EEPROM.
 *
 * PARAMETERS
 *      offset  IN  offset in snapshot buffer to start reading
 *      pBuf    OUT pointer to read data destination
 *      len     IN  length of data
 *
 * RETURN VALUE
 *      None.
 *
 *****************************************************************************/
void configEventLogRead(uint32_t offset, void *pBuf, uint32_t len)
{
    drvEepromRead(CONFIG_EVENT_LOG + offset, pBuf, len);
}


/******************************************************************************
 *
 * configRzwwsDefault
 *
 * PURPOSE
 *      This routine is called to get the default RZWWS value for a given
 *      Plant Type.
 *
 * PARAMETERS
 *      plantType   IN  the plant type (or plant species type)
 *
 * RETURN VALUE
 *      This routine returns the default RZWWS value, as defined in the table
 *      default RZWWS values for plant species types (configPlantTypeRzwws).
 *
 *****************************************************************************/
uint16_t configRzwwsDefault(uint8_t plantType)
{
    uint8_t species = CONFIG_PT_SPECIES(plantType);

    /* Check if species in within valid limits. */
    if (species >= CONFIG_PLANTTYPE_LIMIT)
    {
        /* Species is out-of-range; use default species. */
        species = CONFIG_PLANTTYPE_BERMUDA;
    }
    return configPlantTypeRzwws[species];
}
