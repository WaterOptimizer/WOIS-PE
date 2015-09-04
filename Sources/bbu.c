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
 * Module       : bbu.c
 * Description  : This file implements the Board Bring-up Utility ("BBU").
 *                This is a command-line interface used to support the
 *                electrical design verification and firmware/hardware
 *                integration.  It is accessed via the field communications
 *                serial port.
 *
 *****************************************************************************/

/* MODULE bbu */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "global.h"
#include "platform.h"
#include "system.h"
#include "config.h"
#include "crc.h"
#include "drvAlarm.h"
#include "drvEeprom.h"
#include "drvExtFlash.h"
#include "drvKeypad.h"
#include "drvLcd.h"
#include "drvLed.h"
#include "drvMoist.h"
#include "drvSolenoid.h"
#include "drvSys.h"
#include "hwCpu.h"
#include "hwExpOut.h"
#include "hwI2c.h"

#include "bbu.h"


#ifdef ENABLE_BBU


#define DBG_ECHO 1                      /* echo command input */


static int bbuCmdBuzz(void);
static int bbuCmdEepromReset(void);
static int bbuCmdEepromDump(void);
static int bbuCmdEepromFill(void);
static int bbuCmdEepromRead(void);
static int bbuCmdEepromWrite(void);
static int bbuCmdExtFlashDump(void);
static int bbuCmdExtFlashErase(void);
static int bbuCmdExtFlashFill(void);
static int bbuCmdExtFlashRead(void);
static int bbuCmdExtFlashWrite(void);
static int bbuCmdKeypad(void);
static int bbuCmdLcd(void);
static int bbuCmdLedFunction(void);
static int bbuCmdLedStatus(void);
static int bbuCmdManufacturingData(void);
static int bbuCmdManufacturingInteractive(void);
static int bbuCmdMemoryDump(void);
static int bbuCmdMemoryFill(void);
static int bbuCmdMemoryRead(void);
static int bbuCmdMemoryWrite(void);
static int bbuCmdPowerStatus(void);
static int bbuCmdSensorDump(void);
static int bbuCmdSensorPower(void);
static int bbuCmdSensorRead(void);
static int bbuCmdSolenoid(void);
static int bbuCmdStop(void);
static int bbuCmdVersion(void);
static int bbuCmdHelp(void);
static int bbuCmdExit(void);

typedef struct
{
    const char  *pName;                 /* command name */
    int        (*pFunc)(void);          /* function to implement command */
    const char  *pSyntax;               /* help text - command syntax */
    const char  *pHelpShort;            /* help text - short description */
    const char  *pHelpLong;             /* help text - extended description */
} bbuCmdEnt_t;

/* BBU command lookup table */
static const bbuCmdEnt_t bbuCmds[] =
{
    {
        "buzz",
        bbuCmdBuzz,
        "'on'|'off",
        "Control buzzer output.",
        NULL
    },
    {
        "eep*",
        bbuCmdEepromReset,
        "",
        "Perform a software reset of EEPROM device.",
        ""
    },
    {
        "eepd",
        bbuCmdEepromDump,
        "[eeprom_address [num_bytes]]",
        "Dump an EEPROM region.",
        "If the length is not specified, then 64 bytes will be dumped.  If\n"
        "the starting address is also omitted, then it will dump the region\n"
        "following the previous dump.\n"
        "Valid EEPROM addresses range from 0x0000 through 0x7FFF."
    },
    {
        "eepf",
        bbuCmdEepromFill,
        "eeprom_address eeprom_data num_bytes",
        "Fill an EEPROM region.",
        "The fill data is a one byte value.  The size can be up to 1024 bytes.\n"
        "Valid EEPROM addresses range from 0x0000 through 0x7FFF."
    },
    {
        "eepr",
        bbuCmdEepromRead,
        "eeprom_address",
        "Read an EEPROM location.",
        "Valid EEPROM addresses range from 0x0000 through 0x7FFF."
    },
    {
        "eepw",
        bbuCmdEepromWrite,
        "eeprom_address eeprom_data...",
        "Write one or more EEPROM locations.",
        "The write data is one byte values.  At least one must be provided.\n"
        "Multiple write values are written to consecutive locations.\n"
        "Valid EEPROM addresses range from 0x0000 through 0x7FFF."
    },
    {
        "flsd",
        bbuCmdExtFlashDump,
        "[flash_address [num_bytes]]",
        "Dump an SPI flash region.",
        "If the length is not specified, then 64 bytes will be dumped.  If\n"
        "the starting address is also omitted, then it will dump the region\n"
        "following the previous dump.\n"
        "Valid SPI flash addresses range from 0x00000000 through 0x0007FFFF."
    },
    {
        "flse",
        bbuCmdExtFlashErase,
        "flash_address",
        "Erase the SPI sector containing the specified location.",
        "Valid SPI flash addresses range from 0x00000000 through 0x0007FFFF."
    },
    {
        "flsf",
        bbuCmdExtFlashFill,
        "flash_address flash_data num_bytes",
        "Fill an SPI flash region.",
        "The fill data is a one byte value.  The size can be up to 1024 bytes.\n"
        "Valid SPI flash addresses range from 0x00000000 through 0x0007FFFF."
    },
    {
        "flsr",
        bbuCmdExtFlashRead,
        "flash_address",
        "Read an SPI flash location.",
        "Valid SPI flash addresses range from 0x00000000 through 0x0007FFFF."
    },
    {
        "flsw",
        bbuCmdExtFlashWrite,
        "flash_address flash_data...",
        "Write one or more SPI flash locations.",
        "The write data is one byte values.  At least one must be provided.\n"
        "Multiple write values are written to consecutive locations.\n"
        "Valid SPI flash addresses range from 0x00000000 through 0x0007FFFF."
    },
    {
        "keyp",
        bbuCmdKeypad,
        "",
        "Report keypad events.",
        "Whenever a switch is pressed (or auto-repeats), one of the following\n"
        "messages are output:\n"
        "  'FUNC 0' through 'FUNC 15', for function switches\n"
        "  'ALTF 0' through 'ALTF 15', for alternate function switches\n"
        "  'SOFT 0' through 'SOFT 5', for soft keys\n"
        "  'NAV +', for clockwise rotation of the navigation dial\n"
        "  'NAV -', for counter-clockwise rotation of the navigation dial"
    },
    {
        "lcdt",
        bbuCmdLcd,
        "",
        "Display an LCD test pattern.",
        NULL
    },
    {
        "ledf",
        bbuCmdLedFunction,
        "function#",
        "Control function switch LEDs.",
        "The 'function#' parameter selects which function switch LED to turn on.\n"
        "Valid 'function#' values range from 0 through 15."
    },
    {
        "leds",
        bbuCmdLedStatus,
        "status#|'all' 'on'|'off",
        "Control system status LEDs.",
        "The 'status#' parameter selects which system status LED to turn on/off.\n"
        "Valid 'status#' values are 1 (Fault), 2 (Error), and 3 (Debug); values\n"
        "of 'all' or '*' select all three LEDs."
    },
    {
        "memd",
        bbuCmdMemoryDump,
        "[memory_address [num_bytes]]",
        "Dump a CPU memory region.",
        "If the length is not specified, then 64 bytes will be dumped.  If\n"
        "the starting address is also omitted, then it will dump the region\n"
        "following the previous dump.\n"
        "Valid CPU memory addresses range from 0x00000000 through 0xFFFFFFFF."
    },
    {
        "memf",
        bbuCmdMemoryFill,
        "memory_address memory_data num_bytes",
        "Fill a CPU memory region.",
        "The fill data is a one byte value.  The size can be up to 1024 bytes.\n"
        "Valid CPU memory addresses range from 0x00000000 through 0xFFFFFFFF."
    },
    {
        "memr",
        bbuCmdMemoryRead,
        "memory_address",
        "Read a CPU memory location.",
        "Valid CPU memory addresses range from 0x00000000 through 0xFFFFFFFF."
    },
    {
        "memw",
        bbuCmdMemoryWrite,
        "memory_address eeprom_data...",
        "Write one or more CPU memory locations.",
        "The write data is one byte values.  At least one must be provided.\n"
        "Multiple write values are written to consecutive locations.\n"
        "Valid CPU memory addresses range from 0x00000000 through 0xFFFFFFFF."
    },
    {
        "mfgd",
        bbuCmdManufacturingData,
        "[serial_number system_type board_type system_rev board_rev]",
        "Display or set manufacturing data stored in EEPROM.",
        "If no arguments are provided, then the current data is displayed.\n"
        "Otherwise, all of the manafacturing data values must be supplied:\n"
        "  serial_number: one to eight characters\n"
        "  system_type:   numeric value\n"
        "  board_type:    numeric value\n"
        "  system_rev:    letter.number (e.g. \"A.0\", \"A.1\", \"A.12\")\n"
        "  board_rev:     letter.number (e.g. \"A.0\", \"A.1\", \"A.12\")\n"
        "The checksum is computed automatically."
    },
    {
        "mfgi",
        bbuCmdManufacturingInteractive,
        "",
        "Interactively display and modify manufacturing data stored in EEPROM.",
        "The checksum is computed automatically."
    },
    {
        "pwrs",
        bbuCmdPowerStatus,
        "",
        "Display power status (12VDC and 24VAC).",
        ""
    },
    {
        "send",
        bbuCmdSensorDump,
        "",
        "Read (dump) all moisture sensors.",
        "For each of the twelve moisture sensors: apply power, sample the\n"
        "data, and report the value.\n"
        "Values are in the range of 0 through 4091."
    },
    {
        "senp",
        bbuCmdSensorPower,
        "sensor#",
        "Select moisture sensor to power.",
        "The 'sensor#' parameter selects which moister sensor to power.\n"
        "Valid 'sensor#' values are 0 (Off) and 1 through 12 (Zones)."
    },
    {
        "senr",
        bbuCmdSensorRead,
        "sensor#",
        "Read selected moisture sensor.",
        "The 'sensor#' parameter selects which moister sensor to read.\n"
        "Valid 'sensor#' values are 1 through 12."
    },
    {
        "sole",
        bbuCmdSolenoid,
        "solenoid#|'all' 'on'|'off",
        "Control solenoid outputs.",
        "The 'solenoid#' parameter selects which solenoid outputs to turn on/off.\n"
        "Valid 'solenoid#' values are 0 (Master Valve) and 1 through 12 (Zones);\n"
        "values of 'all' or '*' select all thirteen solenoid outputs."
    },
    {
        "stop",
        bbuCmdStop,
        "",
        "Enter STOP3 mode until interrupt.",
        ""
    },
    {
        "vers",
        bbuCmdVersion,
        "",
        "Display WOIS BBU version information.",
        ""
    },
    {
        "?",
        bbuCmdHelp,
        "[command_name]",
        "Display command summary, or specific command help.",
        NULL
    },
    {
        "exit",
        bbuCmdExit,
        "",
        "Exit from BBU.",
        NULL
    },
    {
        NULL,
        NULL,
        NULL,
        NULL,
        NULL
    }
};

static char bbuCmdBuf[82];
static char bbuCmdLast[82];
static char bbuOutBuf[160];             /* also used for LCD test */

static bool bbuExit = FALSE;
static uint8_t bbuSensorZone = 0;


static void  bbuResetReason(void);
static char *bbuGets(void);
static void  bbuPutc(char ch);
static void  bbuPuts(const char *pStr);


void bbu(void)
{
    drvLedDebug(TRUE);

    bbuPuts("\nWOIS Board Bringup Utility\n\n");
    bbuResetReason();

    memset(bbuOutBuf, 0x00, sizeof(bbuOutBuf));
    strcpy(bbuOutBuf, "WOIS Board Bringup Utility");
    drvLcdWrite(bbuOutBuf, -1);

    while (!bbuExit)
    {
        char *pToken;
        const bbuCmdEnt_t *pEnt;

        bbuPuts("\n> ");
        bbuGets();

        if (strcmp(bbuCmdBuf, "") == 0)
        {
            continue;
        }
        else if (strcmp(bbuCmdBuf, ".") == 0)
        {
            /* repeat last command */
            strcpy(bbuCmdBuf, bbuCmdLast);
        }
        else
        {
            /* save command line for later */
            strcpy(bbuCmdLast, bbuCmdBuf);
        }
        pToken = strtok(bbuCmdBuf, " \t");

        /* lookup command in command table */
        for (pEnt = &bbuCmds[0]; pEnt->pName != NULL; pEnt++)
        {
            if (strcmp(pToken, pEnt->pName) == 0)
            {
                break;
            }
        }
        if (pEnt->pName != NULL)
        {
            if (pEnt->pFunc() != 0)
            {
                sprintf(bbuOutBuf,
                        "Usage: %s %s\n",
                        pEnt->pName,
                        pEnt->pSyntax);
                bbuPuts(bbuOutBuf);
            }
        }
        else
        {
            bbuPuts("INVALID COMMAND\n");
        }
    }

    bbuPuts("\nWOIS Board Bringup Utility TERMINATED\n\n");
    return;
}


static int bbuCmdBuzz(void)
{
    char *pToken;
    bool_t value;

    /* get buzzer value ('on' or 'off') */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    if (strcmp(pToken, "on") == 0)
    {
        value = TRUE;
    }
    else if (strcmp(pToken, "off") == 0)
    {
        value = FALSE;
    }
    else
    {
        return 1;
    }

    drvAlarm(value);

    return 0;
}


static int bbuCmdEepromReset(void)
{
    hwI2c_Disable();

    drvEepromReset();

    hwI2c_Enable();

    return 0;
}


static int bbuCmdEepromDump(void)
{
    char *pToken;
    static uint32_t eeAddr = 0;
    static int32_t eeLen = 0;

    /* get optional EEPROM dump starting address */
    pToken = strtok(NULL, " \t");
    if (pToken != NULL)
    {
        eeAddr = strtol(pToken, NULL, 0);

        /* get optional EEPROM dump length */
        pToken = strtok(NULL, " \t");
        if (pToken != NULL)
        {
            eeLen = strtol(pToken, NULL, 0);
        }
        else
        {
            eeLen = 64;
        }
    }

    /* fix length / set default */
    if (eeLen < 1 || eeLen > 1024)
    {
        eeLen = 64;
    }

    /* round address down and length up to 16 byte boundaries */
    eeAddr &= ~0x000F;
    eeLen = (eeLen + 0x000F) & ~0x000F;

    for (int len = 0; len < eeLen; len += 16)
    {
        uint8_t eeData[16];

        if (!drvEepromRead(eeAddr, eeData, 16))
        {
            bbuPuts("EEPROM read error\n");
            return 0;
        }

        sprintf(bbuOutBuf, "0x%04X =", eeAddr);
        bbuPuts(bbuOutBuf);
        for (int i = 0; i < 16; i++)
        {
            sprintf(bbuOutBuf, " %02X", eeData[i]);
            bbuPuts(bbuOutBuf);
        }
        bbuPuts("   ");
        for (int i = 0; i < 16; i++)
        {
            bbuPutc(isprint(eeData[i]) ? eeData[i] : '.');
        }
        bbuPutc('\n');

        eeAddr += 16;
    }

    return 0;
}


static int bbuCmdEepromFill(void)
{
    char *pToken;
    uint32_t eeAddr;
    uint8_t eeData;
    int32_t eeLen;

    /* get EEPROM fill starting address */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeAddr = strtol(pToken, NULL, 0);

    /* get EEPROM fill data */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeData = strtol(pToken, NULL, 0);

    /* get EEPROM fill length */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeLen = strtol(pToken, NULL, 0);

    if (eeLen < 1 || eeLen > 1024)
    {
        eeLen = 1;
    }

    while (eeLen-- > 0)
    {
        if (!drvEepromWrite(&eeData, eeAddr, 1))
        {
            bbuPuts("EEPROM write error\n");
            return 0;
        }
        drvSysWatchDogClear();
        eeAddr++;
    }

    return 0;
}


static int bbuCmdEepromRead(void)
{
    char *pToken;
    uint32_t eeAddr;
    uint8_t eeData;

    /* get EEPROM address to read */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeAddr = strtol(pToken, NULL, 0);

    if (!drvEepromRead(eeAddr, &eeData, 1))
    {
        bbuPuts("EEPROM read error\n");
        return 0;
    }

    sprintf(bbuOutBuf, "EEPROM[0x%04X] = 0x%02X\n", eeAddr, eeData);
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdEepromWrite(void)
{
    char *pToken;
    uint32_t eeAddr;
    uint8_t eeData;

    /* get EEPROM address to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeAddr = strtol(pToken, NULL, 0);

    /* get EEPROM data to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    eeData = strtol(pToken, NULL, 0);

    for (;;)
    {
        if (!drvEepromWrite(&eeData, eeAddr, 1))
        {
            bbuPuts("EEPROM write error\n");
            return 0;
        }
        eeAddr++;

        /* get optional additional EEPROM data to write */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            break;
        }
        eeData = strtol(pToken, NULL, 0);
    }

    return 0;
}


static int bbuCmdExtFlashDump(void)
{
    char *pToken;
    static uint32_t flashAddr = 0;
    static int32_t flashLen = 0;

    /* get optional SPI flash dump starting address */
    pToken = strtok(NULL, " \t");
    if (pToken != NULL)
    {
        flashAddr = strtol(pToken, NULL, 0);

        /* get optional SPI flash dump length */
        pToken = strtok(NULL, " \t");
        if (pToken != NULL)
        {
            flashLen = strtol(pToken, NULL, 0);
        }
        else
        {
            flashLen = 64;
        }
    }

    /* fix length / set default */
    if (flashLen < 1 || flashLen > 1024)
    {
        flashLen = 64;
    }

    /* round address down and length up to 16 byte boundaries */
    flashAddr &= ~0x000F;
    flashLen = (flashLen + 0x000F) & ~0x000F;

    for (int len = 0; len < flashLen; len += 16)
    {
        uint8_t flashData[16];

        drvExtFlashRead(flashAddr, flashData, 16);

        sprintf(bbuOutBuf, "0x%05X =", flashAddr);
        bbuPuts(bbuOutBuf);
        for (int i = 0; i < 16; i++)
        {
            sprintf(bbuOutBuf, " %02X", flashData[i]);
            bbuPuts(bbuOutBuf);
        }
        bbuPuts("  ");
        for (int i = 0; i < 16; i++)
        {
            bbuPutc(isprint(flashData[i]) ? flashData[i] : '.');
        }
        bbuPutc('\n');

        flashAddr += 16;
    }

    return 0;
}


static int bbuCmdExtFlashErase(void)
{
    char *pToken;
    uint32_t flashAddr;

    /* get SPI flash sector address to erase */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashAddr = strtol(pToken, NULL, 0);

    drvExtFlashErase(flashAddr);
    while (drvExtFlashBusy())
    {
        drvSysWatchDogClear();
    }

    sprintf(bbuOutBuf, "SPI flash sector erased: 0x%05X-0x%05X\n",
            flashAddr & ~(DRV_EXTFLASH_SECTOR_SIZE - 1),
            flashAddr | (DRV_EXTFLASH_SECTOR_SIZE - 1));
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdExtFlashFill(void)
{
    char *pToken;
    uint32_t flashAddr;
    uint8_t flashData;
    int32_t flashLen;

    /* get SPI flash fill starting address */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashAddr = strtol(pToken, NULL, 0);

    /* get SPI flash fill data */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashData = strtol(pToken, NULL, 0);

    /* get SPI flash fill length */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashLen = strtol(pToken, NULL, 0);

    if (flashLen < 1 || flashLen > 1024)
    {
        flashLen = 1;
    }

    while (flashLen-- > 0)
    {
        drvExtFlashWrite(&flashData, flashAddr, 1);
        flashAddr++;
    }

    return 0;
}


static int bbuCmdExtFlashRead(void)
{
    char *pToken;
    uint32_t flashAddr;
    uint8_t flashData;

    /* get SPI flash address to read */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashAddr = strtol(pToken, NULL, 0);

    drvExtFlashRead(flashAddr, &flashData, 1);

    sprintf(bbuOutBuf, "SPI flash[0x%05X] = 0x%02X\n", flashAddr, flashData);
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdExtFlashWrite(void)
{
    char *pToken;
    uint32_t flashAddr;
    uint8_t flashData;

    /* get SPI flash address to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashAddr = strtol(pToken, NULL, 0);

    /* get SPI flash data to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    flashData = strtol(pToken, NULL, 0);

    for (;;)
    {
        drvExtFlashWrite(&flashData, flashAddr, 1);
        flashAddr++;

        /* get optional additional SPI flash data to write */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            break;
        }
        flashData = strtol(pToken, NULL, 0);
    }

    return 0;
}


static int bbuCmdKeypad(void)
{
    bool isDone = FALSE;

    /* clear any queued key events */
    while (drvKeypadGet() != DRV_KEYPAD_TYPE_NONE)
    {
        ;
    }
    bbuPuts("(Press any key on connected terminal to exit.)\n");

    do
    {
        uint8_t key = drvKeypadGet();
        hwExpOut_TComData ch;

        switch (key & DRV_KEYPAD_TYPE_MASK)
        {
            case DRV_KEYPAD_TYPE_NONE:
                if (hwExpOut_RecvChar(&ch) == ERR_OK)
                {
                    isDone = TRUE;
                }
                break;
            case DRV_KEYPAD_TYPE_FUNC:
                sprintf(bbuOutBuf, "FUNC %d\n", key & DRV_KEYPAD_KEY_MASK);
                bbuPuts(bbuOutBuf);
                break;
            case DRV_KEYPAD_TYPE_ALTFUNC:
                sprintf(bbuOutBuf, "ALTF %d\n", key & DRV_KEYPAD_KEY_MASK);
                bbuPuts(bbuOutBuf);
                break;
            case DRV_KEYPAD_TYPE_SOFT:
                sprintf(bbuOutBuf, "SOFT %d\n", key & DRV_KEYPAD_KEY_MASK);
                bbuPuts(bbuOutBuf);
                break;
            case DRV_KEYPAD_TYPE_NAV:
                sprintf(bbuOutBuf, "NAV %c\n",
                        (key == DRV_KEYPAD_KEY_NAV_CW)  ? '+' :
                        (key == DRV_KEYPAD_KEY_NAV_CCW) ? '-' :
                                                          '?');
                bbuPuts(bbuOutBuf);
                break;
            default:
                sprintf(bbuOutBuf, "UNKNOWN 0x%02X\n", key);
                bbuPuts(bbuOutBuf);
                break;
        }

        drvSysWatchDogClear();
    } while (!isDone);

    return 0;
}


static int bbuCmdLedFunction(void)
{
    char *pToken;
    int led;
    char *pEnd;

    /* get function switch number (0..15) */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    led = strtol(pToken, &pEnd, 0);
    if (*pEnd != '\0' || led < 0 || led > 15)
    {
        return 1;
    }

    /* set function switc LED */
    drvLedFunc(led);

    return 0;
}


static int bbuCmdLedStatus(void)
{
    char *pToken;
    int led;
    bool_t value;
    char *pEnd;

    /* get LED number (1..3, 'all', or '*') */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    if (strcmp(pToken, "*") == 0 ||
        strcmp(pToken, "all") == 0)
    {
        led = -1;
    }
    else
    {
        led = strtol(pToken, &pEnd, 0);
        if (*pEnd != '\0' || led < 1 || led > 3)
        {
            return 1;
        }
    }

    /* get LED value ('on' or 'off') */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    if (strcmp(pToken, "on") == 0)
    {
        value = TRUE;
    }
    else if (strcmp(pToken, "off") == 0)
    {
        value = FALSE;
    }
    else
    {
        return 1;
    }

    switch (led)
    {
        case -1:
            /* turn all status LEDs on/off */
            drvLedFault(value);
            drvLedError(value);
            drvLedDebug(value);
            break;
        case 1:
            drvLedFault(value);
            break;
        case 2:
            drvLedError(value);
            break;
        case 3:
            drvLedDebug(value);
            break;
    }

    return 0;
}


static int bbuCmdLcd(void)
{
    memset(bbuOutBuf, 0xFF, sizeof(bbuOutBuf));
    drvLcdWrite(bbuOutBuf, -1);

    return 0;
}


static int bbuCmdManufacturingData(void)
{
    char *pToken;
    char *pEnd;
    char major;
    int minor;
    char extra;
    configManuf_t configManuf;

    /* get WOIS serial number */
    pToken = strtok(NULL, " \t");
    if (pToken != NULL)
    {
        /* clear entire manufacturing data area */
        memset(&configManuf, 0x00, sizeof(configManuf));

        /* parse and validate WOIS serial number */
        if (strlen(pToken) > sizeof(configManuf.serialNumber))
        {
            return 1;
        }
        strncpy((char *)configManuf.serialNumber,
                pToken,
                sizeof(configManuf.serialNumber));
        for (int i = 0; i < sizeof(configManuf.serialNumber); i++)
        {
            if (configManuf.serialNumber[i] == '\0')
            {
                configManuf.serialNumber[i] = ' ';
            }
            else if (!isgraph(configManuf.serialNumber[i]))
            {
                return 1;
            }
        }

        /* get system type */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            return 1;
        }
        configManuf.systemType = strtol(pToken, &pEnd, 0);
        if (*pEnd != '\0')
        {
            return 1;
        }

        /* get board type */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            return 1;
        }
        configManuf.boardType = strtol(pToken, &pEnd, 0);
        if (*pEnd != '\0')
        {
            return 1;
        }

        /* get system rev */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            return 1;
        }
        if (sscanf(pToken, "%c.%d%c", &major, &minor, &extra) != 2)
        {
            return 1;
        }
        if (major < 'A' || major > 'Z')
        {
            return 1;
        }
        if (minor < 0 || minor > 255)
        {
            return 1;
        }
        configManuf.systemRevision = ((major - 'A') << 8) | minor;

        /* get board rev */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            return 1;
        }
        if (sscanf(pToken, "%c.%d%c", &major, &minor, &extra) != 2)
        {
            return 1;
        }
        if (major < 'A' || major > 'Z')
        {
            return 1;
        }
        if (minor < 0 || minor > 255)
        {
            return 1;
        }
        configManuf.boardRevision = ((major - 'A') << 8) | minor;

        /* compute checksum / CRC */
        configManuf.checkSum = crc16(&configManuf, sizeof(configManuf) - 2);

        /* write data to EEPROM */
        if (!drvEepromWrite(&configManuf, 0x0000, sizeof(configManuf)))
        {
            bbuPuts("EEPROM write error\n");
            return 0;
        }
    }

    else

    {
        /* no arguments - just display the current data */
        if (!drvEepromRead(0x0000, &configManuf, sizeof(configManuf)))
        {
            bbuPuts("EEPROM read error\n");
            return 0;
        }
        sprintf(bbuOutBuf, "WOIS serial# = '%8.8s'\n",
                configManuf.serialNumber);
        bbuPuts(bbuOutBuf);
        sprintf(bbuOutBuf, "System type  = 0x%08X\n",
                configManuf.systemType);
        bbuPuts(bbuOutBuf);
        sprintf(bbuOutBuf, "Board type   = 0x%08X\n",
                configManuf.boardType);
        bbuPuts(bbuOutBuf);
        sprintf(bbuOutBuf, "System rev   = %c.%d\n",
                (configManuf.systemRevision >> 8) + 'A',
                configManuf.systemRevision & 0x00FF);
        bbuPuts(bbuOutBuf);
        sprintf(bbuOutBuf, "Board rev    = %c.%d\n",
                (configManuf.boardRevision >> 8) + 'A',
                configManuf.boardRevision & 0x00FF);
        bbuPuts(bbuOutBuf);
        sprintf(bbuOutBuf, "Checksum/CRC = 0x%04X (%s)\n",
                configManuf.checkSum,
                (configManuf.checkSum == crc16(&configManuf, sizeof(configManuf) - 2)) ? "OK" : "*INVALID*");
        bbuPuts(bbuOutBuf);
    }

    return 0;
}


static int bbuCmdManufacturingInteractive(void)
{
    char *pEnd;
    char major;
    int minor;
    char extra;
    configManuf_t configManuf;

    /* read existing values */
    if (!drvEepromRead(0x0000, &configManuf, sizeof(configManuf)))
    {
        bbuPuts("EEPROM read error\n");
        return 0;
    }

    /* clear unused portion of the manufacturing data area */
    memset(configManuf.pad, 0x00, sizeof(configManuf.pad));

    for (;;)
    {
        /* display existing WOIS serial number */
        sprintf(bbuOutBuf, "WOIS serial# ['%8.8s'] = ",
                configManuf.serialNumber);
        bbuPuts(bbuOutBuf);

        /* get new WOIS serial number */
        if (bbuGets() == NULL)
        {
            /* interrupted - abort without saving changes */
            return 0;
        }
        if (strlen(bbuCmdBuf) != 0)
        {
            /* parse and validate WOIS serial number */
            if (strlen(bbuCmdBuf) > sizeof(configManuf.serialNumber))
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
            strncpy((char *)configManuf.serialNumber,
                    bbuCmdBuf,
                    sizeof(configManuf.serialNumber));
            for (int i = 0; i < sizeof(configManuf.serialNumber); i++)
            {
                if (configManuf.serialNumber[i] == '\0')
                {
                    configManuf.serialNumber[i] = ' ';
                }
                else if (!isgraph(configManuf.serialNumber[i]))
                {
                    bbuPuts("Invalid entry; re-enter\n");
                    continue;
                }
            }
        }
        break;
    }

    for (;;)
    {
        /* display existing system type */
        sprintf(bbuOutBuf, "System type  [0x%08X] = ",
                configManuf.systemType);
        bbuPuts(bbuOutBuf);

        /* get new system type */
        if (bbuGets() == NULL)
        {
            /* interrupted - abort without saving changes */
            return 0;
        }
        if (strlen(bbuCmdBuf) != 0)
        {
            configManuf.systemType = strtol(bbuCmdBuf, &pEnd, 0);
            if (*pEnd != '\0')
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
        }
        break;
    }

    for (;;)
    {
        /* display existing board type */
        sprintf(bbuOutBuf, "Board type   [0x%08X] = ",
                configManuf.boardType);
        bbuPuts(bbuOutBuf);

        /* get new board type */
        if (bbuGets() == NULL)
        {
            /* interrupted - abort without saving changes */
            return 0;
        }
        if (strlen(bbuCmdBuf) != 0)
        {
            configManuf.boardType = strtol(bbuCmdBuf, &pEnd, 0);
            if (*pEnd != '\0')
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
        }
        break;
    }

    for (;;)
    {
        /* display existing system rev */
        sprintf(bbuOutBuf, "System rev   [%c.%-3d]      = ",
                (configManuf.systemRevision >> 8) + 'A',
                configManuf.systemRevision & 0x00FF);
        bbuPuts(bbuOutBuf);

        /* get new system rev */
        if (bbuGets() == NULL)
        {
            /* interrupted - abort without saving changes */
            return 0;
        }
        if (strlen(bbuCmdBuf) != 0)
        {
            if (sscanf(bbuCmdBuf, "%c.%d%c", &major, &minor, &extra) != 2)
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
            if (major < 'A' || major > 'Z' || minor < 0 || minor > 255)
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
            configManuf.systemRevision = ((major - 'A') << 8) | minor;
        }
        break;
    }

    for (;;)
    {
        /* display existing board rev */
        sprintf(bbuOutBuf, "Board rev    [%c.%-3d]      = ",
                (configManuf.boardRevision >> 8) + 'A',
                configManuf.boardRevision & 0x00FF);
        bbuPuts(bbuOutBuf);

        /* get new board rev */
        if (bbuGets() == NULL)
        {
            /* interrupted - abort without saving changes */
            return 0;
        }
        if (strlen(bbuCmdBuf) != 0)
        {
            if (sscanf(bbuCmdBuf, "%c.%d%c", &major, &minor, &extra) != 2)
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
            if (major < 'A' || major > 'Z' || minor < 0 || minor > 255)
            {
                bbuPuts("Invalid entry; re-enter\n");
                continue;
            }
            configManuf.boardRevision = ((major - 'A') << 8) | minor;
        }
        break;
    }

    /* compute checksum / CRC */
    configManuf.checkSum = crc16(&configManuf, sizeof(configManuf) - 2);
    sprintf(bbuOutBuf, "Checksum/CRC = 0x%04X\n",
            configManuf.checkSum);
    bbuPuts(bbuOutBuf);

    /* write data to EEPROM */
    if (!drvEepromWrite(&configManuf, 0x0000, sizeof(configManuf)))
    {
        bbuPuts("EEPROM write error\n");
        return 0;
    }

    return 0;
}


static int bbuCmdMemoryDump(void)
{
    char *pToken;
    static uint32_t memAddr = 0;
    static int32_t memLen = 0;

    /* get optional CPU memory dump starting address */
    pToken = strtok(NULL, " \t");
    if (pToken != NULL)
    {
        memAddr = strtol(pToken, NULL, 0);

        /* get optional CPU memory dump length */
        pToken = strtok(NULL, " \t");
        if (pToken != NULL)
        {
            memLen = strtol(pToken, NULL, 0);
        }
        else
        {
            memLen = 64;
        }
    }

    /* fix length / set default */
    if (memLen < 1 || memLen > 1024)
    {
        memLen = 64;
    }

    /* round address down and length up to 16 byte boundaries */
    memAddr &= ~0x000F;
    memLen = (memLen + 0x000F) & ~0x000F;

    for (int len = 0; len < memLen; len += 16)
    {
        uint8_t memData[16];

        memcpy(memData, (const void *)memAddr, 16);

        sprintf(bbuOutBuf, "0x%08X =", memAddr);
        bbuPuts(bbuOutBuf);
        for (int i = 0; i < 16; i++)
        {
            sprintf(bbuOutBuf, " %02X", memData[i]);
            bbuPuts(bbuOutBuf);
        }
        bbuPuts("   ");
        for (int i = 0; i < 16; i++)
        {
            bbuPutc(isprint(memData[i]) ? memData[i] : '.');
        }
        bbuPutc('\n');

        memAddr += 16;
    }

    return 0;
}


static int bbuCmdMemoryFill(void)
{
    char *pToken;
    uint32_t memAddr;
    uint8_t memData;
    int32_t memLen;

    /* get CPU memory fill starting address */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memAddr = strtol(pToken, NULL, 0);

    /* get CPU memory fill data */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memData = strtol(pToken, NULL, 0);

    /* get CPU memory fill length */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memLen = strtol(pToken, NULL, 0);

    if (memLen < 1 || memLen > 1024)
    {
        memLen = 1;
    }

    memset((void *)memAddr, memData, memLen);

    return 0;
}


static int bbuCmdMemoryRead(void)
{
    char *pToken;
    uint32_t memAddr;
    uint8_t memData;

    /* get CPU memory address to read */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memAddr = strtol(pToken, NULL, 0);

    memcpy(&memData, (const void *)memAddr, 1);

    sprintf(bbuOutBuf, "CPU[0x%08X] = 0x%02X\n", memAddr, memData);
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdMemoryWrite(void)
{
    char *pToken;
    uint32_t memAddr;
    uint8_t memData;

    /* get CPU memory address to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memAddr = strtol(pToken, NULL, 0);

    /* get CPU memory data to write */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    memData = strtol(pToken, NULL, 0);

    for (;;)
    {
        memcpy((void *)memAddr, &memData, 1);
        memAddr++;

        /* get optional additional CPU memory data to write */
        pToken = strtok(NULL, " \t");
        if (pToken == NULL)
        {
            break;
        }
        memData = strtol(pToken, NULL, 0);
    }

    return 0;
}


static int bbuCmdPowerStatus(void)
{
    sprintf(bbuOutBuf, "12VDC  = %s\n24VAC  = %s\n",
            drvSys12vdcOk() ? "OK" : "FAILED",
            drvSys24vacOk() ? "OK" : "FAILED");
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdSensorDump(void)
{
    for (int zone = 1; zone <= SYS_N_UNIT_ZONES; zone++)
    {
        uint16_t value;

        drvMoistPower(zone);
        hwCpu_Delay100US(20 * 10);
        value = drvMoistRead(zone);
        sprintf(bbuOutBuf, "Sensor %-2d = %3d%% (ADC=%d)\n",
                zone,
                drvMoistAdc2Percent(value),
                value);
        bbuPuts(bbuOutBuf);
    }
    drvMoistPower(0);
    bbuSensorZone = 0;

    return 0;
}


static int bbuCmdSensorPower(void)
{
    char *pToken;
    int sensor;
    char *pEnd;

    /* get sensor number (0, or 1..12) */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    sensor = strtol(pToken, &pEnd, 0);
    if (*pEnd != '\0' || sensor < 0 || sensor > SYS_N_UNIT_ZONES)
    {
        return 1;
    }

    drvMoistPower(sensor);
    bbuSensorZone = sensor;

    return 0;
}


static int bbuCmdSensorRead(void)
{
    char *pToken;
    int sensor;
    char *pEnd;
    uint16_t value;

    /* get sensor number (1..12) */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    sensor = strtol(pToken, &pEnd, 0);
    if (*pEnd != '\0' || sensor < 1 || sensor > SYS_N_UNIT_ZONES)
    {
        return 1;
    }

    if (bbuSensorZone != sensor)
    {
        bbuPuts("Warning: selected sensor is not powered.\n");
    }

    /* read and report sensor value */
    value = drvMoistRead(sensor);
    sprintf(bbuOutBuf, "Sensor %d = %d%% (ADC=%d)\n",
            sensor,
            drvMoistAdc2Percent(value),
            value);
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdSolenoid(void)
{
    char *pToken;
    int solenoid;
    bool_t value;
    char *pEnd;

    /* get solenoid number (0, 1..12, 'all', or '*') */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    if (strcmp(pToken, "*") == 0 ||
        strcmp(pToken, "all") == 0)
    {
        solenoid = -1;
    }
    else
    {
        solenoid = strtol(pToken, &pEnd, 0);
        if (*pEnd != '\0' || solenoid < 0 || solenoid > SYS_N_UNIT_ZONES)
        {
            return 1;
        }
    }

    /* get solenoid value ('on' or 'off') */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        return 1;
    }
    if (strcmp(pToken, "on") == 0)
    {
        value = TRUE;
    }
    else if (strcmp(pToken, "off") == 0)
    {
        value = FALSE;
    }
    else
    {
        return 1;
    }

    if (solenoid < 0)
    {
        /* turn all solenoids on/off */
        for (int zone = 0; zone <= SYS_N_UNIT_ZONES; zone++)
        {
            drvSolenoidSet(zone, value);
        }
    }
    else
    {
        /* turn specific solenoid on/off */
        drvSolenoidSet(solenoid, value);
    }

    return 0;
}


static int bbuCmdStop(void)
{
    drvSysShutdown();
    drvSysStop();
    drvSysRestart();

    return 0;
}


static int bbuCmdVersion(void)
{
    char version[41];

    sysFormatFirmwareVer(version);
    sprintf(bbuOutBuf,
            "WOIS Version %s (%s)\n",
            version,
            sysVersionDate);
    bbuPuts(bbuOutBuf);

    return 0;
}


static int bbuCmdHelp(void)
{
    char *pToken;

    /* get optional command name */
    pToken = strtok(NULL, " \t");
    if (pToken == NULL)
    {
        const bbuCmdEnt_t *pEnt;

        /* print help summary for all commands */
        for (pEnt = &bbuCmds[0]; pEnt->pName != NULL; pEnt++)
        {
            sprintf(bbuOutBuf,
                    "%-8s %s\n",
                    pEnt->pName,
                    pEnt->pHelpShort);
            bbuPuts(bbuOutBuf);
        }
    }
    else
    {
        const bbuCmdEnt_t *pEnt;

        /* lookup command in command table */
        for (pEnt = &bbuCmds[0]; pEnt->pName != NULL; pEnt++)
        {
            if (strcmp(pToken, pEnt->pName) == 0)
            {
                break;
            }
        }
        if (pEnt->pName != NULL)
        {
            sprintf(bbuOutBuf,
                    "Usage: %s %s\n",
                    pEnt->pName,
                    pEnt->pSyntax);
            bbuPuts(bbuOutBuf);
            bbuPuts(pEnt->pHelpShort);
            bbuPutc('\n');
            if (pEnt->pHelpLong != NULL)
            {
                bbuPuts(pEnt->pHelpLong);
                bbuPutc('\n');
            }
        }
        else
        {
            bbuPuts("help: no such command\n");
        }
    }

    return 0;
}


static int bbuCmdExit(void)
{
    bbuExit = TRUE;

    return 0;
}


static void bbuResetReason(void)
{
    uint8_t srs = drvSysResetReason();

    sprintf(bbuOutBuf,
            "The microcontroller was reset due to %s.  (SRS=0x%02X)\n",
            (srs == 0x00) ? "the BDM interface" :
            ((srs & RSTSRC_PIN) != 0) ? "the RESET_N pin" :
            ((srs & RSTSRC_COP) != 0) ? "the WatchDog Timer" :
            ((srs & RSTSRC_ILOP) != 0) ? "an Illegal OpCode" :
            ((srs & RSTSRC_ILAD) != 0) ? "an Illegal Address" :
            ((srs & RSTSRC_LVI) != 0) ? "a Low Voltage Detection" :
            ((srs & RSTSRC_POR) != 0) ? "a Power-On Reset (normal startup)" :
            "an UNKNOWN REASON",
            srs);
    bbuPuts(bbuOutBuf);
}


static char *bbuGets(void)
{
    int cmdLen = 0;

    for (;;)
    {
        char ch;

        while (hwExpOut_RecvChar((hwExpOut_TComData *)&ch) != ERR_OK)
        {
            drvSysWatchDogClear();
        }

        if (ch == '\b')
        {
            if (cmdLen > 0)
            {
                cmdLen--;
                bbuPuts("\b \b");
            }
        }
        else if (ch == '\025')
        {
            while (cmdLen > 0)
            {
                cmdLen--;
                bbuPuts("\b \b");
            }
        }
        else if (ch == '\r')
        {
            bbuCmdBuf[cmdLen] = '\0';
            bbuPuts("\n");
            return bbuCmdBuf;
        }
        else if (ch == '\003')
        {
            bbuCmdBuf[0] = '\0';
            bbuPuts("^C\n");
            return NULL;
        }
        else
        {
            if (cmdLen < (sizeof(bbuCmdBuf) - 2))
            {
                bbuCmdBuf[cmdLen++] = ch;
                bbuPutc(ch);
            }
        }
    }
}


static void bbuPutc(char ch)
{
    if (ch == '\n')
    {
        bbuPutc('\r');
    }
    while (hwExpOut_SendChar(ch) != ERR_OK)
    {
        drvSysWatchDogClear();
    }
}


static void bbuPuts(const char *pStr)
{
    while (*pStr != '\0')
    {
        bbuPutc(*pStr++);
    }
}


#endif  /* ENABLE_BBU */


/* END bbu */
