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
 * Module       : drvLcd.c
 * Description  : This file implements the LCD driver.
 *
 *****************************************************************************/

/* MODULE drvLcd */

#include "global.h"
#include "hwBusData.h"
#include "hwLcdEnb.h"
#include "hwLcdRs.h"
#include "hwLcdRw.h"

#include "drvLcd.h"


/* **** LCD CONTROLLER DEFINITIONS *******************************************/

/* LCD enables (based on GPIO bean definition) */
#define LCD_ENB_1               0x01    /* controller 1 (lines 1 & 2) */
#define LCD_ENB_2               0x02    /* controller 2 (lines 3 & 4) */
#define LCD_ENB_BOTH            0x03    /* both controllers (write only!) */

/* Register addresses (R/S pin) */
#define LCD_REG_INST            0x00    /* instruction register */
#define LCD_REG_DATA            0x01    /* data register (120us read/write) */

/* Instruction codes */
#define LCD_INST_CLEAR          0x01    /* clear display & home cursor (4.9ms)*/
#define LCD_INST_HOME           0x02    /* home cursor & display (4.8ms)     */
#define LCD_INST_ENTRYM         0x04    /* entry mode set (120us)            */
#define LCD_INST_DISP           0x08    /* display/cursor on/off (120us)     */
#define LCD_INST_SHIFT          0x10    /* cursor & display shift (120us)    */
#define LCD_INST_IFCFG          0x20    /* function set (config I/F) (120us) */
#define LCD_INST_CGADDR         0x40    /* move cursor to CG-RAM (120us)     */
#define LCD_INST_DDADDR         0x80    /* move cursor to display RAM (120us)*/

/* Entry Mode Set command parameters (controls cursor/display shift on write) */
#define LCD_INST_ENTRYM_MSK     0x03    /* select one of:                    */
#define LCD_INST_ENTRYM_DEC     0x00    /*   decrement cursor, no shift      */
#define LCD_INST_ENTRYM_SR      0x01    /*   decrement cursor & shift right  */
#define LCD_INST_ENTRYM_INC     0x02    /*   increment cursor, no shift      */
#define LCD_INST_ENTRYM_SL      0x03    /*   increment cursor & shift left   */

/* Display on/off command parameters */
#define LCD_INST_DISP_MSK       0x07    /* set/reset each bit:               */
#define LCD_INST_DISP_D         0x04    /*   enable display                  */
#define LCD_INST_DISP_C         0x02    /*   enable steady underline cursor  */
#define LCD_INST_DISP_B         0x01    /*   enable blinking block cursor    */

/* Cursor and Display Shift command parameters */
#define LCD_INST_SHIFT_MSK      0x0C    /* select one of:                    */
#define LCD_INST_SHIFT_CURL     0x00    /*   shift cursor left (decr addr)   */
#define LCD_INST_SHIFT_CURR     0x04    /*   shift cursor right (incr addr)  */
#define LCD_INST_SHIFT_DSPL     0x08    /*   shift display left              */
#define LCD_INST_SHIFT_DSPR     0x0C    /*   shift display right             */

/* Function Set command parameters (configure interface) */
#define LCD_INST_IFCFG_MSK      0x1C    /* select one from each group:       */
#define LCD_INST_IFCFG_DL       0x10    /*   data length:                    */
#define LCD_INST_IFCFG_DL4      0x00    /*     4-bit                         */
#define LCD_INST_IFCFG_DL8      0x10    /*     8-bit                         */
#define LCD_INST_IFCFG_N        0x08    /*   display lines:                  */
#define LCD_INST_IFCFG_N1       0x00    /*     single line                   */
#define LCD_INST_IFCFG_N2       0x08    /*     multiple lines (5x7 only)     */
#define LCD_INST_IFCFG_F        0x04    /*   character font:                 */
#define LCD_INST_IFCFG_F57      0x00    /*     5x7                           */
#define LCD_INST_IFCFG_F510     0x04    /*     5x10 (single line only)       */

/* Move Cursor to CG-RAM command parameters */
#define LCD_INST_CGADDR_MSK     0x3F    /* CG-RAM address mask               */

/* Move Cursor to Display RAM command parameters */
#define LCD_INST_DDADDR_MSK     0x7F    /* DD-RAM address mask               */

/* Busy flag */
#define LCD_STATUS_BUSY         0x80    /* busy flag                         */


/* custom character font definitions (see LCD data sheet for details) */
static const uint8_t drvLcdCharG[] =
{
    0x00, 0x00, 0x0F, 0x11, 0x11, 0x0F, 0x01, 0x0E
};
static const uint8_t drvLcdCharY[] =
{
    0x00, 0x00, 0x11, 0x11, 0x11, 0x0F, 0x01, 0x0E
};
static const uint8_t drvLcdCharP[] =
{
    0x00, 0x00, 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10
};
static const uint8_t drvLcdCharJ[] =
{
    0x02, 0x00, 0x06, 0x02, 0x02, 0x02, 0x12, 0x0C
};
static const uint8_t drvLcdCharQ[] =
{
    0x00, 0x00, 0x0D, 0x13, 0x13, 0x0D, 0x01, 0x01
};

static void drvLcdRegWrite(uint8_t lcdEnb, uint8_t lcdReg, uint8_t value);
static void drvLcdBusyWait(uint8_t lcdEnb);
static void drvLcdCharLoad(uint8_t index, const uint8_t *pBitmap);


/******************************************************************************
 *
 *  drvLcdRestart
 *
 *  DESCRIPTION:
 *      This driver internal function (re)initializes the LCD module.  It must
 *      be called after power-up and again whenever AC power is restored to
 *      configure the LCD module prior to performing any LCD writes.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the LCD module.
 *
 *      For better or for worse, the hwCpu_Delay100US() routines includes code
 *      to reset the watchdog timer, so long delays here won't cause problems.
 *
 *****************************************************************************/
void drvLcdRestart(void)
{
    /* wait 40ms after power-up */
    hwCpu_Delay100US(40000 / 100);

    /* initialize both of the two LCD controllers */
    drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_INST, LCD_INST_IFCFG |
                                               LCD_INST_IFCFG_DL8 |
                                               LCD_INST_IFCFG_N2 |
                                               LCD_INST_IFCFG_F57);
    hwCpu_Delay100US(2);                /* wait for >= 120us */
    drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_INST, LCD_INST_IFCFG |
                                               LCD_INST_IFCFG_DL8 |
                                               LCD_INST_IFCFG_N2 |
                                               LCD_INST_IFCFG_F57);
    hwCpu_Delay100US(2);                /* wait for >= 120us */
    drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_INST, LCD_INST_DISP |
                                               LCD_INST_DISP_D);
    hwCpu_Delay100US(2);                /* wait for >= 120us */
    drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_INST, LCD_INST_CLEAR);
    hwCpu_Delay100US(49);               /* wait for >= 4.9ms */
    drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_INST, LCD_INST_ENTRYM |
                                               LCD_INST_ENTRYM_INC);
                                        /* should finish in <= 120us */

    /* load custom characters */
    drvLcdCharLoad(0x00, drvLcdCharG);
    drvLcdCharLoad(0x01, drvLcdCharY);
    drvLcdCharLoad(0x02, drvLcdCharP);
    drvLcdCharLoad(0x03, drvLcdCharJ);
    drvLcdCharLoad(0x04, drvLcdCharQ);
}


/******************************************************************************
 *
 *  drvLcdWrite
 *
 *  DESCRIPTION:
 *      This driver API function writes 160 characters to the LCD module and
 *      positions the cursor (if specified).
 *
 *  PARAMETERS:
 *      pData  (in)  - buffer containing 160 characters of LCD data
 *      cursor (in)  - hardware cursor position (0..159); any negative value
 *                     to disable
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the LCD module.
 *
 *****************************************************************************/
void drvLcdWrite(const char *pData, uint8_t cursor)
{
    int i;

    /* disable cursor display */
    drvLcdBusyWait(LCD_ENB_1);
    drvLcdBusyWait(LCD_ENB_2);
    drvLcdRegWrite(LCD_ENB_BOTH,
                   LCD_REG_INST,
                   LCD_INST_DISP | LCD_INST_DISP_D);

    /* move both cursors to home position (via set data RAM address command) */
    drvLcdBusyWait(LCD_ENB_1);
    drvLcdBusyWait(LCD_ENB_2);
    drvLcdRegWrite(LCD_ENB_BOTH,
                   LCD_REG_INST,
                   LCD_INST_DDADDR | 0x00);

    for (i = 0; i < 160; i++)
    {
        uint8_t lcdEnb = (uint8_t)((i < 80) ? LCD_ENB_1 : LCD_ENB_2);
        uint8_t c = (uint8_t)*pData++;

        switch (c)
        {
            case 0:
                c = ' ';
                break;
            case 'g':
                c = 0x00;
                break;
            case 'y':
                c = 0x01;
                break;
            case 'p':
                c = 0x02;
                break;
            case 'j':
                c = 0x03;
                break;
            case 'q':
                c = 0x04;
                break;
        }
        drvLcdBusyWait(lcdEnb);
        drvLcdRegWrite(lcdEnb, LCD_REG_DATA, c);
    }

    /* Position and enable cursor, if requested */
    if (cursor < 160)
    {
        uint8_t lcdEnb = LCD_ENB_1;

        if (cursor >= 80)
        {
            lcdEnb = LCD_ENB_2;
            cursor -= 80;
        }
        if (cursor >= 40)
        {
            /* bias the cursor position - LCD line 2 starts at address 64 */
            cursor += 64 - 40;
        }
        drvLcdBusyWait(lcdEnb);
        drvLcdRegWrite(lcdEnb,
                       LCD_REG_INST,
                       (uint8_t)(LCD_INST_DDADDR | cursor));
        drvLcdBusyWait(lcdEnb);
        drvLcdRegWrite(lcdEnb,
                       LCD_REG_INST,
                       LCD_INST_DISP | LCD_INST_DISP_D | LCD_INST_DISP_C);
    }
}


/******************************************************************************
 *
 *  drvLcdRegWrite
 *
 *  DESCRIPTION:
 *      This driver internal function writes a single byte to an LCD controller
 *      register.
 *
 *  PARAMETERS:
 *      lcdEnb (in) - enable mask for selecting one or both LCD controllers
 *      lcdReg (in) - LCD instruction/data register selection
 *      value  (in) - value to write to register
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the LCD module.
 *
 *****************************************************************************/
static void drvLcdRegWrite(uint8_t lcdEnb, uint8_t lcdReg, uint8_t value)
{
    EnterCritical();                    /* save and disable interrupts */

    hwBusData_PutVal(value);            /* instruction to data bus */
    hwLcdRs_PutVal(lcdReg);             /* select LCD register */
    hwLcdEnb_PutVal(lcdEnb);            /* strobe controller */
    asm                                 /* 6-cycle (~240nS) delay */
    {
        nop
        nop
        nop
        nop
    }
    hwLcdEnb_PutVal(0);

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvLcdBusyWait
 *
 *  DESCRIPTION:
 *      This driver internal function polls the busy status of an LCD
 *      controller and waits until the controller is not busy.
 *
 *  PARAMETERS:
 *      lcdEnb (in) - enable mask for selecting one of the LCD controllers
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the LCD module.
 *
 *      This cannot be used during the initial steps of controller
 *      initialization.
 *
 *      Exactly one LCD controller must be specified; "both" is not valid.
 *
 *****************************************************************************/
static void drvLcdBusyWait(uint8_t lcdEnb)
{
    volatile uint8_t status;

    do
    {
        EnterCritical();                /* save and disable interrupts */

        hwBusData_SetInput();           /* change data bus GPIOs to input */
        hwLcdRw_SetVal();               /* set LCD and xcvr to read */
        hwLcdRs_PutVal(LCD_REG_INST);   /* select instruction register */
        hwLcdEnb_PutVal(lcdEnb);        /* strobe controller */
        asm                             /* 6-cycle (~240nS) delay */
        {
            nop
            nop
            nop
            nop
        }
        status = hwBusData_GetVal();    /* read LCD status */
        hwLcdEnb_PutVal(0);
        hwLcdRw_ClrVal();               /* set LCD and xcvr to back to write */
        hwBusData_SetOutput();          /* change data bus GPIOs to output */

        ExitCritical();                 /* restore interrupts */
    } while ((status & LCD_STATUS_BUSY) != 0);
}


/******************************************************************************
 *
 *  drvLcdCharLoad
 *
 *  DESCRIPTION:
 *      This driver internal function loads the bitmap definition of a custom
 *      character into both LCD controllers.
 *
 *  PARAMETERS:
 *      index   (in) - custom character index (0..7)
 *      pBitmap (in) - pixel map of character, one byte per row
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the LCD module.
 *
 *****************************************************************************/
static void drvLcdCharLoad(uint8_t index, const uint8_t *pBitmap)
{
    int i;

    /* set CG-RAM address */
    drvLcdBusyWait(LCD_ENB_1);
    drvLcdBusyWait(LCD_ENB_2);
    drvLcdRegWrite(LCD_ENB_BOTH,
                   LCD_REG_INST,
                   (uint8_t)(LCD_INST_CGADDR | ((index & 0x07) * 8)));

    /* load bitmap definition of character */
    for (i = 0; i < 8; i++)
    {
        drvLcdBusyWait(LCD_ENB_1);
        drvLcdBusyWait(LCD_ENB_2);
        drvLcdRegWrite(LCD_ENB_BOTH, LCD_REG_DATA, *pBitmap++);
    }
}


/* END drvLcd */
