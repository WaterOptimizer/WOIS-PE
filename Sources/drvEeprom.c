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
 * Module       : drvEeprom.c
 * Description  : This file implements the EEPROM driver, utilizing the I2C
 *                bean.
 *
 *****************************************************************************/

/* MODULE drvEeprom */


#include "global.h"
#include "hwCpu.h"
#include "hwI2c.h"
#include <string.h>

#include "drvEeprom.h"


/*
 * The following macros are used by the drvEepromReset routine to bit-bang
 * an EEPROM bus reset sequence.  They assume that the IIC controller is
 * disabled and that the routine has set the GPIO pin output values to zeros.
 * These routines control the GPIO lines by switching the data direction
 * between driving the pin low and setting it to high-impedence, where the
 * bus pull-up resistors will pull it high.
 */
#define DRV_EEPROM_SCL_HIGH     PTHDD_PTHDD6 = 0    /* set SCL high */
#define DRV_EEPROM_SCL_LOW      PTHDD_PTHDD6 = 1    /* set SCL low */
#define DRV_EEPROM_SDA_HIGH     PTHDD_PTHDD7 = 0    /* set SDA high */
#define DRV_EEPROM_SDA_LOW      PTHDD_PTHDD7 = 1    /* set SDA low */


static void drvEepromBusyWait(void);


/******************************************************************************
 *
 *  drvEepromReset
 *
 *  DESCRIPTION:
 *      This driver internal function performs an I2C software reset operation
 *      to insure that the EEPROM is not driving the I2C bus following a
 *      system reset.
 *
 *      This reset sequence is defined by the EEPROM datasheet as a START,
 *      followed by nine "1" bits, followed by another START (RESTART), and
 *      finally a STOP.  Because the microcontroller's I2C controller will not
 *      drive the I2C bus if it believe that it is in use by another master,
 *      this code uses direct access to the GPIO pins to "bit-bang" this
 *      sequence.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This should only be used following a microcontroller reset, before the
 *      I2C controller is initialized.
 *
 *      This routine disables the I2C bean and leaves it disabled.
 *
 *      This routine disables interrupts to protect its access to the GPIO
 *      control registers, and it runs with interrupts disabled for several
 *      milliseconds.
 *
 *****************************************************************************/
void drvEepromReset(void)
{
    /* ensure that I2C controller is disabled so that GPIO controls the bus */
    hwI2c_Disable();

    EnterCritical();                    /* save and disable interrupts */

    /* init IIC2 clock and data pin direction to input (high impedance) */
    PTHDD_PTHDD6 = 0;
    PTHDD_PTHDD7 = 0;
    /* init IIC2 clock and data pin values low */
    PTHD &= ~(PTHD_PTHD6_MASK | PTHD_PTHD7_MASK);
    /* allow time for signals to stabilize */
    hwCpu_Delay100US(1);

    /* generate START condition (data goes low while clock is high) */
    DRV_EEPROM_SDA_LOW;                 /* SDA low */
    hwCpu_Delay100US(1);
    DRV_EEPROM_SCL_LOW;                 /* SCL low */
    hwCpu_Delay100US(1);

    /* clock out nine "1" bits */
    DRV_EEPROM_SDA_HIGH;                /* SDA high */
    hwCpu_Delay100US(1);
    for (int i = 0; i < 9; i++)
    {
        DRV_EEPROM_SCL_HIGH;            /* SCL high */
        hwCpu_Delay100US(1);
        DRV_EEPROM_SCL_LOW;             /* SCL low */
        hwCpu_Delay100US(1);
    }

    /* generate RESTART condition (data goes low while clock is high) */
    DRV_EEPROM_SCL_HIGH;                /* SCL high */
    hwCpu_Delay100US(1);
    DRV_EEPROM_SDA_LOW;                 /* SDA low */
    hwCpu_Delay100US(1);
    DRV_EEPROM_SCL_LOW;                 /* SCL low */
    hwCpu_Delay100US(1);

    /* generate STOP condition (data goes high while clock is high) */
    DRV_EEPROM_SCL_HIGH;                /* SCL high */
    hwCpu_Delay100US(1);
    DRV_EEPROM_SDA_HIGH;                /* SDA high */
    hwCpu_Delay100US(1);

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvEepromRead
 *
 *  DESCRIPTION:
 *      This driver API function reads data from the EEPROM connected to the
 *      I2C bus.  Any number of bytes can be read.  This routine performs the
 *      read as a single I2C transaction, copying directly into the caller's
 *      buffer.  Note that there are four bytes of I2C overhead for each call,
 *      so reading EEPROM a byte or two at a time is very inefficient.
 *
 *  PARAMETERS:
 *      offset (in)  - EEPROM address to read (0 up to 32K)
 *      pBuf   (out) - Caller's read buffer to receive EEPROM data
 *      nbytes (in)  - Number of bytes to read
 *
 *  RETURNS:
 *      TRUE on success; FALSE on error
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the I2C bus.
 *
 *****************************************************************************/
bool_t drvEepromRead(uint32_t  offset,
                     void     *pBuf,
                     uint32_t  nbytes)
{
    uint8_t buf[2];
    uint16_t xfer;
    int32_t rtn;

    /* perform I2C write to send EEPROM starting address */
    buf[0] = (uint8_t)(offset >> 8);
    buf[1] = (uint8_t)offset;
    rtn = hwI2c_SendBlock(buf, 2, &xfer);
    if (rtn != ERR_OK)
    {
        /* error - stop I2C transaction and return */
        (void)hwI2c_SendStop();
    }
    else
    {
        /* perform I2C read to retrieve data */
        /* (read routine does I2C restart at beginning and I2C stop at end) */
        rtn = hwI2c_RecvBlock(pBuf, (uint16_t)nbytes, &xfer);
    }

    return (rtn == ERR_OK);
}


/******************************************************************************
 *
 *  drvEepromWrite
 *
 *  DESCRIPTION:
 *      This driver API function writes data to the EEPROM connected to the
 *      I2C bus.  Any number of bytes can be written.  This routine splits the
 *      write into multiple operations as necessary to avoid writing across an
 *      EEPROM page boundary.  To improve efficiency, the caller should avoid
 *      making multiple calls to write to consecutive locations within the same
 *      page as this will require doing multiple page programming operations,
 *      each taking up to 5 mS.  It is better to call this routine once to
 *      perform the entire write, or to make multiple calls that are sized and
 *      aligned to EEPROM page boundaries.
 *
 *  PARAMETERS:
 *      pBuf   (in)  - Caller's write buffer containing data to write to EEPROM
 *      offset (in)  - EEPROM address to write (0 up to 32K)
 *      nbytes (in)  - Number of bytes to write
 *
 *  RETURNS:
 *      TRUE on success; FALSE on error
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the I2C bus.
 *
 *****************************************************************************/
bool_t drvEepromWrite(const void *pBuf,
                      uint32_t    offset,
                      uint32_t    nbytes)
{
    const uint8_t *pUserBuf = (const uint8_t *)pBuf;
    uint8_t buf[2 + DRV_EEPROM_PAGE_SIZE];
    int32_t rtn = ERR_OK;

    while (nbytes > 0)
    {
        uint16_t pageBytes;
        uint16_t pageXfer;

        /* compute maximum bytes to program without crossing a page boundary */
        pageBytes = (uint16_t)(DRV_EEPROM_PAGE_SIZE -
                               (offset & (DRV_EEPROM_PAGE_SIZE - 1)));
        if (pageBytes > nbytes)
        {
            pageBytes = (uint16_t)nbytes;
        }

        /* perform I2C write to send EEPROM starting address and data */
        buf[0] = (uint8_t)(offset >> 8);
        buf[1] = (uint8_t)offset;
        memcpy(&buf[2], pUserBuf, pageBytes);
        rtn = hwI2c_SendBlock(buf, (uint16_t)(2 + pageBytes), &pageXfer);
        (void)hwI2c_SendStop();
        if (rtn != ERR_OK)
        {
            /* error - abort further writes */
            break;
        }

        /* wait for EEPROM internal programming operation to complete */
        drvEepromBusyWait();

        /* continue to next page */
        pUserBuf += pageBytes;
        offset += pageBytes;
        nbytes -= pageBytes;
    }

    return (rtn == ERR_OK);
}


/******************************************************************************
 *
 *  drvEepromBusyWait
 *
 *  DESCRIPTION:
 *      This driver internal function polls the EEPROM device, awaiting the
 *      completion of a programming operation.
 *      Ready/busy status is determined by starting an I2C cycle to the EEPROM
 *      and checking to see if the EEPROM acknowledges the first byte.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the I2C bus.
 *
 *****************************************************************************/
static void drvEepromBusyWait(void)
{
    uint8_t buf[1];
    uint16_t xfer;
    uint8_t rtn;

    do
    {
        /* send just the device address and one address byte */
        /* (dev addr only is enough, but I2C bean doesn't do empty writes) */
        buf[0] = 0x00;
        rtn = hwI2c_SendBlock(buf, sizeof(buf), &xfer);
        /* terminate I2C transaction */
        (void)hwI2c_SendStop();
    } while (rtn != ERR_OK || xfer != sizeof(buf));
}


/* END drvEeprom */
