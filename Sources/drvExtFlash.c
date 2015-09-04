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
 * Module       : drvExtFlash.c
 * Description  : This file implements the SPI flash driver, utilizing the SPI
 *                bean.
 *
 *****************************************************************************/

/* MODULE drvExtFlash */


#include "global.h"
#include "hwSpi.h"
#include "hwSpiSS.h"
#include <string.h>

#include "drvExtFlash.h"
#include "radio.h"


/* ST Micro M25P40 flash SPI command codes */
#define DRV_EXTFLASH_CMD_WREN   0x06    /* Write Enable */
#define DRV_EXTFLASH_CMD_WRDI   0x04    /* Write Disable */
#define DRV_EXTFLASH_CMD_RDSR   0x05    /* Read Status Register */
#define DRV_EXTFLASH_CMD_WRSR   0x01    /* Write Status Register */
#define DRV_EXTFLASH_CMD_READ   0x03    /* Read */
#define DRV_EXTFLASH_CMD_PP     0x02    /* Page Program */
#define DRV_EXTFLASH_CMD_SE     0xD8    /* Sector Erase */
#define DRV_EXTFLASH_CMD_BE     0xC7    /* Bulk Erase */


static void    drvExtFlashSpiWrEn(void);
static uint8_t drvExtFlashSpiWrRd(uint8_t value);


/******************************************************************************
 *
 *  drvExtFlashRead
 *
 *  DESCRIPTION:
 *      This driver API function reads data from the SPI flash connected to the
 *      SPI bus.  Any number of bytes can be read.  This routine performs the
 *      read as a single SPI transaction, copying directly into the caller's
 *      buffer.  Note that there are four bytes of SPI overhead for each call,
 *      so reading SPI flash a byte or two at a time is very inefficient.
 *
 *  PARAMETERS:
 *      offset (in)  - SPI flash address to read (0 up to 512K)
 *      pBuf   (out) - Caller's read buffer to receive SPI flash data
 *      nbytes (in)  - Number of bytes to read
 *
 *  RETURNS:
 *      TRUE on success; FALSE on error (e.g. flash busy)
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *      The caller should wait for any previous erase operation to complete
 *      before calling this routine.
 *
 *****************************************************************************/
bool_t drvExtFlashRead(uint32_t  offset,
                       void     *pBuf,
                       uint32_t  nbytes)
{
    uint8_t *pUserBuf = (uint8_t *)pBuf;

    if (drvExtFlashBusy())
    {
        return FALSE;
    }

    /* assert SPI flash chip select */
    EnterCritical();
    hwSpiSS_ClrVal();
    ExitCritical();

    /* perform SPI writes to send READ command and starting address */
    (void)drvExtFlashSpiWrRd(DRV_EXTFLASH_CMD_READ);
    (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 16));
    (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 8));
    (void)drvExtFlashSpiWrRd((uint8_t)(offset));

    /* perform SPI reads */
    while (nbytes-- > 0)
    {
        *pUserBuf++ = drvExtFlashSpiWrRd(0x00);
    }

    /* deassert SPI flash chip select */
    EnterCritical();
    hwSpiSS_SetVal();
    ExitCritical();

    return TRUE;
}


/******************************************************************************
 *
 *  drvExtFlashWrite
 *
 *  DESCRIPTION:
 *      This driver API function writes data to the SPI flash connected to the
 *      SPI bus.  Any number of bytes can be written.  This routine splits the
 *      write into multiple operations as necessary to avoid writing across a
 *      SPI flash page boundary.  To improve efficiency, the caller should
 *      avoid making multiple calls to write to consecutive locations within
 *      the same page as this will require doing multiple page programming
 *      operations, each taking up to 5 ms.  It is better to call this routine
 *      once to perform the entire write, or to make multiple calls that are
 *      sized and aligned to SPI flash page boundaries.
 *
 *      The caller is also responsible for performing sector erase operations
 *      as necessary, as this write routine can only change flash bits from a
 *      "1" state to a "0" state.  An entire sector must be erased to change
 *      bits from "0" to "1".
 *
 *  PARAMETERS:
 *      pBuf   (in)  - Caller's write buffer containing data to write to flash
 *      offset (in)  - SPI flash address to write (0 up to 512K)
 *      nbytes (in)  - Number of bytes to write
 *
 *  RETURNS:
 *      TRUE on success; FALSE on error (e.g. flash busy)
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *      The caller should wait for any previous erase operation to complete
 *      before calling this routine.
 *
 *****************************************************************************/
bool_t drvExtFlashWrite(const void *pBuf,
                        uint32_t    offset,
                        uint32_t    nbytes)
{
    const uint8_t *pUserBuf = (const uint8_t *)pBuf;
    uint32_t numBytes=nbytes;
    uint8_t readBuf[RADIO_MAXSEGMENT];
    uint32_t i;
    uint32_t readOffset = offset;

    if (drvExtFlashBusy())
    {
        return FALSE;
    }

    while (nbytes > 0)
    {
        uint16_t pageBytes;

        /* compute maximum bytes to program without crossing a page boundary */
        pageBytes = (uint16_t)(DRV_EXTFLASH_PAGE_SIZE -
                               (offset & (DRV_EXTFLASH_PAGE_SIZE - 1)));
        if (pageBytes > nbytes)
        {
            pageBytes = (uint16_t)nbytes;
        }

        /* enable flash write commands */
        drvExtFlashSpiWrEn();

        /* assert SPI flash chip select */
        EnterCritical();
        hwSpiSS_ClrVal();
        ExitCritical();

        /* perform SPI writes to send Program command and starting address */
        (void)drvExtFlashSpiWrRd(DRV_EXTFLASH_CMD_PP);
        (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 16));
        (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 8));
        (void)drvExtFlashSpiWrRd((uint8_t)(offset));

        /* perform SPI reads */
        while (pageBytes-- > 0)
        {
            (void)drvExtFlashSpiWrRd(*pUserBuf++);
            offset++;
            nbytes--;
        }

        /* deassert SPI flash chip select */
        EnterCritical();
        hwSpiSS_SetVal();
        ExitCritical();

        /* wait for SPI flash internal programming operation to complete */
        while (drvExtFlashBusy())
        {
            ;
        }
    }
    
    //read back what was just written to confirm it is correct
    drvExtFlashRead(readOffset,readBuf, numBytes);
    pUserBuf = (const uint8_t *)pBuf;
    
    //do comparison 
    for(i=0; i < numBytes; i++) 
    {
        if(readBuf[i] != pUserBuf[i])
        {
          return FALSE;
        }
    }

    return TRUE;
}


/******************************************************************************
 *
 *  drvExtFlashErase
 *
 *  DESCRIPTION:
 *      This driver API function initiates the erasure of a single sector of
 *      the SPI flash connected to the SPI bus.  Sector erasure takes about one
 *      second to complete, during which time the flash cannot be accessed.
 *      The drvExtFlashBusy() routine can be called to test for completion.
 *
 *  PARAMETERS:
 *      offset (in) - any SPI flash address within the sector to be erased
 *
 *  RETURNS:
 *      TRUE on success; FALSE on error
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *****************************************************************************/
bool_t drvExtFlashErase(uint32_t offset)
{
    /* enable flash write commands */
    drvExtFlashSpiWrEn();

    /* assert SPI flash chip select */
    EnterCritical();
    hwSpiSS_ClrVal();
    ExitCritical();

    /* perform SPI writes to send sector erase command and sector address */
    (void)drvExtFlashSpiWrRd(DRV_EXTFLASH_CMD_SE);
    (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 16));
    (void)drvExtFlashSpiWrRd((uint8_t)(offset >> 8));
    (void)drvExtFlashSpiWrRd((uint8_t)(offset));

    /* deassert SPI flash chip select */
    EnterCritical();
    hwSpiSS_SetVal();
    ExitCritical();

    return TRUE;
}


/******************************************************************************
 *
 *  drvExtFlashBusy
 *
 *  DESCRIPTION:
 *      This driver API function returns the busy status of the SPI flash.
 *      This is used by the application code to determine if an erase command
 *      has completed.  It is also used by this driver to await the completion
 *      of programming a page.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      TRUE if busy; else FALSE (ready)
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *****************************************************************************/
bool_t drvExtFlashBusy(void)
{
    uint8_t status;

    /* assert SPI flash chip select */
    EnterCritical();
    hwSpiSS_ClrVal();
    ExitCritical();

    /* perform SPI write to send RDSR command */
    (void)drvExtFlashSpiWrRd(DRV_EXTFLASH_CMD_RDSR);

    /* read SPI flash status register */
    status = drvExtFlashSpiWrRd(0x00);

    /* deassert SPI flash chip select */
    EnterCritical();
    hwSpiSS_SetVal();
    ExitCritical();

    return ((status & 0x01) != 0);
}


/******************************************************************************
 *
 *  drvExtFlashSpiWrEn
 *
 *  DESCRIPTION:
 *      This driver internal function sends the SPI flash write-enable command
 *      to enable programming and erasure operations.  The flash reverts to
 *      read-only mode once any programming or erasure command is performed.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *****************************************************************************/
static void drvExtFlashSpiWrEn(void)
{
    /* assert SPI flash chip select */
    EnterCritical();
    hwSpiSS_ClrVal();
    ExitCritical();

    /* perform SPI write to send WREN command */
    (void)drvExtFlashSpiWrRd(DRV_EXTFLASH_CMD_WREN);

    /* deassert SPI flash chip select */
    EnterCritical();
    hwSpiSS_SetVal();
    ExitCritical();
}


/******************************************************************************
 *
 *  drvExtFlashSpiWrRd
 *
 *  DESCRIPTION:
 *      This driver internal function writes a byte to the SPI flash device and
 *      reads the byte received from the SPI flash.  The input byte is received
 *      while the output byte is being sent.  It is therefore a response to the
 *      previous byte sent, or it has no meaning.  Refer to the SPI flash data
 *      sheet for the data transfer sequence for each SPI flash command.
 *
 *  PARAMETERS:
 *      value (in) - byte to sent to the SPI flash device
 *
 *  RETURNS:
 *      byte received from the SPI flash device
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, due to its
 *      use of the SPI bus.
 *
 *      It is necessary to wait for reception of the receive byte, even if it
 *      has no value, because the completion of its reception is the only
 *      indication of the completion of the transmission and the calling code
 *      cannot deassert the flash chip select signal until after all data has
 *      been sent to the device.
 *
 *****************************************************************************/
static uint8_t drvExtFlashSpiWrRd(uint8_t value)
{
    uint8_t ch;

    /* clear any residual read data */
    (void)hwSpi_RecvChar(&ch);

    /* write a byte */
    while (hwSpi_SendChar(value) != ERR_OK)
    {
        ;
    }

    /* read the byte received while writing the byte */
    while (hwSpi_RecvChar(&ch) != ERR_OK)
    {
        ;
    }

    return ch;
}


/* END drvExtFlash */
