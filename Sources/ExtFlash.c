/******************************************************************************
 *                       Copyright (c) 2008, King Engineering
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
 * Module       : ExtFlash.c
 * Description  : This file implements the higher level external flash commands
 *
 *****************************************************************************/

/* MODULE ExtFlash */


#include "global.h"
#include "hwSpi.h"
#include "hwSpiSS.h"
#include <string.h>

#include "drvExtFlash.h"
#include "extFlash.h"
#include "system.h"





   

// global variables
uint32_t currentFirmwareAddress;  //holds the external SPI flash address of current firmware
uint32_t newFirmwareAddress;      //holds the external SPI flash address of new firmware



/******************************************************************************
 *
 * extFlashBufferWrite
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
 *      TRUE on success; FALSE on error
 *
 *****************************************************************************/
bool_t extFlashBufferWrite(const void *pBuf, uint32_t offset, uint32_t len)
{
    if(drvExtFlashWrite(pBuf, FW_NEW_CODE_SPI_ADDR+offset, len) == FALSE)
    {
      return FALSE;
    }
    
    return TRUE;
}


/******************************************************************************
 *
 * extFlashBufferRead
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
 *      TRUE on success; FALSE on error
 *
 *****************************************************************************/
bool_t extFlashBufferRead(uint32_t offset, void *pBuf, uint32_t len)
{
    if(drvExtFlashRead(FW_NEW_INFO_SPI_ADDR+offset, pBuf, len)==FALSE){
      return FALSE;
    }
    
    return TRUE;
}

/******************************************************************************
 *
 * extFlashFWErase
 *
 * PURPOSE
 *      This routine erases 2 sectors of the flash for a new FW version
 *
 * PARAMETERS
 *      offset  IN  offset of sector want to start sector erase
 *
 * RETURN VALUE
 *      TRUE on success; FALSE on error
 *
 *****************************************************************************/
bool_t extFlashFWErase(uint32_t offset)
{
    uint8_t i;
    
    for(i=0; i<4; i++)
    {
      while(drvExtFlashBusy())
      {
          sysExecutionExtend();
      }
      if(drvExtFlashErase(offset+ i*EXT_FLASH_SEC_SIZE) == FALSE)
      {
        return FALSE;
      }
    }
    return TRUE;
}

/* end MODULE ExtFlash */