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
 * Module       : drvRadio.h
 * Description  : This file declares the interface to the radio serial port
 *                driver.
 *
 *****************************************************************************/

#ifndef __drvRadio_H
#define __drvRadio_H

/* MODULE drvRadio */


//buffers
uint8_t  drvRadioRxBuf[];
uint8_t  drvRadioTxBuf[];

/*
 * Application Interface
 */

void    drvRadioReset(bool_t state);
void    drvRadioDtr(bool_t state);
bool_t  drvRadioCts(void);

int16_t drvRadioRead(void *pBuf, uint16_t length);
bool_t  drvRadioWrite(const void *pBuf, uint16_t length);
void    drvRadioReadFlush(void);
void    drvRadioWriteFlush(void);
void drvRadioTxEnqueue(uint8_t byte);
void drvRadioTxStart(void);
bool_t drvRadioRxPeek(uint8_t *pByte, uint16_t offset);
bool_t drvRadioAPIModeInit(void);

/* Statistics */
extern uint32_t drvRadioStatRxBytes;
extern uint32_t drvRadioStatRxFrames;
extern uint32_t drvRadioStatRxBufOverflow;
extern uint32_t drvRadioStatRxBadChecksums;
extern uint32_t drvRadioStatTxBytes;
extern uint32_t drvRadioStatTxFrames;
extern uint32_t drvRadioStatTxBufOverflow;


/*
 * Internal Driver Interfaces
 */
void drvRadioRestart(void);
void drvRadioOnRxChar(void);
void drvRadioOnTxChar(void);
void drvRadioOnCtsAsserted(void);


/* END drvRadio */

#endif
