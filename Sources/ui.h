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
 * Module       : ui.h
 * Description  : This file defines the user interface logic public interfaces.
 *
 *****************************************************************************/
#ifndef __ui_H
#define __ui_H

/* MODULE ui */

#include "global.h"



/******************************************************************************
 *
 *  UI GLOBAL VARIABLES
 *
 *****************************************************************************/

extern char uiLcdBuf[160];              /* LCD screen buffer */
extern uint8_t uiLcdCursor;             /* current hardware cursor location */
extern uint8_t uiPosition;              /* current function switch position */
extern uint8_t navEndZone;               /* control variable to set how many zones can navigate through */
extern bool_t TestSkipFlag;

/******************************************************************************
 *
 *  UI FUNCTION PROTOTYPES
 *
 *****************************************************************************/

void uiInit(void);
void uiPoll(void);
void uiStatusShow(void);
void uiLcdRefresh(void);
char *uiFormatSystemMode(char *buf);

/* END ui */

#endif
