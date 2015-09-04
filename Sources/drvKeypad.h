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
 * Module       : drvKeypad.h
 * Description  : This file declares the interface to the keypad driver.
 *
 *****************************************************************************/

#ifndef __drvKeypad_H
#define __drvKeypad_H

/* MODULE drvKeypad */


/*
 * Application Interface
 */

/*
 * Note: Existing key type values should not be changed because they are
 *       used by the "Experimental Remote Console" radio interface and
 *       it is desirable to maintain this radio protocol compatibility with
 *       previously released WOIS units.
 */

/* Key types */
#define DRV_KEYPAD_TYPE_MASK        0xF0
#define DRV_KEYPAD_TYPE_NONE        0x00
#define DRV_KEYPAD_TYPE_FUNC        0x10
#define DRV_KEYPAD_TYPE_ALTFUNC     0x20
#define DRV_KEYPAD_TYPE_SOFT        0x30
#define DRV_KEYPAD_TYPE_NAV         0x40
#define DRV_KEYPAD_TYPE_SOFT_SLOW   0x50
#define DRV_KEYPAD_TYPE_SOFT_FAST   0x60

/* Key values */
#define DRV_KEYPAD_KEY_MASK         0x0F
#define DRV_KEYPAD_KEY_FUNC(n)      (DRV_KEYPAD_TYPE_FUNC      | (n & 0x0F))
#define DRV_KEYPAD_KEY_ALTFUNC(n)   (DRV_KEYPAD_TYPE_ALTFUNC   | (n & 0x0F))
#define DRV_KEYPAD_KEY_SOFT(n)      (DRV_KEYPAD_TYPE_SOFT      | (n & 0x07))
#define DRV_KEYPAD_KEY_SOFT_SLOW(n) (DRV_KEYPAD_TYPE_SOFT_SLOW | (n & 0x07))
#define DRV_KEYPAD_KEY_SOFT_FAST(n) (DRV_KEYPAD_TYPE_SOFT_FAST | (n & 0x07))
#define DRV_KEYPAD_KEY_NAV_CW       (DRV_KEYPAD_TYPE_NAV       | 0x00)
#define DRV_KEYPAD_KEY_NAV_CCW      (DRV_KEYPAD_TYPE_NAV       | 0x01)

uint8_t drvKeypadGet(void);
void    drvKeypadPut(uint8_t key);


/*
 * Internal Driver Interfaces
 */
void drvKeypadSwitchIsr(void);
void drvKeypadNavIsr(void);


/* END drvKeypad */

#endif
