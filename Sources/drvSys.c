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
 * Module       : drvSys.c
 * Description  : This file implements the driver for power management,
 *                including support for mains power fail detection, backup
 *                battery monitoring, and entering/exiting low-power mode.
 *
 *****************************************************************************/

/* MODULE drvSys */

#include "global.h"
#include "platform.h"
#include "bbu.h"
#include "drv.h"
#include "drvCts.h"
#include "drvEeprom.h"
#include "drvKeypad.h"
#include "drvLcd.h"
#include "drvMoist.h"
#include "drvRadio.h"
#include "hwAdc.h"
#include "hwBusData.h"
#include "hwBusKeypad.h"
#include "hwBusLatch1.h"
#include "hwBusLatch2.h"
#include "hwBusLatch3.h"
#include "hwBusLatchReset.h"
#include "hwCpu.h"
#include "hwExpIn.h"
#include "hwExpInRts.h"
#include "hwExpOut.h"
#include "hwExpOutRts.h"
#include "hwI2c.h"
#include "hwLcdEnb.h"
#include "hwLcdRs.h"
#include "hwLcdRw.h"
#include "hwLed.h"
#include "hwPower12vdcStatus.h"
#include "hwPower24vacControl.h"
#include "hwPower24vacStatus.h"
#include "hwRadioDtr.h"
#include "hwRadioReset.h"
#include "hwRadioRts.h"
#include "hwSpi.h"
#include "hwSpiSS.h"
#include "hwTimerKeypad.h"
#include "hwTimerNav.h"
#include "hwUart1Select.h"
#include "hwWatchDog.h"
#include "system.h"

#include "drvSys.h"


static uint8_t drvSysSaveLed = 0x00;    /* saved state of status LEDs */


/******************************************************************************
 *
 *  drvSys12vdcOk
 *
 *  DESCRIPTION:
 *      This driver API function returns the state of the 12VDC power supply.
 *      This is the supply that powers the digital logic (and moisture
 *      sensors), so a failure of this power source indicates a current or
 *      impending loss of power to all of the digital logic except for the
 *      microcontroller.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      TRUE = power OK, FALSE = power failure
 *
 *  NOTES:
 *      This can be invoked from any context.
 *      This can (and must) be used while running on battery power.
 *
 *****************************************************************************/
bool_t drvSys12vdcOk(void)
{
    return (hwPower12vdcStatus_GetVal() == 0);
}


/******************************************************************************
 *
 *  drvSys24vacOk
 *
 *  DESCRIPTION:
 *      This driver API function returns the state of the 24VAC power supply.
 *      This is the supply that powers the solenoids.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      TRUE = power OK, FALSE = power failure
 *
 *  NOTES:
 *      This can be invoked from any context.
 *      This can NOT be used while running on battery power.
 *
 *****************************************************************************/
bool_t drvSys24vacOk(void)
{
    return (hwPower24vacStatus_GetVal() != 0);
}


/******************************************************************************
 *
 *  drvSysStart
 *
 *  DESCRIPTION:
 *      This driver utility function waits for mains power to be stable,
 *      performs the one-time initialization and driver restart functions to
 *      enable all of the peripheral controllers, and then transfers control to
 *      the WOIS application.
 *
 *      This routine is called only at system power-up; only drvSysRestart()
 *      should be called when the system switches from battery to mains power.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This is invoked by the Processor Expert startup code in WOIS.c
 *      following a reset event.
 *
 *****************************************************************************/
void drvSysStart(void)
{
    /* wait for mains power to be present and stable for 100 ms */
 /*   do
    {
        while (!drvSysMainsOk())
        {
            drvSysWatchDogClear();
        }
        hwCpu_Delay100US(1000);
    } while (!drvSysMainsOk());
 */
    /* perform one-time initialization */
    drvCtsInit();
    drvEepromReset();

    /* perform the rest of the initialization */
    drvSysRestart();

    /* insure that mains power is still present */
    while (!drvSysMainsOk())
    {
        drvSysShutdown();
        while (!drvSysMainsOk())
        {
            drvSysStop();               /* (Stop3 operation clears watchdog) */
        }
        drvSysRestart();
    }

    hwCpu_Delay100US(100);              /* give keypad ISR a chance to scan */
#ifdef ENABLE_BBU
    if (drvKeypadGet() == DRV_KEYPAD_KEY_SOFT(5))
    {
        /*
         * The rightmost soft-key (#5) was pressed at startup.
         * Run the Board Bringup Utility to support system test.
         */
        bbu();
        /*
         * BBU generally does not return control.  If it does, we will go ahead
         * and start the Irrigation application, but proper system operation
         * could be impacted by actions performed via the BBU.
         */
    }
#endif

    /* transfer control to Irrigation application */
    sysInit();                          /* DOES NOT RETURN */
}


/******************************************************************************
 *
 *  drvSysRestart
 *
 *  DESCRIPTION:
 *      This driver API function exits Low-Power run mode, sets the clock for
 *      full-speed operation, and restarts all of the peripheral components.
 *      (Restart includes reinitializing external components such as the LCD
 *      controllers.)  This routine is called after main power has been
 *      restored.  It is also called during system initialization.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, while the
 *      system is running in low-power mode (or as part of system init).
 *
 *****************************************************************************/
void drvSysRestart(void)
{
    EnterCritical();                    /* save and disable interrupts */

    /* switch to normal run mode with full-speed clock */
    hwCpu_SetHighSpeed();

    /* first, (re)enable all GPIO outputs by by restoring their proper values */
    hwBusKeypad_PutVal(0x07);           /* keypad row selects to idle state */
    hwExpInRts_SetVal();                /* TODO: revisit when EXP units added */
    hwLed_PutVal(drvSysSaveLed);
    hwSpiSS_SetVal();
    hwUart1Select_SetVal();

    /* reset and then unreset the latches */
    hwBusLatchReset_ClrVal();
    hwCpu_Delay100US(1);
    hwBusLatchReset_SetVal();
    /* restore previous content of output latches */
    hwBusData_PutVal(drvLatch1);
    hwBusLatch1_SetVal();
    hwBusLatch1_ClrVal();
    hwBusData_PutVal(drvLatch2);
    hwBusLatch2_SetVal();
    hwBusLatch2_ClrVal();
    hwBusData_PutVal(drvLatch3);
    hwBusLatch3_SetVal();
    hwBusLatch3_ClrVal();
    /*
     * restore 24VAC power control setting, based on solenoid states
     * (Power enabled if and only if one or more solenoids are energized.  This
     * is determined by checking the saved state of the control latches.
     * Latch 1 controls zones 1..8, latch 2 controls zones 9..12 (4 LSBs),
     * and latch 3 controls the master valve (LSB).)
     */
    hwPower24vacControl_PutVal(((drvLatch1 != 0)          ||
                                ((drvLatch2 & 0x0F) != 0) ||
                                ((drvLatch3 & 0x01) != 0)   ) ? 1 : 0);

    /* now we can (re)initialize the LCD controllers */
    drvLcdRestart();

    /* next, (re)enable the peripheral controllers */
    hwAdc_Enable();
    hwI2c_Enable();
    hwSpi_Enable();
    hwExpIn_Enable();
    hwExpOut_Enable();
    drvMoistRestart();

    /* now we can (re)initialize the radio module */
    drvRadioRestart();

    /* finally, (re)enable the timers */
    hwTimerKeypad_Enable();
    hwTimerNav_Enable();

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvSysShutdown
 *
 *  DESCRIPTION:
 *      This driver API function shuts down all of the peripheral components,
 *      reduces the clock rate to its slowest mode by disabling the FLL, and
 *      enables Low-Power run mode.  This is called in preparation for running
 *      on battery power.  While running with most of the peripheral components
 *      shutdown, the application code must be very carefull not to call driver
 *      functions that would attempt to use any disabled components.
 *
 *      The drvSysRestart routine returns the system to normal operation.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level, while the
 *      system is running in normal mode.
 *
 *****************************************************************************/
void drvSysShutdown(void)
{
    EnterCritical();                    /* save and disable interrupts */

    /* first, disable the timers, as their ISRs use many of the GPIO's */
    hwTimerKeypad_Disable();
    hwTimerNav_Disable();

    /* second, disable the SCIs, as their ISRs also use some of the GPIO's */

    setReg8Bit(PTCDD,PTCDD7);           /* configure TxD2 GPIO to drive low */
    clrReg8Bit(PTCD,PTCD7);
    hwExpIn_Disable();

    setReg8Bit(PTBDD,PTBDD1);           /* configure TxD1 GPIO to drive low */
    clrReg8Bit(PTBD,PTBD1);
    hwExpOut_Disable();

    /* next, disable the other peripheral controllers */

    drvMoistShutdown();                 /* call before ADC & latches disabled */
    hwAdc_Disable();

    setReg8Bit(PTHDD,PTHDD6);           /* configure SCL2 GPIO to drive low */
    clrReg8Bit(PTHD,PTHD6);
    setReg8Bit(PTHDD,PTHDD7);           /* configure SDA2 GPIO to drive low */
    clrReg8Bit(PTHD,PTHD7);
    hwI2c_Disable();

    setReg8Bit(PTBDD,PTBDD2);           /* configure SPSCK1 GPIO to drive low */
    clrReg8Bit(PTBD,PTBD2);
    setReg8Bit(PTBDD,PTBDD3);           /* configure MOSI1 GPIO to drive low */
    clrReg8Bit(PTBD,PTBD3);
    hwSpi_Disable();

    /* reset the latches and hold them in reset */
    hwBusLatchReset_ClrVal();

    /* finally, "disable" all GPIO outputs by driving them low */
    hwBusData_PutVal(0x00);
    hwBusData_SetOutput();
    hwBusKeypad_PutVal(0x00);
    hwExpInRts_ClrVal();                /* TODO: revisit when EXP units added */
    hwExpOutRts_ClrVal();               /* TODO: revisit when EXP units added */
    hwLcdRs_PutVal(0x00);
    hwLcdRw_ClrVal();
    drvSysSaveLed = hwLed_GetVal();
    hwLed_PutVal(0x00);
    hwPower24vacControl_PutVal(0);      /* disable 24VAC power */
    hwRadioDtr_ClrVal();
    hwRadioReset_ClrVal();
    hwRadioRts_ClrVal();
    hwSpiSS_ClrVal();
    hwUart1Select_ClrVal();

    /* switch to Low-Power run mode with 16.384KHz clock */
    hwCpu_SetSlowSpeed();

    ExitCritical();                     /* restore interrupts */
}


/******************************************************************************
 *
 *  drvSysStop
 *
 *  DESCRIPTION:
 *      This driver API function performs a processor STOP instruction.  This
 *      suspends instruction execution until an interrupt occurs, at which
 *      point execution resumes and this routine returns to its caller.
 *
 *      If the processor is in Low-Power run mode when the STOP is executed,
 *      then it enters Stop3 mode, where most peripheral components are also
 *      stopped.  Otherwise, it enters Stop4 mode, where instruction execution
 *      stops but all of the peripheral components continue to run.
 *
 *      When the application detects a mains power failure, it should first
 *      call drvSysShutdown(), then call this routine repeatedly until mains
 *      power returns, and then call drvSysRestart() before resuming normal
 *      operations.  Note that when used in this context, the CPU will be
 *      running in Low-Power run mode at a slow clock rate with most peripheral
 *      controllers disabled when this routine returns control.  In this case,
 *      the application should do as little processing as is necessary, and
 *      then call this routine again if mains power has not been restored.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can only be invoked from task (non-interrupt) level.
 *
 *****************************************************************************/
void drvSysStop(void)
{
    /* enter Stop3/Stop4 mode, until an interrupt occurs */
    /* (Stop3 if in LPrun mode or Stop4 if in normal run mode) */
    hwCpu_SetStopMode();
}


/******************************************************************************
 *
 *  drvSysWatchDogClear
 *
 *  DESCRIPTION:
 *      This driver API function resets the watchdog timer.  This must be
 *      called every 250ms (or less) to prevent the watchdog timer from
 *      performing a system reset.  However, it should be called from as few
 *      places in the application code as possible, and only from places that
 *      indicate that the system is performing all of its normal functions.
 *      Application code should refrain from calling this from within "wait
 *      loops" that could possible run forever.
 *
 *      Note that the watchdog timer is suspended while the CPU is in a Stop
 *      mode, and it is reset upon entry to the Stop mode.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      none
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvSysWatchDogClear(void)
{
    hwWatchDog_Clear();
}


/******************************************************************************
 *
 *  drvSysResetReason
 *
 *  DESCRIPTION:
 *      This driver API function returns the value from the System Reset Status
 *      (SRS) register.  This value indicates the cause of the last CPU reset.
 *      Refer to Section 5.7.2 of the MCF51QE128 Reference Manual for the
 *      definition of this register.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      value of SRS register
 *      (This value never changes, except at reset, so this routine can be
 *      called multiple times and will return the same value.)
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
uint8_t drvSysResetReason(void)
{
    return hwCpu_GetResetSource();
}

/******************************************************************************
 *
 *  drvProcessorReboot
 *
 *  DESCRIPTION:
 *      This driver API function runs in an infinite loop to make the watchdog
 *      timer time out and reboot the processor. This is mainly done to get into 
 *      the bootloader to update the firmware.
 *
 *  PARAMETERS:
 *      none
 *
 *  RETURNS:
 *      nothing
 *
 *  NOTES:
 *      This can be invoked from any context.
 *
 *****************************************************************************/
void drvProcessorReboot(void)
{
    asm(jmp 0xffffffff);    //Jump to illegal address will result a RESET!!!
}


/* END drvSys */
