/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwRtc.H
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : TimerInt
**     Version   : Bean 02.154, Driver 01.14, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 5/14/2010, 1:20 PM
**     Abstract  :
**         This bean "TimerInt" implements a periodic interrupt.
**         When the bean and its events are enabled, the "OnInterrupt"
**         event is called periodically with the period that you specify.
**         TimerInt supports also changing the period in runtime.
**         The source of periodic interrupt can be timer compare or reload
**         register or timer-overflow interrupt (of free running counter).
**     Settings  :
**         Timer name                  : RTC (8-bit)
**         Compare name                : RTCmod
**         Counter shared              : No
**
**         High speed mode
**             Prescaler               : divide-by-1
**           Initial period/frequency
**             Xtal ticks              : 32768
**             microseconds            : 1000000
**             milliseconds            : 1000
**             seconds                 : 1
**             seconds (real)          : 1.0000000
**             Hz                      : 1
**
**         Slow speed mode
**             Prescaler               : divide-by-1
**           Initial period/frequency
**             Xtal ticks              : 32768
**             microseconds            : 1000000
**             milliseconds            : 1000
**             seconds                 : 1
**             seconds (real)          : 1.0000000
**             Hz                      : 1
**
**         Runtime setting             : modes (list of settings)
**            -----------------------------------------------------------------------
**            |  period  |     Prescaler      |        Real period [seconds]        |
**     --------    or    |-----------------------------------------------------------
**     | mode | freqency | high | low  | slow |    high    |    low     |    slow   |
**     ------------------------------------------------------------------------------
**     |  0   |  1Hz     | 1    | -    | 1    | 1.0000000  | -          | 1.0000000 |
**     |  1   |  32Hz    | 1    | -    | 1    | 0.0312500  | -          | 0.0312500 |
**     ------------------------------------------------------------------------------
**
**         Initialization:
**              Timer                  : Enabled
**              Events                 : Enabled
**
**         Timer registers
**              Counter                : RTCCNT    [0xFFFF9831]
**              Mode                   : RTCSC     [0xFFFF9830]
**              Run                    : RTCSC     [0xFFFF9830]
**              Prescaler              : RTCSC     [0xFFFF9830]
**
**         Compare registers
**              Compare                : RTCMOD    [0xFFFF9832]
**
**         Flip-flop registers
**     Contents  :
**         SetPeriodMode - byte hwRtc_SetPeriodMode(byte Mode);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef __hwRtc
#define __hwRtc

/* MODULE hwRtc. */

/*Include shared modules, which are used for whole project*/
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"

/* PUBLISHED CONSTANTS for period mode selection */
#define hwRtc_PM_1Hz         0         /* Constant for switch to mode 0 */
#define hwRtc_Pm_1Hz hwRtc_PM_1Hz      /* Deprecated */
#define hwRtc_PM_32Hz        1         /* Constant for switch to mode 1 */
#define hwRtc_Pm_32Hz hwRtc_PM_32Hz    /* Deprecated */


byte hwRtc_SetPeriodMode(byte Mode);
/*
** ===================================================================
**     Method      :  hwRtc_SetPeriodMode (bean TimerInt)
**
**     Description :
**         Switches the bean to a specified mode (changes the period/frequency
**         using the mode values). This method reduces the time needed
**         for setting of a new period value. This method can be used only when
**         a list of possible period settings is specified at design time.
**         Each of these settings constitutes a mode and Processor Expert
**         assigns them a mode identifier. The prescaler and compare values
**         corresponding to each mode are calculated at design time.
**         Modes can be switched at runtime just by referring to a mode
**         identifier. No run-time calculations are performed, all the
**         calculations are performed at design time. The modes and mode
**         identifiers may be found in the include file *.h.
**     Parameters  :
**         NAME            - DESCRIPTION
**         Mode       - Mode to switch to (0 to 1)
**                         0.   1Hz
**                         1.   32Hz
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
**                           ERR_VALUE - Value out of range,
**                           requested timing mode is not defined
** ===================================================================
*/

__interrupt void hwRtc_Interrupt(void);
/*
** ===================================================================
**     Method      :  hwRtc_Interrupt (bean TimerInt)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes the beans event(s).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwRtc_Init(void);
/*
** ===================================================================
**     Method      :  hwRtc_Init (bean TimerInt)
**
**     Description :
**         Initializes the associated peripheral(s) and the beans 
**         internal variables. The method is called automatically as a 
**         part of the application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwRtc_SetHigh(void);
/*
** ===================================================================
**     Method      :  hwRtc_SetHigh (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the High speed mode. The method is 
**         called automatically as s part of the CPU SetHighSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwRtc_SetSlow(void);
/*
** ===================================================================
**     Method      :  hwRtc_SetSlow (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the Slow speed mode. The method is 
**         called automatically as a part of the CPU SetSlowSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/


#define hwRtc_SetCV(_Val) \
  RTCMOD = (byte)(_Val)
/*
** ===================================================================
**     Method      :  SetCV (bean TimerInt)
**
**     Description :
**         The method computes and sets compare eventually modulo value 
**         for time measuring.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

/* END hwRtc. */

#endif /* ifndef __hwRtc */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
