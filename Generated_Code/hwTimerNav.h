/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwTimerNav.H
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : TimerInt
**     Version   : Bean 02.154, Driver 01.14, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
**     Abstract  :
**         This bean "TimerInt" implements a periodic interrupt.
**         When the bean and its events are enabled, the "OnInterrupt"
**         event is called periodically with the period that you specify.
**         TimerInt supports also changing the period in runtime.
**         The source of periodic interrupt can be timer compare or reload
**         register or timer-overflow interrupt (of free running counter).
**     Settings  :
**         Timer name                  : TPM1 (16-bit)
**         Compare name                : TPM10
**         Counter shared              : No
**
**         High speed mode
**             Prescaler               : divide-by-1
**             Clock                   : 25165824 Hz
**           Initial period/frequency
**             Xtal ticks              : 66
**             microseconds            : 2000
**             milliseconds            : 2
**             seconds (real)          : 0.0020000
**             Hz                      : 500
**
**         Runtime setting             : none
**
**         Initialization:
**              Timer                  : Enabled
**              Events                 : Enabled
**
**         Timer registers
**              Counter                : TPM1CNT   [0xFFFF8041]
**              Mode                   : TPM1SC    [0xFFFF8040]
**              Run                    : TPM1SC    [0xFFFF8040]
**              Prescaler              : TPM1SC    [0xFFFF8040]
**
**         Compare registers
**              Compare                : TPM1C0V   [0xFFFF8046]
**
**         Flip-flop registers
**              Mode                   : TPM1C0SC  [0xFFFF8045]
**     Contents  :
**         Enable  - byte hwTimerNav_Enable(void);
**         Disable - byte hwTimerNav_Disable(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef __hwTimerNav
#define __hwTimerNav

/* MODULE hwTimerNav. */

/*Include shared modules, which are used for whole project*/
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"

byte hwTimerNav_Enable(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_Enable (bean TimerInt)
**
**     Description :
**         This method enables the bean - it starts the timer. Events
**         may be generated (<DisableEvent>/<EnableEvent>).
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/

byte hwTimerNav_Disable(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_Disable (bean TimerInt)
**
**     Description :
**         This method disables the bean - it stops the timer. No
**         events will be generated.
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/

__interrupt void hwTimerNav_Interrupt(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_Interrupt (bean TimerInt)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes the beans event(s).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwTimerNav_Init(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_Init (bean TimerInt)
**
**     Description :
**         Initializes the associated peripheral(s) and the beans 
**         internal variables. The method is called automatically as a 
**         part of the application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwTimerNav_SetHigh(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_SetHigh (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the High speed mode. The method is 
**         called automatically as s part of the CPU SetHighSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwTimerNav_SetSlow(void);
/*
** ===================================================================
**     Method      :  hwTimerNav_SetSlow (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the Slow speed mode. The method is 
**         called automatically as a part of the CPU SetSlowSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/


#define hwTimerNav_SetPV(_Val) \
  TPM1CNTH = 0x00; \
  TPM1SC = (byte)((TPM1SC & 0x78) | (_Val)); /* Set TPM clock source and prescaler and clear overflow interrupt */
/*
** ===================================================================
**     Method      :  SetPV (bean TimerInt)
**
**     Description :
**         The method sets prescaler of the device.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

#define hwTimerNav_SetCV(_Val) ( \
  TPM1C0V = TPM1MOD = (word)(_Val) )
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

/* END hwTimerNav. */

#endif /* ifndef __hwTimerNav */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/