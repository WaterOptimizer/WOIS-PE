/** ###################################################################
**     Filename  : Events.H
**     Project   : WOIS
**     Processor : MCF51QE128LQFP80
**     Beantype  : Events
**     Version   : Driver 01.02
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 8/1/2008, 2:59 PM
**     Abstract  :
**         This is user's event module.
**         Put your event handler code here.
**     Settings  :
**     Contents  :
**         hwTimerNav_OnInterrupt    - void hwTimerNav_OnInterrupt(void);
**         hwTimerKeypad_OnInterrupt - void hwTimerKeypad_OnInterrupt(void);
**         hwRtc_OnInterrupt         - void hwRtc_OnInterrupt(void);
**         hwExpIn_OnRxChar          - void hwExpIn_OnRxChar(void);
**         hwExpIn_OnTxChar          - void hwExpIn_OnTxChar(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2006
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef __Events_H
#define __Events_H
/* MODULE Events */

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwAdc.h"
#include "hwBusData.h"
#include "hwBusKeypad.h"
#include "hwBusLatch1.h"
#include "hwBusLatch2.h"
#include "hwBusLatch3.h"
#include "hwBusLatchReset.h"
#include "hwCts.h"
#include "hwExpIn.h"
#include "hwExpInRts.h"
#include "hwExpOut.h"
#include "hwExpOutRts.h"
#include "hwI2c.h"
#include "hwLcdEnb.h"
#include "hwLcdRs.h"
#include "hwLcdRw.h"
#include "hwLed.h"
#include "hwNav.h"
#include "hwPower12vdcStatus.h"
#include "hwPower24vacStatus.h"
#include "hwRadioDtr.h"
#include "hwRadioReset.h"
#include "hwRadioRts.h"
#include "hwRtc.h"
#include "hwSpi.h"
#include "hwSpiSS.h"
#include "hwTimerKeypad.h"
#include "hwTimerNav.h"
#include "hwUart1Select.h"
#include "hwWatchDog.h"
#include "hwUnusedA0.h"
#include "hwUnusedA3.h"
#include "hwUnusedA6.h"
#include "hwUnusedA7.h"
#include "hwUnusedC5.h"
#include "hwUnusedD0.h"
#include "hwUnusedD4.h"
#include "hwUnusedD7.h"
#include "hwUnusedF0.h"
#include "hwUnusedF1.h"
#include "hwPower24vacControl.h"

void hwTimerNav_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  hwTimerNav_OnInterrupt (module Events)
**
**     From bean   :  hwTimerNav [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the bean is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void hwTimerKeypad_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  hwTimerKeypad_OnInterrupt (module Events)
**
**     From bean   :  hwTimerKeypad [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the bean is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void hwRtc_OnInterrupt(void);
/*
** ===================================================================
**     Event       :  hwRtc_OnInterrupt (module Events)
**
**     From bean   :  hwRtc [TimerInt]
**     Description :
**         When a timer interrupt occurs this event is called (only
**         when the bean is enabled - <Enable> and the events are
**         enabled - <EnableEvent>). This event is enabled only if a
**         <interrupt service/event> is enabled.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void hwExpIn_OnRxChar(void);
/*
** ===================================================================
**     Event       :  hwExpIn_OnRxChar (module Events)
**
**     From bean   :  hwExpIn [AsynchroSerial]
**     Description :
**         This event is called after a correct character is
**         received.
**         The event is available only when the <Interrupt
**         service/event> property is enabled and either the
**         <Receiver> property is enabled or the <SCI output mode>
**         property (if supported) is set to Single-wire mode.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void hwExpIn_OnTxChar(void);
/*
** ===================================================================
**     Event       :  hwExpIn_OnTxChar (module Events)
**
**     From bean   :  hwExpIn [AsynchroSerial]
**     Description :
**         This event is called after a character is transmitted.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/


/* END Events */
#endif /* __Events_H*/

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.01 [03.92]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
