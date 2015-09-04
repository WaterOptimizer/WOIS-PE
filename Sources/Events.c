/** ###################################################################
**     Filename  : Events.C
**     Project   : WOIS
**     Processor : MCF51QE128LQFP80
**     Beantype  : Events
**     Version   : Driver 01.02
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 3/11/2008, 11:35 AM
**     Abstract  :
**         This is user's event module.
**         Put your event handler code here.
**     Settings  :
**     Contents  :
**         hwRtc_OnInterrupt    - void hwRtc_OnInterrupt(void);
**         hwExpOut_OnError     - void hwExpOut_OnError(void);
**         hwExpOut_OnRxChar    - void hwExpOut_OnRxChar(void);
**         hwExpOut_OnTxChar    - void hwExpOut_OnTxChar(void);
**         hwI2c_OnReceiveData  - void hwI2c_OnReceiveData(void);
**         hwI2c_OnTransmitData - void hwI2c_OnTransmitData(void);
**         hwI2c_OnArbitLost    - void hwI2c_OnArbitLost(void);
**         hwI2c_OnNACK         - void hwI2c_OnNACK(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2006
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/
/* MODULE Events */

#include "hwCpu.h"
#include "Events.h"

#include "global.h"
#include "drvKeypad.h"
#include "drvMoist.h"
#include "drvRadio.h"
#include "drvRtc.h"
#include "drvSolenoid.h"

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
void hwRtc_OnInterrupt(void)
{
  /* Write your code here ... */
  drvRtcIsr();
}

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
void hwTimerKeypad_OnInterrupt(void)
{
  /* Write your code here ... */
  drvKeypadSwitchIsr();
  drvSolenoidIsr();
  drvMoistIsr();
}

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
void hwTimerNav_OnInterrupt(void)
{
  /* Write your code here ... */
  drvKeypadNavIsr();
  drvMSIsr();                           /* increment fast tick counter */
}

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
void  hwExpIn_OnTxChar(void)
{
  /* Write your code here ... */
  drvRadioOnTxChar();
}

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
void  hwExpIn_OnRxChar(void)
{
  /* Write your code here ... */
  drvRadioOnRxChar();
}

/* END Events */

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.01 [03.92]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
