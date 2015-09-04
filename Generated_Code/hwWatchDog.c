/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwWatchDog.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : WatchDog
**     Version   : Bean 02.087, Driver 01.10, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
**     Abstract  :
**         This device "WatchDog" implements a watchdog.
**         When enabled, the watchdog timer counts until it reaches
**         a critical value corresponding to the period specified
**         in 'Properties'. Then, the watchdog causes a CPU reset.
**         Applications may clear the timer before it reaches the critical
**         value. The timer then restarts the watchdog counter.
**         Watchdog is used for increasing the safety of the system
**         (unforeseeable situations can cause system crash or an
**         infinite loop - watchdog can restart the system or handle
**         the situation).
**
**         Note: Watchdog can be enabled or disabled in the initialization
**               code only. If the watchdog is once enabled user have
**               to call Clear method in defined time intervals.
**     Settings  :
**         Watchdog causes             : Reset CPU
**
**         Initial Watchdog state      : Enabled
**
**
**
**         High speed mode
**           Watchdog period/frequency
**             microseconds            : 256000
**             milliseconds            : 256
**             Hz                      : 4
**
**         Run register                : SOPT1     [0xFFFF9802]
**         Mode register               : SRS       [0xFFFF9800]
**         Prescaler register          : SOPT1     [0xFFFF9802]
**     Contents  :
**         Clear - byte hwWatchDog_Clear(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/


/* MODULE hwWatchDog. */

#include "hwWatchDog.h"

/*
** ===================================================================
**     Method      :  hwWatchDog_Clear (bean WatchDog)
**
**     Description :
**         Clears the watchdog timer (it makes the timer restart from
**         zero).
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
**                           ERR_DISABLED - The Bean is disabled
** ===================================================================
*/
/*
void hwWatchDog_Clear(void)

**      This method is implemented as macro      **
*/

/* END hwWatchDog. */

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/