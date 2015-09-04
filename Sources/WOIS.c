/** ###################################################################
**     Filename  : WOIS.C
**     Project   : WOIS
**     Processor : MCF51QE128LQFP80
**     Version   : Driver 01.00
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 8/1/2008, 3:57 PM
**     Abstract  :
**         Main module.
**         Here is to be placed user's code.
**     Settings  :
**     Contents  :
**         No public methods
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2006
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/
/* MODULE WOIS */


/* Including used modules for compiling procedure */
#include "hwCpu.h"
#include "Events.h"
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
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

/* WaterOptimizer header files */
#include "global.h"
#include "drvSys.h"

void main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */

  /* initialize WaterOptimizer drivers and start application */
  drvSysStart();                        /* DOES NOT RETURN */

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END WOIS */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.01 [03.92]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
