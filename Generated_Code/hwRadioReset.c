/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwRadioReset.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : BitIO
**     Version   : Bean 02.071, Driver 03.08, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 5/14/2010, 1:20 PM
**     Abstract  :
**         This bean "BitIO" implements an one-bit input/output.
**         It uses one bit/pin of a port.
**         Note: This bean is set to work in Output direction only.
**         Methods of this bean are mostly implemented as a macros
**         (if supported by target language and compiler).
**     Settings  :
**         Used pin                    :
**             ----------------------------------------------------
**                Number (on package)  |    Name
**             ----------------------------------------------------
**                       24            |  PTC3_RGPIO11_TPM3CH3
**             ----------------------------------------------------
**
**         Port name                   : PTC
**
**         Bit number (in port)        : 3
**         Bit mask of the port        : 0x0008
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 0
**         Initial pull option         : off
**
**         Port data register          : PTCD      [0xFFFF8004]
**         Port control register       : PTCDD     [0xFFFF8005]
**
**         Optimization for            : speed
**     Contents  :
**         ClrVal - void hwRadioReset_ClrVal(void);
**         SetVal - void hwRadioReset_SetVal(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

/* MODULE hwRadioReset. */

#include "hwRadioReset.h"
  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwRadioReset_ClrVal (bean BitIO)
**
**     Description :
**         This method clears (sets to zero) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void hwRadioReset_ClrVal(void)

**  This method is implemented as a macro. See hwRadioReset.h file.  **
*/

/*
** ===================================================================
**     Method      :  hwRadioReset_SetVal (bean BitIO)
**
**     Description :
**         This method sets (sets to one) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
/*
void hwRadioReset_SetVal(void)

**  This method is implemented as a macro. See hwRadioReset.h file.  **
*/


/* END hwRadioReset. */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/