/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwRadioDtr.H
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
**                       70            |  PTE2_RGPIO2_MISO1
**             ----------------------------------------------------
**
**         Port name                   : PTE
**
**         Bit number (in port)        : 2
**         Bit mask of the port        : 0x0004
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 1
**         Initial pull option         : off
**
**         Port data register          : PTED      [0xFFFF8008]
**         Port control register       : PTEDD     [0xFFFF8009]
**
**         Optimization for            : speed
**     Contents  :
**         ClrVal - void hwRadioDtr_ClrVal(void);
**         SetVal - void hwRadioDtr_SetVal(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef hwRadioDtr_H_
#define hwRadioDtr_H_

/* MODULE hwRadioDtr. */

  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwRadioDtr_ClrVal (bean BitIO)
**
**     Description :
**         This method clears (sets to zero) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwRadioDtr_ClrVal() ( \
    (void)setReg8(PTECLR, 0x04)        /* PTECLR2=0x01 */ \
  )

/*
** ===================================================================
**     Method      :  hwRadioDtr_SetVal (bean BitIO)
**
**     Description :
**         This method sets (sets to one) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwRadioDtr_SetVal() ( \
    (void)setReg8(PTESET, 0x04)        /* PTESET2=0x01 */ \
  )



/* END hwRadioDtr. */
#endif /* #ifndef __hwRadioDtr_H_ */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/