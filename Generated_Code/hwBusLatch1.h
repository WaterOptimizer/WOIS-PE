/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwBusLatch1.H
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
**                       17            |  PTH2
**             ----------------------------------------------------
**
**         Port name                   : PTH
**
**         Bit number (in port)        : 2
**         Bit mask of the port        : 0x0004
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 0
**         Initial pull option         : off
**
**         Port data register          : PTHD      [0xFFFF801E]
**         Port control register       : PTHDD     [0xFFFF801F]
**
**         Optimization for            : speed
**     Contents  :
**         ClrVal - void hwBusLatch1_ClrVal(void);
**         SetVal - void hwBusLatch1_SetVal(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef hwBusLatch1_H_
#define hwBusLatch1_H_

/* MODULE hwBusLatch1. */

  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwBusLatch1_ClrVal (bean BitIO)
**
**     Description :
**         This method clears (sets to zero) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwBusLatch1_ClrVal() ( \
    (void)clrReg8Bits(PTHD, 0x04)      /* PTHD2=0x00 */ \
  )

/*
** ===================================================================
**     Method      :  hwBusLatch1_SetVal (bean BitIO)
**
**     Description :
**         This method sets (sets to one) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwBusLatch1_SetVal() ( \
    (void)setReg8Bits(PTHD, 0x04)      /* PTHD2=0x01 */ \
  )



/* END hwBusLatch1. */
#endif /* #ifndef __hwBusLatch1_H_ */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/