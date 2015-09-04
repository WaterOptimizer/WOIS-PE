/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwSpiSS.H
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
**                       22            |  PTB5_TPM1CH1_SS1
**             ----------------------------------------------------
**
**         Port name                   : PTB
**
**         Bit number (in port)        : 5
**         Bit mask of the port        : 0x0020
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 1
**         Initial pull option         : off
**
**         Port data register          : PTBD      [0xFFFF8002]
**         Port control register       : PTBDD     [0xFFFF8003]
**
**         Optimization for            : speed
**     Contents  :
**         ClrVal - void hwSpiSS_ClrVal(void);
**         SetVal - void hwSpiSS_SetVal(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef hwSpiSS_H_
#define hwSpiSS_H_

/* MODULE hwSpiSS. */

  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwSpiSS_ClrVal (bean BitIO)
**
**     Description :
**         This method clears (sets to zero) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwSpiSS_ClrVal() ( \
    (void)clrReg8Bits(PTBD, 0x20)      /* PTBD5=0x00 */ \
  )

/*
** ===================================================================
**     Method      :  hwSpiSS_SetVal (bean BitIO)
**
**     Description :
**         This method sets (sets to one) the output value.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
#define hwSpiSS_SetVal() ( \
    (void)setReg8Bits(PTBD, 0x20)      /* PTBD5=0x01 */ \
  )



/* END hwSpiSS. */
#endif /* #ifndef __hwSpiSS_H_ */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
