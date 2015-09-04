/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwNav.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : BitsIO
**     Version   : Bean 02.097, Driver 03.08, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
**     Abstract  :
**         This bean "BitsIO" implements a multi-bit input/output.
**         It uses selected pins of one 1-bit to 8-bit port.
**         Note: This bean is set to work in Input direction only.
**     Settings  :
**         Port name                   : PTD
**
**         Bit mask of the port        : 0x000C
**         Number of bits/pins         : 2
**         Single bit numbers          : 0 to 1
**         Values range                : 0 to 3
**
**         Initial direction           : Input (direction cannot be changed)
**         Initial output value        : 0 = 000H
**         Initial pull option         : off
**
**         Port data register          : PTDD      [0xFFFF8006]
**         Port control register       : PTDDD     [0xFFFF8007]
**
**             ----------------------------------------------------
**                   Bit     |   Pin   |   Name
**             ----------------------------------------------------
**                    0      |    58   |   PTD2_KBI2P2_MISO2
**                    1      |    57   |   PTD3_KBI2P3_SS2
**             ----------------------------------------------------
**
**         Optimization for            : speed
**     Contents  :
**         GetVal - byte hwNav_GetVal(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

/* MODULE hwNav. */

#include "hwNav.h"
  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwNav_GetVal (bean BitsIO)
**
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pins and returns it
**           b) direction = Output : returns the last written value
**         Note: This bean is set to work in Input direction only.
**     Parameters  : None
**     Returns     :
**         ---        - Input value (0 to 3)
** ===================================================================
*/
byte hwNav_GetVal(void)
{
  return (byte)((getReg8(PTDD) & 0x0C) >> 2); /* Return port data */
}


/* END hwNav. */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
