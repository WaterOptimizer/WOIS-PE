/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwLed.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : BitsIO
**     Version   : Bean 02.097, Driver 03.08, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
**     Abstract  :
**         This bean "BitsIO" implements a multi-bit input/output.
**         It uses selected pins of one 1-bit to 8-bit port.
**         Note: This bean is set to work in Output direction only.
**     Settings  :
**         Port name                   : PTE
**
**         Bit mask of the port        : 0x00E0
**         Number of bits/pins         : 3
**         Single bit numbers          : 0 to 2
**         Values range                : 0 to 7
**
**         Initial direction           : Output (direction cannot be changed)
**         Initial output value        : 0 = 000H
**         Initial pull option         : off
**
**         Port data register          : PTED      [0xFFFF8008]
**         Port control register       : PTEDD     [0xFFFF8009]
**
**             ----------------------------------------------------
**                   Bit     |   Pin   |   Name
**             ----------------------------------------------------
**                    0      |    21   |   PTE5_RGPIO5
**                    1      |    20   |   PTE6_RGPIO6
**                    2      |    7    |   PTE7_RGPIO7_TPM3CLK
**             ----------------------------------------------------
**
**         Optimization for            : speed
**     Contents  :
**         GetVal - byte hwLed_GetVal(void);
**         PutVal - void hwLed_PutVal(byte Val);
**         SetBit - void hwLed_SetBit(byte Bit);
**         ClrBit - void hwLed_ClrBit(byte Bit);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

/* MODULE hwLed. */

#include "hwLed.h"
  /* Including shared modules, which are used in the whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "hwCpu.h"


/*
** ===================================================================
**     Method      :  hwLed_GetMsk (bean BitsIO)
**
**     Description :
**         The method returns a bit mask which corresponds to the 
**         required bit position.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
static const byte hwLed_Table[3] = {   /* Table of mask constants */
   0x20, 0x40, 0x80
};

static byte hwLed_GetMsk (byte PinIndex)
{
  return (byte)(PinIndex<3 ? hwLed_Table[PinIndex] : 0); /* Check range and return appropriate bit mask */
}

/*
** ===================================================================
**     Method      :  hwLed_GetVal (bean BitsIO)
**
**     Description :
**         This method returns an input value.
**           a) direction = Input  : reads the input value from the
**                                   pins and returns it
**           b) direction = Output : returns the last written value
**         Note: This bean is set to work in Output direction only.
**     Parameters  : None
**     Returns     :
**         ---        - Input value (0 to 7)
** ===================================================================
*/
byte hwLed_GetVal(void)
{
  return (byte)((getReg8(PTED) & 0xE0) >> 5); /* Return port data */
}

/*
** ===================================================================
**     Method      :  hwLed_PutVal (bean BitsIO)
**
**     Description :
**         This method writes the new output value.
**     Parameters  :
**         NAME       - DESCRIPTION
**         Val        - Output value (0 to 7)
**     Returns     : Nothing
** ===================================================================
*/
void hwLed_PutVal(byte Val)
{
  Val = (byte)((Val & 0x07) << 5);     /* Mask and shift output value */
  setReg8(PTED, (getReg8(PTED) & (~0xE0)) | Val); /* Put masked value on port */
}

/*
** ===================================================================
**     Method      :  hwLed_ClrBit (bean BitsIO)
**
**     Description :
**         This method clears (sets to zero) the specified bit
**         of the output value.
**         [ It is the same as "PutBit(Bit,FALSE);" ]
**     Parameters  :
**         NAME       - DESCRIPTION
**         Bit        - Number of the bit to clear (0 to 2)
**     Returns     : Nothing
** ===================================================================
*/
void hwLed_ClrBit(byte Bit)
{
  byte const Mask = hwLed_GetMsk(Bit); /* Temporary variable - set bit mask */
  setReg8(PTECLR, Mask);               /* [bit (Bit+5)]=0x01 */
}

/*
** ===================================================================
**     Method      :  hwLed_SetBit (bean BitsIO)
**
**     Description :
**         This method sets (sets to one) the specified bit of the
**         output value.
**         [ It is the same as "PutBit(Bit,TRUE);" ]
**     Parameters  :
**         NAME       - DESCRIPTION
**         Bit        - Number of the bit to set (0 to 2)
**     Returns     : Nothing
** ===================================================================
*/
void hwLed_SetBit(byte Bit)
{
  byte const Mask = hwLed_GetMsk(Bit); /* Temporary variable - set bit mask */
  setReg8(PTESET, Mask);               /* [bit (Bit+5)]=0x01 */
}


/* END hwLed. */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
