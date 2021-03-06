/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwTimerKeypad.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : TimerInt
**     Version   : Bean 02.154, Driver 01.14, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
**     Abstract  :
**         This bean "TimerInt" implements a periodic interrupt.
**         When the bean and its events are enabled, the "OnInterrupt"
**         event is called periodically with the period that you specify.
**         TimerInt supports also changing the period in runtime.
**         The source of periodic interrupt can be timer compare or reload
**         register or timer-overflow interrupt (of free running counter).
**     Settings  :
**         Timer name                  : TPM3 (16-bit)
**         Compare name                : TPM30
**         Counter shared              : No
**
**         High speed mode
**             Prescaler               : divide-by-8
**             Clock                   : 3145728 Hz
**           Initial period/frequency
**             Xtal ticks              : 655
**             microseconds            : 20000
**             milliseconds            : 20
**             seconds (real)          : 0.0200001
**             Hz                      : 50
**
**         Runtime setting             : none
**
**         Initialization:
**              Timer                  : Enabled
**              Events                 : Enabled
**
**         Timer registers
**              Counter                : TPM3CNT   [0xFFFF8061]
**              Mode                   : TPM3SC    [0xFFFF8060]
**              Run                    : TPM3SC    [0xFFFF8060]
**              Prescaler              : TPM3SC    [0xFFFF8060]
**
**         Compare registers
**              Compare                : TPM3C0V   [0xFFFF8066]
**
**         Flip-flop registers
**              Mode                   : TPM3C0SC  [0xFFFF8065]
**     Contents  :
**         Enable  - byte hwTimerKeypad_Enable(void);
**         Disable - byte hwTimerKeypad_Disable(void);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

/* MODULE hwTimerKeypad. */

#include "Events.h"
#include "PE_Error.h"
#include "hwTimerKeypad.h"

static bool EnMode;                    /* Enable/Disable device in a given speed CPU mode */
static bool EnUser;                    /* Enable device by user */
/* Internal method prototypes */


/*
** ===================================================================
**     Method      :  HWEnDi (bean TimerInt)
**
**     Description :
**         Enables or disables the peripheral(s) associated with the bean.
**         The method is called automatically as a part of the Enable and 
**         Disable methods and several internal methods.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
static void HWEnDi(void)
{
  if (EnUser && EnMode) {
    TPM3SC |= 0x08;                    /* Run counter (set CLKSB:CLKSA) */
  } else {
    /* TPM3SC: CLKSB=0,CLKSA=0 */
    clrReg8Bits(TPM3SC, 0x18);         /* Stop counter (CLKSB:CLKSA = 00) */ 
    /* TPM3CNTH: BIT15=0,BIT14=0,BIT13=0,BIT12=0,BIT11=0,BIT10=0,BIT9=0,BIT8=0 */
    setReg8(TPM3CNTH, 0x00);           /* Clear counter register - any write clears complete counter */ 
  }
}

/*
** ===================================================================
**     Method      :  hwTimerKeypad_Enable (bean TimerInt)
**
**     Description :
**         This method enables the bean - it starts the timer. Events
**         may be generated (<DisableEvent>/<EnableEvent>).
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte hwTimerKeypad_Enable(void)
{
  if (!EnMode) {                       /* Is the device disabled in the actual speed CPU mode? */
    return ERR_SPEED;                  /* If yes then error */
  }
  if (!EnUser) {                       /* Is the device disabled by user? */
    EnUser = TRUE;                     /* If yes then set the flag "device enabled" */
    HWEnDi();                          /* Enable the device */
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  hwTimerKeypad_Disable (bean TimerInt)
**
**     Description :
**         This method disables the bean - it stops the timer. No
**         events will be generated.
**     Parameters  : None
**     Returns     :
**         ---             - Error code, possible codes:
**                           ERR_OK - OK
**                           ERR_SPEED - This device does not work in
**                           the active speed mode
** ===================================================================
*/
byte hwTimerKeypad_Disable(void)
{
  if (!EnMode) {                       /* Is the device disabled in the actual speed CPU mode? */
    return ERR_SPEED;                  /* If yes then error */
  }
  if (EnUser) {                        /* Is the device enabled by user? */
    EnUser = FALSE;                    /* If yes then set the flag "device disabled" */
    HWEnDi();                          /* Disable the device */
  }
  return ERR_OK;                       /* OK */
}

/*
** ===================================================================
**     Method      :  hwTimerKeypad_SetHigh (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the High speed mode. The method is 
**         called automatically as s part of the CPU SetHighSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void hwTimerKeypad_SetHigh(void)
{
  hwTimerKeypad_SetPV((byte)0x03);     /* Set prescaler */
  /* TPM3CNTH: BIT15=0,BIT14=0,BIT13=0,BIT12=0,BIT11=0,BIT10=0,BIT9=0,BIT8=0 */
  setReg8(TPM3CNTH, 0x00);             /* Reset HW Counter */ 
  EnMode = TRUE;                       /* Set the flag "device enabled" in the actual speed CPU mode */
  HWEnDi();
}

/*
** ===================================================================
**     Method      :  hwTimerKeypad_SetSlow (bean TimerInt)
**
**     Description :
**         The method reconfigures the bean and its selected peripheral(s)
**         when the CPU is switched to the Slow speed mode. The method is 
**         called automatically as a part of the CPU SetSlowSpeed method.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void hwTimerKeypad_SetSlow(void)
{
  EnMode = FALSE;                      /* Set the flag "device disabled" in the actual speed CPU mode */
  HWEnDi();
}

/*
** ===================================================================
**     Method      :  hwTimerKeypad_Init (bean TimerInt)
**
**     Description :
**         Initializes the associated peripheral(s) and the beans 
**         internal variables. The method is called automatically as a 
**         part of the application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
void hwTimerKeypad_Init(void)
{
  /* TPM3SC: TOF=0,TOIE=0,CPWMS=0,CLKSB=0,CLKSA=0,PS2=0,PS1=0,PS0=0 */
  setReg8(TPM3SC, 0x00);               /* Stop HW; disable overflow interrupt and set prescaler to 0 */ 
  /* TPM3C0SC: CH0F=0,CH0IE=1,MS0B=0,MS0A=1,ELS0B=0,ELS0A=0,??=0,??=0 */
  setReg8(TPM3C0SC, 0x50);             /* Set output compare mode and enable compare interrupt */ 
  EnUser = TRUE;                       /* Enable device */
  hwTimerKeypad_SetCV(0xF5C2U);        /* Inicialize appropriate value to the compare/modulo/reload register */
  hwTimerKeypad_SetHigh();             /* Initial speed CPU mode is high */
}


/*
** ===================================================================
**     Method      :  hwTimerKeypad_Interrupt (bean TimerInt)
**
**     Description :
**         The method services the interrupt of the selected peripheral(s)
**         and eventually invokes the beans event(s).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
ISR(hwTimerKeypad_Interrupt)
{
  /* TPM3C0SC: CH0F=0 */
  clrReg8Bits(TPM3C0SC, 0x80);         /* Reset compare interrupt request flag */ 
  hwTimerKeypad_OnInterrupt();         /* Invoke user event */
}



/* END hwTimerKeypad. */

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
