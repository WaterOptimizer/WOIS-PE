/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwCpu.H
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : MCF51QE128_80
**     Version   : Bean 01.011, Driver 01.03, CPU db: 3.00.052
**     Datasheet : MCF51QE128RM, Rev. 3, 9/2007
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 5/14/2010, 1:20 PM
**     Abstract  :
**         This bean "MCF51QE128_80" contains initialization of the
**         CPU and provides basic methods and events for CPU core
**         settings.
**     Comment   :
**         This is the 80-pin QE128 CPU configuration used by the Debug configuration.  The primary difference from the Release 
**         configuration is that the BDM port is enabled.
**     Settings  :
**
**     Contents  :
**         SetHighSpeed   - void hwCpu_SetHighSpeed(void);
**         SetSlowSpeed   - void hwCpu_SetSlowSpeed(void);
**         GetResetSource - byte hwCpu_GetResetSource(void);
**         SetStopMode    - void hwCpu_SetStopMode(void);
**         Delay100US     - void hwCpu_Delay100US(word us100);
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/

#ifndef __hwCpu
#define __hwCpu

/* MODULE hwCpu. */
/* pragma to disable "possibly unassigned ISR handler" message generated by compiler on definition of ISR without vector number */
#pragma warn_absolute off

/* Active configuration define symbol */
#define PEcfg_Debug_E128LQFP80 1

/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"



#define CPU_BUS_CLK_HZ              0x01800000UL /* Initial value of the bus clock frequency in Hz */
#define CPU_BUS_CLK_HZ_HIGH         0x01800000UL /* Value of the bus clock frequency in the high speed mode in Hz */
#define CPU_BUS_CLK_HZ_SLOW         0x4000UL /* Value of the bus clock frequency in the slow speed mode in Hz */

#define CPU_INSTR_CLK_HZ            0x01800000UL /* Initial value of the instruction clock frequency in Hz */
#define CPU_INSTR_CLK_HZ_HIGH       0x01800000UL /* Value of the instruction clock frequency in the high speed mode in Hz */
#define CPU_INSTR_CLK_HZ_SLOW       0x4000UL /* Value of the instruction clock frequency in the slow speed mode in Hz */

#define CPU_EXT_CLK_HZ              0x8000UL /* Value of the main clock frequency (crystal or external clock) in Hz */
#define CPU_INT_CLK_HZ              0x8000UL /* Value of the internal oscillator clock frequency in Hz */

#define CPU_TICK_NS                 0x7736U /* CPU tick is a unit derived from the frequency of external clock source. If no external clock is enabled or available it is derived from the value of internal clock source. The value of this constant represents period of the clock source in ns. */

#define CPU_CORE_ColdFireV1            /* Specification of the core type of the selected cpu */
#define CPU_DERIVATIVE_MCF51QE128      /* Name of the selected cpu derivative */
#define CPU_PARTNUM_MCF51QE128CLK      /* Part number of the selected cpu */

/* Global variables */
extern volatile far word SR_reg;       /* Current CCR reegister */
extern volatile byte SR_lock;
extern far byte CpuMode;               /* Current speed mode */




__interrupt void hwCpu_Interrupt(void);
/*
** ===================================================================
**     Method      :  hwCpu_hwCpu_Interrupt (bean MCF51QE128_80)
**
**     Description :
**         This ISR services an unused interrupt/exception vector.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void __initialize_hardware(void);
/*
** ===================================================================
**     Method      :  __initialize_hardware (bean MCF51QE128_80)
**
**     Description :
**         Configure the basic system functions (timing, etc.).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void hwCpu_SetHighSpeed(void);
/*
** ===================================================================
**     Method      :  hwCpu_SetHighSpeed (bean MCF51QE128_80)
**
**     Description :
**         Sets the high speed mode. The method is enabled only if <Low
**         speed mode> or <Slow speed mode> are enabled in the bean as
**         well.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

void hwCpu_SetSlowSpeed(void);
/*
** ===================================================================
**     Method      :  hwCpu_SetSlowSpeed (bean MCF51QE128_80)
**
**     Description :
**         Sets the slow speed mode. The method is enabled only if
**         <Slow speed mode> is enabled in the bean.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define   hwCpu_SetStopMode()  asm {  mov3q #4,d0; bclr.b d0,SOPT1; nop; stop #0x2000; } /* Set STOP mode */
/*
** ===================================================================
**     Method      :  hwCpu_SetStopMode (bean MCF51QE128_80)
**
**     Description :
**         Sets the low power mode - Stop mode. This method is
**         available only if the STOP instruction is enabled (see <STOP
**         instruction enabled> property).
**         For more information about the stop mode, see the
**         documentation of this CPU.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

 __declspec(register_abi) void hwCpu_Delay100US(word us100:__D0);
/*
** ===================================================================
**     Method      :  hwCpu_Delay100US (bean MCF51QE128_80)
**
**     Description :
**         This method realizes software delay. The length of delay is
**         at least 100 microsecond multiply input parameter [us100].
**         As the delay implementation is not based on real clock, the
**         delay time may be increased by interrupt service routines
**         processed during the delay. The method is independent on
**         selected speed mode.
**     Parameters  :
**         NAME            - DESCRIPTION
**         us100           - Number of 100 us delay repetitions.
**     Returns     : Nothing
** ===================================================================
*/

#define hwCpu_GetResetSource() (SRS)
/*
** ===================================================================
**     Method      :  hwCpu_GetResetSource (bean MCF51QE128_80)
**
**     Description :
**         Gets the reset source of the last reset (value of SRS
**         register).
**     Parameters  : None
**     Returns     :
**         ---             - Reset source register.
** ===================================================================
*/

void PE_low_level_init(void);
/*
** ===================================================================
**     Method      :  PE_low_level_init (bean MCF51QE128_80)
**
**     Description :
**         Initializes beans and provides common register initialization. 
**         The method is called automatically as a part of the 
**         application initialization code.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

/* END hwCpu. */

#endif
/* ifndef __hwCpu */
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
