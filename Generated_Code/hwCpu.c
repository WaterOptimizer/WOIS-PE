/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : hwCpu.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Beantype  : MCF51QE128_80
**     Version   : Bean 01.011, Driver 01.03, CPU db: 3.00.052
**     Datasheet : MCF51QE128RM, Rev. 3, 9/2007
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 7/12/2011, 1:59 PM
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

/* MODULE hwCpu. */
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
#include "hwPower24vacControl.h"
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
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Events.h"
#include "hwCpu.h"

/* Global variables */
volatile far word SR_reg;              /* Current CCR register */
volatile byte SR_lock = 0;
far byte CpuMode = HIGH_SPEED;         /* Current speed mode */


/*
** ===================================================================
**     Method      :  hwCpu_hwCpu_Interrupt (bean MCF51QE128_80)
**
**     Description :
**         This ISR services an unused interrupt/exception vector.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
ISR(hwCpu_Interrupt)
{
}


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
void hwCpu_SetHighSpeed(void)
{
 /* Common peripheral initialization - before High speed mode */
  /* TPM3SC: CLKSB=0,CLKSA=0 */
  clrReg8Bits(TPM3SC, 0x18);            
  /* TPM1SC: CLKSB=0,CLKSA=0 */
  clrReg8Bits(TPM1SC, 0x18);            
  if (CpuMode != HIGH_SPEED) {         /* Is an actual cpu mode other than high speed mode? */
    EnterCritical();                   /* If yes then save the PS register */
    /* ICSC1: CLKS=0,RDIV=0,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
    setReg8(ICSC1, 0x00);              /* Initialization of the ICS control register 1 */ 
    /* ICSC2: BDIV=0,RANGE=0,HGO=0,LP=0,EREFS=1,ERCLKEN=1,EREFSTEN=1 */
    setReg8(ICSC2, 0x07);              /* Initialization of the ICS control register 2 */ 
    while(!ICSSC_OSCINIT) {            /* Wait until the initialization of the external crystal oscillator is completed */
     SRS = 0x00;                       /* Reset watchdog counter */
    }
    /* ICSSC: DRST_DRS=2,DMX32=0 */
    clrSetReg8Bits(ICSSC, 0x60, 0x80); /* Initialization of the ICS status and control */ 
    while((ICSSC & 0xC0) != 0x80) {    /* Wait until the FLL switches to High range DCO mode */
     SRS = 0x00;                       /* Reset watchdog counter */
    }
    ExitCritical();                    /* Restore the PS register */
    CpuMode = HIGH_SPEED;              /* Set actual cpu mode to high speed */
  }
  hwAdc_SetHigh();                     /* Set all beans in project to the high speed mode */
  hwExpIn_SetHigh();                   /* Set all beans in project to the high speed mode */
  hwExpOut_SetHigh();                  /* Set all beans in project to the high speed mode */
  hwI2c_SetHigh();                     /* Set all beans in project to the high speed mode */
  hwRtc_SetHigh();                     /* Set all beans in project to the high speed mode */
  hwSpi_SetHigh();                     /* Set all beans in project to the high speed mode */
  hwTimerKeypad_SetHigh();             /* Set all beans in project to the high speed mode */
  hwTimerNav_SetHigh();                /* Set all beans in project to the high speed mode */
}

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
void hwCpu_SetSlowSpeed(void)
{
  if (CpuMode != SLOW_SPEED) {         /* Is actual cpu mode other than slow slow mode? */
    EnterCritical();                   /* If yes then save the PS register */
    /* ICSC1: CLKS=2,RDIV=0,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
    setReg8(ICSC1, 0x80);              /* Initialization of the ICS control register 1 */ 
    /* ICSC2: BDIV=0,RANGE=0,HGO=0,LP=1,EREFS=1,ERCLKEN=1,EREFSTEN=1 */
    setReg8(ICSC2, 0x0F);              /* Initialization of the ICS control register 2 */ 
    while(!ICSSC_OSCINIT) {            /* Wait until the initialization of the external crystal oscillator is completed */
     SRS = 0x00;                       /* Reset watchdog counter */
    }
    ExitCritical();                    /* Restore the PS register */
    CpuMode = SLOW_SPEED;              /* Set actual cpu mode to high speed */
  }
  hwAdc_SetSlow();                     /* Set all beans in project to the slow speed mode */
  hwExpIn_SetSlow();                   /* Set all beans in project to the slow speed mode */
  hwExpOut_SetSlow();                  /* Set all beans in project to the slow speed mode */
  hwI2c_SetSlow();                     /* Set all beans in project to the slow speed mode */
  hwRtc_SetSlow();                     /* Set all beans in project to the slow speed mode */
  hwSpi_SetSlow();                     /* Set all beans in project to the slow speed mode */
  hwTimerKeypad_SetSlow();             /* Set all beans in project to the slow speed mode */
  hwTimerNav_SetSlow();                /* Set all beans in project to the slow speed mode */
}

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
/*
void hwCpu_SetStopMode(void)

**      This method is implemented as macro in the header module. **
*/

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
__declspec(register_abi) void hwCpu_Delay100US(word us100:__D0)
{
  /* irremovable one time overhead (ignored): 9 cycles */
  /* move: 1 cycles overhead (load parameter into register) */
  /* jsr:  3 cycles overhead (jump to subroutine) */
  /* rts:  5 cycles overhead (return from subroutine) */

  /* aproximate irremovable overhead for each 100us cycle (counted) : 3 cycles */
  /* subq.l 1 cycles overhead  */
  /* bne.b  2 cycles overhead  */

#pragma unused(us100)
  asm {
    naked
loop:
    /* 100 us delay block begin */
    move.b CpuMode,d2                  /* (1 c) get CpuMode */
    cmpi.b #HIGH_SPEED,d2              /* (1 c) compare it to HIGH_SPEED */
    bne.b label0                       /* (1 c (NOT TAKEN)) not equal? goto next section */
    /*
     * Delay
     *   - requested                  : 100 us @ 25.165824MHz,
     *   - possible                   : 2517 c, 100016.59 ns, delta 16.59 ns
     *   - without removable overhead : 2509 c, 99698.7 ns
     */
    move.l #0x0272,d1                  /* (1 c: 39.74 ns) number of iterations */
label1:
    move.b  d0,SRS                     /* Reset watchdog counter */
    subq.l #1,d1                       /* (1 c: 39.74 ns) decrement d1 */
    bne.b label1                       /* (2 c: 79.47 ns) repeat 626x */
    tpf                                /* (1 c: 39.74 ns) wait for 1 c */
    tpf                                /* (1 c: 39.74 ns) wait for 1 c */
    tpf                                /* (1 c: 39.74 ns) wait for 1 c */
    bra.b label2                       /* (2 c) finishing delay, goto end */
label0:
    /*
     * Delay
     *   - requested                  : 100 us @ 0.016384MHz,
     *   - possible                   : 2 c, 122070.31 ns, delta 22070.31 ns
     *   - without removable overhead : -7 c, -427246.09 ns
     *     (negative value means that overhead time is greater than requested delay time)
     */
    tpf                                /* (1 c: 61035.16 ns) minimal delay */
label2:                                /* End of delays */
    /* 100 us delay block end */
    subq.l #1,d0                       /* parameter is passed via d0 register */
    bne.b loop                         /* next loop */
    rts                                /* return from subroutine */
  }
}

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
/*
byte hwCpu_GetResetSource(void)

**      This method is implemented as macro in the header module. **
*/

/*
** ===================================================================
**     Method      :  __initialize_hardware (bean MCF51QE128_80)
**
**     Description :
**         Configure the basic system functions (timing, etc.).
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/

void __initialize_hardware(void)
{
  /* ### MCF51QE128_80 "Cpu" init code ... */
  /*  PE initialization code after reset */
  /* Common initialization of the write once registers */
  /* SOPT1: COPE=1,COPT=1,STOPE=1,WAITE=1,??=0,RSTOPE=0,BKGDPE=1,RSTPE=0 */
  setReg8(SOPT1, 0xF2);                 
  /* SOPT2: COPCLKS=0,??=0,??=0,??=0,SPI1PS=0,ACIC2=0,IIC1PS=0,ACIC1=0 */
  setReg8(SOPT2, 0x00);                 
  /* SPMSC1: LVDF=0,LVDACK=0,LVDIE=0,LVDRE=1,LVDSE=1,LVDE=0,??=0,BGBE=0 */
  setReg8(SPMSC1, 0x18);                
  /* SPMSC2: LPR=0,LPRS=0,LPWUI=0,??=0,PPDF=0,PPDACK=0,PPDE=0,PPDC=0 */
  setReg8(SPMSC2, 0x00);                
  /* SPMSC3: LVDV=0,LVWV=0,LVWIE=0 */
  clrReg8Bits(SPMSC3, 0x38);            
  /* Initialization of CPU registers */
  asm {
    /* VBR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,ADDRESS=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
    clr.l d0
    movec d0,VBR
    /* CPUCR: ARD=0,IRD=0,IAE=0,IME=0,BWD=0,??=0,FSD=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
    clr.l d0
    movec d0,CPUCR
  }
  /*  System clock initialization */
  /* ICSC1: CLKS=0,RDIV=0,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
  setReg8(ICSC1, 0x00);                /* Initialization of the ICS control register 1 */ 
  /* ICSC2: BDIV=0,RANGE=0,HGO=0,LP=0,EREFS=1,ERCLKEN=1,EREFSTEN=1 */
  setReg8(ICSC2, 0x07);                /* Initialization of the ICS control register 2 */ 
  while(!ICSSC_OSCINIT) {              /* Wait until the initialization of the external crystal oscillator is completed */
   SRS = 0x00;                         /* Reset watchdog counter */
  }
  /* ICSSC: DRST_DRS=2,DMX32=0 */
  clrSetReg8Bits(ICSSC, 0x60, 0x80);   /* Initialization of the ICS status and control */ 
  while((ICSSC & 0xC0) != 0x80) {      /* Wait until the FLL switches to High range DCO mode */
   SRS = 0x00;                         /* Reset watchdog counter */
  }
  /*** End of PE initialization code after reset ***/
}

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
void PE_low_level_init(void)
{
  /* Common initialization of the CPU registers */
  /* APCTL2: ADPC15=1,ADPC14=1,ADPC13=1,ADPC12=1 */
  setReg8Bits(APCTL2, 0xF0);            
  /* APCTL3: ADPC23=1,ADPC22=1,ADPC21=1,ADPC20=1,ADPC19=1,ADPC18=1,ADPC17=1,ADPC16=1 */
  setReg8(APCTL3, 0xFF);                
  /* PTJD: PTJD7=0,PTJD6=0,PTJD5=0,PTJD4=0,PTJD3=0,PTJD2=0,PTJD1=0,PTJD0=0 */
  setReg8(PTJD, 0x00);                  
  /* PTJPE: PTJPE7=0,PTJPE6=0,PTJPE5=0,PTJPE4=0,PTJPE3=0,PTJPE2=0,PTJPE1=0,PTJPE0=0 */
  setReg8(PTJPE, 0x00);                 
  /* PTJDD: PTJDD7=1,PTJDD6=1,PTJDD5=1,PTJDD4=1,PTJDD3=1,PTJDD2=1,PTJDD1=1,PTJDD0=1 */
  setReg8(PTJDD, 0xFF);                 
  /* PTCD: PTCD7=1,PTCD5=0,PTCD4=0,PTCD3=0,PTCD2=1,PTCD1=1,PTCD0=1 */
  clrSetReg8Bits(PTCD, 0x38, 0x87);     
  /* PTCPE: PTCPE5=0,PTCPE4=0,PTCPE3=0,PTCPE2=0,PTCPE1=0,PTCPE0=0 */
  clrReg8Bits(PTCPE, 0x3F);             
  /* PTCDD: PTCDD7=1,PTCDD6=0,PTCDD5=1,PTCDD4=1,PTCDD3=1,PTCDD2=1,PTCDD1=1,PTCDD0=1 */
  setReg8(PTCDD, 0xBF);                 
  /* PTHD: PTHD5=0,PTHD4=0,PTHD3=0,PTHD2=0,PTHD1=0,PTHD0=0 */
  clrReg8Bits(PTHD, 0x3F);              
  /* PTHPE: PTHPE5=0,PTHPE4=0,PTHPE3=0,PTHPE2=0,PTHPE1=0,PTHPE0=0 */
  clrReg8Bits(PTHPE, 0x3F);             
  /* PTHDD: PTHDD7=0,PTHDD6=0,PTHDD5=1,PTHDD4=1,PTHDD3=1,PTHDD2=1,PTHDD1=1,PTHDD0=1 */
  setReg8(PTHDD, 0x3F);                 
  /* SCGC2: FLS=1,IRQ=1,KBI=1,ACMP=0,RTC=1,SPI2=0,SPI1=1 */
  clrSetReg8Bits(SCGC2, 0x0A, 0x75);    
  /* PTDPE: PTDPE7=0,PTDPE6=0,PTDPE5=0,PTDPE4=0,PTDPE3=0,PTDPE2=0,PTDPE1=0,PTDPE0=0 */
  setReg8(PTDPE, 0x00);                 
  /* PTED: PTED7=0,PTED6=0,PTED5=0,PTED4=0,PTED3=1,PTED2=1,PTED1=1,PTED0=1 */
  setReg8(PTED, 0x0F);                  
  /* PTEPE: PTEPE7=0,PTEPE6=0,PTEPE5=0,PTEPE4=0,PTEPE3=0,PTEPE2=0,PTEPE1=0,PTEPE0=0 */
  setReg8(PTEPE, 0x00);                 
  /* PTEDD: PTEDD7=1,PTEDD6=1,PTEDD5=1,PTEDD4=1,PTEDD3=1,PTEDD2=1,PTEDD1=1,PTEDD0=1 */
  setReg8(PTEDD, 0xFF);                 
  /* PTBD: PTBD5=1,PTBD2=1,PTBD1=1 */
  setReg8Bits(PTBD, 0x26);              
  /* PTBDD: PTBDD5=1,PTBDD4=0,PTBDD3=0,PTBDD2=1,PTBDD1=1,PTBDD0=0 */
  clrSetReg8Bits(PTBDD, 0x19, 0x26);    
  /* PTGD: PTGD1=0,PTGD0=0 */
  clrReg8Bits(PTGD, 0x03);              
  /* PTGPE: PTGPE1=0,PTGPE0=0 */
  clrReg8Bits(PTGPE, 0x03);             
  /* PTGDD: PTGDD1=1,PTGDD0=1 */
  setReg8Bits(PTGDD, 0x03);             
  /* PTDD: PTDD7=0,PTDD4=0,PTDD0=0 */
  clrReg8Bits(PTDD, 0x91);              
  /* PTDDD: PTDDD7=1,PTDDD4=1,PTDDD3=0,PTDDD2=0,PTDDD0=1 */
  clrSetReg8Bits(PTDDD, 0x0C, 0x91);    
  /* PTAPE: PTAPE7=0,PTAPE6=0,PTAPE3=0,PTAPE2=0,PTAPE1=0,PTAPE0=0 */
  clrReg8Bits(PTAPE, 0xCF);             
  /* PTAD: PTAD7=0,PTAD6=0,PTAD3=0,PTAD0=0 */
  clrReg8Bits(PTAD, 0xC9);              
  /* PTADD: PTADD7=1,PTADD6=1,PTADD3=1,PTADD2=0,PTADD1=0,PTADD0=1 */
  clrSetReg8Bits(PTADD, 0x06, 0xC9);    
  /* PTBPE: PTBPE5=0 */
  clrReg8Bits(PTBPE, 0x20);             
  /* PTFD: PTFD1=0,PTFD0=0 */
  clrReg8Bits(PTFD, 0x03);              
  /* PTFPE: PTFPE1=0,PTFPE0=0 */
  clrReg8Bits(PTFPE, 0x03);             
  /* PTFDD: PTFDD1=1,PTFDD0=1 */
  setReg8Bits(PTFDD, 0x03);             
  /* PTASE: PTASE7=0,PTASE6=0,PTASE4=0,PTASE3=0,PTASE2=0,PTASE1=0,PTASE0=0 */
  clrReg8Bits(PTASE, 0xDF);             
  /* PTBSE: PTBSE7=0,PTBSE6=0,PTBSE5=0,PTBSE4=0,PTBSE3=0,PTBSE2=0,PTBSE1=0,PTBSE0=0 */
  setReg8(PTBSE, 0x00);                 
  /* PTCSE: PTCSE7=0,PTCSE6=0,PTCSE5=0,PTCSE4=0,PTCSE3=0,PTCSE2=0,PTCSE1=0,PTCSE0=0 */
  setReg8(PTCSE, 0x00);                 
  /* PTDSE: PTDSE7=0,PTDSE6=0,PTDSE5=0,PTDSE4=0,PTDSE3=0,PTDSE2=0,PTDSE1=0,PTDSE0=0 */
  setReg8(PTDSE, 0x00);                 
  /* PTESE: PTESE7=1,PTESE6=1,PTESE5=1,PTESE4=0,PTESE3=0,PTESE2=0,PTESE1=0,PTESE0=0 */
  setReg8(PTESE, 0xE0);                 
  /* PTFSE: PTFSE7=0,PTFSE6=0,PTFSE5=0,PTFSE4=0,PTFSE3=0,PTFSE2=0,PTFSE1=0,PTFSE0=0 */
  setReg8(PTFSE, 0x00);                 
  /* PTGSE: PTGSE7=0,PTGSE6=0,PTGSE5=0,PTGSE4=0,PTGSE3=0,PTGSE2=0,PTGSE1=0,PTGSE0=0 */
  setReg8(PTGSE, 0x00);                 
  /* PTHSE: PTHSE7=0,PTHSE6=0,PTHSE5=0,PTHSE4=0,PTHSE3=0,PTHSE2=0,PTHSE1=0,PTHSE0=0 */
  setReg8(PTHSE, 0x00);                 
  /* PTJSE: PTJSE7=0,PTJSE6=0,PTJSE5=0,PTJSE4=0,PTJSE3=0,PTJSE2=0,PTJSE1=0,PTJSE0=0 */
  setReg8(PTJSE, 0x00);                 
  /* PTADS: PTADS7=0,PTADS6=0,PTADS5=0,PTADS4=0,PTADS3=0,PTADS2=0,PTADS1=0,PTADS0=0 */
  setReg8(PTADS, 0x00);                 
  /* PTBDS: PTBDS7=0,PTBDS6=0,PTBDS5=0,PTBDS4=0,PTBDS3=0,PTBDS2=0,PTBDS1=0,PTBDS0=0 */
  setReg8(PTBDS, 0x00);                 
  /* PTCDS: PTCDS7=0,PTCDS6=0,PTCDS5=0,PTCDS4=0,PTCDS3=0,PTCDS2=0,PTCDS1=0,PTCDS0=0 */
  setReg8(PTCDS, 0x00);                 
  /* PTDDS: PTDDS7=0,PTDDS6=0,PTDDS5=0,PTDDS4=0,PTDDS3=0,PTDDS2=0,PTDDS1=0,PTDDS0=0 */
  setReg8(PTDDS, 0x00);                 
  /* PTEDS: PTEDS7=0,PTEDS6=0,PTEDS5=0,PTEDS4=0,PTEDS3=0,PTEDS2=0,PTEDS1=0,PTEDS0=0 */
  setReg8(PTEDS, 0x00);                 
  /* PTFDS: PTFDS7=0,PTFDS6=0,PTFDS5=0,PTFDS4=0,PTFDS3=0,PTFDS2=0,PTFDS1=0,PTFDS0=0 */
  setReg8(PTFDS, 0x00);                 
  /* PTGDS: PTGDS7=0,PTGDS6=0,PTGDS5=0,PTGDS4=0,PTGDS3=0,PTGDS2=0,PTGDS1=0,PTGDS0=0 */
  setReg8(PTGDS, 0x00);                 
  /* PTHDS: PTHDS7=0,PTHDS6=0,PTHDS5=0,PTHDS4=0,PTHDS3=0,PTHDS2=0,PTHDS1=0,PTHDS0=0 */
  setReg8(PTHDS, 0x00);                 
  /* PTJDS: PTJDS7=0,PTJDS6=0,PTJDS5=0,PTJDS4=0,PTJDS3=0,PTJDS2=0,PTJDS1=0,PTJDS0=0 */
  setReg8(PTJDS, 0x00);                 
  /* SCGC1: TPM3=1,TPM2=0,TPM1=1,ADC=1,IIC2=1,IIC1=0,SCI2=1,SCI1=1 */
  setReg8(SCGC1, 0xBB);                 
  /* ### Shared modules init code ... */
  /* ###  "hwAdc" init code ... */
  hwAdc_Init();
  /* ### ByteIO "hwBusData" init code ... */
  /* ### BitsIO "hwBusKeypad" init code ... */
  /* ### BitIO "hwBusLatch1" init code ... */
  /* ### BitIO "hwBusLatch2" init code ... */
  /* ### BitIO "hwBusLatch3" init code ... */
  /* ### BitIO "hwBusLatchReset" init code ... */
  /* ### Init_KBI "hwCts" init code ... */
  hwCts_Init();
  /* ### Asynchro serial "hwExpIn" init code ... */
  hwExpIn_Init();
  /* ### BitIO "hwExpInRts" init code ... */
  /* ### Asynchro serial "hwExpOut" init code ... */
  hwExpOut_Init();
  /* ### BitIO "hwExpOutRts" init code ... */
  /* ### InternalI2C "hwI2c" init code ... */
  hwI2c_Init();
  /* ### BitsIO "hwLcdEnb" init code ... */
  /* ### BitIO "hwLcdRs" init code ... */
  /* ### BitIO "hwLcdRw" init code ... */
  /* ### BitsIO "hwLed" init code ... */
  /* ### BitsIO "hwNav" init code ... */
  /* ### BitIO "hwPower12vdcStatus" init code ... */
  /* ### BitIO "hwPower24vacStatus" init code ... */
  /* ### BitIO "hwPower24vacControl" init code ... */
  /* ### BitIO "hwRadioDtr" init code ... */
  /* ### BitIO "hwRadioReset" init code ... */
  /* ### BitIO "hwRadioRts" init code ... */
  /* ### TimerInt "hwRtc" init code ... */
  hwRtc_Init();
  /* ###  Synchro master "hwSpi" init code ... */
  hwSpi_Init();
  /* ### BitIO "hwSpiSS" init code ... */
  /* ### TimerInt "hwTimerKeypad" init code ... */
  hwTimerKeypad_Init();
  /* ### TimerInt "hwTimerNav" init code ... */
  hwTimerNav_Init();
  /* ### BitIO "hwUart1Select" init code ... */
  /* ###  WatchDog "hwWatchDog" init code ... */
  SRS = 0x00;
  /* ### BitIO "hwUnusedA0" init code ... */
  /* ### BitIO "hwUnusedA3" init code ... */
  /* ### BitIO "hwUnusedA6" init code ... */
  /* ### BitIO "hwUnusedA7" init code ... */
  /* ### BitIO "hwUnusedC5" init code ... */
  /* ### BitIO "hwUnusedD0" init code ... */
  /* ### BitIO "hwUnusedD4" init code ... */
  /* ### BitIO "hwUnusedD7" init code ... */
  /* ### BitIO "hwUnusedF0" init code ... */
  /* ### BitIO "hwUnusedF1" init code ... */
  /* INTC_WCR: ENB=1,??=0,??=0,??=0,??=0,MASK=0 */
  setReg8(INTC_WCR, 0x80);              
  /* Set initial interrupt priority 0 */
  asm {
    move.w SR,D0;
    andi.l #0xF8FF,D0;
    move.w D0,SR;
  }
}

/* Initialization of the CPU registers in FLASH */

/* NVPROT: FPS6=1,FPS5=1,FPS4=1,FPS3=1,FPS2=1,FPS1=1,FPS0=1,FPOPEN=1 */
unsigned char NVPROT_INIT @0x0000040D = 0xFF;

/* NVOPT: KEYEN1=0,KEYEN0=1,??=1,??=1,??=1,??=1,SEC1=1,SEC0=1 */
unsigned char NVOPT_INIT @0x0000040F = 0x7F;
/* END hwCpu. */

/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
