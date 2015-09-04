/** ###################################################################
**     THIS BEAN MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename  : Vectors.C
**     Project   : WOIS
**     Processor : MCF51QE128CLK
**     Version   : Bean 01.011, Driver 01.03, CPU db: 3.00.052
**     Compiler  : CodeWarrior ColdFireV1 C Compiler
**     Date/Time : 1/14/2011, 3:27 PM
**     Abstract  :
**         This bean "MCF51QE128_80" contains initialization of the
**         CPU and provides basic methods and events for CPU core
**         settings.
**     Comment   :
**         This is the 80-pin QE128 CPU configuration used by the Debug configuration.  The primary difference from the Release 
**         configuration is that the BDM port is enabled.
**     Settings  :
**
**
**     (c) Copyright UNIS, spol. s r.o. 1997-2008
**     UNIS, spol. s r.o.
**     Jundrovska 33
**     624 00 Brno
**     Czech Republic
**     http      : www.processorexpert.com
**     mail      : info@processorexpert.com
** ###################################################################*/
#include "hwCpu.h"
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
#include "startcf.h"

extern unsigned long far _SP_INIT[];

/* ISR prototype */
typedef void (*const tIsrFunc)(void);

const tIsrFunc _InterruptVectorTable[103] @0x00000000 = { /* Interrupt vector table */
  /* ISR name                               No. Address      Lvl Pri Name          Description */
  (tIsrFunc)_SP_INIT,                  /* 0x00  0x00000000   -   -   ivINITSP      used by PE */
  _startup,                            /* 0x01  0x00000004   -   -   ivINITPC      used by PE */
  hwCpu_Interrupt,                     /* 0x02  0x00000008   -   -   ivVaccerr     unused by PE */
  hwCpu_Interrupt,                     /* 0x03  0x0000000C   -   -   ivVadderr     unused by PE */
  hwCpu_Interrupt,                     /* 0x04  0x00000010   -   -   ivViinstr     unused by PE */
  hwCpu_Interrupt,                     /* 0x05  0x00000014   -   -   ivVReserved5  unused by PE */
  hwCpu_Interrupt,                     /* 0x06  0x00000018   -   -   ivVReserved6  unused by PE */
  hwCpu_Interrupt,                     /* 0x07  0x0000001C   -   -   ivVReserved7  unused by PE */
  hwCpu_Interrupt,                     /* 0x08  0x00000020   -   -   ivVprviol     unused by PE */
  hwCpu_Interrupt,                     /* 0x09  0x00000024   -   -   ivVtrace      unused by PE */
  hwCpu_Interrupt,                     /* 0x0A  0x00000028   -   -   ivVunilaop    unused by PE */
  hwCpu_Interrupt,                     /* 0x0B  0x0000002C   -   -   ivVunilfop    unused by PE */
  hwCpu_Interrupt,                     /* 0x0C  0x00000030   -   -   ivVdbgi       unused by PE */
  hwCpu_Interrupt,                     /* 0x0D  0x00000034   -   -   ivVReserved13 unused by PE */
  hwCpu_Interrupt,                     /* 0x0E  0x00000038   -   -   ivVferror     unused by PE */
  hwCpu_Interrupt,                     /* 0x0F  0x0000003C   -   -   ivVReserved15 unused by PE */
  hwCpu_Interrupt,                     /* 0x10  0x00000040   -   -   ivVReserved16 unused by PE */
  hwCpu_Interrupt,                     /* 0x11  0x00000044   -   -   ivVReserved17 unused by PE */
  hwCpu_Interrupt,                     /* 0x12  0x00000048   -   -   ivVReserved18 unused by PE */
  hwCpu_Interrupt,                     /* 0x13  0x0000004C   -   -   ivVReserved19 unused by PE */
  hwCpu_Interrupt,                     /* 0x14  0x00000050   -   -   ivVReserved20 unused by PE */
  hwCpu_Interrupt,                     /* 0x15  0x00000054   -   -   ivVReserved21 unused by PE */
  hwCpu_Interrupt,                     /* 0x16  0x00000058   -   -   ivVReserved22 unused by PE */
  hwCpu_Interrupt,                     /* 0x17  0x0000005C   -   -   ivVReserved23 unused by PE */
  hwCpu_Interrupt,                     /* 0x18  0x00000060   -   -   ivVspuri      unused by PE */
  hwCpu_Interrupt,                     /* 0x19  0x00000064   -   -   ivVReserved25 unused by PE */
  hwCpu_Interrupt,                     /* 0x1A  0x00000068   -   -   ivVReserved26 unused by PE */
  hwCpu_Interrupt,                     /* 0x1B  0x0000006C   -   -   ivVReserved27 unused by PE */
  hwCpu_Interrupt,                     /* 0x1C  0x00000070   -   -   ivVReserved28 unused by PE */
  hwCpu_Interrupt,                     /* 0x1D  0x00000074   -   -   ivVReserved29 unused by PE */
  hwCpu_Interrupt,                     /* 0x1E  0x00000078   -   -   ivVReserved30 unused by PE */
  hwCpu_Interrupt,                     /* 0x1F  0x0000007C   -   -   ivVReserved31 unused by PE */
  hwCpu_Interrupt,                     /* 0x20  0x00000080   -   -   ivVtrap0      unused by PE */
  hwCpu_Interrupt,                     /* 0x21  0x00000084   -   -   ivVtrap1      unused by PE */
  hwCpu_Interrupt,                     /* 0x22  0x00000088   -   -   ivVtrap2      unused by PE */
  hwCpu_Interrupt,                     /* 0x23  0x0000008C   -   -   ivVtrap3      unused by PE */
  hwCpu_Interrupt,                     /* 0x24  0x00000090   -   -   ivVtrap4      unused by PE */
  hwCpu_Interrupt,                     /* 0x25  0x00000094   -   -   ivVtrap5      unused by PE */
  hwCpu_Interrupt,                     /* 0x26  0x00000098   -   -   ivVtrap6      unused by PE */
  hwCpu_Interrupt,                     /* 0x27  0x0000009C   -   -   ivVtrap7      unused by PE */
  hwCpu_Interrupt,                     /* 0x28  0x000000A0   -   -   ivVtrap8      unused by PE */
  hwCpu_Interrupt,                     /* 0x29  0x000000A4   -   -   ivVtrap9      unused by PE */
  hwCpu_Interrupt,                     /* 0x2A  0x000000A8   -   -   ivVtrap10     unused by PE */
  hwCpu_Interrupt,                     /* 0x2B  0x000000AC   -   -   ivVtrap11     unused by PE */
  hwCpu_Interrupt,                     /* 0x2C  0x000000B0   -   -   ivVtrap12     unused by PE */
  hwCpu_Interrupt,                     /* 0x2D  0x000000B4   -   -   ivVtrap13     unused by PE */
  hwCpu_Interrupt,                     /* 0x2E  0x000000B8   -   -   ivVtrap14     unused by PE */
  hwCpu_Interrupt,                     /* 0x2F  0x000000BC   -   -   ivVtrap15     unused by PE */
  hwCpu_Interrupt,                     /* 0x30  0x000000C0   -   -   ivVReserved48 unused by PE */
  hwCpu_Interrupt,                     /* 0x31  0x000000C4   -   -   ivVReserved49 unused by PE */
  hwCpu_Interrupt,                     /* 0x32  0x000000C8   -   -   ivVReserved50 unused by PE */
  hwCpu_Interrupt,                     /* 0x33  0x000000CC   -   -   ivVReserved51 unused by PE */
  hwCpu_Interrupt,                     /* 0x34  0x000000D0   -   -   ivVReserved52 unused by PE */
  hwCpu_Interrupt,                     /* 0x35  0x000000D4   -   -   ivVReserved53 unused by PE */
  hwCpu_Interrupt,                     /* 0x36  0x000000D8   -   -   ivVReserved54 unused by PE */
  hwCpu_Interrupt,                     /* 0x37  0x000000DC   -   -   ivVReserved55 unused by PE */
  hwCpu_Interrupt,                     /* 0x38  0x000000E0   -   -   ivVReserved56 unused by PE */
  hwCpu_Interrupt,                     /* 0x39  0x000000E4   -   -   ivVReserved57 unused by PE */
  hwCpu_Interrupt,                     /* 0x3A  0x000000E8   -   -   ivVReserved58 unused by PE */
  hwCpu_Interrupt,                     /* 0x3B  0x000000EC   -   -   ivVReserved59 unused by PE */
  hwCpu_Interrupt,                     /* 0x3C  0x000000F0   -   -   ivVReserved60 unused by PE */
  hwCpu_Interrupt,                     /* 0x3D  0x000000F4   -   -   ivVunsinstr   unused by PE */
  hwCpu_Interrupt,                     /* 0x3E  0x000000F8   -   -   ivVReserved62 unused by PE */
  hwCpu_Interrupt,                     /* 0x3F  0x000000FC   -   -   ivVReserved63 unused by PE */
  hwCpu_Interrupt,                     /* 0x40  0x00000100   -   -   ivVirq        unused by PE */
  hwCpu_Interrupt,                     /* 0x41  0x00000104   -   -   ivVlvd        unused by PE */
  hwTimerNav_Interrupt,                /* 0x42  0x00000108   6   5   ivVtpm1ch0    used by PE */
  hwCpu_Interrupt,                     /* 0x43  0x0000010C   -   -   ivVtpm1ch1    unused by PE */
  hwCpu_Interrupt,                     /* 0x44  0x00000110   -   -   ivVtpm1ch2    unused by PE */
  hwCpu_Interrupt,                     /* 0x45  0x00000114   -   -   ivVtpm1ovf    unused by PE */
  hwCpu_Interrupt,                     /* 0x46  0x00000118   -   -   ivVtpm2ch0    unused by PE */
  hwCpu_Interrupt,                     /* 0x47  0x0000011C   -   -   ivVtpm2ch1    unused by PE */
  hwCpu_Interrupt,                     /* 0x48  0x00000120   -   -   ivVtpm2ch2    unused by PE */
  hwCpu_Interrupt,                     /* 0x49  0x00000124   -   -   ivVtpm2ovf    unused by PE */
  hwCpu_Interrupt,                     /* 0x4A  0x00000128   -   -   ivVspi2       unused by PE */
  hwCpu_Interrupt,                     /* 0x4B  0x0000012C   -   -   ivVspi1       unused by PE */
  hwCpu_Interrupt,                     /* 0x4C  0x00000130   -   -   ivVsci1err    unused by PE */
  hwCpu_Interrupt,                     /* 0x4D  0x00000134   -   -   ivVsci1rx     unused by PE */
  hwCpu_Interrupt,                     /* 0x4E  0x00000138   -   -   ivVsci1tx     unused by PE */
  hwCpu_Interrupt,                     /* 0x4F  0x0000013C   -   -   ivViicx       unused by PE */
  drvCtsISR,                           /* 0x50  0x00000140   3   6   ivVkeyboard   used by PE */
  hwCpu_Interrupt,                     /* 0x51  0x00000144   -   -   ivVadc        unused by PE */
  hwCpu_Interrupt,                     /* 0x52  0x00000148   -   -   ivVacmpx      unused by PE */
  hwExpIn_InterruptError,              /* 0x53  0x0000014C   2   5   ivVsci2err    used by PE */
  hwExpIn_InterruptRx,                 /* 0x54  0x00000150   2   4   ivVsci2rx     used by PE */
  hwExpIn_InterruptTx,                 /* 0x55  0x00000154   2   3   ivVsci2tx     used by PE */
  hwRtc_Interrupt,                     /* 0x56  0x00000158   2   2   ivVrtc        used by PE */
  hwTimerKeypad_Interrupt,             /* 0x57  0x0000015C   1   7   ivVtpm3ch0    used by PE */
  hwCpu_Interrupt,                     /* 0x58  0x00000160   -   -   ivVtpm3ch1    unused by PE */
  hwCpu_Interrupt,                     /* 0x59  0x00000164   -   -   ivVtpm3ch2    unused by PE */
  hwCpu_Interrupt,                     /* 0x5A  0x00000168   -   -   ivVtpm3ch3    unused by PE */
  hwCpu_Interrupt,                     /* 0x5B  0x0000016C   -   -   ivVtpm3ch4    unused by PE */
  hwCpu_Interrupt,                     /* 0x5C  0x00000170   -   -   ivVtpm3ch5    unused by PE */
  hwCpu_Interrupt,                     /* 0x5D  0x00000174   -   -   ivVtpm3ovf    unused by PE */
  hwCpu_Interrupt,                     /* 0x5E  0x00000178   -   -   ivVReserved94 unused by PE */
  hwCpu_Interrupt,                     /* 0x5F  0x0000017C   -   -   ivVReserved95 unused by PE */
  hwCpu_Interrupt,                     /* 0x60  0x00000180   -   -   ivVL7swi      unused by PE */
  hwCpu_Interrupt,                     /* 0x61  0x00000184   -   -   ivVL6swi      unused by PE */
  hwCpu_Interrupt,                     /* 0x62  0x00000188   -   -   ivVL5swi      unused by PE */
  hwCpu_Interrupt,                     /* 0x63  0x0000018C   -   -   ivVL4swi      unused by PE */
  hwCpu_Interrupt,                     /* 0x64  0x00000190   -   -   ivVL3swi      unused by PE */
  hwCpu_Interrupt,                     /* 0x65  0x00000194   -   -   ivVL2swi      unused by PE */
  hwCpu_Interrupt                      /* 0x66  0x00000198   -   -   ivVL1swi      unused by PE */
};
/*
** ###################################################################
**
**     This file was created by UNIS Processor Expert 3.03 [04.07]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
