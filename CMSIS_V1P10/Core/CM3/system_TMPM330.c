/******************************************************************************
 * @file:    system_TMPM330.c
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Source File for the
 *           Toshiba 'TMPM330' Device Series 
 * @version: V1.0
 * @date:    16. Dec. 2008
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#include <stdint.h>
#include "TMPM330.h"


//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------


/*--------------------- Watchdog Configuration -------------------------------
//
// <e> Watchdog Configuration
//   <o1.7>      WDTE: Watchdog Enable
//   <o1.4..7>   WDTP: Watchdog Detection Time
//                     <0=> 2^16 / fSYS
//                     <1=> 2^18 / fSYS
//                     <2=> 2^20 / fSYS
//                     <3=> 2^22 / fSYS
//                     <4=> 2^24 / fSYS
//                     <5=> 2^26 / fSYS
//   <o1.2>      I2WDT: IDLE Start
//   <o1.1>      RESCR: Watchdog Timer out Control
//                     <0=> Generates NMI interrupt
//                     <1=> Connects WDT out to reset
// </e>
*/
#define WDT_SETUP             1
#define WDTMOD_Val            0x00000000


/*--------------------- Clock Configuration ----------------------------------
//
// <e> Clock Configuration
//   <h> System Control Register (SYSCR)
//     <o1.16..17> SCOSEL: SCOUT pin output
//                     <0=> 0: fs
//                     <1=> 1: Reserved
//                     <2=> 2: fsys
//                     <3=> 3: phiT0
//     <o1.12>     FPSEL: fperiph source clock
//                     <0=> 0: fgear
//                     <1=> 1: fc
//     <o1.8..10>  PRCK: Prescaler clock
//                     <0=> 0: fperiph
//                     <1=> 1: fperiph/2
//                     <2=> 2: fperiph/4
//                     <3=> 3: fperiph/8
//                     <4=> 4: fperiph/16
//                     <5=> 5: fperiph/32
//                     <6=> 6: Reserved
//                     <7=> 7: Reserved
//     <o1.0..2>   GEAR: High-speed clock (fc) gear
//                     <0=> 0: fc
//                     <1=> 1: Reserved
//                     <2=> 2: Reserved
//                     <3=> 3: Reserved
//                     <4=> 4: fc/2
//                     <5=> 5: fc/4
//                     <6=> 6: fc/8
//                     <7=> 7: Reserved
//   </h>
//   <h> Oscillation Control Register (OSCCR)
//     <o2.9>      XTEN: Low-speed oscillator enable
//     <o2.8>      XEN: High-speed oscillator enable
//     <o2.4..6>   WUPT: Warm-up timer for oscillator
//                     <0=> 0: No warm-up
//                     <1=> 1: X1:  1024/inp freq | XT1:     64/inp freq 
//                     <2=> 2: X1:  2048/inp freq | XT1:    128/inp freq
//                     <3=> 3: X1:  4096/inp freq | XT1:    256/inp freq
//                     <4=> 4: X1:  8192/inp freq | XT1:  32768/inp freq
//                     <5=> 5: X1: 16384/inp freq | XT1:  65536/inp freq
//                     <6=> 6: X1: 32768/inp freq | XT1: 131072/inp freq
//                     <7=> 7: X1: 65536/inp freq | XT1: 262144/inp freq
//     <o2.3>      WUPSEL: Warm-up counter
//                     <0=> 0: X1
//                     <1=> 1: XT1
//     <o2.2>      PLLON: PLL enable
//     <o2.0>      WUEON: Warm-up timer enable
//   </h>
//   <h> Standby Control Register (STBYCR)
//     <o3.16>     DRVE: Pin status in STOP mode
//                     <0=> 0: Active
//                     <1=> 1: Inactive
//     <o3.9>      RXTEN: Low-speed oscillator after releasing STOP mode enable
//     <o3.8>      RXEN: High-speed oscillator after releasing STOP mode enable
//     <o3.0..2>   STBY: Low power consumption mode
//                     <0=> 0: Reserved
//                     <1=> 1: STOP 
//                     <2=> 2: SLEEP
//                     <3=> 3: IDLE
//                     <4=> 4: Reserved
//                     <5=> 5: Reserved
//                     <6=> 6: Reserved
//                     <7=> 7: Reserved
//   </h>
//   <h> PLL Selection Register (PLLSEL)
//     <o4.0>      PLLSEL: PLL enable
//   </h>
//   <h> System Clock Selection Register (CKSEL)
//     <o5.1>      SYSCK: System clock
//                     <0=> 0: High-speed (fc)
//                     <1=> 1: Low-speed (fs)
//   </h>
// </e>
*/
#define CLOCK_SETUP           1
#define SYSCR_Val             0x00010000
#define OSCCR_Val             0x00000314
#define STBYCR_Val            0x00000103
#define PLLSEL_Val            0x00000001
#define CKSEL_Val             0x00000000


//-------- <<< end of configuration section >>> ------------------------------


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
    
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define XTALH       (10000000UL)        /* External high-speed oscillator freq*/
#define XTALL       (   32768UL)        /* External low-speed oscillator freq */


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency = XTALH;   /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
void SystemInit (void)
{
#if (WDT_SETUP)                         /* Watchdog Setup                     */
  WD->MOD  = WDTMOD_Val;
  if (!(WDTMOD_Val & (1 << 7))) {       /* If watchdog is to be disabled      */
    WD->CR = 0xB1;
  }
#endif

#if (CLOCK_SETUP)                       /* Clock Setup                        */
  CG->SYSCR  = SYSCR_Val;
  CG->OSCCR  = OSCCR_Val;
  CG->STBYCR = STBYCR_Val;
  CG->PLLSEL = PLLSEL_Val;
  CG->CKSEL  = CKSEL_Val;
#endif

  /* Determine clock frequency according to clock register values             */
  if (CG->CKSEL & (1 << 1)) {           /* If system clock is low-speed clock */
    SystemFrequency = XTALL;
  } else {                              /* If system clock is high-speed clock*/
    if ((CG->PLLSEL & (1 << 0)) &&      /* If PLL selected                    */
        (CG->OSCCR  & (1 << 2))  ) {    /* If PLL enabled                     */
      switch (CG->SYSCR & 7) {
        case 0:                         /* Gear -> fc                         */
          SystemFrequency = XTALH * 4;
          break;
        case 1:
        case 2:
        case 3:
        case 7:                         /* Gear -> reserved                   */
          SystemFrequency = XTALH;
          break;
        case 4:                         /* Gear -> fc/2                       */
          SystemFrequency = XTALH * 4 / 2;
          break;
        case 5:                         /* Gear -> fc/4                       */
          SystemFrequency = XTALH * 4 / 4;
          break;
        case 6:                         /* Gear -> fc/8                       */
          SystemFrequency = XTALH * 4 / 8;
          break;
      }
    } else {
      switch (CG->SYSCR & 7) {
        case 0:                         /* Gear -> fc                         */
          SystemFrequency = XTALH;
          break;
        case 1:
        case 2:
        case 3:
        case 7:                         /* Gear -> reserved                   */
          SystemFrequency = XTALH;
          break;
        case 4:                         /* Gear -> fc/2                       */
          SystemFrequency = XTALH / 2;
          break;
        case 5:                         /* Gear -> fc/4                       */
          SystemFrequency = XTALH / 4;
          break;
        case 6:                         /* Gear -> fc/8                       */
          SystemFrequency = XTALH / 8;
          break;
      }
    }                                   /* If PLL not used                    */
  }
}
