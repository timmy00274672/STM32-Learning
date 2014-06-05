/******************************************************************************
 * @file:    system_LM3S6965.c
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Source File for the
 *           Luminary 'LM3S' Device Series 
 * @version: V0.01 (Preliminary)
 * @date:    22. Jan. 2009
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
#include "LM3S6965.h"


//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
/*--------------------- Clock Configuration ----------------------------------
//
// <e> Clock Configuration
//   <h> Run-Mode Clock Configuration (RCC)
//     <o1.23..26> SYSDIV: System Clock Divisor <2-16><#-1>
//                   <i> Specifies which divisor is used to generate the system clock from the
//                   <i> PLL output (PLL VCO frequency is 400 MHz).
//     <o1.22>     USESYSDIV: Enable System Clock Devider
//     <o1.20>     USEPWMDIV: Enable PWM Clock Devider
//     <o1.17..19> PWMDIV: PWM Unit Clock Divisor
//                     <0=> 0: SysClk / 2
//                     <1=> 1: SysClk / 4
//                     <2=> 2: SysClk / 8
//                     <3=> 3: SysClk / 16
//                     <4=> 4: SysClk / 32
//                     <5=> 5: SysClk / 64
//                     <6=> 6: SysClk / 64
//                     <7=> 7: SysClk / 64 (default)
//     <o1.13>     PWRDN: PLL Power Down
//     <o1.11>     BYPASS: PLL Bypass
//     <o1.6..9>   XTAL: Crystal Value
//                     < 0=>  0: 1.0000 MHz  (not using PLL else reserved)
//                     < 1=>  1: 1.8432 MHz  (not using PLL else reserved)
//                     < 2=>  2: 2.0000 MHz  (not using PLL else reserved)
//                     < 3=>  3: 2.4576 MHz  (not using PLL else reserved)
//                     < 4=>  4: 3.579545 MHz
//                     < 5=>  5: 3.6864 MHz
//                     < 6=>  6: 4.0000 MHz
//                     < 7=>  7: 4.096 MHz
//                     < 8=>  8: 4.9152 MHz
//                     < 9=>  9: 5.0000 MHz
//                     <10=> 10: 5.12 MHz
//                     <11=> 11: 6.0000 MHz (default)
//                     <12=> 12: 6.144 MHz
//                     <13=> 13: 7.3728 MHz
//                     <14=> 14: 8.0000 MHz
//                     <15=> 15: 8.192 MHz
//     <o1.4..5>   OSCSRC: Oscillator Source
//                     <0=> 0: MOSC Main oscillator
//                     <1=> 1: IOSC Internal oscillator (default)
//                     <2=> 2: IOSC/4 Internal oscillator / 4 (this is necessary if used as input to PLL)
//                     <3=> 3: 30kHz 30-KHz internal oscillator
//     <o1.1>      IOSCDIS: Internal Oscillator Disable
//     <o1.0>      MOSCDIS: Main Oscillator Disable
//   </h>
//
//   <h> Run-Mode Clock Configuration 2 (RCC2)
//     <o2.31>     USERCC2: Use RCC2
//     <o2.23..28> SYSDIV2: System Clock Divisor <2-64><#-1>
//                   <i> Specifies which divisor is used to generate the system clock from the
//                   <i> PLL output (PLL VCO frequency is 400 MHz).
//     <o2.13>     PWRDN2: Power Down PLL
//     <o2.11>     BYPASS2: Bypass PLL
//     <o2.4..6>   OSCSRC: Oscillator Source
//                     <0=> 0: MOSC Main oscillator
//                     <1=> 1: IOSC Internal oscillator (default)
//                     <2=> 2: IOSC/4 Internal oscillator / 4 (this is necessary if used as input to PLL)
//                     <3=> 3: 30kHz 30-kHz internal oscillator
//                     <7=> 7: 32kHz 32.768-kHz external oscillator
//   </h>
//
// </e>
*/
#define CLOCK_SETUP           1
#define RCC_Val               0x018E3B90
#define RCC2_Val              0x03800000

//-------- <<< end of configuration section >>> ------------------------------


/*----------------------------------------------------------------------------
  Local functions
 *----------------------------------------------------------------------------*/
__inline static uint32_t getOscClk (uint32_t xtal, uint32_t oscSrc);
    
/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define XTALM       ( 6000000UL)            /* Main         oscillator freq */
#define XTALI       (12000000UL)            /* Internal     oscillator freq */
#define XTAL30K     (   30000UL)            /* Internal 30K oscillator freq */
#define XTAL32K     (   32768UL)            /* external 32K oscillator freq */

#define PLL_CLK    (400000000UL)
#define ADC_CLK     (PLL_CLK/25)
#define CAN_CLK     (PLL_CLK/50)


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency         = XTALI;   /*!< System Clock Frequency (Core Clock) */
uint32_t SystemFrequency_PwmClk  = XTALI;   /*!< PWM clock                           */
uint32_t SystemFrequency_AdcClk  = ADC_CLK; /*!< ADC clock                           */
uint32_t SystemFrequency_CanClk  = CAN_CLK; /*!< CAN clock                           */

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
void SystemInit (void) {
#if (CLOCK_SETUP)                       /* Clock Setup                        */
  uint32_t i;
  uint32_t rcc, rcc2;

  rcc  = RCC_Val;  // TEST
  rcc2 = RCC2_Val; // TEST


  SYSCTL->RCC2 = 0x07802810;    /* set default value */
  SYSCTL->RCC  = 0x078E3AD1;    /* set default value */ 

  SYSCTL->RCC  = (RCC_Val  | (1UL<<11) | (1UL<<13)) & ~(1UL<<22); /* set value with BYPASS, PWRDN set, USESYSDIV reset */
  SYSCTL->RCC2 = (RCC2_Val | (1UL<<11) | (1UL<<13));              /* set value with BYPASS, PWRDN set */
  for (i = 0; i < 1000; i++);   /* wait a while */

  SYSCTL->RCC  = (RCC_Val  | (1UL<<11)) & ~(1UL<<22);             /* set value with BYPASS, USESYSDIV reset */
  SYSCTL->RCC2 = (RCC2_Val | (1UL<<11));                          /* set value with BYPASS */
  for (i = 0; i < 1000; i++);   /* wait a while */

  SYSCTL->RCC  = (RCC_Val  | (1<<11));                            /* set value with BYPASS */

  if ( (((RCC_Val  & (1UL<<13)) == 0) && ((RCC2_Val & (1UL<<31)) == 0)) || 
       (((RCC2_Val & (1UL<<13)) == 0) && ((RCC2_Val & (1UL<<31)) != 0))   ) {
    while ((SYSCTL->RIS & (1UL<<6)) != (1UL<<6));                 /* wait until PLL is locked */
  }

  SYSCTL->RCC  = (RCC_Val);                                       /* set value */
  SYSCTL->RCC2 = (RCC2_Val);                                      /* set value */
  for (i = 0; i < 10000; i++);   /* wait a while */

#endif

  /* Determine clock frequency according to clock register values */
  rcc  = SYSCTL->RCC;
  rcc2 = SYSCTL->RCC2;

  if (rcc2 & (1UL<<31)) {                             /* is rcc2 is used ? */
    if (rcc2 & (1UL<<11)) {                           /* check BYPASS */
      SystemFrequency = getOscClk (((rcc>>6) & 0x0F),((rcc2>>4) & 0x07));
    } else {
      SystemFrequency = PLL_CLK;
    }
    if (rcc & (1UL<<22)) {                            /* check USESYSDIV */
      if (rcc2 & (1UL<<11)) {
        SystemFrequency = SystemFrequency / (((rcc2>>23) & (0x3F)) + 1);
      } else {
        SystemFrequency = SystemFrequency / (((rcc2>>23) & (0x3F)) + 1) / 2;
      }
    }
  } else {
    if (RCC_Val & (1UL<<11)) {                            /* check BYPASS */
//    if (rcc & (1UL<<11)) {                            /* check BYPASS */ /* Simulation does not work at theis point */
      SystemFrequency = getOscClk (((rcc>>6) & 0x0F),((rcc>>4) & 0x03));
    } else {
      SystemFrequency = PLL_CLK;
    }
    if (rcc & (1UL<<22)) {                            /* check USESYSDIV */
//      if (rcc & (1UL<<11)) {                          /* check BYPASS */ /* Simulation does not work at theis point */
      if (RCC_Val & (1UL<<11)) {                          /* check BYPASS */
        SystemFrequency = SystemFrequency / (((rcc>>23) & (0x0F)) + 1);
      } else {
        SystemFrequency = SystemFrequency / (((rcc>>23) & (0x0F)) + 1) / 2;
      }
    }
  }

}

/*----------------------------------------------------------------------------
  Get the OSC clock
 *----------------------------------------------------------------------------*/
__inline static uint32_t getOscClk (uint32_t xtal, uint32_t oscSrc) {
  uint32_t oscClk;
   
  switch (oscSrc) {                      /* switch OSCSRC */
    case 0:                              /* MOSC Main oscillator */
      switch (xtal) {                    /* switch XTAL */
        case 0x0:
          oscClk = 1000000UL; 
          break;
        case 0x1:
          oscClk = 1843200UL; 
          break;
        case 0x2:
          oscClk = 2000000UL; 
          break;
        case 0x3:
          oscClk = 2457600UL; 
          break;
        case 0x4:
          oscClk = 3579545UL; 
          break;
        case 0x5:
          oscClk = 3686400UL; 
          break;
        case 0x6:
          oscClk = 4000000UL; 
          break;
        case 0x7:
          oscClk = 4096000UL; 
          break;
        case 0x8:
          oscClk = 4915200UL; 
          break;
        case 0x9:
          oscClk = 5000000UL; 
          break;
        case 0xA:
          oscClk = 5120000UL; 
          break;
        case 0xB:
          oscClk = 6000000UL; 
          break;
        case 0xC:
          oscClk = 6144000UL; 
          break;
        case 0xD:
          oscClk = 7372800UL; 
          break;
        case 0xE:
          oscClk = 8000000UL; 
          break;
        case 0xF:
          oscClk = 8192000UL; 
          break;
       } 
      break;
    case 1:                         /* IOSC Internal oscillator */
      oscClk = XTALI;
      break;
    case 2:                         /* IOSC/4 Internal oscillator/4 */
      oscClk = XTALI/4;
      break;
    case 3:                         /* 30kHz internal oscillator  */
      oscClk = XTAL30K;
      break;
  }

  return oscClk;
}

