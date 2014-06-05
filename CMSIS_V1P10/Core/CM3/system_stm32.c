/******************************************************************************
 * @file:    system_stm32.c
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Source File
 * @version: V1.02
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
#include "stm32.h"

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
//=========================================================================== Clock Configuration
// <e0> Clock Configuration
//   <h> Clock Control Register Configuration (RCC_CR)
//     <e1.24> PLLON: PLL enable         
//       <i> Default: PLL Disabled
//       <o2.18..21> PLLMUL: PLL Multiplication Factor
//         <i> Default: PLLSRC * 2
//                       <0=> PLLSRC * 2
//                       <1=> PLLSRC * 3
//                       <2=> PLLSRC * 4
//                       <3=> PLLSRC * 5
//                       <4=> PLLSRC * 6
//                       <5=> PLLSRC * 7
//                       <6=> PLLSRC * 8
//                       <7=> PLLSRC * 9
//                       <8=> PLLSRC * 10
//                       <9=> PLLSRC * 11
//                       <10=> PLLSRC * 12
//                       <11=> PLLSRC * 13
//                       <12=> PLLSRC * 14
//                       <13=> PLLSRC * 15
//                       <14=> PLLSRC * 16
//       <o2.17> PLLXTPRE: HSE divider for PLL entry
//         <i> Default: HSE
//                       <0=> HSE
//                       <1=> HSE / 2
//       <o2.16> PLLSRC: PLL entry clock source         
//         <i> Default: HSI/2
//                       <0=> HSI / 2
//                       <1=> HSE (PLLXTPRE output)
//     </e>
//     <o1.19> CSSON: Clock Security System enable
//       <i> Default: Clock detector OFF
//     <o1.18> HSEBYP: External High Speed clock Bypass
//       <i> Default: HSE oscillator not bypassed
//     <o1.16> HSEON: External High Speed clock enable 
//       <i> Default: HSE oscillator OFF
//     <o1.3..7> HSITRIM: Internal High Speed clock trimming  <0-31>
//       <i> Default: 0
//     <o1.0> HSION: Internal High Speed clock enable
//       <i> Default: internal 8MHz RC oscillator OFF
//   </h>
//   <h> Clock Configuration Register Configuration (RCC_CFGR)
//     <o2.24..26> MCO: Microcontroller Clock Output   
//       <i> Default: MCO = noClock
//                     <0=> MCO = noClock
//                     <4=> MCO = SYSCLK
//                     <5=> MCO = HSI
//                     <6=> MCO = HSE
//                     <7=> MCO = PLLCLK / 2
//     <o2.22> USBPRE: USB prescaler
//       <i> Default: USBCLK = PLLCLK / 1.5
//                     <0=> USBCLK = PLLCLK / 1.5
//                     <1=> USBCLK = PLLCLK
//     <o2.14..15> ADCPRE: ADC prescaler
//       <i> Default: ADCCLK=PCLK2 / 2
//                     <0=> ADCCLK = PCLK2 / 2
//                     <1=> ADCCLK = PCLK2 / 4
//                     <2=> ADCCLK = PCLK2 / 6
//                     <3=> ADCCLK = PCLK2 / 8
//     <o2.11..13> PPRE2: APB High speed prescaler (APB2)
//       <i> Default: PCLK2 = HCLK
//                     <0=> PCLK2 = HCLK
//                     <4=> PCLK2 = HCLK / 2 
//                     <5=> PCLK2 = HCLK / 4 
//                     <6=> PCLK2 = HCLK / 8 
//                     <7=> PCLK2 = HCLK / 16 
//     <o2.8..10> PPRE1: APB Low speed prescaler (APB1) 
//       <i> Default: PCLK1 = HCLK
//                     <0=> PCLK1 = HCLK
//                     <4=> PCLK1 = HCLK / 2 
//                     <5=> PCLK1 = HCLK / 4 
//                     <6=> PCLK1 = HCLK / 8 
//                     <7=> PCLK1 = HCLK / 16 
//     <o2.4..7> HPRE: AHB prescaler 
//       <i> Default: HCLK = SYSCLK
//                     <0=> HCLK = SYSCLK
//                     <8=> HCLK = SYSCLK / 2
//                     <9=> HCLK = SYSCLK / 4
//                     <10=> HCLK = SYSCLK / 8
//                     <11=> HCLK = SYSCLK / 16
//                     <12=> HCLK = SYSCLK / 64
//                     <13=> HCLK = SYSCLK / 128
//                     <14=> HCLK = SYSCLK / 256
//                     <15=> HCLK = SYSCLK / 512
//     <o2.0..1> SW: System Clock Switch
//       <i> Default: SYSCLK = HSE
//                     <0=> SYSCLK = HSI
//                     <1=> SYSCLK = HSE
//                     <2=> SYSCLK = PLLCLK
//   </h>
//   <o3>HSE: External High Speed Clock [Hz] <4000000-16000000>
//   <i> clock value for the used External High Speed Clock (4MHz <= HSE <= 16MHz).
//   <i> Default: 8000000  (8MHz)
// </e> End of Clock Configuration
#define __CLOCK_SETUP              1
#define __RCC_CR_VAL               0x00000083
#define __RCC_CFGR_VAL             0x001D8000
#define __HSE                      8000000

//=========================================================================== Embedded Flash Configuration
// <e0> Embedded Flash Configuration
//   <h> Flash Access Control Configuration (FLASH_ACR)
//     <o1.0..2> LATENCY: Latency
//       <i> Default: 2 wait states
//                     <0=> 0 wait states
//                     <1=> 1 wait states
//                     <2=> 2 wait states
//     <o1.3> HLFCYA: Flash Half Cycle Access Enable
//     <o1.4> PRFTBE: Prefetch Buffer Enable
//     <o1.5> PRFTBS: Prefetch Buffer Status Enable
//   </h>
// </e>
#define __EFI_SETUP               1
#define __EFI_ACR_Val             0x00000012


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------
  RCC Defines
 *----------------------------------------------------------------------------*/
/* register RCC_CR -----------------------------------------------------------*/
#define RCC_CR_HSION       (0x00000001)              /* Internal High Speed clock enable     */
#define RCC_CR_HSIRDY      (0x00000002)              /* Internal High Speed clock ready flag */
#define RCC_CR_HSEON       (0x00010000)              /* External High Speed clock enable     */
#define RCC_CR_HSERDY      (0x00020000)              /* External High Speed clock ready flag */
#define RCC_CR_PLLON       (0x01000000)              /* PLL enable                           */
#define RCC_CR_PLLRDY      (0x02000000)              /* PLL clock ready flag                 */

/* register RCC_CFGR ---------------------------------------------------------*/
#define RCC_CFGR_SWS       (0x0000000C)              /* System Clock Switch Status           */

#define __HSI (8000000UL)   

/*----------------------------------------------------------------------------
  Clock Definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemFrequency;                            /*!< System Clock Frequency (Core Clock) */
uint32_t SystemFrequency_SysClk;                     /*!< System clock                        */
uint32_t SystemFrequency_AHBClk;                     /*!< AHB System bus speed                */
uint32_t SystemFrequency_APB1Clk;                    /*!< APB Peripheral bus 1 (low)  speed   */
uint32_t SystemFrequency_APB2Clk;                    /*!< APB Peripheral bus 2 (high) speed   */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the Embedded Flash Interface,  initialize the PLL and update th SystemFrequency variable
 */
void SystemInit (void)
{
  uint32_t rccCfgr, pllInClk;

#if __EFI_SETUP
  FLASH->ACR = __EFI_ACR_Val;                        /* set access control register */
#endif

#if __CLOCK_SETUP

  RCC->CFGR = __RCC_CFGR_VAL;                        /* set clock configuration register */
  RCC->CR   = __RCC_CR_VAL;                          /* set clock control register */

  if (__RCC_CR_VAL & RCC_CR_HSION) {                 /* if HSI enabled*/
    while ((RCC->CR & RCC_CR_HSIRDY) == 0);          /* Wait for HSIRDY = 1 (HSI is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_HSEON) {                 /* if HSE enabled*/
    while ((RCC->CR & RCC_CR_HSERDY) == 0);          /* Wait for HSERDY = 1 (HSE is ready)*/
  }

  if (__RCC_CR_VAL & RCC_CR_PLLON) {                 /* if PLL enabled*/
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);          /* Wait for PLLRDY = 1 (PLL is ready)*/
  }

  /* Wait till SYSCLK is stabilized (depending on selected clock) */
  while ((RCC->CFGR & RCC_CFGR_SWS) != ((__RCC_CFGR_VAL<<2) & RCC_CFGR_SWS));
#endif

  /* Determine clock frequency according to clock register values */
  rccCfgr = RCC->CFGR;

  switch ((rccCfgr) & 0x03) {                        /* SW System clock Switch */
    case 0:                                          /* HSI selected as system clock */
      SystemFrequency_SysClk  = (uint32_t)(__HSI);
      break;
    case 1:                                          /* HSE selected as system clock */
      SystemFrequency_SysClk  = (uint32_t)(__HSE);
      break;
    case 2:                                          /* PLL selected as system clock */
      if ((rccCfgr >> 16) & 0x01){                   /* PLLSRC PLL entry clock source */
        if ((rccCfgr >> 17) & 0x01) {                /* PLLXTPRE HSE divider for PLL entry */
          pllInClk = (uint32_t)(__HSE/2);            /* HSE clock divided by 2 */
        } else {
          pllInClk = (uint32_t)(__HSE);              /* HSE clock not divided */
        }
      } else {
        pllInClk = (uint32_t)(__HSI/2);              /* HSI oscillator clock / 2 selected as PLL input clock */
      }
      SystemFrequency_SysClk  = pllInClk * (((rccCfgr >> 18) & 0x0F) + 2);
      break;
  }

  if (rccCfgr & (1<< 7)) {
    SystemFrequency = SystemFrequency_SysClk >> (((rccCfgr >> 4) & 0x07) + 1);
  } else {
    SystemFrequency = SystemFrequency_SysClk;
  }

  SystemFrequency_AHBClk = SystemFrequency;

  if (rccCfgr & (1<<10)) {
    SystemFrequency_APB1Clk = SystemFrequency >> (((rccCfgr >> 8) & 0x03) + 1);
  } else {
    SystemFrequency_APB1Clk = SystemFrequency;
  }

  if (rccCfgr & (1<<13)) {
    SystemFrequency_APB2Clk = SystemFrequency >> (((rccCfgr >>11) & 0x03) + 1);
  } else {
    SystemFrequency_APB2Clk = SystemFrequency;
  }
   
}

