/******************************************************************************
 * @file:    LM3S6965.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File for the 
 *            Luminary 'LM3S' Device Series 
 * @version: V0.01 (Preliminary)
 * @date:    15. Jan. 2009
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


#ifndef __LM3S6965_H__
#define __LM3S6965_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,      /*!< 2 Non Maskable Interrupt                         */
  MemoryManagement_IRQn         = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
  BusFault_IRQn                 = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
  UsageFault_IRQn               = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
  SVCall_IRQn                   = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                   */
  DebugMonitor_IRQn             = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
  PendSV_IRQn                   = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                   */
  SysTick_IRQn                  = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

/******  LM3S6965 Specific Interrupt Numbers ******************************************************/
  GPIOPortA_IRQn                =  0,        /*!<  GPIO Port A                                    */
  GPIOPortB_IRQn                =  1,        /*!<  GPIO Port B                                    */
  GPIOPortC_IRQn                =  2,        /*!<  GPIO Port C                                    */
  GPIOPortD_IRQn                =  3,        /*!<  GPIO Port D                                    */
  GPIOPortE_IRQn                =  4,        /*!<  GPIO Port E                                    */
  UART0_IRQn                    =  5,        /*!<  UART0                                          */
  UART1_IRQn                    =  6,        /*!<  UART1                                          */
  SSI0_IRQn                     =  7,        /*!<  SSI0                                           */
  I2C0_IRQn                     =  8,        /*!<  I2C0                                           */
  PWMFault_IRQn                 =  9,        /*!<  PWM Fault                                      */
  PWMGen0_IRQn                  = 10,        /*!<  PWM Generator 0                                */
  PWMGen1_IRQn                  = 11,        /*!<  PWM Generator 1                                */
  PWMGen2_IRQn                  = 12,        /*!<  PWM Generator 2                                */
  QEI0_IRQn                     = 13,        /*!<  Quadrature Encoder 0                           */
  ADCSeq0_IRQn                  = 14,        /*!<  ADC Sequence 0                                 */
  ADCSeq1_IRQn                  = 15,        /*!<  ADC Sequence 1                                 */
  ADCSeq2_IRQn                  = 16,        /*!<  ADC Sequence 2                                 */
  ADCSeq3_IRQn                  = 17,        /*!<  ADC Sequence 3                                 */
  Watchdog_IRQn                 = 18,        /*!<  Watchdog                                       */
  Timer0A_IRQn                  = 19,        /*!<  Timer 0A                                       */
  Timer0B_IRQn                  = 20,        /*!<  Timer 0B                                       */
  Timer1A_IRQn                  = 21,        /*!<  Timer 1A                                       */
  Timer1B_IRQn                  = 22,        /*!<  Timer 1B                                       */
  Timer2A_IRQn                  = 23,        /*!<  Timer 2A                                       */
  Timer2B_IRQn                  = 24,        /*!<  Timer 2B                                       */
  Comp0_IRQn                    = 25,        /*!<  Comp 0                                         */
  Comp1_IRQn                    = 26,        /*!<  Comp 1                                         */
  Comp2_IRQn                    = 27,        /*!<  Comp 2                                         */
  SysCtrl_IRQn                  = 28,        /*!<  System Control                                 */
  FlashCtrl_IRQn                = 29,        /*!<  Flash Control                                  */
  GPIOPortF_IRQn                = 30,        /*!<  GPIO Port F                                    */
  GPIOPortG_IRQn                = 31,        /*!<  GPIO Port G                                    */
  GPIOPortH_IRQn                = 32,        /*!<  GPIO Port H                                    */
  USART2_IRQn                   = 33,        /*!<  UART2 Rx and Tx                                */
  SSI1_IRQn                     = 34,        /*!<  SSI1 Rx and Tx                                 */
  Timer3A_IRQn                  = 35,        /*!<  Timer 3 subtimer A                             */
  Timer3B_IRQn                  = 36,        /*!<  Timer 3 subtimer B                             */
  I2C1_IRQn                     = 37,        /*!<  I2C1 Master and Slave                          */
  QEI1_IRQn                     = 38,        /*!<  Quadrature Encoder 1                           */
  CAN0_IRQn                     = 39,        /*!<  CAN0                                           */
  CAN1_IRQn                     = 40,        /*!<  CAN1                                           */
  CAN2_IRQn                     = 41,        /*!<  CAN2                                           */
  Ethernet_IRQn                 = 42,        /*!<  Ethernet                                       */
  Hibernate_IRQn                = 43,        /*!<  Hibernate                                      */
  USB0_IRQn                     = 44,        /*!<  USB0                                           */
  PWMGen3_IRQn                  = 45,        /*!<  PWM Generator 3                                */
  uDMA_IRQn                     = 46,        /*!<  uDMA Software Transfer                         */
  uDMAErr_IRQn                  = 47         /*!<  uDMA Error                                     */
} IRQn_Type;
  

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          8         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */


#include "core_cm3.h"                       /* Cortex-M3 processor and core peripherals           */
#include "system_LM3S6965.h"                /* LM3S6965 System                                    */



/**
 * Initialize the system clock
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the PLL and update the SystemFrequency variable.
 */
extern void SystemInit (void);


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

#pragma anon_unions

/*------------- System Control (SYSCTL) --------------------------------------*/
// <g> System Control (SYSCTL)
typedef struct
{
  __I  uint32_t DID0;
  __I  uint32_t DID1;
  __I  uint32_t DC0;
       uint32_t RESERVED0[1];
  __I  uint32_t DC1;
  __I  uint32_t DC2;
  __I  uint32_t DC3;
  __I  uint32_t DC4;
       uint32_t RESERVED1[4];
  __IO uint32_t PBORCTL;
  __IO uint32_t LDOPCTL;
       uint32_t RESERVED2[2];
  __IO uint32_t SRCR0;
  __IO uint32_t SRCR1;
  __IO uint32_t SRCR2;
       uint32_t RESERVED3[1];
  __I  uint32_t RIS;
  __IO uint32_t IMC;
  __IO uint32_t MISC;
  __IO uint32_t RESC;
  __IO uint32_t RCC;
  __I  uint32_t PLLCFG;
       uint32_t RESERVED4[2];
  __IO uint32_t RCC2;
       uint32_t RESERVED5[35];
  __IO uint32_t RCGC0;
  __IO uint32_t RCGC1;
  __IO uint32_t RCGC2;
       uint32_t RESERVED6[1];
  __IO uint32_t SCGC0;
  __IO uint32_t SCGC1;
  __IO uint32_t SCGC2;
       uint32_t RESERVED7[1];
  __IO uint32_t DCGC0;
  __IO uint32_t DCGC1;
  __IO uint32_t DCGC2;
       uint32_t RESERVED8[6];
  __IO uint32_t DSLPCLKCFG;
} SYSCTL_TypeDef;
// </g>


/*------------- General-Purpose Input/Outputs (GPIO) -------------------------*/
// <g> General-Purpose Input/Outputs (GPIO)
typedef struct
{
  __IO uint32_t DATA_Bits[0xFF];
  __IO uint32_t DATA;
  __IO uint32_t DIR;
  __IO uint32_t IS;
  __IO uint32_t IBE;
  __IO uint32_t IEV;
  __IO uint32_t IM;
  __I  uint32_t RIS;
  __I  uint32_t MIS;
  __IO uint32_t ICR;
  __IO uint32_t AFSEL;
       uint32_t RESERVED0[55];
  __IO uint32_t DR2R;
  __IO uint32_t DR4R;
  __IO uint32_t DR8R;
  __IO uint32_t ODR;
  __IO uint32_t PUR;
  __IO uint32_t PDR;
  __IO uint32_t SLR;
  __IO uint32_t DEN;
  __IO uint32_t LOCK;
} GPIO_TypeDef;
// </g>



#pragma no_anon_unions


/******************************************************************************/
/*                              Memory map                                    */
/******************************************************************************/

#define FLASH_BASE            (0x00000000UL)
#define RAM_BASE              (0x20000000UL)
#define PERI_BASE             (0x40000000UL)

#define SYSCTL_BASE           (PERI_BASE  + 0xFE000)

#define GPIOA_BASE            (PERI_BASE  + 0x04000)
#define GPIOB_BASE            (PERI_BASE  + 0x05000)
#define GPIOC_BASE            (PERI_BASE  + 0x06000)
#define GPIOD_BASE            (PERI_BASE  + 0x07000)
#define GPIOE_BASE            (PERI_BASE  + 0x24000)
#define GPIOF_BASE            (PERI_BASE  + 0x25000)
#define GPIOG_BASE            (PERI_BASE  + 0x26000)

/******************************************************************************/
/*                            Peripheral declaration                          */
/******************************************************************************/

#define SYSCTL                (( SYSCTL_TypeDef *)   SYSCTL_BASE)
#define GPIOA                 ((   GPIO_TypeDef *)    GPIOA_BASE)
#define GPIOB                 ((   GPIO_TypeDef *)    GPIOB_BASE)
#define GPIOC                 ((   GPIO_TypeDef *)    GPIOC_BASE)
#define GPIOD                 ((   GPIO_TypeDef *)    GPIOD_BASE)
#define GPIOE                 ((   GPIO_TypeDef *)    GPIOE_BASE)
#define GPIOF                 ((   GPIO_TypeDef *)    GPIOF_BASE)
#define GPIOG                 ((   GPIO_TypeDef *)    GPIOG_BASE)

#endif  /* __LM3S6965_H__ */
