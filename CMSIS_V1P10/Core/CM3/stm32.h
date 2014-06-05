/******************************************************************************
 * @file:    stm32.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File
 * @version: V1.02
 * @date:    06. Feb. 2009
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


#ifndef __STM32_H__
#define __STM32_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  STM32 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
  RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
  DMAChannel1_IRQn            = 11,     /*!< DMA Channel 1 global Interrupt                       */
  DMAChannel2_IRQn            = 12,     /*!< DMA Channel 2 global Interrupt                       */
  DMAChannel3_IRQn            = 13,     /*!< DMA Channel 3 global Interrupt                       */
  DMAChannel4_IRQn            = 14,     /*!< DMA Channel 4 global Interrupt                       */
  DMAChannel5_IRQn            = 15,     /*!< DMA Channel 5 global Interrupt                       */
  DMAChannel6_IRQn            = 16,     /*!< DMA Channel 6 global Interrupt                       */
  DMAChannel7_IRQn            = 17,     /*!< DMA Channel 7 global Interrupt                       */
  ADC_IRQn                    = 18,     /*!< ADC global Interrupt                                 */
  USB_HP_CAN_TX_IRQn          = 19,     /*!< USB High Priority or CAN TX Interrupts               */
  USB_LP_CAN_RX0_IRQn         = 20,     /*!< USB Low Priority or CAN RX0 Interrupts               */
  CAN_RX1_IRQn                = 21,     /*!< CAN RX1 Interrupt                                    */
  CAN_SCE_IRQn                = 22,     /*!< CAN SCE Interrupt                                    */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB WakeUp from suspend through EXTI Line Interrupt  */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                 */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupts              */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare interrupt                       */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel1 global Interrupt                       */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel2 global Interrupt                       */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel3 global Interrupt                       */
  DMA2_Channel4_5_IRQn        = 59      /*!< DMA2 Channel4 and DMA2 Channel5 global Interrupts    */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT           0       /*!< STM32 does not provide a MPU present or not          */
#define __NVIC_PRIO_BITS        4       /*!< STM32 uses 4 Bits for the Priority Levels            */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */


#include "core_cm3.h"                   /* Cortex-M3 processor and core peripherals               */
#include "system_stm32.h"               /* STM32 System                                           */



/**
 * Initialize the system clock
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the PLL and update the SystemFrequency variable
 */
extern void SystemInit (void);


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/*------------------------ Analog to Digital Converter -----------------------*/
typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;

/*------------------------ Backup Registers ----------------------------------*/
typedef struct
{
       uint32_t RESERVED0;
  __IO uint16_t DR1;
       uint16_t RESERVED1;
  __IO uint16_t DR2;
       uint16_t RESERVED2;
  __IO uint16_t DR3;
       uint16_t RESERVED3;
  __IO uint16_t DR4;
       uint16_t RESERVED4;
  __IO uint16_t DR5;
       uint16_t RESERVED5;
  __IO uint16_t DR6;
       uint16_t RESERVED6;
  __IO uint16_t DR7;
       uint16_t RESERVED7;
  __IO uint16_t DR8;
       uint16_t RESERVED8;
  __IO uint16_t DR9;
       uint16_t RESERVED9;
  __IO uint16_t DR10;
       uint16_t RESERVED10;
  __IO uint16_t RTCCR;
       uint16_t RESERVED11;
  __IO uint16_t CR;
       uint16_t RESERVED12;
  __IO uint16_t CSR;
       uint16_t RESERVED13;
} BKP_TypeDef;

/*------------------------ Controller Area Network ---------------------------*/
typedef struct
{
  __IO uint32_t TIR;
  __IO uint32_t TDTR;
  __IO uint32_t TDLR;
  __IO uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct
{
  __IO uint32_t RIR;
  __IO uint32_t RDTR;
  __IO uint32_t RDLR;
  __IO uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct
{
  __IO uint32_t FR0;
  __IO uint32_t FR1;
} CAN_FilterRegister_TypeDef;

typedef struct
{
  __IO uint32_t MCR;
  __IO uint32_t MSR;
  __IO uint32_t TSR;
  __IO uint32_t RF0R;
  __IO uint32_t RF1R;
  __IO uint32_t IER;
  __IO uint32_t ESR;
  __IO uint32_t BTR;
       uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
       uint32_t RESERVED1[12];
  __IO uint32_t FMR;
  __IO uint32_t FM0R;
       uint32_t RESERVED2[1];
  __IO uint32_t FS0R;
       uint32_t RESERVED3[1];
  __IO uint32_t FFA0R;
       uint32_t RESERVED4[1];
  __IO uint32_t FA0R;
       uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

/*------------------------ DMA Controller ------------------------------------*/
typedef struct
{
  __IO uint32_t CCR;
  __IO uint32_t CNDTR;
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
} DMA_TypeDef;

/*------------------------ External Interrupt/Event Controller ---------------*/
typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

/*------------------------ FLASH and Option Bytes Registers ------------------*/
typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
} FLASH_TypeDef;

typedef struct
{
  __IO uint16_t RDP;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WRP0;
  __IO uint16_t WRP1;
  __IO uint16_t WRP2;
  __IO uint16_t WRP3;
} OB_TypeDef;

/*------------------------ General Purpose and Alternate Function IO ---------*/
typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
} AFIO_TypeDef;

/*------------------------ Inter-integrated Circuit Interface ----------------*/
typedef struct
{
  __IO uint16_t CR1;
       uint16_t RESERVED0;
  __IO uint16_t CR2;
       uint16_t RESERVED1;
  __IO uint16_t OAR1;
       uint16_t RESERVED2;
  __IO uint16_t OAR2;
       uint16_t RESERVED3;
  __IO uint16_t DR;
       uint16_t RESERVED4;
  __IO uint16_t SR1;
       uint16_t RESERVED5;
  __IO uint16_t SR2;
       uint16_t RESERVED6;
  __IO uint16_t CCR;
       uint16_t RESERVED7;
  __IO uint16_t TRISE;
       uint16_t RESERVED8;
} I2C_TypeDef;

/*------------------------ Independent WATCHDOG ------------------------------*/
typedef struct
{
  __IO uint32_t KR;
  __IO uint32_t PR;
  __IO uint32_t RLR;
  __IO uint32_t SR;
} IWDG_TypeDef;



/*------------------------ Power Control -------------------------------------*/
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;

/*------------------------ Reset and Clock Control ---------------------------*/
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

/*------------------------ Real-Time Clock -----------------------------------*/
typedef struct
{
  __IO uint16_t CRH;
       uint16_t RESERVED0;
  __IO uint16_t CRL;
       uint16_t RESERVED1;
  __IO uint16_t PRLH;
       uint16_t RESERVED2;
  __IO uint16_t PRLL;
       uint16_t RESERVED3;
  __IO uint16_t DIVH;
       uint16_t RESERVED4;
  __IO uint16_t DIVL;
       uint16_t RESERVED5;
  __IO uint16_t CNTH;
       uint16_t RESERVED6;
  __IO uint16_t CNTL;
       uint16_t RESERVED7;
  __IO uint16_t ALRH;
       uint16_t RESERVED8;
  __IO uint16_t ALRL;
       uint16_t RESERVED9;
} RTC_TypeDef;

/*------------------------ Serial Peripheral Interface -----------------------*/
typedef struct
{
  __IO uint16_t CR1;
       uint16_t RESERVED0;
  __IO uint16_t CR2;
       uint16_t RESERVED1;
  __IO uint16_t SR;
       uint16_t RESERVED2;
  __IO uint16_t DR;
       uint16_t RESERVED3;
  __IO uint16_t CRCPR;
       uint16_t RESERVED4;
  __IO uint16_t RXCRCR;
       uint16_t RESERVED5;
  __IO uint16_t TXCRCR;
       uint16_t RESERVED6;
} SPI_TypeDef;


/*------------------------ Advanced Control Timer ----------------------------*/
typedef struct
{
  __IO uint16_t CR1;
       uint16_t RESERVED0;
  __IO uint16_t CR2;
       uint16_t RESERVED1;
  __IO uint16_t SMCR;
       uint16_t RESERVED2;
  __IO uint16_t DIER;
       uint16_t RESERVED3;
  __IO uint16_t SR;
       uint16_t RESERVED4;
  __IO uint16_t EGR;
       uint16_t RESERVED5;
  __IO uint16_t CCMR1;
       uint16_t RESERVED6;
  __IO uint16_t CCMR2;
       uint16_t RESERVED7;
  __IO uint16_t CCER;
       uint16_t RESERVED8;
  __IO uint16_t CNT;
       uint16_t RESERVED9;
  __IO uint16_t PSC;
       uint16_t RESERVED10;
  __IO uint16_t ARR;
       uint16_t RESERVED11;
  __IO uint16_t RCR;
       uint16_t RESERVED12;
  __IO uint16_t CCR1;
       uint16_t RESERVED13;
  __IO uint16_t CCR2;
       uint16_t RESERVED14;
  __IO uint16_t CCR3;
       uint16_t RESERVED15;
  __IO uint16_t CCR4;
       uint16_t RESERVED16;
  __IO uint16_t BDTR;
       uint16_t RESERVED17;
  __IO uint16_t DCR;
       uint16_t RESERVED18;
  __IO uint16_t DMAR;
       uint16_t RESERVED19;
} TIM1_TypeDef;

/*------------------------ General Purpose Timer -----------------------------*/
typedef struct
{
  __IO uint16_t CR1;
       uint16_t RESERVED0;
  __IO uint16_t CR2;
       uint16_t RESERVED1;
  __IO uint16_t SMCR;
       uint16_t RESERVED2;
  __IO uint16_t DIER;
       uint16_t RESERVED3;
  __IO uint16_t SR;
       uint16_t RESERVED4;
  __IO uint16_t EGR;
       uint16_t RESERVED5;
  __IO uint16_t CCMR1;
       uint16_t RESERVED6;
  __IO uint16_t CCMR2;
       uint16_t RESERVED7;
  __IO uint16_t CCER;
       uint16_t RESERVED8;
  __IO uint16_t CNT;
       uint16_t RESERVED9;
  __IO uint16_t PSC;
       uint16_t RESERVED10;
  __IO uint16_t ARR;
       uint16_t RESERVED11[3];
  __IO uint16_t CCR1;
       uint16_t RESERVED12;
  __IO uint16_t CCR2;
       uint16_t RESERVED13;
  __IO uint16_t CCR3;
       uint16_t RESERVED14;
  __IO uint16_t CCR4;
       uint16_t RESERVED15[3];
  __IO uint16_t DCR;
       uint16_t RESERVED16;
  __IO uint16_t DMAR;
       uint16_t RESERVED17;
} TIM_TypeDef;

/*----------------- Universal Synchronous Asynchronous Receiver Transmitter --*/
typedef struct
{
  __IO uint16_t SR;
       uint16_t RESERVED0;
  __IO uint16_t DR;
       uint16_t RESERVED1;
  __IO uint16_t BRR;
       uint16_t RESERVED2;
  __IO uint16_t CR1;
       uint16_t RESERVED3;
  __IO uint16_t CR2;
       uint16_t RESERVED4;
  __IO uint16_t CR3;
       uint16_t RESERVED5;
  __IO uint16_t GTPR;
       uint16_t RESERVED6;
} USART_TypeDef;

/*------------------------ Window WATCHDOG -----------------------------------*/
typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFR;
  __IO uint32_t SR;
} WWDG_TypeDef;


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Peripheral and SRAM base address in the alias region */
#define PERIPH_BB_BASE        ((     uint32_t)0x42000000)
#define SRAM_BB_BASE          ((     uint32_t)0x22000000)

/* Peripheral and SRAM base address in the bit-band region */
#define SRAM_BASE             ((     uint32_t)0x20000000)
#define PERIPH_BASE           ((     uint32_t)0x40000000)

/* Flash registers base address */
#define FLASH_BASE            ((     uint32_t)0x40022000)
/* Flash Option Bytes base address */
#define OB_BASE               ((     uint32_t)0x1FFFF800)

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN_BASE              (APB1PERIPH_BASE + 0x6400)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)

#define DMA_BASE              (AHBPERIPH_BASE + 0x0000)
#define DMA_Channel1_BASE     (AHBPERIPH_BASE + 0x0008)
#define DMA_Channel2_BASE     (AHBPERIPH_BASE + 0x001C)
#define DMA_Channel3_BASE     (AHBPERIPH_BASE + 0x0030)
#define DMA_Channel4_BASE     (AHBPERIPH_BASE + 0x0044)
#define DMA_Channel5_BASE     (AHBPERIPH_BASE + 0x0058)
#define DMA_Channel6_BASE     (AHBPERIPH_BASE + 0x006C)
#define DMA_Channel7_BASE     (AHBPERIPH_BASE + 0x0080)

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define TIM2                  ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                  ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                  ((TIM_TypeDef *) TIM4_BASE)
#define RTC                   ((RTC_TypeDef *) RTC_BASE)
#define WWDG                  ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                  ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                  ((SPI_TypeDef *) SPI2_BASE)
#define USART2                ((USART_TypeDef *) USART2_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)
#define I2C1                  ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                  ((I2C_TypeDef *) I2C2_BASE)
#define CAN                   ((CAN_TypeDef *) CAN_BASE)
#define BKP                   ((BKP_TypeDef *) BKP_BASE)
#define PWR                   ((PWR_TypeDef *) PWR_BASE)
#define AFIO                  ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                  ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD                 ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE                 ((GPIO_TypeDef *) GPIOE_BASE)
#define ADC1                  ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                  ((ADC_TypeDef *) ADC2_BASE)
#define TIM1                  ((TIM1_TypeDef *) TIM1_BASE)
#define SPI1                  ((SPI_TypeDef *) SPI1_BASE)
#define USART1                ((USART_TypeDef *) USART1_BASE)
#define DMA                   ((DMA_TypeDef *) DMA_BASE)
#define DMA_Channel1          ((DMA_Channel_TypeDef *) DMA_Channel1_BASE)
#define DMA_Channel2          ((DMA_Channel_TypeDef *) DMA_Channel2_BASE)
#define DMA_Channel3          ((DMA_Channel_TypeDef *) DMA_Channel3_BASE)
#define DMA_Channel4          ((DMA_Channel_TypeDef *) DMA_Channel4_BASE)
#define DMA_Channel5          ((DMA_Channel_TypeDef *) DMA_Channel5_BASE)
#define DMA_Channel6          ((DMA_Channel_TypeDef *) DMA_Channel6_BASE)
#define DMA_Channel7          ((DMA_Channel_TypeDef *) DMA_Channel7_BASE)
#define FLASH                 ((FLASH_TypeDef *) FLASH_BASE)
#define OB                    ((OB_TypeDef *) OB_BASE)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)


#endif



