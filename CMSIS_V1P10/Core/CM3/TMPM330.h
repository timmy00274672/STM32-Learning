/******************************************************************************
 * @file:    TMPM330.h
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Header File for the 
 *           Toshiba 'TMPM330' Device Series 
 * @version: V1.0
 * @date:    15. Dec. 2008
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


#ifndef __TMPM330_H__
#define __TMPM330_H__

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

/******  LPC17xx Specific Interrupt Numbers *******************************************************/
  INT0_IRQn                     = 0,        /*!< Interrupt pin PJ0/70 pin                         */
  INT1_IRQn                     = 1,        /*!< Interrupt pin PJ1/49 pin                         */
  INT2_IRQn                     = 2,        /*!< Interrupt pin PJ2/86 pin                         */
  INT3_IRQn                     = 3,        /*!< Interrupt pin PJ3/87 pin                         */
  INT4_IRQn                     = 4,        /*!< Interrupt pin PG3/6  pin                         */
  INT5_IRQn                     = 5,        /*!< Interrupt pin PF7/19 pin                         */
  INTRX0_IRQn                   = 6,        /*!< Serial receive 0                                 */
  INTTX0_IRQn                   = 7,        /*!< Serial transmit 0                                */
  INTRX1_IRQn                   = 8,        /*!< Serial receive 1                                 */
  INTTX1_IRQn                   = 9,        /*!< Serial transmit 1                                */
  INTSBI0_IRQn                  = 10,       /*!< Serial bus interface 0                           */
  INTSBI1_IRQn                  = 11,       /*!< Serial bus interface 1                           */
  INTCECRX_IRQn                 = 12,       /*!< CEC receive                                      */
  INTCECTX_IRQn                 = 13,       /*!< CEC transmit                                     */
  INTAINTRMCRX0_IRQn            = 14,       /*!< Remote ctrl reception 0                          */
  INTADHP_IRQn                  = 15,       /*!< High priority ADC                                */
  INTADM0_IRQn                  = 16,       /*!< ADC monitoring 0                                 */
  INTADM1_IRQn                  = 17,       /*!< ADC monitoring 1                                 */
  INTTB0_IRQn                   = 18,       /*!< TMRB match detection 0                           */
  INTTB1_IRQn                   = 19,       /*!< TMRB match detection 1                           */
  INTTB2_IRQn                   = 20,       /*!< TMRB match detection 2                           */
  INTTB3_IRQn                   = 21,       /*!< TMRB match detection 3                           */
  INTTB4_IRQn                   = 22,       /*!< TMRB match detection 4                           */
  INTTB5_IRQn                   = 23,       /*!< TMRB match detection 5                           */
  INTTB6_IRQn                   = 24,       /*!< TMRB match detection 6                           */
  INTRTC_IRQn                   = 25,       /*!< RTC                                              */
  INTCAP00_IRQn                 = 26,       /*!< TMRB input capture 00                            */
  INTCAP01_IRQn                 = 27,       /*!< TMRB input capture 01                            */
  INTCAP10_IRQn                 = 28,       /*!< TMRB input capture 10                            */
  INTCAP11_IRQn                 = 29,       /*!< TMRB input capture 11                            */
  INTCAP50_IRQn                 = 30,       /*!< TMRB input capture 50                            */
  INTCAP51_IRQn                 = 31,       /*!< TMRB input capture 51                            */
  INTCAP60_IRQn                 = 32,       /*!< TMRB input capture 60                            */
  INTCAP61_IRQn                 = 33,       /*!< TMRB input capture 61                            */
  INT6_IRQn                     = 34,       /*!< Interrupt pin PJ6/39 pin                         */
  INT7_IRQn                     = 35,       /*!< Interrupt pin PJ7/58 pin                         */
  INTRX2_IRQn                   = 36,       /*!< Serial receive 2                                 */
  INTTX2_IRQn                   = 37,       /*!< Serial transmit 2                                */
  INTSBI2_IRQn                  = 38,       /*!< Serial bus interface 2                           */
  INTAINTRMCRX1_IRQn            = 39,       /*!< Remote ctrl reception 1                          */
  INTTB7_IRQn                   = 40,       /*!< TMRB match detection 7                           */
  INTTB8_IRQn                   = 41,       /*!< TMRB match detection 8                           */
  INTTB9_IRQn                   = 42,       /*!< TMRB match detection 9                           */
  INTCAP20_IRQn                 = 43,       /*!< TMRB input capture 20                            */
  INTCAP21_IRQn                 = 44,       /*!< TMRB input capture 21                            */
  INTCAP30_IRQn                 = 45,       /*!< TMRB input capture 30                            */
  INTCAP31_IRQn                 = 46,       /*!< TMRB input capture 31                            */
  INTCAP40_IRQn                 = 47,       /*!< TMRB input capture 40                            */
  INTCAP41_IRQn                 = 48,       /*!< TMRB input capture 41                            */
  INTAD_IRQn                    = 49,       /*!< ADC complete                                     */
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


#include "..\core_cm3.h"                    /* Cortex-M3 processor and core peripherals           */
#include "system_TMPM330.h"                 /* TMPM330 System                                     */



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

/*------------- Clock Generator (CG) -----------------------------------------*/
// <g> Clock Generator (CG)
typedef struct
{
  __IO uint32_t SYSCR;        // <h> System Control Register
                              //   <o.16..17> SCOSEL: SCOUT pin output
                              //                <0=> 0: fs
                              //                <1=> 1: Reserved
                              //                <2=> 2: fsys
                              //                <3=> 3: phiT0
                              //   <o.12>     FPSEL: fperiph source clock
                              //                <0=> 0: fgear
                              //                <1=> 1: fc
                              //   <o.8..10>  PRCK: Prescaler clock
                              //                <0=> 0: fperiph
                              //                <1=> 1: fperiph/2
                              //                <2=> 2: fperiph/4
                              //                <3=> 3: fperiph/8
                              //                <4=> 4: fperiph/16
                              //                <5=> 5: fperiph/32
                              //                <6=> 6: Reserved
                              //                <7=> 7: Reserved
                              //   <o.0..2>   GEAR: High-speed clock (fc) gear
                              //                <0=> 0: fc
                              //                <1=> 1: Reserved
                              //                <2=> 2: Reserved
                              //                <3=> 3: Reserved
                              //                <4=> 4: fc/2
                              //                <5=> 5: fc/4
                              //                <6=> 6: fc/8
                              //                <7=> 7: Reserved
                              // </h>
  __IO uint32_t OSCCR;        // <h> Oscillation Control Register
                              //   <o.9>      XTEN: Low-speed oscillator enable
                              //   <o.8>      XEN: High-speed oscillator enable
                              //   <o.4..6>   WUPT: Warm-up timer for oscillator
                              //                <0=> 0: No warm-up
                              //                <1=> 1: X1:  1024/inp freq | XT1:     64/inp freq 
                              //                <2=> 2: X1:  2048/inp freq | XT1:    128/inp freq
                              //                <3=> 3: X1:  4096/inp freq | XT1:    256/inp freq
                              //                <4=> 4: X1:  8192/inp freq | XT1:  32768/inp freq
                              //                <5=> 5: X1: 16384/inp freq | XT1:  65536/inp freq
                              //                <6=> 6: X1: 32768/inp freq | XT1: 131072/inp freq
                              //                <7=> 7: X1: 65536/inp freq | XT1: 262144/inp freq
                              //   <o.3>      WUPSEL: Warm-up counter
                              //                <0=> 0: X1
                              //                <1=> 1: XT1
                              //   <o.2>      PLLON: PLL enable
                              //   <o.1>      WUEF: Status of Warm-up timer <r>
                              //                <0=> 0: Warm-up completed
                              //                <1=> 1: Warm-up in operation
                              //   <o.0>      WUEON: Warm-up timer enable <w>
                              // </h>
  __IO uint32_t STBYCR;       // <h> Standby Control Register
                              //   <o.16>     DRVE: Pin status in STOP mode
                              //                <0=> 0: Active
                              //                <1=> 1: Inactive
                              //   <o.9>      RXTEN: Low-speed oscillator after releasing STOP mode enable
                              //   <o.8>      RXEN: High-speed oscillator after releasing STOP mode enable
                              //   <o.0..2>   STBY: Low power consumption mode
                              //                <0=> 0: Reserved
                              //                <1=> 1: STOP 
                              //                <2=> 2: SLEEP
                              //                <3=> 3: IDLE
                              //                <4=> 4: Reserved
                              //                <5=> 5: Reserved
                              //                <6=> 6: Reserved
                              //                <7=> 7: Reserved
                              // </h>
  __IO uint32_t PLLSEL;       // <h> PLL Selection Register
                              //   <o.0>      PLLSEL: PLL enable
                              // </h>
  __IO uint32_t CKSEL;        // <h> System Clock Selection Register
                              //   <o.1>      SYSCK: System clock
                              //                <0=> 0: High-speed (fc)
                              //                <1=> 1: Low-speed (fs)
                              //   <o.0>      SYSCKFLG: System clock status <r>
                              //                <0=> 0: High-speed (fc)
                              //                <1=> 1: Low-speed (fs)
                              // </h>
  __IO uint32_t ICRCG;        // <h> CG Interrupt Request Clear Register <w>
                              //   <o.0..3>   ICRCG: Clear interrupt
                              //                <0=> 0: INT0
                              //                <1=> 1: INT1 
                              //                <2=> 2: INT2
                              //                <3=> 3: INT3
                              //                <4=> 4: INT4
                              //                <5=> 5: INT5
                              //                <6=> 6: INTCECRX
                              //                <7=> 7: INTRMCRX0
                              //                <8=> 8: INTRTC
                              //                <9=> 9: INT6
                              //                <10=> 10: INT7
                              //                <11=> 11: INTRMCRX1
                              // </h>
  __IO uint32_t NMIFLG;       // <h> NMI Flag Register <r>
                              //   <o.1>      NMIFLG1: NMI generated from NMI pin <r>
                              //   <o.0>      NMIFLG0: NMI generated from watchdog <r>
                              // </h>
  __IO uint32_t RSTFLG;       // <h> Reset Flag Register
                              //   <o.4>      DBGRSTF: Debug reset flag
                              //   <o.2>      WDTRSTF: Watchdog reset flag
                              //   <o.1>      PINRSTF: RESET pin flag
                              //   <o.0>      PONRSTF: Power-on reset flag
                              // </h>
  __IO uint32_t IMCGA;        // <h> CG Interrupt Mode Control Register A
                              //   <o.28..30> EMCG3: INT3 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.26..27> EMST3: INT3 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.24>     INT3EN: INT3 enable
                              //   <o.20..22> EMCG2: INT2 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.18..19> EMST2: INT2 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.16>     INT2EN: INT2 enable
                              //   <o.12..14> EMCG1: INT1 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.10..11> EMST1: INT1 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.8>      INT1EN: INT1 enable
                              //   <o.4..6>   EMCG0: INT0 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.2..3>   EMST0: INT0 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.0>      INT0EN: INT0 enable
                              // </h>
  __IO uint32_t IMCGB;        // <h> CG Interrupt Mode Control Register B
                              //   <o.28..30> EMCG7: INTRMCRX0 active state setting
                              //                <0=> 0: none
                              //                <2=> 2: none
                              //                <3=> 3: rising edge
                              //   <o.26..27> EMST7: INTRMCRX0 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.24>     INT7EN: INTRMCRX0 enable
                              //   <o.20..22> EMCG6: INTCECRX active state setting
                              //                <0=> 0: none
                              //                <2=> 2: none
                              //                <3=> 3: rising edge
                              //   <o.18..19> EMST6: INTCECRX active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.16>     INT6EN: INTCECRX enable
                              //   <o.12..14> EMCG5: INT5 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.10..11> EMST5: INT5 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.8>      INT5EN: INT5 enable
                              //   <o.4..6>   EMCG4: INT4 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.2..3>   EMST4: INT4 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.0>      INT4EN: INT4 enable
                              // </h>
  __IO uint32_t IMCGC;        // <h> CG Interrupt Mode Control Register C
                              //   <o.28..30> EMCGB: INTRMCRX1 active state setting
                              //                <0=> 0: none
                              //                <2=> 2: none
                              //                <3=> 3: rising edge
                              //   <o.26..27> EMSTB: INTRMCRX1 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.24>     INTBEN: INTRMCRX1 enable
                              //   <o.20..22> EMCGA: INT7 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.18..19> EMSTA: INT7 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.16>     INTAEN: INT7 enable
                              //   <o.12..14> EMCG9: INT6 active state setting
                              //                <0=> 0: low level
                              //                <1=> 1: high level
                              //                <2=> 2: falling edge
                              //                <3=> 3: rising edge
                              //                <4=> 4: both edges
                              //   <o.10..11> EMST9: INT6 active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.8>      INT9EN: INT6 enable
                              //   <o.4..6>   EMCG8: INTRTC active state setting
                              //                <0=> 0: none
                              //                <2=> 2: falling edge
                              //   <o.2..3>   EMST8: INTRTC active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.0>      INT8EN: INTRTC enable
                              // </h>
  __IO uint32_t IMCGD;        // <h> CG Interrupt Mode Control Register D
                              //   <o.4..6>   EMCGC: INTCECTX active state setting
                              //                <0=> 0: none
                              //                <2=> 2: falling edge
                              //   <o.2..3>   EMSTC: INTCECTX active state <r>
                              //                <0=> 0: -
                              //                <1=> 1: rising edge
                              //                <2=> 2: falling edge
                              //                <3=> 3: both edges
                              //   <o.0>      INTCEN: INTCECTX enable
                              // </h>
} CG_TypeDef;
// </g>

/*------------- General Purpose Input/Output Port (PORT) ---------------------*/
typedef struct
{
  __IO uint32_t DATA;
  __IO uint32_t CR;
  __IO uint32_t FR1;
       uint32_t RESERVED0[8];
  __IO uint32_t PUP;
  __IO uint32_t PDN;
       uint32_t RESERVED1;
  __IO uint32_t IE;
} PORT_TypeDef;

/*------------- 16-bit Timer/Event Counter (TMRB) ----------------------------*/
typedef struct
{
  __IO uint32_t EN;
  __IO uint32_t RUN;
  __IO uint32_t CR;
  __IO uint32_t MOD;
  __IO uint32_t FFCR;
  __IO uint32_t ST;
  __IO uint32_t IM;
  __IO uint32_t UC;
  __IO uint32_t RG0;
  __IO uint32_t RG1;
  __IO uint32_t CP0;
  __IO uint32_t CP1;
} TMRB_TypeDef;

/*------------- Serial Bus Interface (SBI) -----------------------------------*/
typedef struct
{
  __IO uint32_t CR0;
  __IO uint32_t CR1;
  __IO uint32_t DBR;
  __IO uint32_t I2CAR;
  union {
  __O  uint32_t CR2;
  __I  uint32_t SR;
  };
  __IO uint32_t BR0;
} SBI_TypeDef;

/*------------- Serial Channel (SIO) -----------------------------------------*/
typedef struct
{
  __IO uint32_t EN;
  __IO uint32_t BUF;
  __IO uint32_t CR;
  __IO uint32_t MOD0;
  __IO uint32_t BRCR;
  __IO uint32_t BRADD;
  __IO uint32_t MOD1;
  __IO uint32_t MOD2;
  __IO uint32_t RFC;
  __IO uint32_t TFC;
  __I  uint32_t RST;
  __I  uint32_t TST;
  __IO uint8_t  FCNF;
} SIO_TypeDef;

/*------------- Analog-to-Digital Converter (ADC) ----------------------------*/
typedef struct
{
  __IO uint32_t CLK;
  __IO uint32_t MOD0;
  __IO uint32_t MOD1;
  __IO uint32_t MOD2;
  __IO uint32_t MOD3;
  __IO uint32_t MOD4;
  __IO uint32_t MOD5;
       uint32_t RESERVED0[5];
  __IO uint32_t REG08;
  __IO uint32_t REG19;
  __IO uint32_t REG2A;
  __IO uint32_t REG3B;
  __IO uint32_t REG4C;
  __IO uint32_t REG5D;
  __IO uint32_t REG6E;
  __IO uint32_t REG7F;
  __IO uint32_t REGSP;
  __IO uint32_t CMP0;
  __IO uint32_t CMP1;
} ADC_TypeDef;

/*------------- Watchdog Timer (WDT) -----------------------------------------*/
typedef struct
{
  __IO uint32_t MOD;
  __IO uint32_t CR;
} WDT_TypeDef;

/*------------- Real Time Clock (RTC) ----------------------------------------*/
typedef struct
{
  __IO uint8_t  SECR;
  __IO uint8_t  MINR;
  __IO uint8_t  HOURR;
       uint8_t  RESERVED0;
  __IO uint8_t  DAYR;
  __IO uint8_t  DATER;
  __IO uint8_t  MONTHR;
  __IO uint8_t  YEARR;
  __IO uint32_t PAGER;
  __O  uint32_t RESTR;
} RTC_TypeDef;

/*------------- Consumer Electronics Control (CEC) ---------------------------*/
typedef struct
{
  __IO uint32_t EN;
  __IO uint32_t ADD;
  __O  uint32_t RESET;
  __IO uint32_t REN;
  __I  uint32_t RBUF;
  __IO uint32_t RCR1;
  __IO uint32_t RCR2;
  __IO uint32_t RCR3;
  __IO uint32_t TEN;
  __IO uint32_t TBUF;
  __IO uint32_t TCR;
  __I  uint32_t RSTAT;
  __I  uint32_t TSTAT;
} CEC_TypeDef;

/*------------- Remote Control Signal Preprocessor (RMC) ---------------------*/
typedef struct
{
  __IO uint32_t EN;
  __IO uint32_t REN;
  __I  uint32_t RBUF1;
  __I  uint32_t RBUF2;
  __I  uint32_t RBUF3;
  __IO uint32_t RCR1;
  __IO uint32_t RCR2;
  __IO uint32_t RCR3;
  __IO uint32_t RCR4;
  __I  uint32_t RSTAT;
} RMC_TypeDef;

/*------------- Flash Control (FC) -------------------------------------------*/
typedef struct
{
  __I  uint32_t SECBIT;
       uint32_t RESERVED0[7];
  __I  uint32_t FLCS;
} FC_TypeDef;

/*------------- Chip ID (CID) ------------------------------------------------*/
typedef struct
{
  __IO uint32_t ID0;
  __IO uint32_t ID1;
       uint32_t RESERVED0[2];
  __IO uint32_t STCALIBV;
} CID_TypeDef;

/*------------- Boot (BOOT) --------------------------------------------------*/
typedef struct
{
  __IO uint32_t IDAD;
  __IO uint32_t PRDC1;
  __IO uint32_t PRDC2;
  __IO uint32_t PRDC3;
  __IO uint32_t RABG;
  __IO uint32_t RAEN;
  __IO uint32_t FLBG;
  __IO uint32_t FLEN;
  __IO uint32_t BLNM;
  __IO uint32_t BG16;
  __IO uint32_t BG32;
  __IO uint32_t BG64;
  __IO uint32_t BG12;
} BOOT_TypeDef;


#pragma no_anon_unions


/******************************************************************************/
/*                              Memory map                                    */
/******************************************************************************/

#define FLASH_BASE            (0x00000000UL)
#define RAM_BASE              (0x20000000UL)
#define PERI_BASE             (0x40000000UL)

#define PA_BASE               (PERI_BASE  + 0x00000)
#define PB_BASE               (PERI_BASE  + 0x00040)
#define PC_BASE               (PERI_BASE  + 0x00080)
#define PD_BASE               (PERI_BASE  + 0x000C0)
#define PE_BASE               (PERI_BASE  + 0x00100)
#define PF_BASE               (PERI_BASE  + 0x00140)
#define PG_BASE               (PERI_BASE  + 0x00180)
#define PH_BASE               (PERI_BASE  + 0x001C0)
#define PI_BASE               (PERI_BASE  + 0x00200)
#define PJ_BASE               (PERI_BASE  + 0x00240)
#define PK_BASE               (PERI_BASE  + 0x00280)
#define TB0_BASE              (PERI_BASE  + 0x10000)
#define TB1_BASE              (PERI_BASE  + 0x10040)
#define TB2_BASE              (PERI_BASE  + 0x10080)
#define TB3_BASE              (PERI_BASE  + 0x100C0)
#define TB4_BASE              (PERI_BASE  + 0x10100)
#define TB5_BASE              (PERI_BASE  + 0x10140)
#define TB6_BASE              (PERI_BASE  + 0x10180)
#define TB7_BASE              (PERI_BASE  + 0x101C0)
#define TB8_BASE              (PERI_BASE  + 0x10200)
#define TB9_BASE              (PERI_BASE  + 0x10240)
#define SBI0_BASE             (PERI_BASE  + 0x20000)
#define SBI1_BASE             (PERI_BASE  + 0x20020)
#define SBI2_BASE             (PERI_BASE  + 0x20040)
#define SIO0_BASE             (PERI_BASE  + 0x20080)
#define SIO1_BASE             (PERI_BASE  + 0x200C0)
#define SIO2_BASE             (PERI_BASE  + 0x20100)
#define ADC_BASE              (PERI_BASE  + 0x30000)
#define WDT_BASE              (PERI_BASE  + 0x40000)
#define RTC_BASE              (PERI_BASE  + 0x40100)
#define CG_BASE               (PERI_BASE  + 0x40200)
#define CEC_BASE              (PERI_BASE  + 0x40300)
#define RMC0_BASE             (PERI_BASE  + 0x40400)
#define RMC1_BASE             (PERI_BASE  + 0x40440)
#define FC_BASE               (PERI_BASE  + 0x40500)
#define CID_BASE              (PERI_BASE  + 0x40540)
#define BOOT_BASE             (PERI_BASE  + 0x40560)


/******************************************************************************/
/*                            Peripheral declaration                          */
/******************************************************************************/

#define PA                    ((   PORT_TypeDef *)       PA_BASE)
#define PB                    ((   PORT_TypeDef *)       PB_BASE)
#define PC                    ((   PORT_TypeDef *)       PC_BASE)
#define PD                    ((   PORT_TypeDef *)       PD_BASE)
#define PE                    ((   PORT_TypeDef *)       PE_BASE)
#define PF                    ((   PORT_TypeDef *)       PF_BASE)
#define PG                    ((   PORT_TypeDef *)       PG_BASE)
#define PH                    ((   PORT_TypeDef *)       PH_BASE)
#define PI                    ((   PORT_TypeDef *)       PI_BASE)
#define PJ                    ((   PORT_TypeDef *)       PJ_BASE)
#define PK                    ((   PORT_TypeDef *)       PK_BASE)
#define TB0                   ((   TMRB_TypeDef *)      TB0_BASE)
#define TB1                   ((   TMRB_TypeDef *)      TB1_BASE)
#define TB2                   ((   TMRB_TypeDef *)      TB2_BASE)
#define TB3                   ((   TMRB_TypeDef *)      TB3_BASE)
#define TB4                   ((   TMRB_TypeDef *)      TB4_BASE)
#define TB5                   ((   TMRB_TypeDef *)      TB5_BASE)
#define TB6                   ((   TMRB_TypeDef *)      TB6_BASE)
#define TB7                   ((   TMRB_TypeDef *)      TB7_BASE)
#define TB8                   ((   TMRB_TypeDef *)      TB8_BASE)
#define TB9                   ((   TMRB_TypeDef *)      TB9_BASE)
#define SBI0                  ((    SBI_TypeDef *)     SBI0_BASE)
#define SBI1                  ((    SBI_TypeDef *)     SBI1_BASE)
#define SBI2                  ((    SBI_TypeDef *)     SBI2_BASE)
#define SIO0                  ((    SIO_TypeDef *)     SIO0_BASE)
#define SIO1                  ((    SIO_TypeDef *)     SIO1_BASE)
#define SIO2                  ((    SIO_TypeDef *)     SIO2_BASE)
#define AD                    ((    ADC_TypeDef *)      ADC_BASE)
#define WD                    ((    WDT_TypeDef *)      WDT_BASE)
#define RTC                   ((    RTC_TypeDef *)      RTC_BASE)
#define CG                    ((     CG_TypeDef *)       CG_BASE)
#define CEC                   ((    CEC_TypeDef *)      CEC_BASE)
#define RMC0                  ((    RMC_TypeDef *)     RMC0_BASE)
#define RMC1                  ((    RMC_TypeDef *)     RMC1_BASE)
#define FC                    ((     FC_TypeDef *)       FC_BASE)
#define CID                   ((    CID_TypeDef *)      CID_BASE)
#define BOOT                  ((   BOOT_TypeDef *)     BOOT_BASE)


#endif  // __TMPM330_H__
