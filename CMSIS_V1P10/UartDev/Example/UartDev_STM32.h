/*
 * @file:    UartDev_STM32.h
 * @purpose: USART Serial Device Definitions for STM32
 * @version: V1.10 First Release
 * @date:    24. Feb. 2009
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
 */

#ifndef __UARTDEV_STM32_H
#define __UARTDEV_STM32_H

/*----------------------------------------------------------------------------
  Reset and clock control (RCC)
 *----------------------------------------------------------------------------*/
/* register RCC_APB1ENR ------------------------------------------------------*/
#define RCC_APB1ENR_UART5EN      ((uint32_t)(1 <<20))
#define RCC_APB1ENR_UART4EN      ((uint32_t)(1 <<19))
#define RCC_APB1ENR_USART3EN     ((uint32_t)(1 <<18))
#define RCC_APB1ENR_USART2EN     ((uint32_t)(1 <<17))

/* register RCC_APB2ENR ------------------------------------------------------*/
#define RCC_APB2ENR_USART1EN     ((uint32_t)(1 <<14))
#define RCC_APB2ENR_IOPDEN       ((uint32_t)(1 << 5))
#define RCC_APB2ENR_IOPCEN       ((uint32_t)(1 << 4))
#define RCC_APB2ENR_IOPBEN       ((uint32_t)(1 << 3))
#define RCC_APB2ENR_IOPAEN       ((uint32_t)(1 << 2))
#define RCC_APB2ENR_AFIOEN       ((uint32_t)(1 << 0))

/*----------------------------------------------------------------------------
  General-purpose and alternate-function I/Os (GPIOs and AFIOs)
 *----------------------------------------------------------------------------*/
/* register AFIO_MAPR --------------------------------------------------------*/
#define AFIO_MAPR_USART3_REMAP_F ((uint32_t)(3 << 4))
#define AFIO_MAPR_USART3_REMAP_P ((uint32_t)(1 << 4))
#define AFIO_MAPR_USART2_REMAP   ((uint32_t)(1 << 3))
#define AFIO_MAPR_USART1_REMAP   ((uint32_t)(1 << 2))


/*----------------------------------------------------------------------------
  Universal synchronous asynchronous receiver transmitter (USART)
 *----------------------------------------------------------------------------*/
/* register USART_SR ---------------------------------------------------------*/
#define USART_SR_CTS             ((uint32_t)(1 << 9))
#define USART_SR_LBD             ((uint32_t)(1 << 8))
#define USART_SR_TXE             ((uint32_t)(1 << 7))
#define USART_SR_TC              ((uint32_t)(1 << 6))
#define USART_SR_RXNE            ((uint32_t)(1 << 5))
#define USART_SR_IDLE            ((uint32_t)(1 << 4))
#define USART_SR_ORE             ((uint32_t)(1 << 3))
#define USART_SR_NE              ((uint32_t)(1 << 2))
#define USART_SR_FE              ((uint32_t)(1 << 1))
#define USART_SR_PE              ((uint32_t)(1 << 0))

/* register USART_CR1 --------------------------------------------------------*/
#define USART_CR1_UE             ((uint32_t)(1 <<13))   /* USART Enable */
#define USART_CR1_M              ((uint32_t)(1 <<12))   /* word length */
#define USART_CR1_PCE            ((uint32_t)(1 <<10))   /* Parity Control Enable */
#define USART_CR1_PS             ((uint32_t)(1 << 9))   /* Parity Selection */
#define USART_CR1_TXEIE          ((uint32_t)(1 << 7))   /* TXE Interrupt Enable */
#define USART_CR1_RXNEIE         ((uint32_t)(1 << 5))   /* RXNE Interrupt Enable */
#define USART_CR1_TE             ((uint32_t)(1 << 3))   /* Transmitter Enable */
#define USART_CR1_RE             ((uint32_t)(1 << 2))   /* Receiver Enable */

/* register USART_CR2 --------------------------------------------------------*/
#define USART_CR2_STOP           ((uint32_t)(3 <<12))
#define USART_CR2_STOP_0_5       ((uint32_t)(1 <<12))
#define USART_CR2_STOP_1         ((uint32_t)(0 <<12))
#define USART_CR2_STOP_1_5       ((uint32_t)(3 <<12))
#define USART_CR2_STOP_2         ((uint32_t)(2 <<12))

/* register USART_CR3 --------------------------------------------------------*/
#define USART_CR3_CTSE           ((uint32_t)(1 << 9))
#define USART_CR3_RTSE           ((uint32_t)(1 << 8))


/*----------------------------------------------------------------------------
 Define  Baudrate setting (BRR) for USART 
 *----------------------------------------------------------------------------*/
#define __DIV(_PCLK_, _BAUD_)       ((_PCLK_*25)/(4*_BAUD_))
#define __DIVMANT(_PCLK_, _BAUD_)   (__DIV(_PCLK_, _BAUD_)/100)
#define __DIVFRAQ(_PCLK_, _BAUD_)   (((__DIV(_PCLK_, _BAUD_) - (__DIVMANT(_PCLK_, _BAUD_) * 100)) * 16 + 50) / 100)
#define __USART_BRR(_PCLK_, _BAUD_) ((__DIVMANT(_PCLK_, _BAUD_) << 4)|(__DIVFRAQ(_PCLK_, _BAUD_) & 0x0F))



#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

