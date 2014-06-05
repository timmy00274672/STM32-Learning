/*
 * @file:    SpiDev_STM32.h
 * @purpose: SPI Serial Device Definitions for STM32
 * @version: V1.10 First Release
 * @date:    24. Feb. 2009
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2008 ARM Limited. All rights reserved.
 *
 * ARM Limited (ARM) is supplying this software for use with Cortex-M3
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

#ifndef __SPIDEV_STM32_H
#define __SPIDEV_STM32_H


/*----------------------------------------------------------------------------
  Reset and clock control (RCC)
 *----------------------------------------------------------------------------*/
/* register RCC_APB1ENR ------------------------------------------------------*/
#define RCC_APB1ENR_SPI3EN       ((uint32_t)(1 <<15))
#define RCC_APB1ENR_SPI2EN       ((uint32_t)(1 <<14))

/* register RCC_APB2ENR ------------------------------------------------------*/
#define RCC_APB2ENR_SPI1EN       ((uint32_t)(1 <<12))
#define RCC_APB2ENR_IOPDEN       ((uint32_t)(1 << 5))
#define RCC_APB2ENR_IOPCEN       ((uint32_t)(1 << 4))
#define RCC_APB2ENR_IOPBEN       ((uint32_t)(1 << 3))
#define RCC_APB2ENR_IOPAEN       ((uint32_t)(1 << 2))
#define RCC_APB2ENR_AFIOEN       ((uint32_t)(1 << 0))

/*----------------------------------------------------------------------------
  General-purpose and alternate-function I/Os (GPIOs and AFIOs)
 *----------------------------------------------------------------------------*/
/* register AFIO_MAPR --------------------------------------------------------*/
#define AFIO_MAPR_SPI1_REMAP     ((uint32_t)(1 << 0))


/*----------------------------------------------------------------------------
  Serial peripheral interface (SPI)
 *----------------------------------------------------------------------------*/
/* register SPI_CR1 ----------------------------------------------------------*/
#define SPI_CR1_SPE              ((uint32_t)(1 << 6))   /* SPI enable */

/* register SPI_CR2 ----------------------------------------------------------*/
#define SPI_CR2_TXEIE            ((uint32_t)(1 << 7))   /* TX Buffer empty Interrupt */
#define SPI_CR2_RXNEIE           ((uint32_t)(1 << 6))   /* RX Buffer not empty Interrupt */

/* register SPI_SR -----------------------------------------------------------*/
#define SPI_SR_BSY               ((uint32_t)(1 << 7))   /* Busy flag */
#define SPI_SR_TXE               ((uint32_t)(1 << 1))   /* Transmitter empty */
#define SPI_SR_RXNE              ((uint32_t)(1 << 0))   /* Receiver not empty */




#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

