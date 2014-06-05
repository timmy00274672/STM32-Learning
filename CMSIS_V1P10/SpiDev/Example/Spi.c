/*----------------------------------------------------------------------------
 * Name:    Spi.c
 * Purpose: SPI usage for STM32
 * Version: V1.00
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------
 * History:
 *          V1.00 Initial Version
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "stm32.h"
#include "SpiDev.h"

#include "spi_AT25.h"


char bufTx[256];
char bufRx[256];

int  retCode;

/*----------------------------------------------------------------------------
  Interface to SpiDev
 *----------------------------------------------------------------------------*/
extern SpiDev_IOB  spi0;
       SpiDev_IOB *pSpi;


/*----------------------------------------------------------------------------
  Initialize SPI Chip Select
 *----------------------------------------------------------------------------*/
void SPI_SlaveSelectInit (void) {
  RCC->APB2ENR |= (1 << 3);                          /* enable clock for GPIOB */
  GPIOB->BSRR    = (0x00000100);                     /* SlaveSelect high */
  GPIOB->CRH   &= ~(0x0FUL    << 0);                 /* Clear PB8 */
  GPIOB->CRH   |=  (0x03UL    <<  0);                /* PB8 output push-pull 50MHz */
}


/*----------------------------------------------------------------------------
  Enable/Disable SPI Chip Select
 *----------------------------------------------------------------------------*/
void SPI_SlaveSelect (unsigned char ss) {
  if (ss) {
    GPIOB->BSRR = (0x00000100);                      /* SlaveSelect high */
  } else {
    GPIOB->BRR  = (0x00000100);                      /* SlaveSelect low */
  }
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

  SystemInit ();                                     /* initialize the clocks */

  pSpi = &spi0;            /* Select 'spi0' as active communication interface */
  pSpi->Cfg.SlaveSelect = SpiDev_SSO_DISABLED; /* use own generated SS signal */
  SPI_SlaveSelectInit();
  pSpi->Init();

  strcpy (bufTx, "Testing AT25 SPI Flash on STM32103F ...");
  SPI_MemWrite(0x400, 36,  (unsigned char*)bufTx);
  strcpy (bufTx, "Page Offset 0x10: ...");
  SPI_MemWrite(0x450, 21,  (unsigned char*)bufTx);
  SPI_MemRead (0x400, 128, (unsigned char*)bufRx);

//  strcpy (bufTx, "1234567890123456");
//  retCode =  pSpi->BufTxRx ((void *)&bufTx[0], (void *)&bufRx[0], 16);
//
//  retCode = pSpi->SetBaudrate (18000000);
//
//  strcpy (bufTx, "abcdefghijklmnop");
//  retCode =  pSpi->BufTxRx ((void *)&bufTx[0], (void *)&bufRx[0], 16);

  while (1) ;
}
