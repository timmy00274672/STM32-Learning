/*----------------------------------------------------------------------------
 * Name:    Usart.c
 * Purpose: USART usage for STM32
 * Version: V1.00
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
#include "UartDev.h"


char bufTx[20];
char bufRx[20];

int  retCode;

/*----------------------------------------------------------------------------
  Interface to UartDev
 *----------------------------------------------------------------------------*/
extern UartDev_IOB  uart0;
       UartDev_IOB *pUart;

/*----------------------------------------------------------------------------
  SendChar
  Write character to Serial Port.
 *----------------------------------------------------------------------------*/
int SendChar (int ch)  {
  int size = 1;

  retCode =  pUart->BufTx ((void *)&ch, &size, UartDev_FLAG_BLOCKING);
  return (ch);
}


/*----------------------------------------------------------------------------
  GetKey
  Read character to Serial Port.
 *----------------------------------------------------------------------------*/
int GetKey (void)  {
  int ch;
  int size = 1;


  retCode =  pUart->BufRx ((void *)&ch, &size, UartDev_FLAG_BLOCKING);
  if (size == 0) 
    ch = -1;

  return (ch);
}


/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
  int i;
  int sizeRx, sizeTx;

  SystemInit ();                                  /* initialize the clocks */
  pUart = &uart0;          /* Select 'uart0' as active communication interface. */
  pUart->Init();

  printf ("CMSIS Serial Device Example\r\n\r\n");

  while (1) {
    unsigned char c;

    printf ("\r\nPress a key. ");
    c = getchar ();
    printf ("\r\n");
    printf ("You pressed '%c'.\r\n\r\n", c);

    while (1) {
      for (i = 0; i < 1000000; i++);
      sizeRx = 10;
      retCode =  pUart->BufRx ((void *)&bufRx[0], &sizeRx, UartDev_FLAG_NONBLOCKING);
      for (i = 0; i < sizeRx; i++) bufTx[i] = bufRx[i];
      sizeTx = sizeRx;
      retCode =  pUart->BufTx ((void *)&bufTx[0], &sizeTx, UartDev_FLAG_NONBLOCKING);
      if (sizeRx > 0) {
        strcpy (bufTx, "\r\n");
        sizeTx = 2;
        retCode =  pUart->BufTx ((void *)&bufTx[0], &sizeTx, UartDev_FLAG_BLOCKING);
      }
    }
  }
}
