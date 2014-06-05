/*
 * @file:    UartDev_STM32.c
 * @purpose: UART Device Functions for STM32
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

#include "STM32.h"
#include "UartDev.h"
#include "UartDev_STM32.h"


/*----------------------------------------------------------------------------
  UART Device "STM32"
  defines to configure the UART device
 *----------------------------------------------------------------------------*/
#define  _USART_         1       /* used USART (1..3) */
#define  _USART_REMAP_   0       /* 0 = Pins NOT Remapped (default)
                                    1 = USART1 remap
                                    2 = USART2 remap
                                    3 = USART3 partial remap
                                    4 = USART3 full remap */

/*----------------------------------------------------------------------------
  UART Device "STM32"
  defines to configure the buffer handling
  Note: The length of the buffers must be a power of 2.
        The defined size is used for transmit and receive buffer
 *----------------------------------------------------------------------------*/
#define _BUF_SIZE_   128	     /* Must be a power of 2 (2,4,8,16,32,64,128,...) */



/*----------------------------------------------------------------------------
  UART Device "STM32" 
  function prototypes
 *----------------------------------------------------------------------------*/
static int  UartDev_Init    (void);
static int  UartDev_UnInit  (void);
static int  UartDev_BufTx   (void *pData, int* pSize, unsigned int flags);
static int  UartDev_BufRx   (void *pData, int* pSize, unsigned int flags);
static int  UartDev_BufFlush(void);

static void ConfigureUsart (UartDev_CFG* pCfg);
static void ConfigurePins  (void);




/*----------------------------------------------------------------------------
  UART Device "STM32" IO Block
 *----------------------------------------------------------------------------*/

#define UART_DEV     uart0


UartDev_IOB UART_DEV = {
                     {115200, UartDev_DATABITS_8, UartDev_STOPBITS_1, UartDev_PARITY_NONE},

                     UartDev_Init,
                     UartDev_UnInit,
                     UartDev_BufTx,
                     UartDev_BufRx,
                     UartDev_BufFlush
                       };


#define _STATE_UNKNOWN_          0
#define _STATE_INITIALIZED_      1

/*----------------------------------------------------------------------------
  Serial Device "STM32"
  local typedefs
 *----------------------------------------------------------------------------*/
typedef struct {
            int    UsartNr;                           /* USART number 1..n */
            int    State;                             /* Status */ 
  USART_TypeDef  *pUsart;                             /* pointer to USART struct */
       uint32_t    UsartPeriph;                       /* USART Paripheral Clock */
      IRQn_Type    UsartIRQn;                         /* USART Interrupt number */
} UART_INFO;


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  local variables
 *----------------------------------------------------------------------------*/
static UART_INFO UsartInfo = {
#if   (_USART_ == 1)
  1,                                                  /* USART#1 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  USART1,
  RCC_APB2ENR_USART1EN,
  USART1_IRQn

#elif (_USART_ == 2)
  2,                                                  /* USART#2 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  USART2,
  RCC_APB1ENR_USART2EN,
  USART2_IRQn

#elif (_USART_ == 3)
  3,                                                  /* USART#3 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  USART3,
  RCC_APB1ENR_USART3EN,
  USART3_IRQn

#else
  #error USART not defined!  
#endif

};



/*----------------------------------------------------------------------------
  Serial Device "STM32"
  initialize
 *----------------------------------------------------------------------------*/
static int UartDev_Init (void) {
  int i;

  if (UsartInfo.State != _STATE_UNKNOWN_) {
    return (-1);                                      /* Device initialized */
  }

  ConfigurePins ();                                   /* configure the USART pins */

  #if   (_USART_ == 1)
    RCC->APB2ENR |= UsartInfo.UsartPeriph;            /* enable clock for USART */
  #else
    RCC->APB1ENR |= UsartInfo.UsartPeriph;            /* enable clock for USART */
  #endif

  ConfigureUsart (&UART_DEV.Cfg); 

  UsartInfo.State = _STATE_INITIALIZED_;


  (UsartInfo.pUsart)->CR1 |=  (USART_CR1_RE | USART_CR1_TE); /* RX, TX enable */
  for (i = 0; i < 800; i++);                                 /* eleminate schmierzeichen */
  (UsartInfo.pUsart)->CR1 |=  (USART_CR1_UE);                /* USART enable */

  return(0);                                          /* success */
}


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  uninitialize
 *----------------------------------------------------------------------------*/
static int UartDev_UnInit (void) {

  (UsartInfo.pUsart)->CR1 &= ~(USART_CR1_RE | USART_CR1_TE); /* RX, TX disable */
  (UsartInfo.pUsart)->CR1 &= ~(USART_CR1_UE);                /* USART disable */

  UsartInfo.State = _STATE_UNKNOWN_;

  return(0);                                          /* Success */
}


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  Send buffer 
 *----------------------------------------------------------------------------*/
static int UartDev_BufTx (void *pData, int* pSize, unsigned int flags) {
  char *pChar;
   int   bytesToWrite;

  if (UsartInfo.State != _STATE_INITIALIZED_) {
    return (-1);                                      /* Device not initialized */
  }

  if ((pData == 0 ) && (*pSize > 0)) {
    return (-1);                                      /* Parameter not valid */
  }

  pChar = (char *)pData;
  bytesToWrite = *pSize;

  if (flags == UartDev_FLAG_BLOCKING) {               /* Blocking transmission */

    while (bytesToWrite-- > 0) {
      while (!(UsartInfo.pUsart->SR & USART_SR_TXE));
      UsartInfo.pUsart->DR = (*pChar++ & 0x1FF);      /* transmit character */
    }
  } else {                                            /* nonBlocking transmission */
    if (UsartInfo.pUsart->SR & USART_SR_TXE) {
      if (bytesToWrite-- > 0) {
        UsartInfo.pUsart->DR = (*pChar++ & 0x1FF);    /* transmit character */
        *pSize = 1;
      } else {
        *pSize = 0;
      }
    } else {
      *pSize = 0;
    } 
  }

  return(0);                                          /* success */
}


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  Read Buffer
 *----------------------------------------------------------------------------*/
static int UartDev_BufRx (void *pData, int* pSize, unsigned int flags) {
  char *pChar;
  int    bytesToRead;

  if (UsartInfo.State != _STATE_INITIALIZED_) {
    return (-1);                                      /* Device not initialized */
  }

  if ((pData == 0 ) && (*pSize > 0)) {
    return (-1);                                      /* Parameter not valid */
  }

  pChar = (char *)pData;
  bytesToRead = *pSize;

  if (flags == UartDev_FLAG_BLOCKING) {               /* Blocking transmission */
    while (bytesToRead-- > 0) {
      while (!(UsartInfo.pUsart->SR & USART_SR_RXNE));
      *pChar++ = (UsartInfo.pUsart->DR & 0x1FF);      /* receive character */
    }
  } else {                                            /* nonBlocking transmission */
    if (UsartInfo.pUsart->SR & USART_SR_RXNE) {
      *pChar++ = (UsartInfo.pUsart->DR & 0x1FF);      /* receive character */
      *pSize = 1;
    } else {
      *pSize = 0;
    } 
  }

  return(0);                                          /* Success */
}


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  Read Buffer
 *----------------------------------------------------------------------------*/
static int UartDev_BufFlush(void) {

  if (UsartInfo.State != _STATE_INITIALIZED_) {
    return (-1);                                      /* Device not initialized */
  }

  return(0);                                          /* Success */
}



/*----------------------------------------------------------------------------
  UART Device "STM32"
  Configure USART
 *----------------------------------------------------------------------------*/
static void ConfigureUsart (UartDev_CFG *pCfg) {

  #if   (_USART_ == 1)
    UsartInfo.pUsart->BRR = __USART_BRR(SystemFrequency_APB2Clk, pCfg->BaudRate); /* set baudrate */
  #else
    UsartInfo.pUsart->BRR = __USART_BRR(SystemFrequency_APB1Clk, pCfg->BaudRate); /* set baudrate */
  #endif

  switch (pCfg->DataBits) {                           /* set Data Bits */
    case UartDev_DATABITS_9:                          /* 9 Data bits */
      UsartInfo.pUsart->CR1 |=  (USART_CR1_M);
    break;
    case UartDev_DATABITS_8:                          /* 8 Data bits */
    default:
      UsartInfo.pUsart->CR1 &= ~(USART_CR1_M);
    break;
  }

  switch (pCfg->StopBits) {                           /* set Stop Bits */
    case UartDev_STOPBITS_2:                          /* 2   Stop Bits */
      UsartInfo.pUsart->CR2 &= ~(USART_CR2_STOP);
      UsartInfo.pUsart->CR2 |=  (USART_CR2_STOP_2);
      break;
    case UartDev_STOPBITS_0_5:                        /* 0,5 Stop Bits */
      UsartInfo.pUsart->CR2 &= ~(USART_CR2_STOP);
      UsartInfo.pUsart->CR2 |=  (USART_CR2_STOP_0_5);
      break;
    case UartDev_STOPBITS_1_5:                        /* 1,5 Stop Bits */
      UsartInfo.pUsart->CR2 &= ~(USART_CR2_STOP);
      UsartInfo.pUsart->CR2 |=  (USART_CR2_STOP_1_5);
      break;
    case UartDev_STOPBITS_1:                          /* 1   Stop Bit */
    default:
      UsartInfo.pUsart->CR2 &= ~(USART_CR2_STOP);
      UsartInfo.pUsart->CR2 |=  (USART_CR2_STOP_1);
      break;
  }

  switch (pCfg->Parity) {                             /* set Parity */
    case UartDev_PARITY_ODD:                          /* Parity Odd */
      UsartInfo.pUsart->CR1 |=  (USART_CR1_PS);
      UsartInfo.pUsart->CR1 |=  (USART_CR1_PCE);
    break;
    case UartDev_PARITY_EVEN:                         /* Parity Even */
      UsartInfo.pUsart->CR1 &= ~(USART_CR1_PS);
      UsartInfo.pUsart->CR1 |=  (USART_CR1_PCE);
    break;
    case UartDev_PARITY_NONE:                         /* Parity None */
    default:
      UsartInfo.pUsart->CR1 &= ~(USART_CR1_PCE);
      break;
  }

}

/*----------------------------------------------------------------------------
  UART Device "STM32"
  Configure USART Pins
 *----------------------------------------------------------------------------*/
static void ConfigurePins (void) {

#if   (_USART_ == 1)
  AFIO->MAPR   &= ~(AFIO_MAPR_USART1_REMAP);          /* clear USART1 remap */
  #if  (_USART_REMAP_ == 1)                           /* USART1 remap */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               /* enable clock for Alternate Function */
    AFIO->MAPR   |= AFIO_MAPR_USART1_REMAP;           /* set   USART1 remap */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;               /* enable clock for GPIOB */
    GPIOB->CRL   &= ~(0xFFUL  << 24);                 /* Clear PB6, PB7 */
    GPIOB->CRL   |=  (0x0BUL  << 24);                 /* USART1 Tx (PB6)  alternate output push-pull */
    GPIOB->CRL   |=  (0x04UL  << 28);                 /* USART1 Rx (PB7) input floating */
  #else                                               /* USART1 no remap */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;               /* enable clock for GPIOA */
    GPIOA->CRH   &= ~(0xFFUL  << 4);                  /* Clear PA9, PA10 */
    GPIOA->CRH   |=  (0x0BUL  << 4);                  /* USART1 Tx (PA9)  alternate output push-pull */
    GPIOA->CRH   |=  (0x04UL  << 8);                  /* USART1 Rx (PA10) input floating */
  #endif

#elif (_USART_ == 2)
  AFIO->MAPR   &= ~(AFIO_MAPR_USART2_REMAP);          /* clear USART2 remap */
  #if  (_USART_REMAP_ == 2)                           /* USART2 remap */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               /* enable clock for Alternate Function */
    AFIO->MAPR   |= AFIO_MAPR_USART2_REMAP;           /* set   USART2 remap */
    RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;              /* enable clock for GPIOD */
    GPIOD->CRL   &= ~(0xFFUL  << 20);                 /* Clear PD5, PD6 */
    GPIOD->CRL   |=  (0x0BUL  << 20);                 /* USART2 Tx (PD5)  alternate output push-pull */
    GPIOD->CRL   |=  (0x04UL  << 24);                 /* USART2 Rx (PD6)  input floating */
  #else                                               /* USART2 no remap */
    RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;              /* enable clock for GPIOA */
    GPIOA->CRL   &= ~(0xFFUL  << 8);                  /* Clear PA2, PA3 */
    GPIOA->CRL   |=  (0x0BUL  << 8);                  /* USART2 Tx (PA2)  alternate output push-pull */
    GPIOA->CRL   |=  (0x04UL  << 12);                 /* USART2 Rx (PA3)  input floating */
  #endif                                                

#elif (_USART_ == 3)
  AFIO->MAPR   &= ~(AFIO_MAPR_USART3_REMAP_F);        /* clear USART3 remap */
  #if   (_USART_REMAP_ == 4)                          /* USART3 full remap */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               /* enable clock for Alternate Function */
    AFIO->MAPR   |= AFIO_MAPR_USART3_REMAP_F;         /* set   USART3 remap */
    RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;              /* enable clock for GPIOD */
    GPIOD->CRH   &= ~(0xFFUL  <<  0);                 /* Clear PD8, PD9 */
    GPIOD->CRH   |=  (0x0BUL  <<  0);                 /* USART3 Tx (PD8) alternate output push-pull */
    GPIOD->CRH   |=  (0x04UL  <<  4);                 /* USART3 Rx (PD9) input floating */
  #elif (_USART_REMAP_ == 3)                          /* USART3 partial remap */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               /* enable clock for Alternate Function */
    AFIO->MAPR   |= AFIO_MAPR_USART3_REMAP_P;         /* set   USART3 remap */
    RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;              /* enable clock for GPIOC */
    GPIOC->CRH   &= ~(0xFFUL  <<  8);                 /* Clear PC10, PC11 */
    GPIOC->CRH   |=  (0x0BUL  <<  8);                 /* USART3 Tx (PC10) alternate output push-pull */
    GPIOC->CRH   |=  (0x04UL  << 12);                 /* USART3 Rx (PC11) input floating */
  #else                                               /* USART3 no remap */
    RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;              /* enable clock for GPIOB */
    GPIOB->CRH   &= ~(0xFFUL  <<  8);                 /* Clear PB10, PB11 */
    GPIOB->CRH   |=  (0x0BUL  <<  8);                 /* USART3 Tx (PB10) alternate output push-pull */
    GPIOB->CRH   |=  (0x04UL  << 12);                 /* USART3 Rx (PB11) input floating */
  #endif

#else
  #error USART not defined!  
#endif
}


