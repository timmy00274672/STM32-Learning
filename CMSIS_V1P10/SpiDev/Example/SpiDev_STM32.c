/*
 * @file:    SpiDev_STM32.c
 * @purpose: SPI Device Functions for STM32
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
#include "SpiDev.h"
#include "SpiDev_STM32.h"

/*----------------------------------------------------------------------------
  Note
  SPI1 Remapped uses JTAG pins
  SPI3          uses JTAG pins
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
  SPI Device "STM32"
  defines to configure the SPI device
 *----------------------------------------------------------------------------*/
#define  _SPI_         1         /* used SPI (1..3) */
#define  _SPI_REMAP_   0         /* 0 = Pins NOT Remapped (default)
                                    1 = SPI1 remap */

/*----------------------------------------------------------------------------
  SPI Device "STM32" 
  function prototypes
 *----------------------------------------------------------------------------*/
static int      SpiDev_Init    (void);
static int      SpiDev_UnInit  (void);
static int      SpiDev_BufTxRx (void *pDataTx, void *pDataRx, unsigned int Size);
static int      SpiDev_SetBaudrate(unsigned int Baudrate);

static uint32_t SpiCalculateBrc (unsigned int fPclk, unsigned int Baudrate);
static void     SpiConfigure    (SpiDev_CFG* pCfg);
static void     SpiConfigurePins(SpiDev_CFG *pCfg);
static uint8_t  SpiWriteRead    (uint8_t Data);



/*----------------------------------------------------------------------------
  SPI Device "STM32" IO Block
 *----------------------------------------------------------------------------*/
#define SPI_DEV     spi0


SpiDev_IOB SPI_DEV = {
                       { 500000, 
                         SpiDev_CLOCK_POLARITY_IDLELOW, 
                         SpiDev_CLOCK_PHASE_FIRSTEDGE, 
                         SpiDev_SSO_DISABLED
                       },

                       SpiDev_Init,
                       SpiDev_UnInit,
                       SpiDev_BufTxRx,
                       SpiDev_SetBaudrate
                     };


#define _STATE_UNKNOWN_          0
#define _STATE_INITIALIZED_      1

/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  local typedefs
 *----------------------------------------------------------------------------*/
typedef struct {
            int    SpiNr;                             /* Spi number 1..n */
            int    State;                             /* Status */ 
    SPI_TypeDef   *pSpi;                              /* pointer to USART struct */
   unsigned int    SpiPeriph;                         /* SPI Paripheral Clock */
      IRQn_Type    SpiIRQn;                           /* SPI Interrupt number */
} SPI_INFO;


/*----------------------------------------------------------------------------
  Serial Device "STM32"
  local variables
 *----------------------------------------------------------------------------*/
static SPI_INFO SpiInfo = {
#if   (_SPI_ == 1)
  1,                                                  /* SPI#1 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  SPI1,
  RCC_APB2ENR_SPI1EN,
  SPI1_IRQn

#elif (_SPI_ == 2)
  2,                                                  /* SPI#2 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  SPI2,
  RCC_APB1ENR_SPI2EN,
  SPI2_IRQn

#elif (_SPI_ == 3)
  3,                                                  /* SPI#3 */
  _STATE_UNKNOWN_,                                    /* unintialized */
  SPI3,
  RCC_APB1ENR_SPI3EN,
  SPI3_IRQn

#else
  #error SPI not defined!  
#endif

};


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  initialize
 *----------------------------------------------------------------------------*/
static int SpiDev_Init (void) {

  if (SpiInfo.State != _STATE_UNKNOWN_) {
    return (-1);                                      /* Device initialized */
  }

  SpiConfigurePins (&SPI_DEV.Cfg);                    /* configure the SPI pins */

  #if   (_SPI_ == 1)
    RCC->APB2ENR |= SpiInfo.SpiPeriph;                /* enable clock for SPI */
  #else
    RCC->APB1ENR |= SpiInfo.SpiPeriph;                /* enable clock for SPI */
  #endif

  SpiConfigure (&SPI_DEV.Cfg); 

  SpiInfo.State = _STATE_INITIALIZED_;


  (SpiInfo.pSpi)->CR1 |=  SPI_CR1_SPE;                /* enable SPI */

//  (SpiInfo.pSpi)->CR2 |=  (SPI_CR2_TXEIE | SPI_CR2_RXNEIE); 
//   NVIC_EnableIRQ(SpiInfo.SpiIRQn);                   /* enable Interrupt */      

  return(0);                                          /* success */
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  uninitialize
 *----------------------------------------------------------------------------*/
static int SpiDev_UnInit (void) {

//   NVIC_DisableIRQ(SpiInfo.SpiIRQn);                  /* disable Interrupt */
//  (SpiInfo.pSpi)->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE); 

  (SpiInfo.pSpi)->CR1 &= ~SPI_CR1_SPE;                /* disable SPI */

  SpiInfo.State = _STATE_UNKNOWN_;

  return(0);                                          /* Success */
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  Send / receive buffer 
 *----------------------------------------------------------------------------*/
static int SpiDev_BufTxRx (void *pDataTx, void *pDataRx, unsigned int Size) {
  unsigned char *pCharTx, *pCharRx;
            int   bytesToWrite;

  if (SpiInfo.State != _STATE_INITIALIZED_) {
    return (-1);                                      /* Device not initialized */
  }

  if ((pDataTx == 0) || (pDataRx == 0)) {
    return (-1);                                      /* Parameter not valid */
  }

  pCharTx = (uint8_t *)pDataTx;
  pCharRx = (uint8_t *)pDataRx;
  bytesToWrite = Size;

  while (bytesToWrite-- > 0) {
    *pCharRx++ = SpiWriteRead (*pCharTx++);
  }

  return(0);                                          /* success */
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  Set Baudrate
 *----------------------------------------------------------------------------*/
static int SpiDev_SetBaudrate (unsigned int Baudrate) {
  uint32_t brc;

  if (SpiInfo.State != _STATE_INITIALIZED_) {
    return (-1);                                      /* Device not initialized */
  }

  #if   (_SPI_ == 1)
    brc = SpiCalculateBrc (SystemFrequency_APB2Clk, Baudrate);
  #else
    brc = SpiCalculateBrc (SystemFrequency_APB1Clk, Baudrate);
  #endif

  (SpiInfo.pSpi)->CR1 &= ~(          0x07 <<  3);     /* clear Baude Rate Control */ 
  (SpiInfo.pSpi)->CR1 |=  (brc            <<  3);     /* Baude Rate Control */

  return(0);                                          /* Success */
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  Configure SPI
 *----------------------------------------------------------------------------*/
static void SpiConfigure (SpiDev_CFG *pCfg) {
  uint32_t brc;

  #if   (_SPI_ == 1)
    brc = SpiCalculateBrc (SystemFrequency_APB2Clk, pCfg->Baudrate);
  #else
    brc = SpiCalculateBrc (SystemFrequency_APB1Clk, pCfg->Baudrate);
  #endif

  (SpiInfo.pSpi)->CR1  =
                (             0   << 11) |            /* 8-bit data frame format */
                (             1   <<  9) |            /* Software Slave Management */
                (             1   <<  8) |            /* Internal Slave Select */
                (             0   <<  7) |            /* MSB transmitted first */
//                (             1   <<  6) |            /* SPI enable */
                (brc              <<  3) |            /* Baude Rate Control */
                (             1   <<  2) |            /* Master mode */
                (pCfg->Polarity   <<  1) |            /* Clock Polarity */
                (pCfg->Phase      <<  0);             /* Clock Phase */

  (SpiInfo.pSpi)->CR2  = 
                (pCfg->SlaveSelect <<  2);            /* Slave Select Output */
         
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  calculate Baude Rate Control value
 *----------------------------------------------------------------------------*/
static uint32_t SpiCalculateBrc (unsigned int fPclk, unsigned int Baudrate) {
      uint32_t brc;
  unsigned int BaudrateCur;

  for (brc = 0; brc < 7; brc = brc + 1) {
    BaudrateCur = fPclk / (1<<(brc+1));
    if (BaudrateCur <= Baudrate) break;
  }

  return (brc);
} 


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  Write and Read a byte on SPI interface
 *----------------------------------------------------------------------------*/
static uint8_t SpiWriteRead (uint8_t Data) {

   while (!((SpiInfo.pSpi)->SR & SPI_SR_TXE));        /* Wait if TXE cleared, Tx FIFO is full. */
   (SpiInfo.pSpi)->DR = Data;

   while (!((SpiInfo.pSpi)->SR & SPI_SR_RXNE));       /* Wait if RNE cleared, Rx FIFO is empty. */
   return ((SpiInfo.pSpi)->DR);
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  Configure SPI Pins
 *----------------------------------------------------------------------------*/
static void SpiConfigurePins (SpiDev_CFG *pCfg) {

#if   (_SPI_ == 1)
  AFIO->MAPR   &= ~(AFIO_MAPR_SPI1_REMAP);            /* clear SPI1 remap */
  #if  (_SPI_REMAP_ == 1)                             /* SPI1 remap */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;               /* enable clock for Alternate Function */
    AFIO->MAPR   |= AFIO_MAPR_SPI1_REMAP;             /* set   USART1 remap */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;               /* enable clock for GPIOB */
    GPIOB->CRL   &= ~(0x0FFFUL  << 12);               /* Clear PB3..PB5 */
    GPIOB->CRL   |=  (0x0BUL    << 12);               /* SPI SCK  (PB3)  alternate output push-pull 50MHz */
    GPIOB->CRL   |=  (0x04UL    << 16);               /* SPI MISO (PB4)  input Pull up/donw */
    GPIOB->CRL   |=  (0x0BUL    << 20);               /* SPI MOSI (PB5)  alternate output push-pull 50MHz */
    if (pCfg->SlaveSelect == SpiDev_SSO_ENABLED) {
      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;             /* enable clock for GPIOA */
      GPIOA->CRH   &= ~(0x0FUL    << 28);             /* Clear PA15 */
      GPIOA->CRH   |=  (0x0BUL    << 18);             /* SPI NSS  (PA15) alternate output push-pull 50MHz */
    }
  #else                                               /* USART1 no remap */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;               /* enable clock for GPIOA */
    GPIOA->CRL   &= ~(0x0FFFUL  << 20);               /* Clear PA5..PA7 */
    GPIOA->CRL   |=  (0x0BUL    << 20);               /* SPI SCK  (PA5)  alternate output push-pull 50MHz */
    GPIOA->CRL   |=  (0x04UL    << 24);               /* SPI MISO (PA6)  input Pull up/donw */
    GPIOA->CRL   |=  (0x0BUL    << 28);               /* SPI MOSI (PA7)  alternate output push-pull 50MHz */
    if (pCfg->SlaveSelect == SpiDev_SSO_ENABLED) {
    GPIOA->CRL   &= ~(0x000FUL  << 16);               /* Clear PA4 */
    GPIOA->CRL   |=  (0x0BUL    << 16);               /* SPI NSS  (PA4)  alternate output push-pull 50MHz */
    }
  #endif

#elif (_SPI_ == 2)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;               /* enable clock for GPIOB */
    GPIOB->CRH   &= ~(0x0FFFUL  << 20);               /* Clear PB13..PB15 */
    GPIOB->CRH   |=  (0x0BUL    << 20);               /* SPI SCK  (PB13) alternate output push-pull 50MHz */
    GPIOB->CRH   |=  (0x04UL    << 24);               /* SPI MISO (PB14) input Pull up/donw */
    GPIOB->CRH   |=  (0x0BUL    << 28);               /* SPI MOSI (PB15) alternate output push-pull 50MHz */
    if (pCfg->SlaveSelect == SpiDev_SSO_ENABLED) {
    GPIOB->CRH   &= ~(0x000FUL  << 16);               /* Clear PB12 */
    GPIOB->CRH   |=  (0x0BUL    << 16);               /* SPI NSS  (PB12) alternate output push-pull 50MHz */
    }

#elif (_SPI_ == 3)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;               /* enable clock for GPIOB */
    GPIOB->CRL   &= ~(0x0FFFUL  << 12);               /* Clear PB3..PB5 */
    GPIOB->CRL   |=  (0x0BUL    << 12);               /* SPI SCK  (PB3)  alternate output push-pull 50MHz */
    GPIOB->CRL   |=  (0x04UL    << 16);               /* SPI MISO (PB4)  input Pull up/donw */
    GPIOB->CRL   |=  (0x0BUL    << 20);               /* SPI MOSI (PB5)  alternate output push-pull 50MHz */
    if (pCfg->SlaveSelect == SpiDev_SSO_ENABLED) {
      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;             /* enable clock for GPIOA */
      GPIOA->CRH   &= ~(0x0FUL    << 28);             /* Clear PA15 */
      GPIOA->CRH   |=  (0x0BUL    << 18);             /* SPI NSS  (PA15) alternate output push-pull 50MHz */
    }

#else
  #error SPI not defined!  
#endif
}


/*----------------------------------------------------------------------------
  SPI Serial Device "STM32"
  SPI Interrupt function.
  (not static because of startup_stm32f10x.s) 
 *----------------------------------------------------------------------------*/
#if   (_SPI_ == 1)
void SPI1__IRQHandler (void)
#elif (_SPI_ == 2)
void SPI2_IRQHandler (void)
#elif (_SPI_ == 3)
void SPI3_IRQHandler (void)
#else
  #error SPI not defined!  
#endif
{
  volatile unsigned int iir;

    iir = SpiInfo.pSpi->SR;

    SpiInfo.pSpi->SR &=  ~iir;
}
