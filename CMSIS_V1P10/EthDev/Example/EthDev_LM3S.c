/*
 * @file:    EthDev_LM3S.c
 * @purpose: Ethernet Device Functions for LM3S6965
 * @version: V1.10
 * @date:    24. Feb. 2009
 *----------------------------------------------------------------------------
 *
 * Copyright (C) 2009 ARM Limited. All rights reserved.
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

#include <LM3Sxxxx.H>

#include "EthDev.h"
#include "EthDev_LM3S.h"

/*----------------------------------------------------------------------------
  Ethernet Device local functions
 *----------------------------------------------------------------------------*/
static int           EthDev_Init        (void);
static int           EthDev_UnInit      (void);
static int           EthDev_SetMCFilter (int NumHwAddr, unsigned char *pHwAddr);
static int           EthDev_TxFrame     (void *pData, int size);
static void          EthDev_Lock        (void);
static void          EthDev_UnLock      (void);
static EthDev_STATUS EthDev_LinkChk     (void);

static int           phy_Rd             (unsigned int PhyReg);
static int           phy_Wr             (unsigned int PhyReg, unsigned short Data);

/*----------------------------------------------------------------------------
  Ethernet Device IO Block
 *----------------------------------------------------------------------------*/

#define ETH     eth0


EthDev_IOB ETH = { EthDev_MODE_AUTO,
                   0,0,0,0,0,0,
                   NULL,
                   NULL,

                   EthDev_Init,
                   EthDev_UnInit,
                   EthDev_SetMCFilter,
                   EthDev_TxFrame,
                   EthDev_Lock,
                   EthDev_UnLock,
                   EthDev_LinkChk
};

static EthDev_STATUS Status;

/*----------------------------------------------------------------------------
  Ethernet Device initialize
 *----------------------------------------------------------------------------*/
static int EthDev_Init (void) {
  int regv,tout;

  SysCtlPeripheralEnable (SYSCTL_PERIPH_ETH);        /* Enable and Reset the Ethernet Controller */
  SysCtlPeripheralReset (SYSCTL_PERIPH_ETH);

  SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);      /* Enable Port F for Ethernet LEDs */
  GPIODirModeSet  (GPIO_PORTF_BASE, (GPIO_PIN_2|GPIO_PIN_3), GPIO_DIR_MODE_HW);
  GPIOPadConfigSet(GPIO_PORTF_BASE, (GPIO_PIN_2|GPIO_PIN_3), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

  pEMAC->IEN   = 0;                                  /* Disable all Ethernet Interrupts */
  pEMAC->ISR   = INT_ALL;
  pEMAC->MCDIV = 10;                                 /* Set MII interface clock (max. 2.5MHz) */
  pEMAC->MADR  = PHY_DEF_ADR;                        /* set the PHY address */

  phy_Wr (PHY_REG_CTRL, 0x8000);

  /* Wait for hardware reset to end. */
  for (tout = 0x20000; ; tout--) {
    regv = phy_Rd (PHY_REG_CTRL);
    if (regv < 0 || tout == 0) {
       /* Error */
       return (-1);
    }
    if (!(regv & 0x8000)) {
       /* Reset complete. */
       break;
    }
  }

  switch (ETH.Mode) {
    case EthDev_MODE_AUTO:
      phy_Wr (PHY_REG_CTRL, PHY_AUTO_NEG);
      break;

    case EthDev_MODE_10M_FULL:
      phy_Wr (PHY_REG_CTRL, PHY_10M_FULL);
      break;

    case EthDev_MODE_10M_HALF:
      phy_Wr (PHY_REG_CTRL, PHY_10M_HALF);
      break;

    case EthDev_MODE_100M_FULL:
      phy_Wr (PHY_REG_CTRL, PHY_100M_FULL);
      break;

    case EthDev_MODE_100M_HALF:
      phy_Wr (PHY_REG_CTRL, PHY_100M_HALF);
      break;

    default:
       return (-1);
  }

  /* Configure TX/RX Control Registers, enable Multicast. */
  pEMAC->RXCTL = RXCTL_BAD_CRC | RXCTL_RST_FIFO;

  pEMAC->TXCTL = TXCTL_PAD_EN  | TXCTL_CRC_EN;

  /* Set the Ethernet MAC Address registers */
  pEMAC->IAR1   = ((unsigned int)ETH.HwAddr[5] << 8)  |
                   (unsigned int)ETH.HwAddr[4];
  pEMAC->IAR0   = ((unsigned int)ETH.HwAddr[3] << 24) |
                  ((unsigned int)ETH.HwAddr[2] << 16) |
                  ((unsigned int)ETH.HwAddr[1] << 8)  |
                   (unsigned int)ETH.HwAddr[0];

  pEMAC->RXCTL |= RXCTL_RX_EN;                       /* Enable the Ethernet Controller */
  pEMAC->TXCTL |= TXCTL_TX_EN;

  pEMAC->IEN    = INT_RX;
  IntEnable (INT_ETH);

  Status.Link = EthDev_LINK_DOWN;
  return (0);
}


/*----------------------------------------------------------------------------
  Ethernet Device Uninitialize
 *----------------------------------------------------------------------------*/
static int EthDev_UnInit (void) {

  pEMAC->IEN = 0;                                    /* Disable all Ethernet Interrupts */
  pEMAC->ISR = INT_ALL;
  IntDisable (INT_ETH);                              /* Disable ENET Interrupt */

  /* Power Down PHY */
  phy_Wr (PHY_REG_CTRL, 0x0800);

  return (0);
}


/*----------------------------------------------------------------------------
  Ethernet Device Interrupt Disable
 *----------------------------------------------------------------------------*/
static void EthDev_Lock (void) {
  pEMAC->IEN = 0;
}


/*----------------------------------------------------------------------------
  Ethernet Device Interrupt Enable
 *----------------------------------------------------------------------------*/
static void EthDev_UnLock (void) {
  pEMAC->IEN = INT_RX;
}


/*----------------------------------------------------------------------------
  Set Multicast Filter Address to ethernet controller 
 *----------------------------------------------------------------------------*/
static int EthDev_SetMCFilter (int NumHwAddr, unsigned char *pHwAddr) {

  /* luminary profides no address filter nor hash table only enable /disable */
  if (NumHwAddr) {
    pEMAC->RXCTL |=  RXCTL_MCAST_EN;                 /* Enable Multicast */
  } else {
    pEMAC->RXCTL &= ~RXCTL_MCAST_EN;                 /* Disable Multicast */
  }

  return 0;
}


/*----------------------------------------------------------------------------
  Send frame to ENET ethernet controller 
 *----------------------------------------------------------------------------*/
static int EthDev_TxFrame (void *pData, int size) {
  unsigned short *sp;
  unsigned int len,val,tout;

  for (tout = TX_FRAME_TOUT;  ; tout--) {
    if ((pEMAC->TXRQ & TXRQ_NEW_TX) == 0) {
      break;
    }
    if (tout == 0) {
      return (-1);                                   /* Error Timeout */
    }
  }

  sp   = (unsigned short *)pData;
  val  = size - 14;
  val |= (*sp++) << 16;
  pEMAC->DATA = val;

  for (len = (size + 1)>>2; len; sp += 2, len--) {   /* Copy frame data to EMAC packet buffers. */
    pEMAC->DATA = sp[0] | ((unsigned int)sp[1] << 16);
  }
  
  pEMAC->TXRQ = TXRQ_NEW_TX;                         /* Start Transmitting */

  return (0);
}


/*----------------------------------------------------------------------------
  Ethernet Device Ethernet Controller Interrupt function.
  (not static because of Startup.s) 
 *----------------------------------------------------------------------------*/
void EthernetIntHandler (void) __irq {               /* EthDev_ISR */
  unsigned int int_stat,val,RxLen,len;
  unsigned short *dp;

  while ((int_stat = pEMAC->ISR & INT_ALL) != 0) {
    pEMAC->ISR = int_stat;                           /* Acknowledge Interrupt */

    if (int_stat & INT_RX) {                         /* Receive interrupt */
      val   = pEMAC->DATA;
      RxLen = (val & 0xFFFF) - 6;
      /* DMA adds also 4-byte CRC and 2-byte 'size' to packet size. */
      if (RxLen > EthDev_MTU_SIZE || (int_stat & INT_RX_ERR)) {
        /* Invalid frame, ignore it and free buffer. */
        goto rel;
      }
      dp = ETH.RxFrame (RxLen);
      if (dp != NULL) {
//        pEMAC->RXCTL &= ~RXCTL_RX_EN;
        for (len = (RxLen + 3) >> 2; len; dp += 2, len--) {
          dp[0] = val >> 16;
          val = pEMAC->DATA;
          dp[1] = val & 0xFFFF;
        }
        if (ETH.RxFrameReady) {                      /* call RxFrameReady() if provided */
          ETH.RxFrameReady (RxLen);
        }
        if ((RxLen - 1) & 0x02) {
          /* Drain the remaining 1 or 2 byte(s) of the CRC. */
          val = pEMAC->DATA;
        }
//        pEMAC->RXCTL |= RXCTL_RX_EN;
      }
      else {
        /* In case of error discard the entire RX FIFO. */
rel:    pEMAC->RXCTL &= ~RXCTL_RX_EN;
        pEMAC->RXCTL |= RXCTL_RST_FIFO;
        pEMAC->RXCTL |= RXCTL_RX_EN;
      }
    }
  }
}


/*----------------------------------------------------------------------------
  Ethernet Device Check the Link
 *----------------------------------------------------------------------------*/
static EthDev_STATUS EthDev_LinkChk (void) {
  int phyData;

  phyData = phy_Rd (PHY_REG_STAT);
  if (phyData < 0) {
    return (Status);
  }

  if (Status.Link == EthDev_LINK_DOWN) {
    if (phyData & 0x0004) {
      Status.Link = EthDev_LINK_UP;
      phyData = phy_Rd (PHY_REG_DIAG);
      if (phyData < 0) {
        return (Status);
      }
      Status.Duplex = (phyData & 0x0800) ? EthDev_DUPLEX_FULL : EthDev_DUPLEX_HALF;
      Status.Speed  = (phyData & 0x0400) ? EthDev_SPEED_100M  : EthDev_SPEED_10M;
      if (Status.Duplex == EthDev_DUPLEX_FULL) {
        /* Full duplex is enabled. */
        pEMAC->TXCTL |= TXCTL_DUP_EN;
      }
    }
  }
  else {
    if ((phyData & 0x0004) == 0) {
      Status.Link   = EthDev_LINK_DOWN;
    }
  }
  return (Status);
}


/*----------------------------------------------------------------------------
  Write a data 'Data' to PHY register 'PhyReg'.  
 *----------------------------------------------------------------------------*/
static int phy_Wr (unsigned int PhyReg, unsigned short Data) {
  unsigned int timeOut;

  pEMAC->MTXD = Data;
  pEMAC->MCTL = (PhyReg << 3) | MCTL_WR | MCTL_START;

  for (timeOut = 0; timeOut < MII_WR_TOUT; timeOut++) { /* Wait until operation completed */
    if ((pEMAC->MCTL & MCTL_START) == 0) {
      return (0);
    }
  }
  return (-1);
}


/*----------------------------------------------------------------------------
  Read a PHY register 'PhyReg'. 
 *----------------------------------------------------------------------------*/
static int phy_Rd (unsigned int PhyReg) {
  unsigned int timeOut;

  pEMAC->MCTL = (PhyReg << 3) | MCTL_START;

  for (timeOut = 0; timeOut < MII_RD_TOUT; timeOut++) { /* Wait until operation completed */
    if ((pEMAC->MCTL & MCTL_START) == 0) {
      /* Return a 16-bit value. */
      return (pEMAC->MRXD & MRXD_MASK);
    }
  }
  return (-1);
}


