/*
 * @file:    EthDev_LM3S.h
 * @purpose: Ethernet Device Definitions for LM3S6965
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

#ifndef __ETHDEV_LM3S_H
#define __ETHDEV_LM3S_H

/* EMAC Register Access Structure */
typedef struct {
   unsigned long ISR;                   /* Interrupt Status/Acknowledge      */
   unsigned long IEN;                   /* Interrupt Enable Mask             */
   unsigned long RXCTL;                 /* Receove Control                   */
   unsigned long TXCTL;                 /* Transmit Control                  */
   unsigned long DATA;                  /* TX/RX FIFO Data Access            */
   unsigned long IAR0;                  /* Individual Address Register 0     */
   unsigned long IAR1;                  /* Individual Address Register 1     */
   unsigned long THR;                   /* Threshold                         */
   unsigned long MCTL;                  /* Management Control                */
   unsigned long MCDIV;                 /* Management Clock Divider          */
   unsigned long MADR;                  /* Management Address                */
   unsigned long MTXD;                  /* Management Transmit Data          */
   unsigned long MRXD;                  /* Management Receive Data           */
   unsigned long RXFC;                  /* Number of RX Packets              */
   unsigned long TXRQ;                  /* Transimt Request                  */
   unsigned long TSEN;                  /* Timer Support Enable              */
} EMAC_Regs;

#define pEMAC   ((volatile EMAC_Regs *)ETH_BASE)

/* Interrupt Status/Acknowledge Register */
#define INT_RX              0x00000001  /* Receive Interrupt                 */
#define INT_TX_ERR          0x00000002  /* Transmit Error Interrupt          */
#define INT_TX_EMPTY        0x00000004  /* Transmit FIFO Empty Interrupt     */
#define INT_RX_OVR          0x00000008  /* Receive FIFO Overrun Interrupt    */
#define INT_RX_ERR          0x00000010  /* Receive Error Interrupt           */
#define INT_MII             0x00000020  /* Management MII TX/RX Done         */
#define INT_PHY             0x00000040  /* PHY Interrupt                     */
#define INT_ALL             0x0000007F  /* All Interrupts Mask               */

/* Receive Control Register */
#define RXCTL_RX_EN         0x00000001  /* Enable Ethernet Receiver          */
#define RXCTL_MCAST_EN      0x00000002  /* Accept Multicast Packets          */
#define RXCTL_PROM_EN       0x00000004  /* Accept All Packets Mode           */
#define RXCTL_BAD_CRC       0x00000008  /* Reject Packets with Bad CRC       */
#define RXCTL_RST_FIFO      0x00000010  /* Reset Receive FIFO                */

/* Transmit Control Register */
#define TXCTL_TX_EN         0x00000001  /* Enable Ethernet Transmitter       */
#define TXCTL_PAD_EN        0x00000002  /* Enable Packet Padding             */
#define TXCTL_CRC_EN        0x00000004  /* Enable Generation of the CRC      */
#define TXCTL_DUP_EN        0x00000010  /* Enable Duplex Mode                */

#define TX_FRAME_TOUT       0x00100000  /* Frame Transmit timeout count      */

/* MII Management Control Register */
#define MCTL_START          0x00000001  /* Start MII Tx/Rx Transaction       */
#define MCTL_WR             0x00000002  /* MII Write                         */
#define MCTL_REG_ADR        0x000000F8  /* MII Register Address Mask         */

#define MII_WR_TOUT         0x00050000  /* MII Write timeout count           */
#define MII_RD_TOUT         0x00050000  /* MII Read timeout count            */

/* Management Clock Divider */
#define MCDIV_MASK          0x000000FF  /* MII Data Clock Divider Mask       */

/* Management Transmit/Receive Data */
#define MTXD_MASK           0x0000FFFF  /* Management Transmit Data mask     */
#define MRXD_MASK           0x0000FFFF  /* Management Receive Data mask      */

/* Number of RX Packets */
#define RXFC_MASK           0x0000003F  /* Number of RX Packets in FIFO      */

/* Tramsmit Request */
#define TXRQ_NEW_TX         0x00000001  /* Start New Transmission            */

/* Timer Support */
#define TSEN_EN             0x00000001  /* Enable Timer Support              */

/* Embedded PHY Registers */
#define PHY_REG_CTRL        0x00        /* Basic Mode Control Register       */
#define PHY_REG_STAT        0x01        /* Basic Mode Status Register        */
#define PHY_REG_IDR1        0x02        /* PHY Identifier 1                  */
#define PHY_REG_IDR2        0x03        /* PHY Identifier 2                  */
#define PHY_REG_ANA         0x04        /* Auto-Negotiation Advertisement    */
#define PHY_REG_ANLPA       0x05        /* Auto-Neg. Link Partner Abitily    */
#define PHY_REG_ANE         0x06        /* Auto-Neg. Expansion Register      */
#define PHY_REG_VSP         0x10        /* Vendor Specific Register          */
#define PHY_REG_INT         0x11        /* Interrupt Control/Status Register */
#define PHY_REG_DIAG        0x12        /* Diagnostic Register               */
#define PHY_REG_TRANC       0x13        /* Transceiver Control               */
#define PHY_REG_LEDC        0x17        /* LED Configuration                 */
#define PHY_REG_MDIX        0x18        /* MDI/MDIX Control                  */

#define PHY_100M_FULL       0x2100      /* Full Duplex 100Mbit               */
#define PHY_100M_HALF       0x2000      /* Half Duplex 100Mbit               */
#define PHY_10M_FULL        0x0100      /* Full Duplex 10Mbit                */
#define PHY_10M_HALF        0x0000      /* Half Duplex 10MBit                */
#define PHY_AUTO_NEG        0x1000      /* Select Auto Negotiation           */

/* Misc */
#define PHY_DEF_ADR         0x0000      /* Default PHY device address        */

#endif

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/

