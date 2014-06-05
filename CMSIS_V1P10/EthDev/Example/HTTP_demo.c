/*----------------------------------------------------------------------------
 *      R T L   T C P N E T   E x a m p l e
 *----------------------------------------------------------------------------
 *      Name:    HTTP_DEMO.C
 *      Purpose: HTTP Server demo example
 *      Rev.:    V3.22
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <RTL.h>
#include <Net_Config.h>
#include <LM3Sxxxx.H>
#include <string.h>
#ifdef LM3S6965_RevC
  #include "rit128x96x4.h"
#else
  #include "osram128x64x4.h"
#endif
#include "EthDev.h"

BOOL LEDrun;
BOOL LCDupdate;
BOOL tick;
U32  dhcp_tout;
U8   lcd_text[6][16+1] = {"    RL-ARM",       /* Buffer for LCD text         */
                          "  HTTP example",
                          " ",
                          "Waiting for Link",
                          " ",
                          " "
                         };

/*---Interface to TCP Stack--------------------------------------------------*/

extern EthDev_IOB eth0;
EthDev_IOB *pEth;
EthDev_STATUS EthState;


/*---------------------------------------------------------------------------*/


extern LOCALM localm[];                       /* Local Machine Settings      */
#define MY_IP localm[NETIF_ETH].IpAdr
#define DHCP_TOUT   50                        /* DHCP timeout 5 seconds      */

static void init_io (void);
static void init_display (void);

/*--------------------------- init ------------------------------------------*/

static void init () {
   /* Add System initialisation code here */ 

   init_io ();
   init_display ();

   /* Select 'eth0' as active communication interface. */
   pEth = &eth0;
   /* This is required before the TCP_ARM.lib is fixed. */
   /* Othervise it ends in FAULT handler. */
   SysCtlPeripheralEnable (SYSCTL_PERIPH_ETH);

   EthState.Link = EthDev_LINK_DOWN;
   init_TcpNet ();

   /* Setup and enable the SysTick timer for 100ms. */
   SysTickPeriodSet(SysCtlClockGet() / 10);
   SysTickEnable();
}


/*--------------------------- timer_poll ------------------------------------*/

static void timer_poll () {
   /* System tick timer running in poll mode */

   if ((HWREG (NVIC_ST_CTRL) >> 16) & 1) {
      /* Timer tick every 100 ms */
      timer_tick ();
      tick = __TRUE;
   }
}

/*--------------------------- init_io ---------------------------------------*/

static void init_io () {

   /* Set the clocking to run from the PLL at 50 MHz */
   SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

   /* Configure the GPIO for the LED. */
   SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOF);
   GPIOPadConfigSet (GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
   GPIODirModeSet (GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);

   /* Configure UART0 for 115200 baud. */
   SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
   SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
   GPIOPinTypeUART (GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
   UARTConfigSet(UART0_BASE, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
   UARTEnable(UART0_BASE);
}


/*--------------------------- fputc -----------------------------------------*/

int fputc (int ch, FILE *f)  {
   /* Debug output to serial port. */

   if (ch == '\n')  {
      UARTCharPut (UART0_BASE, '\r');        /* output CR                    */
   }
   UARTCharPut (UART0_BASE, ch);
   return (ch);
}


/*--------------------------- LED_out ---------------------------------------*/

void LED_out (U32 val) {
   GPIOPinWrite (GPIO_PORTF_BASE, GPIO_PIN_0, val & 1);
}


/*--------------------------- upd_display -----------------------------------*/

static void upd_display () {
   /* Update LCD Module display text. */

#ifdef LM3S6965_RevC
   RIT128x96x4Clear ();
   RIT128x96x4StringDraw ((const char *)&lcd_text[0], 20, 10, 11);
   RIT128x96x4StringDraw ((const char *)&lcd_text[1], 20, 25, 11);
   RIT128x96x4StringDraw ((const char *)&lcd_text[2], 20, 40, 11);
   RIT128x96x4StringDraw ((const char *)&lcd_text[3], 20, 55, 11);
   RIT128x96x4StringDraw ((const char *)&lcd_text[4], 20, 70, 11);
   RIT128x96x4StringDraw ((const char *)&lcd_text[5], 20, 85, 11);
#else
   OSRAM128x64x4Clear ();
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[0], 20, 0, 11);
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[1], 20, 10, 11);
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[2], 20, 20, 11);
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[3], 20, 30, 11);
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[4], 20, 40, 11);
   OSRAM128x64x4StringDraw ((const char *)&lcd_text[5], 20, 50, 11);
#endif
   LCDupdate =__FALSE;
}


/*--------------------------- init_display ----------------------------------*/

static void init_display () {
   /* OLED Module init */

#ifdef LM3S6965_RevC
   RIT128x96x4Init(1000000);
#else
   OSRAM128x64x4Init(1000000);
#endif
   upd_display ();
}


/*--------------------------- dhcp_check ------------------------------------*/

static void dhcp_check () {
   /* Monitor DHCP IP address assignment. */

   if (tick == __FALSE || dhcp_tout == 0) {
      return;
   }

   if (mem_test (&MY_IP, 0, IP_ADRLEN) == __FALSE && !(dhcp_tout & 0x80000000)) {
      /* Success, DHCP has already got the IP address. */
      dhcp_tout = 0;
      sprintf((char *)lcd_text[2],"IP address:");
      sprintf((char *)lcd_text[3],"%d.%d.%d.%d", MY_IP[0], MY_IP[1],
                                               MY_IP[2], MY_IP[3]);
      sprintf((char *)lcd_text[4],((EthState.Speed  == EthDev_SPEED_100M) ?
                                    "speed : 100M" : "speed: 10M"));
      sprintf((char *)lcd_text[5],((EthState.Duplex == EthDev_DUPLEX_FULL)?
                                    "duplex: FULL" : "duplex: HALF"));
      LCDupdate = __TRUE;
      return;
   }
   if (--dhcp_tout == 0) {
      /* A timeout, disable DHCP and use static IP address. */
      dhcp_disable ();
      sprintf((char *)lcd_text[3]," DHCP failed    " );
      LCDupdate = __TRUE;
      dhcp_tout = 30 | 0x80000000;            /* set timer to 3 sec to display message */
      return;
   }
   if (dhcp_tout == 0x80000000) {
      dhcp_tout = 0;
      sprintf((char *)lcd_text[2],"IP address:");
      sprintf((char *)lcd_text[3],"%d.%d.%d.%d", MY_IP[0], MY_IP[1],
                                               MY_IP[2], MY_IP[3]);
      sprintf((char *)lcd_text[4],((EthState.Speed  == EthDev_SPEED_100M) ?
                                    "speed : 100M" : "speed: 10M"));
      sprintf((char *)lcd_text[5],((EthState.Duplex == EthDev_DUPLEX_FULL)?
                                    "duplex: FULL" : "duplex: HALF"));
      LCDupdate = __TRUE;
   }
}


/*--------------------------- blink_led -------------------------------------*/

static void blink_led () {
   /* Blink the LEDs on an eval board */
   static U32 LEDstat = 1;

   if (tick == __TRUE) {
      /* Every 100 ms */
      tick = __FALSE;
      if (LEDrun == __TRUE) {
         LEDstat = ~LEDstat & 0x01;
         LED_out (LEDstat);
      }
      if (LCDupdate == __TRUE) {
         upd_display ();
      }
   }
}


/*---------------------------------------------------------------------------*/

void link_check (void) {
   EthDev_STATUS newState;

   if (tick == __FALSE) {
      return;
   }
   /* Every 100 ms */
   newState = pEth->LinkChk ();
   if (newState.Link == EthState.Link) {
      /* No change. */
      return;
   }
   EthState = newState;

   if (EthState.Link == EthDev_LINK_DOWN) {
      MY_IP[0] = MY_IP[1] =  MY_IP[2] = MY_IP[3] = 0;          /* reset IP address */
      sprintf((char *)lcd_text[2]," ");
      sprintf((char *)lcd_text[3],"Waiting for Link");
      sprintf((char *)lcd_text[4]," ");
      sprintf((char *)lcd_text[5]," ");
      LCDupdate = __TRUE;
      pEth->UnInit ();
      init_TcpNet ();
   }
   else {
      sprintf((char *)lcd_text[2]," ");
      sprintf((char *)lcd_text[3],"Waiting for DHCP");
      sprintf((char *)lcd_text[4]," ");
      sprintf((char *)lcd_text[5]," ");
      LCDupdate = __TRUE;
      dhcp_tout = DHCP_TOUT;
   }
}


/*---------------------------------------------------------------------------*/

int main (void) {
  /* Main Thread of the TcpNet */

  init ();
  dhcp_tout = DHCP_TOUT;
  LEDrun    = __TRUE;

  while (1) {
    blink_led ();
    switch (EthState.Link) {
      case 0:              /* link down */
        timer_poll ();
        link_check ();
        break;
  
      case 1:              /* link up */
        timer_poll ();
        link_check ();
        main_TcpNet ();
        dhcp_check ();
        break;
     }
  }
}



/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/


