/******************************************************************************
 * @file:    main.c
 * @purpose: CMSIS Cortex-M3 Core Peripheral Access Layer Source File
 *           Blink a LED using CM3 SysTick
 * @version: V1.0
 * @date:    22. Jan. 2009
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
 ******************************************************************************/

#include "LM3S6965.h"


uint32_t msTicks;                               /* counts 1ms timeTicks */
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                                    /* increment counter necessary in Delay() */
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
__inline static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/*------------------------------------------------------------------------------
  configer LED pins
 *------------------------------------------------------------------------------*/
__inline static LED_Config(void) {
  uint32_t i;

  SYSCTL->RCGC2 |= (1<<5);                       /* enable clock for GPIOF */
  for (i = 0; i < 0x10; i++);                    /* wait */
  GPIOF->DIR     = 0x01;                         /* Enable the GPIO pin for output */
  GPIOF->DEN     = 0x01;                         /* enable the GPIO pin */
}

/*------------------------------------------------------------------------------
  Switch on LEDs
 *------------------------------------------------------------------------------*/
__inline static void LED_On (uint32_t led) {

  GPIOF->DATA |=  (led);                        /* Turn On  LED */
}


/*------------------------------------------------------------------------------
  Switch off LEDs
 *------------------------------------------------------------------------------*/
__inline static void LED_Off (uint32_t led) {

  GPIOF->DATA &= ~(led);                        /* Turn Off LED */
}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {

  SystemInit();									/* setup clocks */
  if (SysTick_Config(SystemFrequency / 1000)) { /* Setup SysTick Timer for 1 msec interrupts  */
    while (1);                                  /* Capture error */
  }
  
  LED_Config();                             
 
  while(1) {
    LED_On (0x01);                              /* Turn on the LED. */
    Delay (100);                                /* delay  100 Msec */
    LED_Off (0x01);                             /* Turn off the LED. */
    Delay (100);                                /* delay  100 Msec */
  }
  
}

