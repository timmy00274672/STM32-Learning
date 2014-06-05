/******************************************************************************
 * @file:    system_armcm0.c
 * @purpose: CMSIS ARM Cortex-M0 Device Peripheral Access Layer Source File
 * @version: V0.01
 * @date:    17. Feb. 2009
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
 ******************************************************************************/


#include <stdint.h>
#include "armcm0.h"

/*----------------------------------------------------------------------------
  Define SYSCLK
 *----------------------------------------------------------------------------*/
#define __HSI (8000000UL)

/*----------------------------------------------------------------------------
  Clock Definitions
 *----------------------------------------------------------------------------*/
//uint32_t SystemFrequency  = __HSI;   /*!< System Clock Frequency (Core Clock) */
uint32_t SystemFrequency  = 72000000UL;   /*!< System Clock Frequency (Core Clock) */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialise GPIO directions and values
 */
void SystemInit (void)
{
  GPIO0->DATA[0].WORD = 0;
  GPIO0->IE = 0;
  GPIO0->DIR = 0xff83;
  
  GPIO1->DATA[0].WORD = 0;
  GPIO1->IE = 0;
  GPIO1->DIR = 0;
  
  GPIO2->DATA[0].WORD = 0;
  GPIO2->IE = 0;
  GPIO2->DIR = 0;
}

