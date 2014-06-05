/******************************************************************************
 * @file:    system_stm32.h
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Header File
 * @version: V1.02
 * @date:    22. Jan. 2009
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


#ifndef __SYSTEM_STM32_H
#define __SYSTEM_STM32_H

extern uint32_t SystemFrequency;                   /*!< System Clock Frequency (Core Clock) */
extern uint32_t SystemFrequency_SysClk;            /*!< System clock                        */
extern uint32_t SystemFrequency_AHBClk;            /*!< AHB System bus speed                */
extern uint32_t SystemFrequency_APB1Clk;           /*!< APB Peripheral Bus 1 (low)  speed   */
extern uint32_t SystemFrequency_APB2Clk;           /*!< APB Peripheral Bus 2 (high) speed   */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the Embedded Flash Interface,  initialize the PLL and update th SystemFrequency variable
 */
extern                   void SystemInit     (void);


#endif
