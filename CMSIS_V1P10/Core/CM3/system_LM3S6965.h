/******************************************************************************
 * @file:    system_LM3S6965.h
 * @purpose: CMSIS Cortex-M3 Device Peripheral Access Layer Header File for the
 *           Toshiba 'TMPM330' Device Series 
 * @version: V0.01 (Preliminary)
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


#ifndef __SYSTEM_LM3S6965_H
#define __SYSTEM_LM3S6965_H

extern uint32_t SystemFrequency;                   /*!< System Clock Frequency (Core Clock) */
extern uint32_t SystemFrequency_PwnClk;            /*!< PWM clock                           */
extern uint32_t SystemFrequency_AdcClk;            /*!< ADC clock                           */
extern uint32_t SystemFrequency_CanClk;            /*!< CAN clock                           */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemFrequency variable.
 */
extern void SystemInit (void);
#endif
