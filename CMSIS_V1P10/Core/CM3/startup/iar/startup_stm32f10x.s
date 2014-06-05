;/******************************************************************************
; * @file:    startupstm32f10x.s
; * @purpose: CMSIS Cortex-M3 Core Device Startup File
; * @version: V1.01
; * @date:    04. Feb. 2009
; *----------------------------------------------------------------------------
; *
; * Copyright (C) 2009 ARM Limited. All rights reserved.
; *
; * ARM Limited (ARM) is supplying this software for use with Cortex-Mx 
; * processor based microcontrollers.  This file can be freely distributed 
; * within development tools that are supporting such ARM based processors. 
; *
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)
	
        EXTERN  __iar_program_start
        PUBLIC  __vector_table
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     __iar_program_start

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     WWDG_IRQHandler
        DCD     PVD_IRQHandler
        DCD     TAMPER_IRQHandler
        DCD     RTC_IRQHandler
        DCD     FLASH_IRQHandler
        DCD     RCC_IRQHandler
        DCD     EXTI0_IRQHandler
        DCD     EXTI1_IRQHandler
        DCD     EXTI2_IRQHandler
        DCD     EXTI3_IRQHandler
        DCD     EXTI4_IRQHandler
        DCD     DMAChannel1_IRQHandler
        DCD     DMAChannel2_IRQHandler
        DCD     DMAChannel3_IRQHandler
        DCD     DMAChannel4_IRQHandler
        DCD     DMAChannel5_IRQHandler
        DCD     DMAChannel6_IRQHandler
        DCD     DMAChannel7_IRQHandler
        DCD     ADC_IRQHandler
        DCD     USB_HP_CAN_TX_IRQHandler
        DCD     USB_LP_CAN_RX0_IRQHandler
        DCD     CAN_RX1_IRQHandler
        DCD     CAN_SCE_IRQHandler
        DCD     EXTI9_5_IRQHandler
        DCD     TIM1_BRK_IRQHandler
        DCD     TIM1_UP_IRQHandler
        DCD     TIM1_TRG_COM_IRQHandler
        DCD     TIM1_CC_IRQHandler
        DCD     TIM2_IRQHandler
        DCD     TIM3_IRQHandler
        DCD     TIM4_IRQHandler
        DCD     I2C1_EV_IRQHandler
        DCD     I2C1_ER_IRQHandler
        DCD     I2C2_EV_IRQHandler
        DCD     I2C2_ER_IRQHandler
        DCD     SPI1_IRQHandler
        DCD     SPI2_IRQHandler
        DCD     USART1_IRQHandler
        DCD     USART2_IRQHandler
        DCD     USART3_IRQHandler
        DCD     EXTI15_10_IRQHandler
        DCD     RTCAlarm_IRQHandler
        DCD     USBWakeUp_IRQHandler
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B NMI_Handler
        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER(1)
HardFault_Handler
        B HardFault_Handler
        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER(1)
MemManage_Handler
        B MemManage_Handler
        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER(1)
BusFault_Handler
        B BusFault_Handler
        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER(1)
UsageFault_Handler
        B UsageFault_Handler
        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER(1)
SVC_Handler
        B SVC_Handler
        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER(1)
DebugMon_Handler
        B DebugMon_Handler
        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER(1)
PendSV_Handler
        B PendSV_Handler
        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER(1)
SysTick_Handler
        B SysTick_Handler
        PUBWEAK WWDG_IRQHandler
        SECTION .text:CODE:REORDER(1)
WWDG_IRQHandler
        B WWDG_IRQHandler
        PUBWEAK PVD_IRQHandler
        SECTION .text:CODE:REORDER(1)
PVD_IRQHandler
        B PVD_IRQHandler
        PUBWEAK TAMPER_IRQHandler
        SECTION .text:CODE:REORDER(1)
TAMPER_IRQHandler
        B TAMPER_IRQHandler
        PUBWEAK RTC_IRQHandler
        SECTION .text:CODE:REORDER(1)
RTC_IRQHandler
        B RTC_IRQHandler
        PUBWEAK FLASH_IRQHandler
        SECTION .text:CODE:REORDER(1)
FLASH_IRQHandler
        B FLASH_IRQHandler
        PUBWEAK RCC_IRQHandler
        SECTION .text:CODE:REORDER(1)
RCC_IRQHandler
        B RCC_IRQHandler
        PUBWEAK EXTI0_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI0_IRQHandler
        B EXTI0_IRQHandler
        PUBWEAK EXTI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI1_IRQHandler
        B EXTI1_IRQHandler
        PUBWEAK EXTI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI2_IRQHandler
        B EXTI2_IRQHandler
        PUBWEAK EXTI3_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI3_IRQHandler
        B EXTI3_IRQHandler
        PUBWEAK EXTI4_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI4_IRQHandler
        B EXTI4_IRQHandler
        PUBWEAK DMAChannel1_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel1_IRQHandler
        B DMAChannel1_IRQHandler
        PUBWEAK DMAChannel2_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel2_IRQHandler
        B DMAChannel2_IRQHandler
        PUBWEAK DMAChannel3_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel3_IRQHandler
        B DMAChannel3_IRQHandler
        PUBWEAK DMAChannel4_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel4_IRQHandler
        B DMAChannel4_IRQHandler
        PUBWEAK DMAChannel5_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel5_IRQHandler
        B DMAChannel5_IRQHandler
        PUBWEAK DMAChannel6_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel6_IRQHandler
        B DMAChannel6_IRQHandler
        PUBWEAK DMAChannel7_IRQHandler
        SECTION .text:CODE:REORDER(1)
DMAChannel7_IRQHandler
        B DMAChannel7_IRQHandler
        PUBWEAK ADC_IRQHandler
        SECTION .text:CODE:REORDER(1)
ADC_IRQHandler
        B ADC_IRQHandler
        PUBWEAK USB_HP_CAN_TX_IRQHandler
        SECTION .text:CODE:REORDER(1)
USB_HP_CAN_TX_IRQHandler
        B USB_HP_CAN_TX_IRQHandler
        PUBWEAK USB_LP_CAN_RX0_IRQHandler
        SECTION .text:CODE:REORDER(1)
USB_LP_CAN_RX0_IRQHandler
        B USB_LP_CAN_RX0_IRQHandler
        PUBWEAK CAN_RX1_IRQHandler
        SECTION .text:CODE:REORDER(1)
CAN_RX1_IRQHandler
        B CAN_RX1_IRQHandler
        PUBWEAK CAN_SCE_IRQHandler
        SECTION .text:CODE:REORDER(1)
CAN_SCE_IRQHandler
        B CAN_SCE_IRQHandler
        PUBWEAK EXTI9_5_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI9_5_IRQHandler
        B EXTI9_5_IRQHandler
        PUBWEAK TIM1_BRK_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM1_BRK_IRQHandler
        B TIM1_BRK_IRQHandler
        PUBWEAK TIM1_UP_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM1_UP_IRQHandler
        B TIM1_UP_IRQHandler
        PUBWEAK TIM1_TRG_COM_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM1_TRG_COM_IRQHandler
        B TIM1_TRG_COM_IRQHandler
        PUBWEAK TIM1_CC_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM1_CC_IRQHandler
        B TIM1_CC_IRQHandler
        PUBWEAK TIM2_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM2_IRQHandler
        B TIM2_IRQHandler
        PUBWEAK TIM3_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM3_IRQHandler
        B TIM3_IRQHandler
        PUBWEAK TIM4_IRQHandler
        SECTION .text:CODE:REORDER(1)
TIM4_IRQHandler
        B TIM4_IRQHandler
        PUBWEAK I2C1_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C1_EV_IRQHandler
        B I2C1_EV_IRQHandler
        PUBWEAK I2C1_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C1_ER_IRQHandler
        B I2C1_ER_IRQHandler
        PUBWEAK I2C2_EV_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C2_EV_IRQHandler
        B I2C2_EV_IRQHandler
        PUBWEAK I2C2_ER_IRQHandler
        SECTION .text:CODE:REORDER(1)
I2C2_ER_IRQHandler
        B I2C2_ER_IRQHandler
        PUBWEAK SPI1_IRQHandler
        SECTION .text:CODE:REORDER(1)
SPI1_IRQHandler
        B SPI1_IRQHandler
        PUBWEAK SPI2_IRQHandler
        SECTION .text:CODE:REORDER(1)
SPI2_IRQHandler
        B SPI2_IRQHandler
        PUBWEAK USART1_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART1_IRQHandler
        B USART1_IRQHandler
        PUBWEAK USART2_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART2_IRQHandler
        B USART2_IRQHandler
        PUBWEAK USART3_IRQHandler
        SECTION .text:CODE:REORDER(1)
USART3_IRQHandler
        B USART3_IRQHandler
        PUBWEAK EXTI15_10_IRQHandler
        SECTION .text:CODE:REORDER(1)
EXTI15_10_IRQHandler
        B EXTI15_10_IRQHandler
        PUBWEAK RTCAlarm_IRQHandler
        SECTION .text:CODE:REORDER(1)
RTCAlarm_IRQHandler
        B RTCAlarm_IRQHandler
        PUBWEAK USBWakeUp_IRQHandler
        SECTION .text:CODE:REORDER(1)
USBWakeUp_IRQHandler
        B USBWakeUp_IRQHandler

        END
