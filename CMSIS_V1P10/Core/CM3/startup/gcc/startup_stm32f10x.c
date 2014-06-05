/******************************************************************************
 * @file:    startupstm32f10x.c
 * @purpose: CMSIS Cortex-M3 Core Device Startup File
 * @version: V1.01
 * @date:    4. Feb. 2009
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




#define WEAK __attribute__ ((weak))
//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void WEAK Reset_Handler(void);
void WEAK Nmi_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK SysTick_Handler(void);

// External Interrupts
void WEAK WWDG_IRQHandler(void) ;
void WEAK PVD_IRQHandler(void) ;
void WEAK TAMPER_IRQHandler(void) ;
void WEAK RTC_IRQHandler(void) ;
void WEAK FLASH_IRQHandler(void) ;
void WEAK RCC_IRQHandler(void) ;
void WEAK EXTI0_IRQHandler(void) ;
void WEAK EXTI1_IRQHandler(void) ;
void WEAK EXTI2_IRQHandler(void) ;
void WEAK EXTI3_IRQHandler(void) ;
void WEAK EXTI4_IRQHandler(void) ;
void WEAK DMAChannel1_IRQHandler(void) ;
void WEAK DMAChannel2_IRQHandler(void) ;
void WEAK DMAChannel3_IRQHandler(void) ;
void WEAK DMAChannel4_IRQHandler(void) ;
void WEAK DMAChannel5_IRQHandler(void) ;
void WEAK DMAChannel6_IRQHandler(void) ;
void WEAK DMAChannel7_IRQHandler(void) ;
void WEAK ADC_IRQHandler(void) ;
void WEAK USB_HP_CAN_TX_IRQHandler(void) ;
void WEAK USB_LP_CAN_RX0_IRQHandler(void) ;
void WEAK CAN_RX1_IRQHandler(void) ;
void WEAK CAN_SCE_IRQHandler(void) ;
void WEAK EXTI9_5_IRQHandler(void) ;
void WEAK TIM1_BRK_IRQHandler(void) ;
void WEAK TIM1_UP_IRQHandler(void) ;
void WEAK TIM1_TRG_COM_IRQHandler(void) ;
void WEAK TIM1_CC_IRQHandler(void) ;
void WEAK TIM2_IRQHandler(void) ;
void WEAK TIM3_IRQHandler(void) ;
void WEAK TIM4_IRQHandler(void) ;
void WEAK I2C1_EV_IRQHandler(void) ;
void WEAK I2C1_ER_IRQHandler(void) ;
void WEAK I2C2_EV_IRQHandler(void) ;
void WEAK I2C2_ER_IRQHandler(void) ;
void WEAK SPI1_IRQHandler(void) ;
void WEAK SPI2_IRQHandler(void) ;
void WEAK USART1_IRQHandler(void) ;
void WEAK USART2_IRQHandler(void) ;
void WEAK USART3_IRQHandler(void) ;
void WEAK EXTI15_10_IRQHandler(void) ;
void WEAK RTCAlarm_IRQHandler(void) ;
void WEAK USBWakeUp_IRQHandler(void) ;
 


/* Exported types --------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern unsigned long _etext;
extern unsigned long _sidata;		/* start address for the initialization values of the .data section. defined in linker script */
extern unsigned long _sdata;		/* start address for the .data section. defined in linker script */
extern unsigned long _edata;		/* end address for the .data section. defined in linker script */

extern unsigned long _sbss;			/* start address for the .bss section. defined in linker script */
extern unsigned long _ebss;			/* end address for the .bss section. defined in linker script */

extern void _estack;		/* init value for the stack pointer. defined in linker script */



/* Private typedef -----------------------------------------------------------*/
/* function prototypes ------------------------------------------------------*/
void Reset_Handler(void) __attribute__((__interrupt__));
extern int main(void);


/******************************************************************************
*
* The minimal vector table for a Cortex M3.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/


__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{       
        &_estack,                   // The initial stack pointer
        Reset_Handler,              // Reset Handler
		Nmi_Handler,                // NMI Handler
		HardFault_Handler,          // Hard Fault Handler
		MemManage_Handler,          // MPU Fault Handler
		BusFault_Handler,           // Bus Fault Handler
		UsageFault_Handler,         // Usage Fault Handler
		0,                          // Reserved
		0,                          // Reserved
		0,                          // Reserved
		0,                          // Reserved
		SVC_Handler,                // SVCall Handler
		DebugMon_Handler,           // Debug Monitor Handler
		0,                          // Reserved
		PendSV_Handler,             // PendSV Handler
		SysTick_Handler,            // SysTick Handler

		// External Interrupts
		WWDG_IRQHandler,  			// Window Watchdog
		PVD_IRQHandler,   			// PVD through EXTI Line detect
		TAMPER_IRQHandler,			// Tamper
		RTC_IRQHandler,   			// RTC
		FLASH_IRQHandler, 			// Flash
		RCC_IRQHandler,   			// RCC
		EXTI0_IRQHandler, 			// EXTI Line 0
		EXTI1_IRQHandler, 			// EXTI Line 1
		EXTI2_IRQHandler, 			// EXTI Line 2
		EXTI3_IRQHandler, 			// EXTI Line 3
		EXTI4_IRQHandler, 			// EXTI Line 4
		DMAChannel1_IRQHandler, 	// DMA Channel 1
		DMAChannel2_IRQHandler, 	// DMA Channel 2
		DMAChannel3_IRQHandler, 	// DMA Channel 3
		DMAChannel4_IRQHandler, 	// DMA Channel 4
		DMAChannel5_IRQHandler, 	// DMA Channel 5
		DMAChannel6_IRQHandler, 	// DMA Channel 6
		DMAChannel7_IRQHandler, 	// DMA Channel 7
		ADC_IRQHandler, 			// ADC
		USB_HP_CAN_TX_IRQHandler, 	// USB High Priority or CAN TX
		USB_LP_CAN_RX0_IRQHandler, 	// USB Low  Priority or CAN RX0
		CAN_RX1_IRQHandler, 		// CAN RX1
		CAN_SCE_IRQHandler, 		// CAN SCE
		EXTI9_5_IRQHandler, 		// EXTI Line 9..5
		TIM1_BRK_IRQHandler,		// TIM1 Break
		TIM1_UP_IRQHandler, 		// TIM1 Update
		TIM1_TRG_COM_IRQHandler, 	// TIM1 Trigger and Commutation
		TIM1_CC_IRQHandler, 		// TIM1 Capture Compare
		TIM2_IRQHandler, 			// TIM2
		TIM3_IRQHandler, 			// TIM3
		TIM4_IRQHandler, 			// TIM4
		I2C1_EV_IRQHandler, 		// I2C1 Event
		I2C1_ER_IRQHandler, 		// I2C1 Error
		I2C2_EV_IRQHandler, 		// I2C2 Event
		I2C2_ER_IRQHandler, 		// I2C2 Error
		SPI1_IRQHandler, 			// SPI1
		SPI2_IRQHandler, 			// SPI2
		USART1_IRQHandler,		 	// USART1
		USART2_IRQHandler,		 	// USART2
		USART3_IRQHandler,		 	// USART3
		EXTI15_10_IRQHandler, 		// EXTI Line 15..10
		RTCAlarm_IRQHandler, 		// RTC Alarm through EXTI Line
		USBWakeUp_IRQHandler, 		// USB Wakeup from suspend
};

/*******************************************************************************
* Function Name  : Reset_Handler
* Description    : This is the code that gets called when the processor first starts execution
*		       following a reset event.  Only the absolutely necessary set is performed,
*		       after which the application supplied main() routine is called. 
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void Reset_Handler(void)
{
    unsigned long *pulSrc, *pulDest;

    //
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_sidata;
    for(pulDest = &_sdata; pulDest < &_edata; )
    {
        *(pulDest++) = *(pulSrc++);
    }

    //
    // Zero fill the bss segment.
    //
    for(pulDest = &_sbss; pulDest < &_ebss; )
    {
        *(pulDest++) = 0;
    }

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// Provide weak aliases for each Exception handler to the Default_Handler. 
// As they are weak aliases, any function with the same name will override 
// this definition.
//
//*****************************************************************************

#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
#pragma weak WWDG_IRQHandler = Default_Handler
#pragma weak PVD_IRQHandler = Default_Handler
#pragma weak TAMPER_IRQHandler = Default_Handler
#pragma weak RTC_IRQHandler = Default_Handler
#pragma weak FLASH_IRQHandler = Default_Handler
#pragma weak RCC_IRQHandler = Default_Handler
#pragma weak EXTI0_IRQHandler = Default_Handler
#pragma weak EXTI1_IRQHandler = Default_Handler
#pragma weak EXTI2_IRQHandler = Default_Handler
#pragma weak EXTI3_IRQHandler = Default_Handler
#pragma weak EXTI4_IRQHandler = Default_Handler
#pragma weak DMAChannel1_IRQHandler = Default_Handler
#pragma weak DMAChannel2_IRQHandler = Default_Handler
#pragma weak DMAChannel3_IRQHandler = Default_Handler
#pragma weak DMAChannel4_IRQHandler = Default_Handler
#pragma weak DMAChannel5_IRQHandler = Default_Handler
#pragma weak DMAChannel6_IRQHandler = Default_Handler
#pragma weak DMAChannel7_IRQHandler = Default_Handler
#pragma weak ADC_IRQHandler = Default_Handler
#pragma weak USB_HP_CAN_TX_IRQHandler = Default_Handler
#pragma weak USB_LP_CAN_RX0_IRQHandler = Default_Handler
#pragma weak CAN_RX1_IRQHandler = Default_Handler
#pragma weak CAN_SCE_IRQHandler = Default_Handler
#pragma weak EXTI9_5_IRQHandler = Default_Handler
#pragma weak TIM1_BRK_IRQHandler = Default_Handler
#pragma weak TIM1_UP_IRQHandler = Default_Handler
#pragma weak TIM1_TRG_COM_IRQHandler = Default_Handler
#pragma weak TIM1_CC_IRQHandler = Default_Handler
#pragma weak TIM2_IRQHandler = Default_Handler
#pragma weak TIM3_IRQHandler = Default_Handler
#pragma weak TIM4_IRQHandler = Default_Handler
#pragma weak I2C1_EV_IRQHandler = Default_Handler
#pragma weak I2C1_ER_IRQHandler = Default_Handler
#pragma weak I2C2_EV_IRQHandler = Default_Handler
#pragma weak I2C2_ER_IRQHandler = Default_Handler
#pragma weak SPI1_IRQHandler = Default_Handler
#pragma weak SPI2_IRQHandler = Default_Handler
#pragma weak USART1_IRQHandler = Default_Handler
#pragma weak USART2_IRQHandler = Default_Handler
#pragma weak USART3_IRQHandler = Default_Handler
#pragma weak EXTI15_10_IRQHandler = Default_Handler
#pragma weak RTCAlarm_IRQHandler = Default_Handler
#pragma weak USBWakeUp_IRQHandler = Default_Handler

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
void Default_Handler(void) {
	// Go into an infinite loop.
	//
	while (1) {
	}
}
