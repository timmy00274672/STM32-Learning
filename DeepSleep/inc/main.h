/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/08/2014
* Version        : 	Version 1.0
* Description    : 	This head file for DeepSleep. 
*******************************************************************************/
#include "stm32f10x_lib.h"

/**
*	open clock on APB1 : PWR
*	open clock on APB2 : GPIOA PA[0,4] used for I/O
**/
void RCC_Configuration(void);

/**
*	PA0 Input in IPU mode
*	PA4 Output in PP mode @ 50 MHz
**/
void GPIO_Configuration(void);

/**
*	Set PA0 as EXTI source (Line 0) and enable it
*	Mode : Interrupt
*	Trigger : as falling 
**/
void EXTI_Configuration(void);

/**
*	setting priority grouping
*	enable EXTI0_IRQHandler with proper priority
**/
void NVIC_Configuration(void);

/**
*	Select HCLK as SysTick clock souce(AHB clock selected as SysTick clock source.)
*	Set Reload time as 100ms
**/
void SysTick_Configuration(void);

/**
*	Use SysTick counter to implement
*	@param Tick time to delay in second
**/
void Delay_NSecond(u8 Tick);

