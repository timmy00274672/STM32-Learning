/*******************************************************************************
* File Name			: 	stm32f10x_it.h
* Author			: 	timmy00274672 (timmy00274672@gmail.com)
* Date				: 	06/21/2014
* Version			: 	Version 1.0
* Description		: 	This source file of overwritting handler for TIMPWMInput. 
*******************************************************************************/
#include "stm32f10x_it.h"
#include "stdio.h"

/**

*	1. 	show frequency and duty cycle via USART1
*	2.	TIM_ClearITPendingBit(TIM2, TIM_TI_CC2)
**/
void TIM2_IRQHandler(void)
{
	static float IC2Value = 0;
	static float DutyCycle = 0;
	static float Frequency = 0;
	static float Paulse = 0;

	IC2Value = TIM_GetCapture2(TIM2);
	Paulse = TIM_GetCapture1(TIM2);

	DutyCycle = Paulse / IC2Value;
	Frequency = 72000000 / IC2Value;

	printf("\r\n The DutyCycle of input pulse is %d %%\r\n", (u32)(DutyCycle * 100));
	printf("\r\n The Frequency of input pulse is %.2f KHz\r\n", Frequency/1000);
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
}
