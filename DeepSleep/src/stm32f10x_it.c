/*******************************************************************************
* File Name      : 	stm32f10x_it.c
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/08/2014
* Version        : 	Version 1.0
* Description    : 	This source file of overwritting handler for DeepSleep. 
*******************************************************************************/
#include "stm32f10x_it.h"

void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0);
}