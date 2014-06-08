/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/08/2014
* Version        : 	Version 1.0
* Description    : 	This head file for DeepSleep. 
*******************************************************************************/
#include "stm32f10x_lib.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void SysTick_Configuration(void);
void Delay_NSecond(u8 Tick);

