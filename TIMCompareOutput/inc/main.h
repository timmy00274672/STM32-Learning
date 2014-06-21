/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/21/2014
* Version        : 	Version 1.0
* Description    : 	This head file for TIMCompareOutput. 
*******************************************************************************/
#include "stm32f10x_lib.h"

//For Time Pause
uc16 CCR1_Val = 40000; 	//4s
uc16 CCR2_Val = 20000; 	//2s
uc16 CCR3_Val = 10000; 	//1s
uc16 CCR4_Val = 5000; 	//0.5s
/**
*	open clock on APB1 : TIM2
*	open clock on APB2 : GPIOA PA[0:3] used for output
**/
void RCC_Configuration(void);

/**
*	PA[0:3] Output in AF_PP mode @ 50 MHz
**/
void GPIO_Configuration(void);

/**
*	Configurate TIM2 Base Unit (@ 1us)
*		Period : 65535 
*		Prescaler : 7199
*		ClockDivision : 0
*		CounterMode : up
*	Configurate TIM2 OC[1:4]
*		OCMode : OCMode_Toggle
*		OutputState : Enable
*		OCPloarity : High
*		TIMPause : CCR[1:4]_Val
*		Perload : disable for all OCs
*	Enable interrupt : TIM2 CC[1:4]
*	Enable TIM2
**/
void TIM_Configuration(void);

/**
*	setting priority grouping
*	enable TIM2_IRQHandler with proper priority
**/
void NVIC_Configuration(void);

