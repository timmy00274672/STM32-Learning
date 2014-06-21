/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/21/2014
* Version        : 	Version 1.0
* Description    : 	This head file for TIMPWMOutput. 
*******************************************************************************/
#include "stm32f10x_lib.h"

//For Time Pause
uc16 CCR1_Val = 60000; 	//50%
uc16 CCR2_Val = 30000; 	//37.5%
uc16 CCR3_Val = 15000; 	//25%
uc16 CCR4_Val = 7500; 	//12.5%
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
*		Period : 60000 
*		Prescaler : 7199
*		ClockDivision : 0
*		CounterMode : up
*	Configurate TIM2 OC[1:4]
*		OCMode : TIM_OCMode_PWM1
*		OutputState : Enable
*		OCPloarity : High
*		TIMPause : CCR[1:4]_Val
*		Perload : enable for all OCs
*	Enable TIM2
**/
void TIM_Configuration(void);

