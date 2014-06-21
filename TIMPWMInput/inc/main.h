/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/21/2014
* Version        : 	Version 1.0
* Description    : 	This head file for TIMPWMInput. 
*******************************************************************************/
#include "stm32f10x_lib.h"
#include "stdio.h"

/**
*	open clock on APB1 : TIM2 for I/P 
*						 TIM3 for O/P
*	open clock on APB2 : GPIOA + USART1 PA for USART1 [9:10]
*						 
**/
void RCC_Configuration(void);

/**
	Set the clock including CPOL, CPHA, LastBit
	Set the Frame including BaudRate, WordLength, StopBits, Parity, HardareFlowControl, Mode
**/
void USART_Configuration(void);


/**
*	all @ 50 MHz
*	PA[1] in floating mode (Default AF : TIM2CH2)
*	PA[6] in AF-PP mode (Default AF : TIM3_CH1)
*	PA[9] in AF-PP mode (Default AF : USART1_TX)
*	PA[10] in floating mode (Default AF : USART1_RX)
**/
void GPIO_Configuration(void);

/**
*	TIM3 (output)
*		Configurate TIM3 Base Unit 
*			Period : 60000
*			Prescaler : 0
*			ClockDivision : 0
*			CounterMode : up
*		Configurate TIM3 OC1
*			OCMode : PWM1
*			OutputState : Enable
*			OCPloarity : High
*			TIMPause : 15000 (25% * 60000)
*			Preload : enable
*		Enable TIM3 ARR Preload
*		Enable TIM3
*	TIM2 (input)
*		Configurate TIM2 PWMI
			Channel 2 (as master)
			IC Polarity : Rising
			IC Selection : Direct TI
			IC Prescaler : 1
			IC Filter : 0
		Input Trigger : TI2FP2 (Channel 2 as master)
		Slave Mode : Reset
		Enable IT : TIM_IT_CC2
		Enable TIM2
**/
void TIM_Configuration(void);

/**
*	setting priority grouping
*	enable TIM2_IRQHandler with proper priority
**/
void NVIC_Configuration(void);

/**
*	Write the `ch` to USART1
**/
int fputc(int ch, FILE *f);
