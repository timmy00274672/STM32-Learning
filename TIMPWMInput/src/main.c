/*******************************************************************************
* File Name			: 	main.c
* Author			: 	timmy00274672 (timmy00274672@gmail.com)
* Date				: 	06/21/2014
* Version			: 	Version 1.0
* Description		: 	This source file for TIMPWMInput. 
*******************************************************************************/
#include "main.h"
#include "stm32f10x_lib.h"

int main(void)
{
	RCC_Configuration();
	NVIC_Configuration();
	USART_Configuration();
	GPIO_Configuration();
	TIM_Configuration();
	while(1);
}

/**
*	open clock on APB1 : TIM2 for I/P 
*						 TIM3 for O/P
*	open clock on APB2 : GPIOA + USART1 PA for USART1 [9:10]
*						 
**/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartupStatus;
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartupStatus = RCC_WaitForHSEStartUp();

    if(HSEStartupStatus == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while(RCC_GetSYSCLKSource() != 0x08);
    }

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 ,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 ,ENABLE);
}

/**
	Set the clock including CPOL, CPHA, LastBit
	Set the Frame including BaudRate, WordLength, StopBits, Parity, HardareFlowControl, Mode
**/
void USART_Configuration(void)
{
	/**
        Set the USART_CR2
        Clock enable(CLKEN) : enable the CK bit or not

        Clock polarity(CPOL) : polarity of the clock output on the CK pin in synchronour mode.
        
            It works in conjunction with the CPHA bit to produce the desired clock/data relationship.
            CPOL_Low : Stedady low value on CK pin outside transmission window.
        
        Clock phase (CPHA) : phase output on the CK pin in synchronous mode.

            CPHA_2Edge : the second clock transition is the first data capture edge

        Last bit clock pulse(LBCL) : select whether the clock pulse associated with the MSB
            has to be output on the CK pin in synchronous mode. The last bit is the 8th or
            9th data bit depending on the M bit in USART_CR1 register.
    **/
    USART_ClockInitTypeDef USART_ClockInitStructure;
    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable; 
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART1,&USART_ClockInitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600; //Set the USART_BRR automatically
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //Set the USART_CR1 M bit
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //Set the USART_CR2 STOP[0:1]
    USART_InitStructure.USART_Parity = USART_Parity_No; //Set the USART_CR1 PCE, PS
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Set the USART_CR3 RTSE, CTSE
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

/**
*	all @ 50 MHz
*	PA[1] in floating mode (Default AF : TIM2CH2)
*	PA[6] in AF-PP mode (Default AF : TIM3_CH1)
*	PA[9] in AF-PP mode (Default AF : USART1_TX)
*	PA[10] in floating mode (Default AF : USART1_RX)
**/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
*	setting priority grouping
*	enable TIM2_IRQHandler with proper priority
**/
void NVIC_Configuration(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

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
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 60000;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;	
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStruct.TIM_Pulse = 15000;

	TIM_OC1Init(TIM3, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);


	TIM_ICInitTypeDef TIM_ICInitStruct;

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;

	TIM_PWMIConfig(TIM2, &TIM_ICInitStruct);

	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

/**
*	Write the `ch` to USART1
**/
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (u16)ch);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    return ch;
}