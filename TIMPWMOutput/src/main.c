/*******************************************************************************
* File Name			: 	main.c
* Author			: 	timmy00274672 (timmy00274672@gmail.com)
* Date				: 	06/21/2014
* Version			: 	Version 1.0
* Description		: 	This source file for TIMPWMOutput. 
*******************************************************************************/
#include "main.h"
#include "stm32f10x_lib.h"

int main(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	TIM_Configuration();
	while(1);
}
/**
*	open clock on APB1 : TIM2
*	open clock on APB2 : GPIOA PA[0:3] used for output
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);
}

/**
*	PA[0:3] Output in AF_PP mode @ 50 MHz
**/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*
	*	PA4 Output in PP mode @ 50 MHz
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

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
void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

	TIM_TimeBaseInitStruct.TIM_Prescaler = 7199;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 60000;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;	
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

	TIM_OCInitTypeDef TIM_OCInitStruct;

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OCInitStruct.TIM_Pulse = CCR1_Val;
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);

	TIM_OCInitStruct.TIM_Pulse = CCR2_Val;
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);

	TIM_OCInitStruct.TIM_Pulse = CCR3_Val;
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);

	TIM_OCInitStruct.TIM_Pulse = CCR4_Val;
	TIM_OC4Init(TIM2, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_Cmd(TIM2, ENABLE);
}