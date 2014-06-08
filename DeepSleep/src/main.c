/*******************************************************************************
* File Name      : 	main.c
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/08/2014
* Version        : 	Version 1.1
* Description    : 	This souce file for DeepSleep. 
*******************************************************************************/
#include "main.h"
#include "stm32f10x_lib.h"

int main(void)
{
	
}
/**
*	open clock on APB1 : PWR
*	open clock on APB2 : GPIOA PA[0,4] used for I/O
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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);
}

/**
*	PA0 Input in IPU mode
*	PA4 Output in PP mode @ 50 MHz
**/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*
	*	PA4 Output in PP mode @ 50 MHz
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
	*	PA0 Input in IPU mode
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
*	Set PA0 as EXTI source (Line 0) and enable it
*	Mode : Interrupt
*	Trigger : as falling 
**/
void EXTI_Configuration(void)
{
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
}

/**
*	setting priority grouping
*	enable EXTI0_IRQHandler with proper priority
**/
void NVIC_Configuration(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

/**
*	Select HCLK as SysTick clock souce(AHB clock selected as SysTick clock source.)
*	Set Reload time as 100ms
**/
void SysTick_Configuration(void)
{
	SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_SetReload( 72000 * 100 ); // 100ms, for HCLK @ 72 MHz
}

/**
*	Use SysTick counter to implement
*	@param Tick time to delay in second
**/
void Delay_NSecond(u8 Tick)
{
	u32 TickCounter = 10 * Tick; // because SysTick reload = 100 ms 
	SysTick_CounterCmd(SysTick_Counter_Enable);
	while(TickCounter--)
	{
		while(SysTick_GetFlagStatus(SysTick_FLAG_COUNT) == 0);
	}
	SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
}

