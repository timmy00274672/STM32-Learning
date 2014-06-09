/*******************************************************************************
* File Name      : 	main.c
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/09/2014
* Version        : 	Version 1.1
* Description    : 	This source file for ADC_Basic 
*******************************************************************************/
#include "main.h"
#include "stdio.h"
#include "stm32f10x_lib.h"

int main(void)
{
	
	RCC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	NVIC_Configuration();
	ADC_Configuration();

	printf("\r\nThe AD_Value : \r\n");
	while(1);
}

/**
*	open clock on APB2 : USART1 for communication
						 GPIOA PA[9:10] 
						 GPIOB PB0 is connected to VR3
						 ADC1

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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
    	RCC_APB2Periph_USART1 |
    	RCC_APB2Periph_ADC1 | 
    	RCC_APB2Periph_GPIOB |
    	RCC_APB2Periph_AFIO,
    	ENABLE);
}

/**
*	PA9 as Tx in AF_PP mode @ 50 MHz
*	PA10 as Rx in floating mode
*	PB0 as analog signal in AIN mode
**/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*
	*	PA9 as Tx in AF_PP mode @ 50 MHz
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	*	PA10 as Rx in floating mode
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	*	PB0 as analog signal in AIN mode
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
}

/**
*	1.	Configurate ADC1 clock as PCLK2/4
*	2.	Mode : independent
		Scan Mode : enable
		Continuous Mode : enable
		Trigger : S/W
		Align : right
		Channel # : 1
	3.	ADC1 Channel
		Channel 8 (Default alternative function of PB0 is ADC_IN8.)
		Rank 1
		Sampling cycle : 55.5
	4.	Calibrate
	5.	Enable interrupt for end of the conversion
**/
void ADC_Configuration(void)
{
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE; 
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStruct);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);

	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	ADC_SoftwareStartConvCmd(ADC1,ENABLE);

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
*	Open the ADC1_2_IRQChannel
**/
void NVIC_Configuration(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQChannel;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStruct);	
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