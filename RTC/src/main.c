/*******************************************************************************
* File Name       	: main.c
* Author          	: timmy00274672 (timmy00274672@gmail.com)
* Date            	: 06/05/2014
* Version			: Version 1.0
* Description     	: This source file for RTC project
*******************************************************************************/
#include "main.h"
#include "stm32f10x_lib.h"
#include "time.h"
/**
*	0 -> Not to be displayed
*	Vice Versa.
**/
vu32 TimeDisplay = 0; 	

/**
*	Time for now in Calendar format
**/
Calendar time_now = {2014,6,6,17,0,0}; //The inital time in Calendar format

int main(void)
{

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	RTC_Configuration();
	Time_SetCalendarTime(time_now);
	while(1) Time_Show();
	
}

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

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO
    	,ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    /**
        Configurate the USART1 Tx : PA[9] to be the 
            alternate function push-pull output mode
            Maximum output frequency : 50MHz
    **/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /**
        Configurate the USART1 Rx : PA[10] to be the floating input mode
    **/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);     
}

/**
*	1. Enable Backup Access
*	2. Initialize BKP register
*	3. Use LSE as clock source (32.768 KHz)
*	4. Set Prescaler as 92767 such that the frequency = 32.768 KHz / ( 32767 + 1 ) = 1 Hz
*	Before each access to those backup registers, make sure the last action is over.
**/
void RTC_Configuration(void)
{
	PWR_BackupAccessCmd(ENABLE);
	BKP_DeInit();
	//RCC_LSEConfig(RCC_LSE_ON);  // LSE oscillator ON
	//while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET); // wait for LSE oscillator clock ready
	//RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // LSE selected as RTC clock
	RCC_LSICmd(ENABLE);  // Enable LSI
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);               /* Select the RTC Clock Source */

	RCC_RTCCLKCmd(ENABLE);

	/*
	*	Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
	*	are synchronized with RTC APB clock.
	*	This function must be called before any read operation after
	*	an APB reset or an APB clock stop.
	*/
	RTC_WaitForSynchro();
	RTC_ITConfig(RTC_IT_SEC,ENABLE); // Enable Second interrupt
	
	/* 
	*	Waits until last write operation on RTC registers has finished. 
	*	This function must be called before any write to RTC registers.
	*/
	RTC_WaitForLastTask(); 
	// RTC_SetPrescaler(32767);
	RTC_SetPrescaler(39999);

	RTC_WaitForLastTask(); 
}

void Time_Show(void)
{
	u32 CurrentTime = 0;
	if(TimeDisplay)
	{
		CurrentTime = Time_GetUnitTime();
		time_now = Time_ConvUnixToCalendar(CurrentTime);

		printf("\r\nTime : %d-%d-%d, %d:%d:%d \r\n",
			time_now.tm_year,
			time_now.tm_mon,
			time_now.tm_mday,
			time_now.tm_hour,
			time_now.tm_min,
			time_now.tm_sec);

		TimeDisplay = 0;
	}
}

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

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (u16)ch);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    return ch;
}

void Time_SetCalendarTime(Calendar t)
{
	Time_SetUnixTime(Time_ConvCalendarToUnix(t));
}

u32 Time_ConvCalendarToUnix(Calendar t)
{
	t.tm_year -=1900;
	return mktime(&t);
}

u32 Time_GetUnitTime(void)
{
	return (u32)RTC_GetCounter();
}

Calendar Time_ConvUnixToCalendar(time_t t)
{
	Calendar* temp_t;
	temp_t  =  localtime( &t);
	temp_t -> tm_year += 1900 ;
	return *temp_t;
}

void Time_SetUnixTime(u32 t)
{
	RTC_WaitForLastTask();
	RTC_SetCounter(t);
	RTC_WaitForLastTask();
}