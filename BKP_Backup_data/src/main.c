#include "main.h"
#include "stm32f10x_lib.h"		
#include "stm32f10x_pwr.h"

int main(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	USART_Configuration();
	BKP_Configuration();
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);
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

void BKP_Configuration(void)
{
	PWR_BackupAccessCmd(ENABLE);
	BKP_ClearFlag(); //Clears Tamper Pin Event pending flag
}