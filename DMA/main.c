#include "stm32f10x_lib.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_systick.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

#define BUFFER_SIZE 32

vu16 currDataCounter = 0;
vu32 tick = 0; //used for systick
uc32 srcConstBuffer[BUFFER_SIZE] = 
{
  0x01020304,0x05060708,0x090a0b0c,0x0d0e0f10,
  0x11121314,0x15161718,0x191a1b1c,0x1d1e1f20,
  0x21222324,0x25262728,0x292a2b2c,0x2d2e2f30,
  0x31323334,0x35363738,0x393a3b3c,0x3d3e3f40,
  0x41424344,0x45464748,0x494a4b4c,0x4d4e4f50,
  0x51525354,0x55565758,0x595a5b5c,0x5d5e5f60,
  0x61626364,0x65666768,0x696a6b6c,0x6d6e6f70,
  0x71727374,0x75767778,0x797a7b7c,0x7d7e7f80,
};

u32 dstBuffer[BUFFER_SIZE];
int main(void)
{
    u8 i = 0;
    u8 tickCountCpu = 0;
    u8 tickCountDma = 0;

    RCC_Configuration();
    GPIO_Configuration();
    NVIC_Configuration();
    USART_Configuration();
    DMA_Configuration();
    SysTick_Configuration();

    tick = 0 ;
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        dstBuffer[i] = srcConstBuffer[i];
    }
    tickCountCpu = tick;
    
    for(i = 0; i < BUFFER_SIZE; i++)
    {
        dstBuffer[i] = 0;
    }
   
   tick = 0;
   DMA_Cmd(DMA1_Channel6, ENABLE);
   while(currDataCounter!=0);
   tickCountDma = tick;

   if(strncmp( (const char*)srcConstBuffer, (const char*)dstBuffer , BUFFER_SIZE ) == 0){
    printf("\r\nTransmit Success\r\n");
   }else{
    printf("\r\nTransmit Fault\r\n");
   }
   printf("\r\nCPU cost %d us\r\n",tickCountCpu);
   printf("\r\nDMA cost %d us\r\n",tickCountDma);
   while(1);
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA 
        ,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
#ifdef VECT_TAB_RAM
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    //NVIC Grouping level : 2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    //EXTI0 Priority 0:0
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel6);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) srcConstBuffer;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32) dstBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC ;
    DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable ;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;

    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
    currDataCounter = DMA_GetCurrDataCounter(DMA1_Channel6);
}

void SysTick_Configuration(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    SysTick_SetReload(9);
    SysTick_CounterCmd(SysTick_Counter_Enable);
    SysTick_ITConfig(ENABLE);
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

void SysTick_Handler(void)
{
    tick++;
}

void DMAChannel6_IRQHandler(void)
{
    currDataCounter = DMA_GetCurrDataCounter(DMA1_Channel6);
    DMA_ClearITPendingBit(DMA1_IT_GL6);
}