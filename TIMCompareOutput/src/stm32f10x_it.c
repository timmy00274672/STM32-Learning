/*******************************************************************************
* File Name			: 	stm32f10x_it.h
* Author			: 	timmy00274672 (timmy00274672@gmail.com)
* Date				: 	06/21/2014
* Version			: 	Version 1.0
* Description		: 	This source file of overwritting handler for TIMCompareOutput. 
*******************************************************************************/
#include "stm32f10x_it.h"
extern uc16 CCR1_Val;
extern uc16 CCR2_Val;
extern uc16 CCR3_Val;
extern uc16 CCR4_Val;
/**
*	When TIM_IT_CCx is set, 
*		1. 	PA(x+3) will be toggled. x = 1...4 //automatically
*		2.	Set Compare as the counter + CCRx_Val (which is defined in main.h)
*		3.	TIM_ClearITPendingBit(TIM2, TIM_TI_CCx)
**/
void TIM2_IRQHandler(void)
{
	vu16 capture = 0;

	if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
	{
		// GPIO_WriteBit(GPIOA,GPIO_Pin_4, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_4)));
		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_Val);
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

	}else if(TIM_GetITStatus(TIM2,TIM_IT_CC2) != RESET)
	{
		// GPIO_WriteBit(GPIOA,GPIO_Pin_5, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5)));
		capture = TIM_GetCapture2(TIM2);
		TIM_SetCompare2(TIM2, capture + CCR2_Val);
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

	}else if(TIM_GetITStatus(TIM2,TIM_IT_CC3) != RESET)
	{
		// GPIO_WriteBit(GPIOA,GPIO_Pin_6, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_6)));
		capture = TIM_GetCapture3(TIM2);
		TIM_SetCompare3(TIM2, capture + CCR3_Val);
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);

	}if(TIM_GetITStatus(TIM2,TIM_IT_CC4) != RESET)
	{
		// GPIO_WriteBit(GPIOA,GPIO_Pin_7, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7)));
		capture = TIM_GetCapture4(TIM2);
		TIM_SetCompare4(TIM2, capture + CCR4_Val);
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		
	}
}
