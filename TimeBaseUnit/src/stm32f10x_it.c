/*******************************************************************************
* File Name      : 	stm32f10x_it.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/10/2014
* Version        : 	Version 1.0
* Description    : 	This head file of overwritting handler for TimeBaseUnit. 
*******************************************************************************/

/**
*	When TIM_IT_CCx is set, 
*		1. 	PA(x+3) will be toggled. x = 1...4
*		2.	Set Compare as the counter + CCRx_Val (which is defined in main.h)
*		3.	TIM_ClearITPendingBit(TIM2, TIM_TI_CCx)
**/
void TIM2_IRQHandler(void)
{
	//TODO
}
