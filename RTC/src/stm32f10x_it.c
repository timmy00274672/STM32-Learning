/*******************************************************************************
* File Name      : 	stm32f10x_it.c
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/06/2014
* Version        : 	Version 1.0
* Description    : 	This head file for RTC project. Here implements handlers needed 
					in this project.
*******************************************************************************/
#include "stm32f10x_it.h"

extern vu32 TimeDisplay;
void RTC_IRQHandler(void)
{
	if(RTC_GetFlagStatus(RTC_FLAG_SEC) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_SEC);
		TimeDisplay = 1;
	}
}