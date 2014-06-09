/*******************************************************************************
* File Name      : 	stm32f10x_it.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/06/2014
* Version        : 	Version 1.0
* Description    : 	This source file for RTC project. Here defines handlers needed 
					in this project.
*******************************************************************************/

#include "stm32f10x_it.h"
#include "stdio.h"

void ADC_IRQHandler(void)
{
	float VolValue = 0.00;
	VolValue = 2.56 * ADC_GetConversionValue(ADC1) / 0x0FFF;
	printf("\r\n = %.2fv \r\n", VolValue);
}
