/*******************************************************************************
* File Name      : 	stm32f10x_it.c
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/06/2014
* Version        : 	Version 1.0
* Description    : 	This source file for BKP_Tamper. Here defines handlers needed 
					in this project.
*******************************************************************************/
#include "stm32f10x_it.h"
#include "stdio.h"
#include "stm32f10x_bkp.h"

void TAMPER_IRQHandler(void)
{
	printf("\r\nA tamper event is comming!!\r\n");
	PrintBackupReg();
	BKP_ClearITPendingBit();
}