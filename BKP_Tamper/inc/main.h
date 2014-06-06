/*******************************************************************************
* File Name      : 	main.h
* Author         : 	timmy00274672 (timmy00274672@gmail.com)
* Date           : 	06/06/2014
* Version        : 	Version 1.0
* Description    : 	This source file for BKP_Tamper. Part of the code are directly
					copy from the BKP_Backup_data, and add tamper-related code, 
					including NVIC_Configuration() and corresponding handler.
*******************************************************************************/
#include "stm32f10x_lib.h"
#include "stdio.h"

/**
*	Open PWR BKP clock (BKP is enabled when PWR backup access is enabled)
*	Open USART1 GPIOA clock (USART1 use PA[9:10])
**/
void RCC_Configuration(void);

/*
*	Set PA[9:10] as Tx and Rx mode.
*/
void GPIO_Configuration(void);

/**
	Set the clock including CPOL, CPHA, LastBit
	Set the Frame including BaudRate, WordLength, StopBits, Parity, HardareFlowControl, Mode
**/
void USART_Configuration(void);

/**
*	Enable Backup Access (PWR), clear flag, set TPAL, IT for Tamper, and enalbe Tamper
**/
void BKP_Configuration(void);

/**
*	Print all BKP_DRx x = 1 ... 10
**/
void PrintBackupReg(void);

/**
*	BKP_DRx will be set FirstBackupData + OFFSETx (x = 1 ... 10)
*	OFFSETx is constant.
*	@param FirstBackupData will be write to BKP_DR1
**/
void WriteToBackupReg(u16 FirstBackupData);

/**
*	@param FirstBackupData will be compared to BKP_DR1
*	@retrun 0 if all BKP_DRxs are equal	to FirstBackupData + OFFSETx (x = 1 ... 10);
*
*		i (i != 0) all BKP_DRxs are equal to FirstBackupData + OFFSETx (x = 1 ... i-1) 
*		but neither does BKP_DRi
**/
u8 CheckBackupReg(u16 FirstBackupData);

/**
*	@retrun 0 if all BKP_DRxs are equal	to 0;
*
*		i (i != 0) all BKP_DRxs are equal to 0 
*		but neither does BKP_DRi
**/
u8 IsBackupRegReset(u16);

/**
*	Write the `ch` to USART1
**/
int fputc(int ch, FILE *f);

