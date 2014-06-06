/*******************************************************************************
* File Name       	: main.h
* Author          	: timmy00274672 (timmy00274672@gmail.com)
* Date            	: 06/05/2014
* Version			: Version 1.0
* Description     	: This head file for RTC project
*******************************************************************************/
#include "time.h"
#include "stm32f10x_lib.h"
#include "stdio.h"

typedef struct tm Calendar;
/**
*	Open PWR BKP clock (BKP is enabled when PWR backup access is enabled)
*	Open USART1 GPIOA clock (USART1 use PA[9:10])
**/
void RCC_Configuration(void);


/**
*	Open the RTC_IRQChannel
**/
void NVIC_Configuration(void);

/**
*	Set PA[9:10] as Tx and Rx mode.
**/
void GPIO_Configuration(void);

/**
*	1. Enable Backup Access
*	2. Initialize BKP register
*	3. Use LSE as clock source (32.768 KHz)
*	4. Set Prescaler as 92767 such that the frequency = 32.768 KHz / ( 32767 + 1 ) = 1 Hz
*	Before each access to those backup registers, make sure the last action is over.
**/
void RTC_Configuration(void);

/**
	Set the clock including CPOL, CPHA, LastBit
	Set the Frame including BaudRate, WordLength, StopBits, Parity, HardareFlowControl, Mode
**/
void USART_Configuration(void);

/**
*	Write the `ch` to USART1
**/
int fputc(int ch, FILE *f);

/**
*	Print the time through USART1
**/
void Time_Show(void);

/**
*	Convert the calendar to UNIX format and set the RTC counter
*	@param t : calendar structure
**/
void Time_SetCalendarTime(Calendar t);

/**
*	Convert the calendar to UNIX format
*	@param t : calendar structure
*	@return the UNIX time format of t
**/
u32 Time_ConvCalendarToUnix(Calendar t);

/**
*	Read the RTC counter.
*	@return the RTC counter in UNIX format
**/
u32 Time_GetUnitTime(void);

/**
*	Convert the UNIX to calendar format
*	@param t : time in UNIX format
*	@return t in calendar format
**/
Calendar Time_ConvUnixToCalendar(time_t t);

/**
*	Set the RTC counter as t
*	@param t : time in UNIX format
**/
void Time_SetUnixTime(u32 t);