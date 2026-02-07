/******************************************************************************
* FILENAME : isr.h 
* DESCRIPTION : This file contains the prototypes of all functions and macros
*               which are defined/used in isr.c | Interrupt Service Routines
*               are being prototyped in this file.
* AUTHOR : Mariyam Shahid
******************************************************************************/

#ifndef _ISR_H_
#define _ISR_H_

/******************************************************************************
*                            FUNCTION-PROTOTYPES
******************************************************************************/
void Isr_Timer0A(void);
void Isr_GpioF(void);
void Isr_Uart0(void);
void Isr_Uart1(void);

#endif /*_ISR_H_*/