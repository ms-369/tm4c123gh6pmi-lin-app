/******************************************************************************
* FILENAME : isr.c
* DESCRIPTION : This file contains the definitions of all functions prototypes
*               which are defined in isr.h | Interrupt Service Routines (ISRs)
*               are being implemented in this file.
* AUTHOR : Mariyam Shahid
******************************************************************************/

/***************************** Header Inclusion ******************************/
#include "isr.h"
#include "..\inc\gptm.h"
#include "..\inc\uart.h"
#include "..\inc\lin.h"
#include "..\inc\lin_app.h"


/********************************* Externs ***********************************/
extern gptm_Config timer;
extern gpio_Config switches;
extern uart_Config uart_config;
extern uart_Config Terminal_uart;
extern uart_Config Master_uart;
extern uart_Config Slave_uart;
extern volatile uint8_t data;
/********************************* Globals ***********************************/

/**
 * @brief Isr_Timer0A : Interrupt service routine for timer 0 channel A.
 * 
 * @return
 */
void Isr_Timer0A(void)
{
    gptm_interruptCallback(&timer);
}

/**
 * @brief Isr_GpioF : Interrupt service routine for GPIO port F.
 * 
 * @return
 */
void Isr_GpioF(void)
{
    gpio_interruptCallback(&switches);
}

/**
 * @brief UART0_IRQHandler : Interrupt service routine for UART 0.
 * 
 * @return
 */
void Isr_Uart0(void)
{
    // uart0_interruptCallback();
    lin_receiveByte(&Terminal_uart);
}

/**
 * @brief UART1_IRQHandler : Interrupt service routine for UART 1.
 * 
 * @return
 */
void Isr_Uart1(void)
{
    #ifdef LIN_MASTER
    lin_receiveByte(&Master_uart);
    #endif
    #ifdef LIN_SLAVE
    lin_receiveByte(&Slave_uart);
    #endif
}