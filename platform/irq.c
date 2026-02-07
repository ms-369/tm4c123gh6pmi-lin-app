/******************************************************************************
* FILENAME : irq.c
* DESCRIPTION : This file contains the IRQ Handlers for interrupts which might
*               have been used in rest of the code.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
******************************************************************************/

/***************************** Header Inclusion ******************************/
#include "isr.h"

/******************************* IRQ Handlers ********************************/

/**
 * @brief TIM0A_IRQHandler : Interrupt handler for timer 0 channel A.
 * 
 * @return
 */
void TIM0A_IRQHandler() {

    Isr_Timer0A();
}

/**
 * @brief GPIOF_IRQHandler : Interrupt handler for gpio port F.
 * 
 * @return
 */
void GPIOF_IRQHandler() {

    Isr_GpioF();
}

/**
 * @brief UART0_IRQHandler : Interrupt handler for UART 0.
 * 
 * @return
 */
void UART0_IRQHandler() {

    Isr_Uart0();
}

/**
 * @brief UART1_IRQHandler : Interrupt handler for UART 1.
 * 
 * @return
 */
void UART1_IRQHandler() {

    Isr_Uart1();
}