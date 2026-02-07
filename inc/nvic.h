/******************************************************************************
* FILENAME : nvic.h 
* DESCRIPTION : This file defines NVIC register address to enable interrupts.
*
* AUTHOR : Mariyam Shahid
******************************************************************************/
#ifndef _NVIC_H_
#define _NVIC_H_
#include "../inc/gptm.h"

/******************************** Macros *********************************/
#define NVIC_EN0     (*((volatile uint32_t *)  0xE000E100))
#define NVIC_EN1     (*((volatile uint32_t *)  0xE000E104))
#define NVIC_EN2     (*((volatile uint32_t *)  0xE000E108))
#define NVIC_EN3     (*((volatile uint32_t *)  0xE000E10C))

/* GPIO NVIC enable pin masks */
#define GPIOA_IRQ    (1U << 0)
#define GPIOB_IRQ    (1U << 1)
#define GPIOC_IRQ    (1U << 2)
#define GPIOD_IRQ    (1U << 3)
#define GPIOE_IRQ    (1U << 4)
#define GPIOF_IRQ    (1U << 30)

/* Timer NVIC enable pin masks */
/* Timer A */
#define TIM0A_IRQ    (1U << 19)
#define TIM1A_IRQ    (1U << 21)
#define TIM2A_IRQ    (1U << 23)
#define TIM3A_IRQ    (1U << 3)
#define TIM4A_IRQ    (1U << 6)
#define TIM5A_IRQ    (1U << 28)
/* Timer B */
#define TIM0B_IRQ    (1U << 20)
#define TIM1B_IRQ    (1U << 22)
#define TIM2B_IRQ    (1U << 24)
#define TIM3B_IRQ    (1U << 4)
#define TIM4B_IRQ    (1U << 7)
#define TIM5B_IRQ    (1U << 29)
/* Wide timer A */
#define WTIM0A_IRQ    (1U << 30)
#define WTIM1A_IRQ    (1U << 0)
#define WTIM2A_IRQ    (1U << 2)
#define WTIM3A_IRQ    (1U << 4)
#define WTIM4A_IRQ    (1U << 6)
#define WTIM5A_IRQ    (1U << 8)
/* Wide timer B */
#define WTIM0B_IRQ    (1U << 31)
#define WTIM1B_IRQ    (1U << 1)
#define WTIM2B_IRQ    (1U << 3)
#define WTIM3B_IRQ    (1U << 5)
#define WTIM4B_IRQ    (1U << 7)
#define WTIM5B_IRQ    (1U << 9)

/* UART NVIC enable pin masks */
#define UART0_IRQ     (1U << 5)
#define UART1_IRQ     (1U << 6)

/*************************************** Function Prototypes **************************************/
void nvic_enbleTimerInterupt(gptm_SelectTimer timer, gptm_Channel channel);

#endif /*_NVIC_H_*/