/***************************************************************************************************
* FILENAME : nvic.c
* DESCRIPTION : Enables NVIC interrupts.
*
* NOTES : 
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/nvic.h"

/************************************* Function Implementations ***********************************/

/**
 * @brief nvic_enbleTimerInterupt : Enables NVIC interrupt for timer specified
 * 
 * @param timer     : Timer specified by the user
 * @param channel  : Channel of timer specified by the user
 * 
 * @return 
 */
void nvic_enbleTimerInterupt(gptm_SelectTimer timer, gptm_Channel channel)
{
    switch (timer)
    {
        /* Enable NVIC interrupt for timer 0 */
        case GPTM_TIMER0_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM0A_IRQ;
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM0B_IRQ;
            }
            break;
        /* Enable NVIC interrupt for timer 1 */
        case GPTM_TIMER1_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM1A_IRQ;
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM1B_IRQ;
            }
            break;
        /* Enable NVIC interrupt for timer 2 */    
        case GPTM_TIMER2_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM2A_IRQ;
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN0 |= TIM2B_IRQ;
            }
            break;
        /* Enable NVIC interrupt for timer 3 */
        case GPTM_TIMER3_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN1 |= (TIM3A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN1 |= (TIM3B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for timer 4 */
        case GPTM_TIMER4_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (TIM4A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (TIM4B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for timer 5 */
        case GPTM_TIMER5_16_32:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (TIM5A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (TIM5B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 0 */
        case GPTM_WTIMER0_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (WTIM0A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN2 |= (WTIM0B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 1 */
        case GPTM_WTIMER1_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM1A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM1B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 2 */
        case GPTM_WTIMER2_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM2A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM2B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 3 */
        case GPTM_WTIMER3_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM3A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM3B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 4 */
        case GPTM_WTIMER4_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM4A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM4B_IRQ);
            }
            break;
        /* Enable NVIC interrupt for wide timer 5 */
        case GPTM_WTIMER5_32_64:
            if(channel == GPTM_CHANNEL_A || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM5A_IRQ);
            }
            if(channel == GPTM_CHANNEL_B || channel == GPTM_CHANNEL_BOTH) {
                NVIC_EN3 |= (WTIM5B_IRQ);
            }
            break;
        default:
            break;
    }
}