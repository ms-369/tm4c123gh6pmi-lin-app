/***************************************************************************************************
* FILENAME : state_machine.c
* DESCRIPTION : Implements the application-level state machine controlling system behavior.
*               The state machine coordinates GPIO inputs (SW1, SW2), timer events, and LED outputs.
*               State transitions are triggered by GPIO and GPTM interrupts, while state execution
*               is handled in the main loop.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/state_machine.h"
#include "../inc/gpio.h"
#include "../inc/gptm.h"

/****************************************  Externs  ***********************************************/
extern volatile state_machine_States Current_state;

/************************************* Function Implementations ***********************************/

/**
 * @brief state_machine_initState : Initializes the timer and GPIO.
 * 
 * @param timer     : Configuration of timer specified by the user
 * @param switches  : Configuration of gpio as switch specified by the user
 * @param red_led   : Configuration of gpio as red led specified by the user
 * @param green_led : Configuration of gpio as green led specified by the user
 * 
 * 
 * @return 
 */
void state_machine_initState(gptm_Config* timer, gpio_Config* switches,
                             gpio_Config* red_led, gpio_Config* green_led)
{
    /* Initialising GPIO for input */
    switches->PORT = PORT_F_AHB;
    /* Mention on page # 20 of user_manual */
    /* PIN0 is sw2 on the board */
    /* PIN4 is sw1 on the board */
    switches->PIN  = PIN0 | PIN4;
    switches->MODE = DIGITAL_INPUT;
    switches->GPIO_INTERRUPT = 1;
    gpio_initGpioPort(switches);

    /* Initialising GPIO for output */
    red_led->PORT = PORT_F_AHB;
    red_led->PIN  = PIN1;
    red_led->MODE = DIGITAL_OUTPUT;
    red_led->GPIO_INTERRUPT = 0;
    gpio_initGpioPort(red_led);

    /* Initialising GPIO for output */
    green_led->PORT = PORT_F_AHB;
    green_led->PIN  = PIN3;
    green_led->MODE = DIGITAL_OUTPUT;
    green_led->GPIO_INTERRUPT = 0;
    gpio_initGpioPort(green_led);
    
    /* Initialising timer */
    timer->TIMER = GPTM_TIMER0_16_32;
    timer->CHANNEL = GPTM_CHANNEL_A;
    timer->MODE = GPTM_MODE_ONE_SHOT;
    timer->COUNT_DIR = GPTM_COUNT_UP;
    timer->INTERVAL = INTERVAL_VAL;
    timer->GPTM_INTERRUPT = 1;
    timer->CONFIG = GPTM_CONCATENATE;
    timer->PRESCALER = PRESCALER_VAL;
    gptm_initTimer(timer);

    /* Turning on on-board red LED */
    gpio_digitalWrite(green_led, 0);
    gpio_digitalWrite(red_led, 1);
}

/**
 * @brief state_machine_restart : Restarts the state machine.
 * 
 * 
 * @return 
 */
void state_machine_restart(gptm_Config* config, uint32_t new_ticks, gpio_Config* red_led, gpio_Config* green_led)
{
    gptm_reloadTimer(config, new_ticks);

    /* Turn on red LED */
    gpio_digitalWrite(red_led, 1);

    /* Turn off green LED */
    gpio_digitalWrite(green_led, 0);
}

/**
 * @brief state_machine_startTimer : Starts the timer.
 * 
 * @param ticks : Load value for timer.
 *
 * @return
 */
void state_machine_startTimer(gptm_Config* config)
{
    gptm_startTimer(config);
}

/**
 * @brief state_machine_turnOnGreenLed : Turn off red LED and turns on green LED.
 * 
 *
 * @return
 */
void state_machine_turnOnGreenLed(gpio_Config* red_led, gpio_Config* green_led)
{
    /* Turn off red LED */
    gpio_digitalWrite(red_led, 0);

    /* Turn on green LED */
    gpio_digitalWrite(green_led, 1);
}