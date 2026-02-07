/***************************************************************************************************
* FILENAME : state_machine.h
* DESCRIPTION : Defines enums and function prototypes for state meachine.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

#ifndef __STATE_MACHINE_H_
#define __STATE_MACHINE_H_

/*************************************** Header Inclusion *****************************************/
#include "gpio.h"
#include "gptm.h"

/********************************************* Macros *********************************************/
#define INTERVAL_VAL  160000000
#define PRESCALER_VAL 100
#define DELAY         10

/****************************************** Enumerations ******************************************/
typedef enum {
    /* Initializes SW1,SW2 and timer and turns on the on-board red LED  */
    STATE_INIT,
    /* Does nothing until an interrupt is fired */
    STATE_IDLE,
    /* Starts 10 sec timer on interrupt fired by SW1 */
    STATE_START_TIMER,
    /* Turns on on-board green LED on interrupt fired by timer */
    STATE_TURN_ON_GREEN_LED,
    /* State error */
    STATE_ERROR,
    /* Restarts then state */
    STATE_RESTART
    // STATE_STOP_TIMER,
} state_machine_States;

/*************************************** Function Prototypes **************************************/
void state_machine_initState(gptm_Config* timer, gpio_Config* switches,
                             gpio_Config* red_led, gpio_Config* green_led);
void state_machine_startTimer(gptm_Config* config);
void state_machine_restart(gptm_Config* config, uint32_t new_ticks, gpio_Config* red_led, gpio_Config* green_led);
void state_machine_turnOnGreenLed(gpio_Config* red_led, gpio_Config* green_led);

#endif /*__STATE_MACHINE_H_*/