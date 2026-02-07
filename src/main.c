/***************************************************************************************************
* FILENAME : main.c
* DESCRIPTION : Main appllication for testing LIN driver.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/gpio.h"
#include "../inc/gptm.h"
#include "../inc/nvic.h"
#include "../inc/uart.h"
#include "../inc/lin.h"
#include "../inc/state_machine.h"
#include "../inc/lin_app.h"

/********************************************* Globals ********************************************/
volatile state_machine_States Current_state = STATE_IDLE;
gptm_Config timer;
gpio_Config switches;
uart_Config uart_config;

volatile uint8_t data;
uart_Config Terminal_uart;
uart_Config Master_uart;
uart_Config Slave_uart;
lin_FrameStructure lin_frame = {0};
gptm_Config timer_config;
gpio_Config red_config, green_config, blue_config;
lin_app_ErrorType ret;
#ifdef LIN_MASTER
volatile lin_Masterstate Lin_master_state = LIN_MASTER_INIT;
#endif
#ifdef LIN_SLAVE
volatile lin_Slavestate Lin_slave_state = LIN_SLAVE_INIT;
#endif
/************************************** Main Implementation ***************************************/

/**
 * @brief main : Main function for testing lin app driver
 * 
 * @return int
 */
int main(void)
{
/* LIN slave states */
#ifdef LIN_SLAVE
    while(1)
    {
        switch (Lin_slave_state) 
        {
            case LIN_SLAVE_INIT:
                lin_app_initSlave(&Slave_uart, &lin_frame,
                                   &red_config, &green_config, &blue_config);
                break;
            case LIN_SLAVE_IDLE:
                break;
            case LIN_SLAVE_GET_COMMAND:
                lin_receiveFrame(&lin_frame);
                break;
            case LIN_SLAVE_SEND_ACK:
                lin_app_sendAck(&Slave_uart, &lin_frame);
                break;
            case LIN_SLAVE_SEND_ERR:
                lin_app_sendErr(&Slave_uart, &lin_frame);
                break;
            case LIN_SLAVE_SEND_OFF:
                lin_app_sendOff(&Slave_uart, &lin_frame);
                break;
            case LIN_SLAVE_EXECUTE_COMMAND:
                lin_app_executeCommand(&lin_frame, &red_config, &green_config, &blue_config);
                break;
            case LIN_SLAVE_OFF:
                lin_app_offSlave(&Slave_uart, &red_config, &green_config, &blue_config);
                break;
            defualt:
                    
        }
    }
#endif

/* LIN master states */
#ifdef LIN_MASTER
    while(1)
    {
        switch (Lin_master_state) 
        {
            case LIN_MASTER_INIT:
                lin_app_initMaster(&Terminal_uart, &Master_uart, &lin_frame, &timer);                
                break;
            case LIN_MASTER_IDLE:
                break;
            case LIN_MASTER_WAITING:
                break;
            case LIN_MASTER_SEND_COMMAND:
                lin_app_sendCommandFrame(&Master_uart, &lin_frame);
                break;
            case LIN_MASTER_WAIT_FOR_ACK:
                lin_app_waitForAck(&timer, ACK_TIMEOUT);
                break;
            case LIN_MASTER_GET_RESPONSE:
                lin_app_getResponseFrame(&lin_frame);
                break;
            case LIN_MASTER_ACK_RECEIVED:
                lin_app_ackReceived(&Terminal_uart, &timer);
                break;
            case LIN_MASTER_ACK_TIMEOUT:
                lin_app_errReceived(&Terminal_uart);
                break;
            case LIN_MASTER_ERR_RECEIVED:
                lin_app_errReceived(&Terminal_uart);
                break;
            case LIN_MASTER_OFF_RECEIVED:
                lin_app_offMaster(&Terminal_uart, &Master_uart, &lin_frame, &timer);
                break;
            defualt:
  
        }
    }
#endif

    return 0;
}
