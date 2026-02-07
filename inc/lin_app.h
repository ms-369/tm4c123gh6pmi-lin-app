/***********************************************************************************************
* FILENAME : lin_app.h
* DESCRIPTION : Toggles LEDs specified by the user.
*
* NOTES :
* AUTHOR : Mariyam Shahid
***********************************************************************************************/

#ifndef __LIN_APP_H_
#define __LIN_APP_H_

/*************************************** Header Inclusion *************************************/
#include "uart.h"
#include "types.h"
#include "lin.h"
#include "gptm.h"

/******************************************** Macros ******************************************/
#define ACK_TIMEOUT 10
/* #define LIN_MASTER */
#define LIN_SLAVE
#define SLAVE_PID 0x3F
#define UART_DATA_BAUD_RATE     9600
#define RED 3
#define BLUE 4
#define GREEN 5
#define IS_SAME 1
#define IS_NOT_SAME 0
#define RESPONSE_SIZE 3

/****************************************** Enumerations ******************************************/
#ifdef LIN_SLAVE
typedef enum
{
    LIN_SLAVE_INIT = 0,
    LIN_SLAVE_IDLE,
    LIN_SLAVE_GET_COMMAND,
    LIN_SLAVE_EXECUTE_COMMAND,
    LIN_SLAVE_SEND_ACK,
    LIN_SLAVE_SEND_ERR,
    LIN_SLAVE_SEND_OFF,
    LIN_SLAVE_OFF,
    LIN_SLAVE_ERROR
} lin_Slavestate;
#endif

#ifdef LIN_MASTER
typedef enum
{
    LIN_MASTER_INIT,
    LIN_MASTER_IDLE,
    LIN_MASTER_SEND_COMMAND,
    LIN_MASTER_WAIT_FOR_ACK,
    LIN_MASTER_GET_RESPONSE,
    LIN_MASTER_WAITING,
    LIN_MASTER_ACK_RECEIVED,
    LIN_MASTER_ACK_TIMEOUT,
    LIN_MASTER_ERR_RECEIVED,
    LIN_MASTER_OFF_RECEIVED,
    LIN_MASTER_ERROR
} lin_Masterstate;
#endif

typedef enum {
#ifdef LIN_SLAVE
    LIN_SLAVE_INIT_SUCCESSFUL,
    LIN_SLAVE_INIT_UNSUCCESSFUL,
    LIN_GET_COMMAND_SUCCESSFUL,
    LIN_GET_COMMAND_UNSUCCESSFUL,
    LIN_EXECUTE_COMMAND_UNSUCCESSFUL,
    LIN_EXECUTE_COMMAND_SUCCESSFUL,
    LIN_SEND_ACK_SUCCESSFUL,
    LIN_SEND_ACK_UNSUCCESSFUL,
    LIN_SEND_ERR_SUCCESSFUL,
    LIN_SEND_ERR_UNSUCCESSFUL,
    LIN_SEND_OFF_SUCCESSFUL,
    LIN_SEND_OFF_UNSUCCESSFUL,
    LIN_SLAVE_OFF_UNSUCCESSFUL,
    LIN_SLAVE_OFF_SUCCESSFUL
#endif

#ifdef LIN_MASTER
    LIN_MASTER_INIT_SUCCESSFUL,
    LIN_MASTER_INIT_UNSUCCESSFUL,
    LIN_MASTER_OFF_SUCCESSFUL,
    LIN_MASTER_OFF_UNSUCCESSFUL,
    LIN_SEND_COMMAND_SUCCESSFUL,
    LIN_SEND_COMMAND_UNSUCCESSFUL,
    LIN_ACK_INIT_TIMER_UNSUCCESSFUL,
    LIN_ACK_START_TIMER_UNSUCCESSFUL,
    LIN_GET_ACK_SUCCESSFUL,
    LIN_GET_RESPONSE_SUCCESSFUL,
    LIN_GET_RESPONSE_UNSUCCESSFUL
#endif

} lin_app_ErrorType;

/******************************************* Data Types *******************************************/



/*************************************** Function Prototypes **************************************/
#ifdef LIN_SLAVE
lin_app_ErrorType lin_app_initSlave(uart_Config* uart_config, lin_FrameStructure* lin_frame,
                                  gpio_Config* red_config, gpio_Config* green_config,
                                  gpio_Config* blue_config);
lin_app_ErrorType lin_app_getCommandFrame(lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_executeCommand(lin_FrameStructure* lin_frame, gpio_Config* red_config,
                                         gpio_Config* green_config, gpio_Config*  blue_config);
lin_app_ErrorType lin_app_sendAck(uart_Config* uart_config, lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_sendErr(uart_Config* uart_config, lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_sendOff(uart_Config* lave_uart, lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_offSlave(uart_Config* uart_config, gpio_Config* red_config, gpio_Config* green_config,
                                  gpio_Config* blue_config);
#endif /* LIN_SLAVE */

#ifdef LIN_MASTER
lin_app_ErrorType lin_app_initMaster(uart_Config* Terminal_uart, uart_Config* uart_config, lin_FrameStructure* lin_frame,
                                  gptm_Config* timer);
lin_app_ErrorType lin_app_sendCommandFrame(uart_Config* uart_config,lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_waitForAck(gptm_Config* timer, uint32_t new_ticks);
lin_app_ErrorType lin_app_getResponseFrame(lin_FrameStructure* lin_frame);
lin_app_ErrorType lin_app_ackReceived(uart_Config* uart_config, gptm_Config* timer);
lin_app_ErrorType lin_app_errReceived(uart_Config* uart_config);
lin_app_ErrorType lin_app_offMaster(uart_Config* Terminal_uart, uart_Config* uart_config, lin_FrameStructure* lin_frame,
                                  gptm_Config* timer);
#endif /* LIN_MASTER */

#endif /* __LIN_APP_H_ */
