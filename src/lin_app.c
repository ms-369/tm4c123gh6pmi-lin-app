/*************************************************************************************************
* FILENAME : lin_app.c
* DESCRIPTION : LIN protocol implementation over UART1 following LIN 2.1 spec.
*               Supports frame init, transmission, reception with break, sync, PID, data,
*               checksum, and verification.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
*************************************************************************************************/

/*************************************** Header Inclusion ***************************************/
#include "../inc/Lin.h"
#include "../inc/uart.h"
#include "../inc/gptm.h"
#include "../inc/state_machine.h"
#include "../inc/lin_app.h"

/*************************************** Global Data ********************************************/
#ifdef LIN_MASTER
extern volatile lin_Masterstate Lin_master_state;
#endif
#ifdef LIN_SLAVE
extern volatile lin_Slavestate Lin_slave_state;
extern uint8_t Lin_ResponseBuffer[LIN_RX_BUFFER_SIZE];
#endif
extern uint8_t Lin_RxBuffer[LIN_RX_BUFFER_SIZE];
extern uint8_t Lin_RxIndex;

/************************************  Function Implementations *********************************/

/**
 * @brief is_same : Compares two null-terminated byte strings for equality.
 *
 * @param str1  : Pointer to input string
 * @param str2 : Pointer to reference string
 *
 * @return uint8_t : 1 if strings match, 0 otherwise
 */
uint8_t is_same(uint8_t* str1, uint8_t* str2, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++) {
        if (str1[i] != str2[i]) {
            return IS_NOT_SAME;
        }
    }
    return IS_SAME;
}

#ifdef LIN_SLAVE
/**
 * @brief lin_app_initSlave : Initializes LIN slave application.
 *                            Configures UART, GPIOs for LEDs,
 *                            and initializes LIN frame parameters.
 *
 * @param uart_config  : UART configuration specified by the user
 * @param lin_frame    : LIN frame structure specified by the user
 * @param red_config   : GPIO configuration for red LED
 * @param green_config : GPIO configuration for green LED
 * @param blue_config  : GPIO configuration for blue LED
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_initSlave(uart_Config* uart, lin_FrameStructure* lin_frame,
                                  gpio_Config* red_config, gpio_Config* green_config,
                                  gpio_Config* blue_config)
{   
    /* Initialising UART */
    uart->UART_NUM = UART_1;
    uart->BAUD_RATE = LIN_DATA_BAUD_RATE;
    uart->WORD_LENGTH = WORDLENGTH_8_BITS;
    uart->STOP_BITS = STOPBITS_1;
    uart->PARITY = PARITY_NONE;
    uart->UART_INTERRUPT = UART_INTERRUPT_ENABLE;
    uart_init(uart);

    /* Initialising GPIO for output */
    red_config->PORT = PORT_F_AHB;
    red_config->PIN  = PIN1;
    red_config->MODE = DIGITAL_OUTPUT;
    red_config->GPIO_INTERRUPT = GPIO_INTERRUPT_DISABLE;
    gpio_initGpioPort(red_config);
    green_config->PORT = PORT_F_AHB;
    green_config->PIN  = PIN3;
    green_config->MODE = DIGITAL_OUTPUT;
    green_config->GPIO_INTERRUPT = GPIO_INTERRUPT_DISABLE;
    gpio_initGpioPort(green_config);
    blue_config->PORT = PORT_F_AHB;
    blue_config->PIN  = PIN2;
    blue_config->MODE = DIGITAL_OUTPUT;
    blue_config->GPIO_INTERRUPT = GPIO_INTERRUPT_DISABLE;
    gpio_initGpioPort(blue_config);

    /* Initialising LIN */
    lin_frame->PID = SLAVE_PID;
    lin_frame->CHECKSUM_TYPE = LIN_CHECKSUM_ENHANCED;

    if (lin_init(uart, LIN_DATA_BAUD_RATE) == LIN_SUCCESS) {
        Lin_slave_state = LIN_SLAVE_IDLE;
        return LIN_SLAVE_INIT_SUCCESSFUL;
    }

    return LIN_SLAVE_INIT_UNSUCCESSFUL;
}


/**
 * @brief lin_app_getCommandFrame : Processes a received LIN command frame
 *                                  and validates it.
 *
 * @param lin_frame : Pointer to LIN frame structure
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_getCommandFrame(lin_FrameStructure* lin_frame)
{
    if (lin_receiveFrame(lin_frame) == LIN_RECEIVE_SUCCESS) {
        /* change state here to send ack */
        return LIN_GET_COMMAND_SUCCESSFUL;
    }

    /* change state here to send err */
    return LIN_GET_COMMAND_UNSUCCESSFUL;
}

/**
 * @brief lin_app_executeCommand : Executes slave command based on received
 *                                 LIN frame data (RED, GREEN, BLUE).
 *                                 Toggles corresponding GPIO and updates state.
 *
 * @param lin_frame   : Pointer to received LIN frame
 * @param red_config  : GPIO configuration for red LED
 * @param green_config: GPIO configuration for green LED
 * @param blue_config : GPIO configuration for blue LED
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_executeCommand(lin_FrameStructure* lin_frame, gpio_Config* red_config,
                                          gpio_Config* green_config, gpio_Config*  blue_config)
{
    if ((is_same(lin_frame->DATA, "RED", RED)) || 
        (is_same(lin_frame->DATA, "Red", RED)) ||
        (is_same(lin_frame->DATA, "red", RED))) {
        gpio_digitalToggle(red_config);
        Lin_slave_state = LIN_SLAVE_SEND_ACK;
        goto SUCCESSFUL;
    }
    else if ((is_same(lin_frame->DATA, "GREEN", GREEN)) || 
             (is_same(lin_frame->DATA, "Green", GREEN)) || 
             (is_same(lin_frame->DATA, "green", GREEN))) {
             gpio_digitalToggle(green_config);
             Lin_slave_state = LIN_SLAVE_SEND_ACK;
             goto SUCCESSFUL;
    }
    else if ((is_same(lin_frame->DATA, "BLUE", BLUE)) || 
             (is_same(lin_frame->DATA, "Blue", BLUE)) ||
             (is_same(lin_frame->DATA, "blue", BLUE))) {
             gpio_digitalToggle(blue_config);
             Lin_slave_state = LIN_SLAVE_SEND_ACK;
             goto SUCCESSFUL;
    }
    else if ((is_same(lin_frame->DATA, "OFF", BLUE)) || 
             (is_same(lin_frame->DATA, "Off", BLUE)) ||
             (is_same(lin_frame->DATA, "off", BLUE))) {
             Lin_slave_state = LIN_SLAVE_SEND_OFF;
             goto SUCCESSFUL;
    }
    else {
        Lin_slave_state = LIN_SLAVE_SEND_ERR;
        return LIN_EXECUTE_COMMAND_UNSUCCESSFUL;
    }
    SUCCESSFUL:
        /* Clear/refresh receive buffer */
        for (uint32_t i = 0; i < LIN_RX_BUFFER_SIZE; i++) {
            Lin_RxBuffer[i] = '\0';
            Lin_ResponseBuffer[i] = '\0';
        }    

        return LIN_EXECUTE_COMMAND_SUCCESSFUL;
}

/**
 * @brief lin_app_sendAck : Sends an ACK response frame from slave to master
 *                          using LIN frame format.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure used for transmission
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_sendAck(uart_Config* uart, lin_FrameStructure* lin_frame)
{
    Lin_ResponseBuffer[0] = 'A';
    Lin_ResponseBuffer[1] = 'C';
    Lin_ResponseBuffer[2] = 'K';
    Lin_ResponseBuffer[3] = '!';
    Lin_ResponseBuffer[4] = '\0';
    Lin_ResponseBuffer[5] = '\0';
    Lin_ResponseBuffer[6] = '\0';
    Lin_ResponseBuffer[7] = '\0';
    if (lin_sendFrame(uart, lin_frame) == LIN_SUCCESS) {
        Lin_slave_state = LIN_SLAVE_IDLE;
        return LIN_SEND_ACK_SUCCESSFUL;
    }

    return LIN_SEND_ACK_UNSUCCESSFUL;
}

/**
 * @brief lin_app_sendErr : Sends an ERR response frame from slave to master
 *                          indicating command or checksum failure.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure used for transmission
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_sendErr(uart_Config* uart_config, lin_FrameStructure* lin_frame)
{
    Lin_ResponseBuffer[0] = 'E';
    Lin_ResponseBuffer[1] = 'R';
    Lin_ResponseBuffer[2] = 'R';
    Lin_ResponseBuffer[3] = '!';
    Lin_ResponseBuffer[4] = '\0';
    Lin_ResponseBuffer[5] = '\0';
    Lin_ResponseBuffer[6] = '\0';
    Lin_ResponseBuffer[7] = '\0';
    if (lin_sendFrame(uart_config, lin_frame) == LIN_SUCCESS) {
        Lin_slave_state = LIN_SLAVE_IDLE;
        return LIN_SEND_ERR_SUCCESSFUL;
    }
 
    return LIN_SEND_ERR_UNSUCCESSFUL;
}

/**
 * @brief lin_app_sendOff : Sends OFF as a response frame, to turn off master.
 *                          
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure used for transmission
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_sendOff(uart_Config* uart, lin_FrameStructure* lin_frame)
{
    Lin_ResponseBuffer[0] = 'O';
    Lin_ResponseBuffer[1] = 'F';
    Lin_ResponseBuffer[2] = 'F';
    Lin_ResponseBuffer[3] = '!';
    Lin_ResponseBuffer[4] = '\0';
    Lin_ResponseBuffer[5] = '\0';
    Lin_ResponseBuffer[6] = '\0';
    Lin_ResponseBuffer[7] = '\0';
    if (lin_sendFrame(uart, lin_frame) == LIN_SUCCESS) {
        Lin_slave_state = LIN_SLAVE_OFF;
        return LIN_SEND_OFF_SUCCESSFUL;
    }

    return LIN_SEND_OFF_UNSUCCESSFUL;
}

/**
 * @brief lin_app_offSlave : Turns off LIN slave application.
 *                           Disables interrupts for UART and turns off LEDs.
 *                          
 * @param uart_config  : UART configuration specified by the user
 * @param red_config   : GPIO configuration for red LED
 * @param green_config : GPIO configuration for green LED
 * @param blue_config  : GPIO configuration for blue LED
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_offSlave(uart_Config* uart, gpio_Config* red_config, gpio_Config* green_config,
                                  gpio_Config* blue_config)
{   
    /* Disabling UART interrupt */
    uart->UART_INTERRUPT = UART_INTERRUPT_DISABLE;
    if (uart_init(uart) == UART_SUCCESS) {
        /* Turning off LED's */
        gpio_digitalWrite(red_config, 0);
        gpio_digitalWrite(green_config, 0);
        gpio_digitalWrite(blue_config, 0);
        Lin_slave_state = LIN_SLAVE_IDLE;
        return LIN_SLAVE_OFF_SUCCESSFUL;
    } 
    
    return LIN_SLAVE_OFF_UNSUCCESSFUL;
}

#endif

#ifdef LIN_MASTER
/**
 * @brief lin_app_initMaster : Initializes LIN master application.
 *                             Configures UART, GPTM timer for ACK timeout,
 *                             and initializes LIN frame parameters.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure
 * @param timer       : GPTM configuration for timeout handling
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_initMaster(uart_Config* terminal_uart, uart_Config* master_uart, lin_FrameStructure* lin_frame,
                                      gptm_Config* timer)
{
    /* Initialising timer */
    timer->TIMER = GPTM_TIMER0_16_32;
    timer->CHANNEL = GPTM_CHANNEL_A;
    timer->MODE = GPTM_MODE_ONE_SHOT;
    timer->COUNT_DIR = GPTM_COUNT_UP;
    timer->INTERVAL = INTERVAL_VAL;
    timer->GPTM_INTERRUPT = GPTM_INTERRUPT_ENABLE;
    timer->CONFIG = GPTM_CONCATENATE;
    timer->PRESCALER = PRESCALER_VAL;
    gptm_initTimer(timer);

    /* Initialising terminal UART */
    terminal_uart->UART_NUM = UART_0;
    terminal_uart->BAUD_RATE = UART_DATA_BAUD_RATE;
    terminal_uart->WORD_LENGTH = WORDLENGTH_8_BITS;
    terminal_uart->STOP_BITS = STOPBITS_1;
    terminal_uart->PARITY = PARITY_NONE;
    terminal_uart->UART_INTERRUPT = UART_INTERRUPT_ENABLE;
    uart_init(terminal_uart);
    

    /* Initialising Master UART */
    master_uart->UART_NUM = UART_1;
    master_uart->BAUD_RATE = LIN_DATA_BAUD_RATE;
    master_uart->WORD_LENGTH = WORDLENGTH_8_BITS;
    master_uart->STOP_BITS = STOPBITS_1;
    master_uart->PARITY = PARITY_NONE;
    master_uart->UART_INTERRUPT = UART_INTERRUPT_ENABLE;
    uart_init(master_uart);
    
    /* Initialising LIN */
    lin_frame->PID = SLAVE_PID;
    lin_frame->CHECKSUM_TYPE = LIN_CHECKSUM_ENHANCED;
    if (lin_init(master_uart, LIN_DATA_BAUD_RATE) != LIN_SUCCESS) {
        return LIN_MASTER_INIT_UNSUCCESSFUL;
    }
    uart_sendString(terminal_uart, "\r\n<S> \0");
    Lin_master_state = LIN_MASTER_IDLE;
    return LIN_MASTER_INIT_SUCCESSFUL;
}                                 

/**
 * @brief lin_app_offMaster : Turns off LIN master application.
 *                            Disables interrupts for UART and GPTM timer.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure
 * @param timer       : GPTM configuration for timeout handling
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_offMaster(uart_Config* terminal_uart, uart_Config* master_uart, lin_FrameStructure* lin_frame,
                                      gptm_Config* timer)
{
    /* Initialising timer */
    timer->GPTM_INTERRUPT = GPTM_INTERRUPT_DISABLE;
    gptm_initTimer(timer);

    /* Initialising terminal UART */
    terminal_uart->UART_INTERRUPT = UART_INTERRUPT_DISABLE;
    uart_init(terminal_uart);
    

    /* Initialising Master UART */
    master_uart->UART_INTERRUPT = UART_INTERRUPT_DISABLE;
    uart_init(master_uart);
    
    /* Initialising LIN */
    lin_frame->PID = SLAVE_PID;
    lin_frame->CHECKSUM_TYPE = LIN_CHECKSUM_ENHANCED;
    if (lin_init(master_uart, LIN_DATA_BAUD_RATE) != LIN_SUCCESS) {
        return LIN_MASTER_OFF_UNSUCCESSFUL;
    }
    uart_sendString(terminal_uart, "\r\nBYE BYE! \0");
    Lin_master_state = LIN_MASTER_IDLE;
    return LIN_MASTER_OFF_SUCCESSFUL;
}

/**
 * @brief lin_app_sendCommandFrame : Sends a LIN command frame from master
 *                                  to slave and updates master state.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame to be transmitted
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_sendCommandFrame(uart_Config* uart, lin_FrameStructure* lin_frame)
{
    if (lin_sendFrame(uart, lin_frame) == LIN_SUCCESS) {
        Lin_RxIndex = 0;
        /* Clear/refresh receive buffer */
        for (uint32_t i = 0; i < LIN_RX_BUFFER_SIZE; i++) {
            Lin_RxBuffer[i] = '\0';
        }
        Lin_master_state = LIN_MASTER_WAIT_FOR_ACK;
        return LIN_SEND_COMMAND_SUCCESSFUL;
    }
    Lin_master_state = LIN_MASTER_ERR_RECEIVED;
    return LIN_SEND_COMMAND_UNSUCCESSFUL;
}

/**
 * @brief lin_app_waitForAck : Arms and starts ACK timeout timer
 *                             and transitions master to waiting state.
 *
 * @param timer     : GPTM configuration
 * @param new_ticks : New timeout value in ticks (unused)
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_waitForAck(gptm_Config* timer, uint32_t new_ticks)
{
    if (gptm_reloadTimer(timer, gptm_convertSecToTicks(ACK_TIMEOUT)) != GPTM_SUCCESS) {
        return LIN_ACK_INIT_TIMER_UNSUCCESSFUL;
    }
    if (gptm_startTimer(timer) != GPTM_SUCCESS) {
        return LIN_ACK_START_TIMER_UNSUCCESSFUL;
    }
    Lin_master_state = LIN_MASTER_WAITING;
}

/**
 * @brief lin_app_getResponseFrame : Interprets received LIN response data
 *                                  and updates master state based on ACK/ERR.
 *
 * @param lin_frame : LIN frame containing response data
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_getResponseFrame(lin_FrameStructure* lin_frame)
{
    // Recieve frame and then decide if ACK is recived or not
    if (lin_receiveFrame(lin_frame) == LIN_RECEIVE_SUCCESS) {

        if (is_same(lin_frame->DATA, "ERR", RESPONSE_SIZE)) {
        Lin_master_state = LIN_MASTER_ERR_RECEIVED;
        }
        else if (is_same(lin_frame->DATA, "ACK", RESPONSE_SIZE)) {
            Lin_master_state = LIN_MASTER_ACK_RECEIVED;
        }
        else if (is_same(lin_frame->DATA, "OFF", RESPONSE_SIZE)) {
            Lin_master_state = LIN_MASTER_OFF_RECEIVED;
        }
        return LIN_GET_RESPONSE_SUCCESSFUL; 
    }
    else {
        return LIN_GET_RESPONSE_UNSUCCESSFUL;
    }
    
}

/**
 * @brief lin_app_ackReceived : Handles ACK reception by stopping timeout timer,
 *                              updating state, and notifying user.
 *
 * @param uart_config : UART configuration
 * @param timer       : GPTM timer used for ACK timeout
 * @param lin_frame   : LIN frame structure (unused)
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_ackReceived(uart_Config* uart_config, gptm_Config* timer)
{
    /* Stop the timer and move to next state */
    gptm_stopTimer(timer);

    Lin_master_state = LIN_MASTER_INIT;

    /* Displayes that ack received */
    uart_sendString(uart_config, "\r\nAck recieved! ;o;\0");
    return LIN_GET_ACK_SUCCESSFUL;
}

/**
 * @brief lin_app_errReceived : Handles ERR reception by resetting master state
 *                              and notifying user.
 *
 * @param uart_config : UART configuration
 * @param lin_frame   : LIN frame structure (unused)
 *
 * @return lin_app_ErrorType
 */
lin_app_ErrorType lin_app_errReceived(uart_Config* uart_config)
{
    Lin_master_state = LIN_MASTER_INIT;
    uart_sendString(uart_config, "\r\nSomething bad happened ;-;\0");
}
#endif
