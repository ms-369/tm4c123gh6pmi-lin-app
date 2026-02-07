/*************************************************************************************************
* FILENAME : lin.c
* DESCRIPTION : LIN protocol implementation over UART1 following LIN 2.1 spec.
*               Supports frame init, transmission, reception with break, sync, PID, data,
*               checksum, and verification.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
*************************************************************************************************/

/*************************************** Header Inclusion ***************************************/
#include "../inc/lin.h"
#include "../inc/uart.h"
#include "../inc/gptm.h"
#include "../inc/lin_app.h"

/*************************************** Global Data ********************************************/
#ifdef LIN_MASTER
extern volatile lin_Masterstate Lin_master_state;
#endif
#ifdef LIN_SLAVE
extern volatile lin_Slavestate Lin_slave_state;
uint8_t Lin_ResponseBuffer[LIN_RX_BUFFER_SIZE];
#endif
uint8_t Lin_RxBuffer[LIN_RX_BUFFER_SIZE];
uint8_t Lin_RxIndex = 0U;

/************************************  Function Implementations *********************************/

/**
 * @brief Initializes UART for LIN communication with the specified baud rate.
 *
 * @param uart_config Pointer to UART configuration structure.
 * @param baudRate Desired LIN baud rate.
 *
 * @return lin_ErrorType Returns LIN_SUCCESS on success, LIN_UART_INIT_FAILURE otherwise.
 */
lin_ErrorType lin_init(uart_Config* uart_config, uint32_t baudRate)
{
    uart_config->BAUD_RATE = baudRate;

    if (uart_init(uart_config) != UART_SUCCESS)
    {
        return LIN_UART_INIT_FAILURE;
    }

    Lin_RxIndex = 0;
    /* Clear/refresh receive buffer */
    for (uint32_t i = 0; i < LIN_RX_BUFFER_SIZE; i++) {
        Lin_RxBuffer[i] = '\0';
    }
    #ifdef LIN_SLAVE
    /* Clear/refresh receive and response buffers */
    for (uint32_t i = 0; i < LIN_RX_BUFFER_SIZE; i++) {
        Lin_ResponseBuffer[i] = '\0';
        Lin_RxBuffer[i] = '\0';
    }
    #endif
    return LIN_SUCCESS;
}

/**
 * @brief Calculates the LIN frame checksum according to
 *        classic or enhanced checksum model.
 *        In enhanced mode, PID is included in the checksum.
 *
 * @param lin_frame Pointer to LIN frame structure.
 *
 * @return uint8_t Calculated checksum byte.
 */
uint8_t lin_calculateChecksum(lin_FrameStructure* lin_frame)
{
    uint16_t sum = 0U;
    uint8_t i = 0;

    /* Include PID in checksum for enhanced model */
    if (lin_frame->CHECKSUM_TYPE == LIN_CHECKSUM_ENHANCED) {
        sum += lin_frame->PID;
    }

    #ifdef LIN_SLAVE
    if (Lin_slave_state == LIN_SLAVE_SEND_ACK ||
         Lin_slave_state == LIN_SLAVE_SEND_OFF) {
        for (; i < LIN_RX_BUFFER_SIZE; i++) {
            sum += Lin_ResponseBuffer[i];
        }
    }
    else {
        for (; i < LIN_MAX_DATA_LENGTH; i++) {
            sum += lin_frame->DATA[i];
        }
    }
    #endif

    #ifdef LIN_MASTER
    for (; i < LIN_MAX_DATA_LENGTH; i++) {
        sum += lin_frame->DATA[i];
    }
    #endif
    
    /* Fold carry */
    while (sum > 0xFFU)
    {
        sum = (sum & 0xFFU) + (sum >> 8U);
    }

    return (uint8_t)(~sum);
}

/**
 * @brief Returns expected data length based on
 *        the LIN frame ID (lower 6 bits of PID).
 *        This follows a fixed grouping scheme.
 *
 * @param pid Protected identifier byte.
 *
 * @return uint8_t Expected data length.
 */
static uint8_t lin_getDataLengthFromPID(uint8_t pid)
{
    uint8_t frameID = pid & LIN_PID_MAX_VALUE;

    /* IDs 0-31 => 2 bytes data */
    if (frameID <= PID_RANGE_0_TO_31) {
        return LIN_RCVR_GROUP_1_DATA_LENGTH;
    } 
    /* IDs 32-47 => 4 bytes data */
    else if (frameID <= PID_RANGE_32_TO_47) {
        return LIN_RCVR_GROUP_2_DATA_LENGTH;
    } 
    /* IDs 48-63 => 8 bytes data */
    else {
        return LIN_RCVR_GROUP_3_DATA_LENGTH;
    }
}

/**
 * @brief Generates a LIN break field by transmitting a 0x00 byte
 *        at a reduced baud rate to emulate a dominant low
 *        for at least 13 bit times.
 *
 * @param uart Pointer to UART configuration.
 *
 * @return void
 */
void lin_sendBreak(uart_Config* uart)
{
    lin_sendByte(uart, 0);
}

/**
 * @brief Sends a single byte over UART.
 *
 * @param uart Pointer to UART configuration.
 * @param byte Byte to be transmitted.
 *
 * @return lin_ErrorType Currently always successful.
 */
static lin_ErrorType lin_sendByte(uart_Config* uart, uint8_t byte)
{
    uart_sendChar(uart, byte);
}

/**
 * @brief Transmits a complete LIN frame including:
 *        Break field, Sync byte, PID, Data bytes, and Checksum.
 *
 * @param uart Pointer to UART configuration.
 * @param lin_frame Pointer to LIN frame structure.
 *
 * @return lin_ErrorType Returns LIN_SUCCESS on success,
 *                       LIN_INVALID_DATA_LENGTH or LIN_UART_INIT_FAILURE on failure.
 */
lin_ErrorType lin_sendFrame(uart_Config* uart, lin_FrameStructure* lin_frame)
{
    uint8_t checksum;
    uint8_t i = 0;

    if (lin_getDataLengthFromPID(lin_frame->PID) < 0 || 
        lin_getDataLengthFromPID(lin_frame->PID) > LIN_MAX_DATA_LENGTH)
    {
        return LIN_INVALID_DATA_LENGTH;
    }

    uart->BAUD_RATE = BREAK_BAUD_RATE;

    if (uart_init(uart) != UART_SUCCESS)
    {
        return LIN_UART_INIT_FAILURE;
    }

    /* Send break field */
    lin_sendBreak(uart);

    uart->BAUD_RATE = LIN_DATA_BAUD_RATE;

    if (uart_init(uart) != UART_SUCCESS)
    {
        return LIN_UART_INIT_FAILURE;
    }

    /* Send sync byte (0x55) */
    lin_sendByte(uart, LIN_SYNC_BYTE);

    /* Send protected identifier (PID) */
    lin_sendByte(uart, lin_frame->PID);

    /* Send data bytes */
    for(i = 0; i < LIN_MAX_DATA_LENGTH; i++) {
        #ifdef LIN_MASTER
        if (Lin_master_state == LIN_MASTER_SEND_COMMAND) {
            lin_sendByte(uart, Lin_RxBuffer[i+1]);
            /* Copying for checksum calculation */
            lin_frame->DATA[i] = Lin_RxBuffer[i+1];
        }
        else {
            lin_sendByte(uart, lin_frame->DATA[i]);
        }
        #endif
        #ifdef LIN_SLAVE
        lin_sendByte(uart, Lin_ResponseBuffer[i]);
        #endif
    }

    /* Calculate and send checksum */
    checksum = lin_calculateChecksum(lin_frame);
    lin_sendByte(uart, checksum);

    return LIN_SUCCESS;
}

/**
 * @brief Validates the PID by checking whether the
 *        frame ID (lower 6 bits) falls within
 *        the supported ID range.
 *
 * @param pid Protected Identifier byte.
 *
 * @return uint8_t VALID_PID if valid, INVALID_PID otherwise.
 */
static uint8_t lin_validatePID(uint8_t pid)
{
    /* Mask to get the lower 6 bits (frame ID) */
    uint8_t id = pid & LIN_PID_MAX_VALUE;

    /* Check if frame ID is between 48 (0x30) and 63 (0x3F) */
    if (id > PID_RANGE_32_TO_47 && id <= PID_RANGE_48_TO_63) {
        return VALID_PID;
    } else {
        return INVALID_PID;
    }
}

/**
 * @brief UART RX handler for LIN communication.
 *        Receives one byte at a time and stores it
 *        into the LIN receive buffer based on the
 *        current master/slave state.
 *
 * @param uart_config Pointer to UART configuration.
 *
 * @return void
 */
void lin_receiveByte(uart_Config* uart_config)
{
    uint8_t data;
    static uint8_t frame_start = 0;
    uart_Regs *uart = uart_getBase(uart_config->UART_NUM);
    
    uart_receiveChar(uart_config,&data);

    if (data == LIN_SYNC_BYTE && Lin_RxIndex == LIN_IDX_SYNC) {
        return;
    }
    else if (data == LIN_BREAK_BYTE && Lin_RxIndex == LIN_MIN_DATA_LENGTH) {
        return;
    }
    
    #ifdef LIN_SLAVE
    if (data == SLAVE_PID) {
        frame_start = 1;
    }
    if (frame_start) {
        if (Lin_RxIndex < LIN_RX_BUFFER_SIZE) {
            Lin_RxBuffer[Lin_RxIndex] = data;
            Lin_RxIndex++;
        }
        if (Lin_RxIndex >= LIN_RX_BUFFER_SIZE) {
            Lin_slave_state = LIN_SLAVE_GET_COMMAND;
            Lin_RxIndex = 0;
            frame_start = 0;
        }
    }
    #endif

    #ifdef LIN_MASTER
    /* Master does not echo received bytes.
     * It only processes the slave response and updates its state machine */
    if (data == SLAVE_PID) {
        frame_start = 1;
    }
    if (frame_start) {
        if (Lin_RxIndex < LIN_RX_BUFFER_SIZE) {
            Lin_RxBuffer[Lin_RxIndex] = (uint8_t)data;
            Lin_RxIndex++;
        }
        if (Lin_RxIndex >= LIN_RX_BUFFER_SIZE) {
            Lin_master_state = LIN_MASTER_GET_RESPONSE;
            Lin_RxIndex = 0;
            frame_start = 0;
        }
    }
    else {
        if (Lin_RxIndex == LIN_MIN_DATA_LENGTH) {
            Lin_RxIndex = LIN_IDX_DATA_START;
        }
        if (data == '\r') {
            /* CHANGE STATE HERE TO SEND DATA TO THE SLAVE FROM MASTER */
            Lin_master_state = LIN_MASTER_SEND_COMMAND;
        }
        else if (Lin_RxIndex <= LIN_MAX_DATA_LENGTH ) {
            uart->UARTDR = (uint8_t)data;
            Lin_RxBuffer[Lin_RxIndex] = (uint8_t)data;
            Lin_RxIndex++;
        }
    }
    #endif

    /* Clear interrupt */
    uart->UARTICR |= UART_ICR_RXIC;
}

/**
 * @brief Processes a received LIN frame stored in the RX buffer.
 *        Verifies PID, extracts data bytes, and validates checksum.
 *        Updates master/slave state machines accordingly.
 *
 * @param lin_frame Pointer to LIN frame structure.
 *
 * @return lin_ErrorType Returns LIN_RECEIVE_SUCCESS on success,
 *                       LIN_INVALID_PID or LIN_INVALID_CHECKSUM on failure.
 */
lin_ErrorType lin_receiveFrame(lin_FrameStructure* lin_frame)
{
    /* When whole frame is received verify pid */
    if (lin_frame->PID != Lin_RxBuffer[LIN_IDX_PID]) {
        /* change state here to send error if slave is defined send err else send master's message */
        #ifdef LIN_MASTER
            Lin_master_state = LIN_MASTER_ERR_RECEIVED;
        #endif
        #ifdef LIN_SLAVE
            Lin_slave_state = LIN_SLAVE_SEND_ERR;
        #endif
        return LIN_INVALID_PID;
    }

    for (int i = 0; i < LIN_MAX_DATA_LENGTH; i++) {
        lin_frame->DATA[i] = Lin_RxBuffer[i + LIN_IDX_DATA_START];
    }

    if (lin_calculateChecksum(lin_frame) == Lin_RxBuffer[LIN_IDX_CHECKSUM_START]) {
        /* The slave should send an ack after frame validation */
        #ifdef LIN_SLAVE
            Lin_slave_state = LIN_SLAVE_EXECUTE_COMMAND;
        #endif
        return LIN_RECEIVE_SUCCESS;
    }
    else {
        #ifdef LIN_SLAVE
            Lin_slave_state = LIN_SLAVE_SEND_ERR;
        #endif
        #ifdef LIN_MASTER
            Lin_master_state = LIN_MASTER_ERR_RECEIVED;
        #endif
        return LIN_INVALID_CHECKSUM;
        /* change state here to send error if slave is defined send err else send master's message */
    }
}