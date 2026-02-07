/***********************************************************************************************
* FILENAME : lin.h
* DESCRIPTION : Contains data types, macros, and function prototypes for LIN driver
*               implemented over UART1.
*
* NOTES :
* AUTHOR : Mariyam Shahid
***********************************************************************************************/

#ifndef __LIN_H_
#define __LIN_H_

/*************************************** Header Inclusion *************************************/
#include "uart.h"
#include "types.h"

/******************************************** Macros ******************************************/

/* LIN Protocol */
#define LIN_BREAK_BYTE              (0x00U)
#define LIN_SYNC_BYTE               (0x55U)

/* LIN Data Length */
#define LIN_MAX_DATA_LENGTH               (8U)
#define LIN_MIN_DATA_LENGTH               (0U)
#define LIN_RCVR_GROUP_1_DATA_LENGTH      (2U)
#define LIN_RCVR_GROUP_2_DATA_LENGTH      (4U)
#define LIN_RCVR_GROUP_3_DATA_LENGTH      (8U)
#define LIN_IDLE_BYTE                     0xFF

/* LIN PID */
#define LIN_PID_MAX_VALUE           (0x3FU)
#define LIN_PID_MIN_VALUE           (0x00U)

/* Buffer size = PID(1) + max data(8) + checksum(1) */
#define LIN_RX_BUFFER_SIZE   (1U + LIN_MAX_DATA_LENGTH + 1U)

/* LIN Frame field indexes */
#define LIN_IDX_SYNC             0U
#define LIN_IDX_PID              0U
#define LIN_IDX_DATA_START       1U
#define LIN_IDX_CHECKSUM_START   9U


/* PID Range */
#define PID_RANGE_0_TO_31   31
#define PID_RANGE_32_TO_47  47
#define PID_RANGE_48_TO_63  63
#define VALID_PID           1
#define INVALID_PID         0


#define LIN_DATA_BAUD_RATE     9600
#define BREAK_BAUD_RATE    6676
#define IDLE_DATA          0xFF

/****************************************** Enumerations ******************************************/

typedef enum {
    LIN_SUCCESS = 48,
    LIN_SEND_SUCCESS,
    LIN_SYNC_UNSUCCESSFUL,
    LIN_NULL_POINTER,
    LIN_FRAME_NOT_INITIALISED,
    LIN_INVALID_PID,
    LIN_INVALID_DATA_LENGTH,
    LIN_INVALID_CHECKSUM,
    LIN_UART_INIT_FAILURE,
    LIN_BUSY,
    LIN_RECEIVE_SUCCESS,

    __LIN_RESERV_END = 63
} lin_ErrorType;

typedef enum {
    LIN_CHECKSUM_CLASSIC = 0,
    LIN_CHECKSUM_ENHANCED,
    LIN_CHECKSUM_INVALID
} lin_ChecksumModelType;

/******************************************* Data Types *******************************************/

typedef struct
{
    uint8_t  PID;
    uint8_t  DATA[LIN_MAX_DATA_LENGTH];
    lin_ChecksumModelType CHECKSUM_TYPE;
} lin_FrameStructure;

/*************************************** Function Prototypes **************************************/

lin_ErrorType lin_init(uart_Config* uart_config, uint32_t baudRate);
uint8_t lin_calculateChecksum(lin_FrameStructure*);
static uint8_t lin_getDataLengthFromPID(uint8_t pid);
void lin_sendBreak(uart_Config* uart);
static lin_ErrorType lin_sendByte(uart_Config* uart, uint8_t byte);
lin_ErrorType lin_sendFrame(uart_Config* uart_config, lin_FrameStructure* lin_frame);
static uint8_t lin_validatePID(uint8_t pid);
void lin_receiveByte(uart_Config* uart_config);
lin_ErrorType lin_receiveFrame(lin_FrameStructure*);

#endif /* __LIN_H_ */