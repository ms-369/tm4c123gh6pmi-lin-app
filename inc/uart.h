/***************************************************************************************************
* FILENAME : uart.h
* DESCRIPTION : Defines macros, enums, structs and declares functions for UART driver.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

#ifndef _UART_H_
#define _UART_H_

/*************************************** Header Inclusion *****************************************/
#include "gpio.h"

/********************************************* Macros *********************************************/
/* UART clock configuration — UARTCC */
/* System clock */
#define UART_CC_CS_SYSCLK       0x0U
/* Precision internal oscillator */
#define UART_CC_CS_PIOSC        0x5U

/* Enabling clock for UART */
#define RCGCUART  (*((uint32_t *)0x400FE618))
/* Bit masks for enabling respective UART */
#define RCGC_UART0_EN_CLK (1U << 0)
#define RCGC_UART1_EN_CLK (1U << 1)

/* UARTs base addresses */
#define UART0_BASE     0x4000C000
#define UART1_BASE     0x4000D000

/* Defining UARTs */
#define UART0       ((uart_Regs *)UART0_BASE)
#define UART1       ((uart_Regs *)UART1_BASE)

/* Configurations */
/* UART enable — UARTCTL */
/* UART enable */
#define UART_CTL_UARTEN         (1U << 0)
/* Transmit enable */
#define UART_CTL_TXE            (1U << 8)
/* Receive enable */
#define UART_CTL_RXE            (1U << 9)

/* UART line control — UARTLCRH */
/* Parity enable */
#define UART_LCRH_PEN           (1U << 1)
/* Even parity select */
#define UART_LCRH_EPS           (1U << 2)
/* Two stop bits select */
#define UART_LCRH_STP2          (1U << 3)
/* Word length 5 bits */
#define UART_LCRH_WLEN_5        (0U << 5)
/* Word length 6 bits */
#define UART_LCRH_WLEN_6        (1U << 5)
/* Word length 7 bits */
#define UART_LCRH_WLEN_7        (2U << 5)
/* Word length 8 bits */
#define UART_LCRH_WLEN_8        (3U << 5)
/* Send break */
#define UART_LCRH_BRK           (1U << 0)

/* UART flag register — UARTFR */
/* Receive FIFO empty */
#define UART_FR_RXFE            (1U << 4)
/* Transmit FIFO full */
#define UART_FR_TXFF            (1U << 5)

/* UART interrupt mask — UARTIM */
/* Receive interrupt mask */
#define UART_IM_RXIM            (1U << 4)

/* UART interrupt clear — UARTICR */
/* Clear receive interrupt */
#define UART_ICR_RXIC           (1U << 4)

#define UART0_PIN_MUX                 0x1
#define UART1_PIN_MUX                 0x2
#define UART_SYSTEM_CLC_EN            0x0
#define UART_OVERSAMPLING_RATE        16U
#define UART_FRACTIONAL_DIVISOR_SCALE 64U
#define UART_ROUNDING_DIVISOR         2U

#define MINIMUM_BAUD_RATE             0U
#define MAXIMUM_BAUD_RATE             1000000U

/****************************************** Enumerations ******************************************/

typedef enum {
    UART_0,
    UART_1,
} uart_Number;

typedef enum {
    WORDLENGTH_5_BITS,
    WORDLENGTH_6_BITS,
    WORDLENGTH_7_BITS,
    WORDLENGTH_8_BITS
} uart_WordLength;

typedef enum {
    STOPBITS_1 = 1,
    STOPBITS_2 = 2
} uart_StopBits;

typedef enum {
    UART_INTERRUPT_DISABLE = 0,
    UART_INTERRUPT_ENABLE
}uart_InterruptEnable;

typedef enum {
    PARITY_NONE = 0,
    PARITY_EVEN,
    PARITY_ODD
} uart_Parity;

typedef enum {
    UART_SUCCESS = 32,
    UART_NOT_CONFIGURED,
    UART_INVALID_UART_NUMBER,
    UART_UNSUPPORTED_UART_NUMBER,
    UART_UNSUPPORTED_BAUD_RATE,
    UART_INVALID_WORDLENGTH,
    UART_INVALID_STOPBITS,

    __UART_RESERVED_START = 39,
    __UART_RESERVED_END = 47
} uart_Error;

/******************************************* Data Types *******************************************/

/* UART registers */
typedef struct uart_Regs {
    /* 0x000 Data register */
    volatile uint32_t UARTDR;
    /* 0x004 Receive status / error clear */
    volatile uint32_t UARTRSR_ECR;
    uint32_t _reserved1[4];
    /* 0x018 Flag register */
    volatile uint32_t UARTFR;
    uint32_t _reserved2;
    /* 0x020 IrDA low-power register */
    volatile uint32_t UARTILPR;
    /* 0x024 Integer baud-rate divisor */
    volatile uint32_t UARTIBRD;
    /* 0x028 Fractional baud-rate divisor */
    volatile uint32_t UARTFBRD;
    /* 0x02C Line control */
    volatile uint32_t UARTLCRH;
    /* 0x030 Control register */
    volatile uint32_t UARTCTL;
    /* 0x034 Interrupt FIFO level select */
    volatile uint32_t UARTIFLS;
    /* 0x038 Interrupt mask */
    volatile uint32_t UARTIM;
    /* 0x03C Raw interrupt status */
    volatile uint32_t UARTRIS;
    /* 0x040 Masked interrupt status */
    volatile uint32_t UARTMIS;
    /* 0x044 Interrupt clear */
    volatile uint32_t UARTICR;
    /* 0x048 DMA control */
    volatile uint32_t UARTDMACTL;
    uint32_t _reserved3[19];
    /* 0x0A4 9-Bit self address */
    volatile uint32_t UART9BITADDR;
    /* 0x0A8 9-Bit self address mask */
    volatile uint32_t UART9BITAMASK;
    uint32_t _reserved4[203];
    /* 0xFC0 Peripheral properties */
    volatile uint32_t UARTPP;
    /* 0xFC8 Clock configuration */
    volatile uint32_t UARTCC;
    uint32_t _reserved5[7];
    /* 0xFD0 Peripheral ID 4 */
    volatile uint32_t UARTPeriphID4;
    /* 0xFD4 Peripheral ID 5 */
    volatile uint32_t UARTPeriphID5;
    /* 0xFD8 Peripheral ID 6 */
    volatile uint32_t UARTPeriphID6;
    /* 0xFDC Peripheral ID 7 */
    volatile uint32_t UARTPeriphID7;
    /* 0xFE0 Peripheral ID 0 */
    volatile uint32_t UARTPeriphID0;
    /* 0xFE4 Peripheral ID 1 */
    volatile uint32_t UARTPeriphID1;
    /* 0xFE8 Peripheral ID 2 */
    volatile uint32_t UARTPeriphID2;
    /* 0xFEC Peripheral ID 3 */
    volatile uint32_t UARTPeriphID3;
    /* 0xFF0 PrimeCell ID 0 */
    volatile uint32_t UARTPCellID0;
    /* 0xFF4 PrimeCell ID 1 */
    volatile uint32_t UARTPCellID1;
    /* 0xFF8 PrimeCell ID 2 */
    volatile uint32_t UARTPCellID2;
    /* 0xFFC PrimeCell ID 3 */
    volatile uint32_t UARTPCellID3;
} uart_Regs;

typedef struct {
    uart_Number UART_NUM;
    uint32_t BAUD_RATE;               
    uart_WordLength WORD_LENGTH;
    uart_StopBits STOP_BITS;
    uart_Parity PARITY;
    uart_InterruptEnable UART_INTERRUPT; 
} uart_Config;

/*************************************** Function Prototypes **************************************/
uart_Regs* uart_getBase(uart_Number uart_number);
static inline void uart_enableClock(uart_Number uart_number);
uart_Error uart_init(uart_Config* config);
uart_Error uart_sendChar(uart_Config* config, uint8_t data);
uart_Error uart_sendString(uart_Config* config, uint8_t* data);
uart_Error uart_receiveChar(uart_Config* config, uint8_t* data);
void uart0_interruptCallback(void);
void uart1_interruptCallback(void);

#endif /*_UART_H_*/