/***************************************************************************************************
* FILENAME : uart.c
* DESCRIPTION : Functions (APIs) for UART initialization, UART send character, UART send string
*               (assuming it always ends with ‘\0’), UART receive character, UART Rx (receive)
*               interrupt callback.
*
* NOTES : This Uart driver only handles UART 0 and UART 1.
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/uart.h"
#include "../inc/gpio.h"
#include "../inc/nvic.h"
#include "../inc/gptm.h"

/*************************************** Global Data ********************************************/
extern volatile uint8_t data;

/************************************* Function Implementations ***********************************/

/**
 * @brief uart_getBase : Returns base memory address of the UART selected
 * 
 * @param uart_number : UART selected by the user
 * 
 * @return uart_Regs* 
 */
uart_Regs* uart_getBase(uart_Number uart_number)
{
    switch (uart_number)
    {
        case UART_0:
            return UART0;
        case UART_1:
            return UART1;
        default:
            return 0;
    }
}

/**
 * @brief uart_enableClock : Enables clock for the UART selected
 * 
 * @param uart_number : UART selected by the user
 * 
 * @return 
 */
static inline void uart_enableClock(uart_Number uart_number)
{
    if (uart_number == UART_0) {
        RCGCUART |= RCGC_UART0_EN_CLK;
    }
    else if (uart_number == UART_1) {
        RCGCUART |= RCGC_UART1_EN_CLK;
    }
}

/**
 * @brief uart_init : Initialises UART specified by the user
 * 
 * @param config : Configuration specified by the user
 * 
 * @return uart_Error
 */
uart_Error uart_init(uart_Config* config)
{
    /* Validate UART */
    uart_Regs *uart = uart_getBase(config->UART_NUM);
    if (!uart) return UART_NOT_CONFIGURED;

    /* Validate configuration */
    /* UART selected number validation */
    if (config->UART_NUM < UART_0) {
        return UART_INVALID_UART_NUMBER;
    }
    /* This driver supports only UART 0 and UART 1.
       But can support rest in future. */
    if (config->UART_NUM > UART_1) {
        return UART_UNSUPPORTED_UART_NUMBER;
    }
    /* Baud rate validation */
    if (config->BAUD_RATE < MINIMUM_BAUD_RATE || config->BAUD_RATE > MAXIMUM_BAUD_RATE) {
        return UART_UNSUPPORTED_BAUD_RATE;
    }
    /* Word length validation */
    if (config->WORD_LENGTH < WORDLENGTH_5_BITS || config->WORD_LENGTH > WORDLENGTH_8_BITS) {
        return UART_INVALID_WORDLENGTH;
    }
    /* Stop bits validation */
    if (config->STOP_BITS < STOPBITS_1 || config->STOP_BITS > STOPBITS_2) {
        return UART_INVALID_STOPBITS;
    }

    /* Enable clock */
    uart_enableClock(config->UART_NUM);

    /* Respective gpio pin configuration */
    gpio_Config uart_pin;

    if (config->UART_NUM == UART_0) {
        /* Mentoined on page number 925:
           Table 14-1. UART Signals (144LQFP) */
        /* UART0 on PORTA PIN0 (Rx) and PIN1 (Tx) */
        /* RX Pin */
        uart_pin.PORT = PORT_A_AHB;
        uart_pin.PIN = PIN0;
        uart_pin.MODE = DIGITAL_INPUT;
        uart_pin.ALT_FUNC = UART_ENABLE;
        uart_pin.PCTL_VAL = UART0_PIN_MUX;
        uart_pin.GPIO_INTERRUPT = GPIO_INTERRUPT_DISABLE;
        gpio_initGpioPort(&uart_pin);

        /* TX Pin */
        uart_pin.PIN = PIN1;
        uart_pin.MODE = DIGITAL_OUTPUT;
        uart_pin.ALT_FUNC = UART_ENABLE;
        uart_pin.PCTL_VAL = UART0_PIN_MUX;
        gpio_initGpioPort(&uart_pin);
    }
    else if (config->UART_NUM == UART_1) {
        /* UART1 on PORTB PIN0 (Rx) and PIN1 (Tx) */
        /* RX Pin */
        uart_pin.PORT = PORT_C_AHB;
        uart_pin.PIN = PIN4;
        uart_pin.MODE = DIGITAL_INPUT;
        uart_pin.ALT_FUNC = UART_ENABLE;
        uart_pin.PCTL_VAL = UART1_PIN_MUX;
        uart_pin.GPIO_INTERRUPT = GPIO_INTERRUPT_DISABLE;
        gpio_initGpioPort(&uart_pin);

        /* TX Pin */
        uart_pin.PIN = PIN5;
        uart_pin.MODE = DIGITAL_OUTPUT;
        uart_pin.ALT_FUNC = UART_ENABLE;
        uart_pin.PCTL_VAL = UART1_PIN_MUX;
        gpio_initGpioPort(&uart_pin);
    }

    /* Disable UART before configuration */
    uart->UARTCTL &= ~UART_CTL_UARTEN;

    /* Baud rate calculation */
    /* Mentioned on page number 934:
       14.4 Initialization and Configuration */
    uart->UARTIBRD = SYSTEM_CLOCK / (UART_OVERSAMPLING_RATE * config->BAUD_RATE);
    uart->UARTFBRD = ((SYSTEM_CLOCK % (UART_OVERSAMPLING_RATE * config->BAUD_RATE)) *
                     UART_FRACTIONAL_DIVISOR_SCALE + (config->BAUD_RATE / UART_ROUNDING_DIVISOR))
                     / config->BAUD_RATE;

    /* Line control */
    uint32_t lcrh_val = 0;

    /* Word length */
    switch (config->WORD_LENGTH)
    {
        case WORDLENGTH_5_BITS:
            lcrh_val |= UART_LCRH_WLEN_5;
            break;
        case WORDLENGTH_6_BITS:
            lcrh_val |= UART_LCRH_WLEN_6;
            break;
        case WORDLENGTH_7_BITS:
            lcrh_val |= UART_LCRH_WLEN_7;
            break;
        case WORDLENGTH_8_BITS:
            lcrh_val |= UART_LCRH_WLEN_8;
            break;
        default:
         return UART_INVALID_WORDLENGTH;
    }

    /* Stop bits */
    if (config->STOP_BITS == STOPBITS_2) {
        lcrh_val |= UART_LCRH_STP2;
    }

    /* Parity */
    if (config->PARITY != PARITY_NONE) {
        lcrh_val |= UART_LCRH_PEN;
        if (config->PARITY == PARITY_EVEN) {
            lcrh_val |= UART_LCRH_EPS;
        }
    }

    /* Write onto lcrh at once */
    uart->UARTLCRH = lcrh_val; 

    /* Set UART clock source to system clock*/
    /* Mentioned on page number 971:
       Register 18: UART Clock Configuration (UARTCC), offset 0xFC8
       Value        Description
        0x0         System clock (based on clock source and divisor factor)
        0x1-0x4     reserved
        0x5         PIOSC
        0x5-0xF     Reserved */
    uart->UARTCC = UART_CC_CS_SYSCLK;

    /* Enable RX, TX and UART */
    uart->UARTCTL |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);

    /* Interrupt enable */
    if (config->UART_INTERRUPT == UART_INTERRUPT_ENABLE ) {
        /* Clear any prior interrupt */
        uart->UARTICR |= UART_ICR_RXIC;

        /* Enable receive interrupt mask */
        uart->UARTIM |= UART_IM_RXIM;

        /* Enable nvic for the selected UART */
        switch(config->UART_NUM)
        {
            case UART_0:
                NVIC_EN0 |= UART0_IRQ;
                break;
            case UART_1:
                NVIC_EN0 |= UART1_IRQ;
                break;
            default:
                return UART_UNSUPPORTED_UART_NUMBER;
        }
    }

    return UART_SUCCESS;
}

/**
 * @brief uart_sendChar : Sends a character via transmitter
 * 
 * @param config : Configuration specified by the user
 * @param data : Character data to be sent
 * 
 * @return uart_Error
 */
uart_Error uart_sendChar(uart_Config* config, uint8_t data)
{
    uart_Regs *uart = uart_getBase(config->UART_NUM);
    if (!uart) {
        return UART_NOT_CONFIGURED;
    }

    /* Wait until TX has space */
    while (uart->UARTFR & UART_FR_TXFF);

    uart->UARTDR = data;

    return UART_SUCCESS;
}

/**
 * @brief uart_sendString : Sends a string via transmitter
 * 
 * @param config : Configuration specified by the user
 * @param data : String data to be sent
 * 
 * @return uart_Error
 */
uart_Error uart_sendString(uart_Config* config, uint8_t* data)
{
    /* Keep sending until null terminator */
    while (*data != '\0')
    {
        uart_sendChar(config, *data);
        data++;
    }

    return UART_SUCCESS;
}

/**
 * @brief uart_receiveChar : Recieves a character via reciever
 * 
 * @param config : Configuration specified by the user
 * @param data : Variable to hold character data
 * 
 * @return uart_Error
 */
uart_Error uart_receiveChar(uart_Config* config, uint8_t* data)
{
    uart_Regs *uart = uart_getBase(config->UART_NUM);
    if (!uart) return UART_NOT_CONFIGURED;

    /* Wait until RX has data */
    while (uart->UARTFR & UART_FR_RXFE);

    *data = (uint8_t)(uart->UARTDR & MAX_8_BIT);

    return UART_SUCCESS;
}

/**
 * @brief uart0_interruptCallback : Interrupt callback routine for UART 0.
 * 
 * @return
 */
void uart0_interruptCallback(void)
{
    while (!(UART0->UARTFR & UART_FR_RXFE)) {
    data = (uint8_t)(UART0->UARTDR & MAX_8_BIT);
    /* In case of 'Enter' enter new line instead of carriage return */
    if (data == '\r') {
        UART0->UARTDR = '\r';
        UART0->UARTDR = '\n';
    }
    else {
        UART0->UARTDR = data;
    }
    }
    /* Clear interrupt */
    UART0->UARTICR |= UART_ICR_RXIC;
}

/**
 * @brief uart1_interruptCallback : Interrupt callback routine for UART 1.
 * 
 * @return
 */
void uart1_interruptCallback(void)
{
    uint8_t rx;
    while (!(UART1->UARTFR & UART_FR_RXFE)) {
    rx = (uint8_t)(UART1->UARTDR & MAX_8_BIT);
    /* In case of 'Enter' enter new line instead of carriage return */
    if (rx == '\r') {
        UART1->UARTDR = '\r';
        UART1->UARTDR = '\n';
    }
    else {
        UART1->UARTDR = rx;
    }
    }
    /* Clear interrupt */
    UART1->UARTICR |= UART_ICR_RXIC;
}