/***************************************************************************************************
* FILENAME : gpio.h
* DESCRIPTION : Contains function prototypes and macros for GPIO driver.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

#ifndef __GPIO_H_
#define __GPIO_H_

/*************************************** Header Inclusion *****************************************/
#include "types.h"

/****************************************** Macros *********************************************/
/* Clock configuration  */
#define RCGCGPIO_CLK_EN   (*((uint32_t *) 0x400FE608))

/* AHB bus configuration */
#define GPIOHBCTL  (*((uint32_t *) 0x400FE06C))

/* Ports base addresses */
#define PORT_A_AHB_BASE  0x40058000
#define PORT_B_AHB_BASE  0x40059000
#define PORT_C_AHB_BASE  0x4005A000
#define PORT_D_AHB_BASE  0x4005B000
#define PORT_E_AHB_BASE  0x4005C000
#define PORT_F_AHB_BASE  0x4005D000

/* Defining ports */
#define GPIOA_AHB  ((gpio_Regs *)PORT_A_AHB_BASE)
#define GPIOB_AHB  ((gpio_Regs *)PORT_B_AHB_BASE)
#define GPIOC_AHB  ((gpio_Regs *)PORT_C_AHB_BASE)
#define GPIOD_AHB  ((gpio_Regs *)PORT_D_AHB_BASE)
#define GPIOE_AHB  ((gpio_Regs *)PORT_E_AHB_BASE)
#define GPIOF_AHB  ((gpio_Regs *)PORT_F_AHB_BASE)

#define UNLOCK  0x4C4F434B
#define LED_OFF 0

/* PCTL configuration */
#define PCTL_PER_PIN_MASK 0xFU

/****************************************** Enumerations ******************************************/

typedef enum {
    /* Advanced High-Performance Bus */
    PORT_A_AHB = 0,
    PORT_B_AHB,
    PORT_C_AHB,
    PORT_D_AHB,
    PORT_E_AHB,
    PORT_F_AHB
} gpio_Port;

typedef enum {
    PIN0 = 0x01,
    PIN1 = 0x02,
    PIN2 = 0x04,
    PIN3 = 0x08,
    PIN4 = 0x10,
    PIN5 = 0x20,
    PIN6 = 0x40,
    PIN7 = 0x80
} gpio_Pin;

typedef enum {
    DIGITAL_OUTPUT = 0,
    DIGITAL_INPUT,
} gpio_Mode;

typedef enum {
    GPIO_INTERRUPT_DISABLE = 0,
    GPIO_INTERRUPT_ENABLE
}gpio_InterruptEnable;

typedef enum {
    NONE = 0,
    UART_ENABLE,
} gpio_AlternateFunction;

typedef enum {
    GPIO_SUCCESS = 0,
    GPIO_ERROR_INVALID_PORT,
    GPIO_ERROR_INVALID_PIN,
    GPIO_ERROR_INVALID_MODE,
    GPIO_ERROR_PIN_LOCKED,
    GPIO_INVALID_CONFIGURATION,
    __GPIO_RESERV_START = 6,
    __GPIO_RESERV_END = 15
} gpio_Error; 

/******************************************* Data Types *******************************************/

typedef struct gpio_Regs {
    /* Data register */
    volatile uint32_t GPIODATA[256]; 
    /* Data direction register */
    volatile uint32_t GPIODIR;
    volatile uint32_t GPIOIS;
    volatile uint32_t GPIOIBE;
    volatile uint32_t GPIOIEV;
    volatile uint32_t GPIOIM;
    volatile uint32_t GPIORIS;
    volatile uint32_t GPIOMIS;
    volatile uint32_t GPIOICR;
    volatile uint32_t GPIOAFSEL;
    uint32_t _reserved1[55];
    volatile uint32_t GPIODR2R;
    volatile uint32_t GPIODR4R;
    volatile uint32_t GPIODR8R;
    volatile uint32_t GPIOODR;
    volatile uint32_t GPIOPUR;
    volatile uint32_t GPIOPDR;
    volatile uint32_t GPIOSLR;
    volatile uint32_t GPIODEN;
    volatile uint32_t GPIOLOCK;
    volatile uint32_t GPIOCR;
    volatile uint32_t GPIOAMSEL;
    volatile uint32_t GPIOPCTL; 
    volatile uint32_t GPIOADCCTL;
    volatile uint32_t GPIODMACTL;
    volatile uint32_t GPIOPeriphID4;
    volatile uint32_t GPIOPeriphID5;
    volatile uint32_t GPIOPeriphID6;
    volatile uint32_t GPIOPeriphID7;
    volatile uint32_t GPIOPeriphID0;
    volatile uint32_t GPIOPeriphID1;
    volatile uint32_t GPIOPeriphID2;
    volatile uint32_t GPIOPeriphID3;
    volatile uint32_t GPIOPCellID0;
    volatile uint32_t GPIOPCellID1;
    volatile uint32_t GPIOPCellID2;
    volatile uint32_t GPIOPCellID3;
} gpio_Regs;

typedef struct gpio_Config {
    gpio_Port PORT;
    gpio_Mode MODE;
    gpio_Pin PIN;
    gpio_AlternateFunction ALT_FUNC;
    gpio_InterruptEnable GPIO_INTERRUPT;
    uint32_t PCTL_VAL;
} gpio_Config;

/*************************************** Function Prototypes **************************************/
gpio_Error gpio_initGpioPort(gpio_Config* const config);
gpio_Error gpio_digitalToggle(gpio_Config* const config);
gpio_Error gpio_digitalWrite(gpio_Config* const config, uint8_t pinState);
gpio_Error gpio_digitalRead(gpio_Config* const config, uint8_t* pinState);
void gpio_interruptCallback(gpio_Config* const config);

#endif /*__GPIO_H_*/