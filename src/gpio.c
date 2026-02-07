/***************************************************************************************************
* FILENAME : gpio.c
* DESCRIPTION : Functions (APIs) for GPIO initialization, GPIO digital write (0/1), GPIO digital write toggle 
*               (if pin is set to 1, then set it to 0 and vice-versa) and GPIO digital read 
*
* NOTES : This Driver now supports configuration of PD7 and PF0.
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/gpio.h"
#include "../inc/nvic.h"
#include "../inc/state_machine.h"

/****************************************  Externs  ***********************************************/
extern volatile state_machine_States Current_state;

/************************************* Function Implementations ***********************************/

/**
 * @brief gpio_getPort : Returns memory address of the port selected
 * 
 * @param port : Port selected by the user
 * 
 * @return gpio_Regs* 
 */
static inline gpio_Regs* gpio_getPort(gpio_Port port) 
{
    switch(port)
    {
        case PORT_A_AHB:
            return GPIOA_AHB;
        case PORT_B_AHB:
            return GPIOB_AHB;
        case PORT_C_AHB:
            return GPIOC_AHB;
        case PORT_D_AHB:
            return GPIOD_AHB;
        case PORT_E_AHB:
            return GPIOE_AHB;
        case PORT_F_AHB:
            return GPIOF_AHB;
        default:
            return 0;
    }
}

static inline uint8_t gpio_pinIndex(gpio_Pin pin)
{
    switch(pin)
    {
        case PIN0:
            return 0;
        case PIN1:
            return 1;
        case PIN2:
            return 2;
        case PIN3:
            return 3;
        case PIN4:
            return 4;
        case PIN5:
            return 5;
        case PIN6:
            return 6;
        case PIN7:
            return 7;
        default:
            return 0;
    }
}

/**
 * @brief gpio_initGpioPort : Initialises GPIO port specified by the user
 * 
 * @param config : Configuration specified by the user
 * 
 * @return gpio_Error 
 */
gpio_Error gpio_initGpioPort(gpio_Config* const config) 
{

    if (!config) {
        return GPIO_INVALID_CONFIGURATION;
    }
    /* Port validation */
    gpio_Regs* port_ptr = gpio_getPort(config->PORT);
    if (!port_ptr) {
        return GPIO_ERROR_INVALID_PORT;
    }
    /* Pin validation
       As 8 pins combination can have a maximum value of 0xFF */
    if (config->PIN < PIN0 || config->PIN > 0xFF) {
        return GPIO_ERROR_INVALID_PIN;
    }
    /* Mode validation */
    if (config->MODE > DIGITAL_INPUT || config->MODE < DIGITAL_OUTPUT) {
        return GPIO_ERROR_INVALID_MODE;
    }

    /* Enable clock for the port */
    RCGCGPIO_CLK_EN |= (1 << config->PORT);
    /* Enable AHB for the port */
    GPIOHBCTL |= (1 << config->PORT);
    
    /* Mentioned in tm4c123gh6pm datasheet, at page number 659(Table 10-5),
       that port D's pin 7 and port F's pin 0 is locked.
       In order to use we have to first unlock them */
    if ((config->PORT == PORT_D_AHB && config->PIN & PIN7) ||
       (config->PORT == PORT_F_AHB && config->PIN & PIN0)) {
        
        /* Unlock the GPIO port 
           0x4C4F434B : Key given on page # 684*/
        port_ptr->GPIOLOCK = UNLOCK;   
        /* Commit the pins */
        port_ptr->GPIOCR |= config->PIN;
    }

    uint32_t mask = config->PIN;

    /* Alternate function configuration */
    if (config->ALT_FUNC == UART_ENABLE) {
        /* Disable analog */
        port_ptr->GPIOAMSEL &= ~config->PIN;
        /* Enable alternate function for selected pin
        GPIOAFSEL: 1 = pin controlled by peripheral */
        port_ptr->GPIOAFSEL |= config->PIN;
        /* Clear the 4-bit PCTL field corresponding to the pin
        Each pin uses 4 bits in GPIOPCTL */
        port_ptr->GPIOPCTL &= ~(PCTL_PER_PIN_MASK << (gpio_pinIndex(config->PIN) * 4U));
        /* UART0 = 0x1, UART1 = 0x2 */
        port_ptr->GPIOPCTL |= ((uint32_t)config->PCTL_VAL << (gpio_pinIndex(config->PIN) * 4U));
        port_ptr->GPIOPUR |= config->PIN;
    }
    /* Configure pin direction */
    if (config->MODE == DIGITAL_OUTPUT) {
        /* Set direction as output */
        port_ptr->GPIODIR |= mask;
    } 
    else if (config->MODE == DIGITAL_INPUT) {
        /* Set direction as input */
        port_ptr->GPIODIR &= ~mask;
    }
    /* Enable digital function */
    port_ptr->GPIODEN |= config->PIN;

    /* Interrupt configuration */
    if (config->GPIO_INTERRUPT == GPIO_INTERRUPT_ENABLE) {
        /* Interrupts are only valid for input pins */
        if (config->MODE != DIGITAL_INPUT) {
            return GPIO_ERROR_INVALID_MODE;
        }
        /* Mask interrupt during configuration */
        port_ptr->GPIOIM &= ~mask;
        /* Configure as edge-sensitive */
        port_ptr->GPIOIS &= ~mask;
        /* Disable both-edge triggering */
        port_ptr->GPIOIBE &= ~mask;
        /* Falling edge trigger (button press) */
        port_ptr->GPIOIEV &= ~mask;
        /* Clear any prior interrupt */
        port_ptr->GPIOICR |= mask;
        /* Unmask interrupt */
        port_ptr->GPIOIM |= mask;
        /* Enable interrupt on NVIC */
        switch(config->PORT)
        {
            case PORT_A_AHB:
                NVIC_EN0 |= GPIOA_IRQ;
                break;
            case PORT_B_AHB:
                NVIC_EN0 |= GPIOB_IRQ;
                break;
            case PORT_C_AHB:
                NVIC_EN0 |= GPIOC_IRQ;
                break;
            case PORT_D_AHB:
                NVIC_EN0 |= GPIOD_IRQ;
                break;
            case PORT_E_AHB:
                NVIC_EN0 |= GPIOE_IRQ;
                break;
            case PORT_F_AHB:
                NVIC_EN0 |= GPIOF_IRQ;
                break;
    }
    }

    return GPIO_SUCCESS;
}


/**
 * @brief gpio_digitalWrite : Writes to the GPIO port's pin specified by the user
 * 
 * @param config   : Configuration specified by the user
 * @param pinState : Pin to write on
 * 
 * @return gpio_Error 
 */
gpio_Error gpio_digitalWrite(gpio_Config* const config, uint8_t pinState)
{
    if (!config) {
        return GPIO_INVALID_CONFIGURATION;
    }
    /* Port validation */
    gpio_Regs *port_ptr = gpio_getPort(config->PORT);
    if (!port_ptr) {
        return GPIO_ERROR_INVALID_PORT;
    }

    uint32_t mask = config->PIN;

    /* Writing on data register of the port specified */
    if (pinState) {
        port_ptr->GPIODATA[mask] = mask;
    }
    else {
        port_ptr->GPIODATA[mask]  = 0;
    }      

    return GPIO_SUCCESS;
}

/**
 * @brief gpio_digitalToggle : Toggles the GPIO port's pin specified by the user
 * 
 * @param config   : Configuration specified by the user
 * 
 * @return gpio_Error 
 */
gpio_Error gpio_digitalToggle(gpio_Config* const config)
{
    if (!config) {
        return GPIO_INVALID_CONFIGURATION;
    }
    /* Port validation */
    gpio_Regs *port_ptr = gpio_getPort(config->PORT);
    if (!port_ptr) {
        return GPIO_ERROR_INVALID_PORT;
    }
    /* Pin validation 
       As 8 pins combination can have a maximum value of 0xFF*/
    if (config->PIN < PIN0 || config->PIN > 0xFF) {
        return GPIO_ERROR_INVALID_PIN;
    }

    uint32_t mask = config->PIN;
    port_ptr->GPIODATA[mask] ^= mask;

    return GPIO_SUCCESS;
}


/**
 * @brief gpio_digitalRead : Reads the GPIO port's pin specified by the user
 * 
 * @param config   : Configuration specified by the user
 * @param pinState : Pin to read from
 * 
 * @return gpio_Error 
 */
gpio_Error gpio_digitalRead(gpio_Config* const config, uint8_t* pinState)
{
    if (!config) {
        return GPIO_INVALID_CONFIGURATION;
    }
    /* Port validation */
    gpio_Regs *port_ptr = gpio_getPort(config->PORT);
    if (!port_ptr) {
        return GPIO_ERROR_INVALID_PORT;
    }
    /* Pin validation
       As 8 pins combination can have a maximum value of 0xFF */
    if (config->PIN < PIN0 || config->PIN > 0xFF) {
        return GPIO_ERROR_INVALID_PIN;
    }

    uint32_t mask = config->PIN;
    *pinState = (port_ptr->GPIODATA[mask] & mask) ? 1 : 0;

    return GPIO_SUCCESS;
}

/**
 * @brief gpio_interruptCallback : Interrupt callback function for GPIO.
 * 
 * @return
 */
void gpio_interruptCallback(gpio_Config* config)
{
    gpio_Regs* port = gpio_getPort(config->PORT);
    /* Checks if interrupt is raised by the button which starts the timer.
       In this case, its sw1. */
    if (port->GPIOMIS & PIN4) {
        Current_state = STATE_START_TIMER;
    /* Clears interrupt */
        port->GPIOICR |= PIN4;
    }
    /* Checks if interrupt is raised by the button which restarts the state machine.
       In this case, its sw2. */
    else if (port->GPIOMIS & PIN0) {
        Current_state = STATE_RESTART;
    /* Clears interrupt */
        port->GPIOICR |= PIN0;
    }
}