/***************************************************************************************************
* FILENAME : gptm.c
* DESCRIPTION : Function (APIs) for timer initialization, timer blocking delay (milliseconds)
*               and timer interrupt callback.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

/*************************************** Header Inclusion *****************************************/
#include "../inc/gpio.h"
#include "../inc/gptm.h"
#include "../inc/nvic.h"
#include "../inc/state_machine.h"
#include "../inc/lin_app.h"

/******************************************  GLOBALS  *********************************************/
extern volatile state_machine_States Current_state;
#ifdef LIN_MASTER
extern volatile lin_Masterstate Lin_master_state;
#endif

/************************************* Function Implementations ***********************************/

/**
 * @brief gptm_getBase : Returns base memory address of the timer selected.
 * 
 * @param timer : Timer selected by the user.
 * 
 * @return gptm_Regs*
 */
static gptm_Regs* gptm_getBase(gptm_SelectTimer timer)
{
    switch (timer)
    {
        case GPTM_TIMER0_16_32:
            return TIMER0_16_32;
        case GPTM_TIMER1_16_32:
            return TIMER1_16_32;
        case GPTM_TIMER2_16_32:
            return TIMER2_16_32;
        case GPTM_TIMER3_16_32:
            return TIMER3_16_32;
        case GPTM_TIMER4_16_32:
            return TIMER4_16_32;
        case GPTM_TIMER5_16_32:
            return TIMER5_16_32;
        case GPTM_WTIMER0_32_64:
            return WTIMER0_32_64;
        case GPTM_WTIMER1_32_64:
            return WTIMER1_32_64;
        case GPTM_WTIMER2_32_64:
            return WTIMER2_32_64;
        case GPTM_WTIMER3_32_64:
            return WTIMER3_32_64;
        case GPTM_WTIMER4_32_64:
            return WTIMER4_32_64;
        case GPTM_WTIMER5_32_64:
            return WTIMER5_32_64;
        default:
            return 0;
    }

}

/**
 * @brief gptm_validateConfiguration : Validates the configuration.
 * 
 * @param config : Configuration specified by the user.
 *
 * @return gptm_Error
 */
static inline gptm_Error gptm_validateConfiguration(gptm_Config* config)
{
    /* Configuration validation */
    if (config == 0) {
        return GPTM_INVALID_CONFIGURATION;
    }
    /* Channel validation */
    if (config->CHANNEL < GPTM_CHANNEL_A || config->CHANNEL > GPTM_CHANNEL_BOTH) {
        return GPTM_INVALID_CHANNEL;
    }
    /* Mode validation */
    if (config->MODE == GPTM_MODE_RTC) {
        return GPTM_UNSUPPORTED_MODE;
    }
    if (config->MODE < GPTM_MODE_ONE_SHOT || config->MODE > GPTM_MODE_PWM) {
        return GPTM_INVALID_MODE;
    }
    /* Count direction validation */
    if (config->COUNT_DIR < GPTM_COUNT_DOWN || config->COUNT_DIR > GPTM_COUNT_UP) {
        return GPTM_INVALID_COUNT_DIRECTION;
    }
}

/**
 * @brief gptm_enableClockForTimer : Enables the clock for timer.
 * 
 * @param config : Configuration specified by the user.
 *
 * @return gptm_Error
 */
static inline gptm_Error gptm_enableClockForTimer(gptm_Config* config)
{
    switch (config->TIMER)
    {
        case GPTM_TIMER0_16_32:
            RCGCTIMER |= RCGC_EN_TIMER0_CLK;
            while((PRTIMER & 0x01) == 0);
            break;
        case GPTM_TIMER1_16_32:
            RCGCTIMER |= RCGC_EN_TIMER1_CLK;
            break;
        case GPTM_TIMER2_16_32:
            RCGCTIMER |= RCGC_EN_TIMER2_CLK;
            break;
        case GPTM_TIMER3_16_32:
            RCGCTIMER |= RCGC_EN_TIMER3_CLK;
            break;
        case GPTM_TIMER4_16_32:
            RCGCTIMER |= RCGC_EN_TIMER4_CLK;
            break;
        case GPTM_TIMER5_16_32:
            RCGCTIMER |= RCGC_EN_TIMER5_CLK;
            break;
        case GPTM_WTIMER0_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER0_CLK;
            break;
        case GPTM_WTIMER1_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER1_CLK;
            break;
        case GPTM_WTIMER2_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER2_CLK;
            break;
        case GPTM_WTIMER3_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER3_CLK;
            break;
        case GPTM_WTIMER4_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER4_CLK;
            break;
        case GPTM_WTIMER5_32_64:
            RCGCWTIMER |= RCGC_EN_WTIMER5_CLK;
            break;
        default:
            return GPTM_INVALID_TIMER;
    }
}

/**
 * @brief gptm_initTimer : Initialises timer specified by the user.
 * 
 * @param config : Configuration specified by the user.
 *
 * @return gptm_Error
 */
gptm_Error gptm_initTimer(gptm_Config* config)
{
    /* Validate configuration */
    gptm_validateConfiguration(config);

    /* Resolves base address */
    gptm_Regs* timer;
    timer = gptm_getBase(config->TIMER);

    /* ENABLING CLOCK FOR SELECTED TIMER */
    gptm_enableClockForTimer(config);

    /* Disable timers before configuring */
    if (config->CHANNEL == GPTM_CHANNEL_A || config->CHANNEL == GPTM_CHANNEL_BOTH) {
         timer->GPTMCTL &= ~GPTM_CTL_TAEN;
    }
    if (config->CHANNEL == GPTM_CHANNEL_B || config->CHANNEL == GPTM_CHANNEL_BOTH) {
        timer->GPTMCTL &= ~GPTM_CTL_TBEN;
    }

    /* Timer configuration: concatenated or split */
    /* 32/64-bit concatenated mode */
    if (config->CONFIG == GPTM_CONCATENATE) {
        timer->GPTMCFG = CONCATENATE_MODE;
    }
    /* 16/32-bit split mode */
    else if (config->CONFIG == GPTM_SPLIT) {
        timer->GPTMCFG = SPLIT_MODE;
    }
    else {
        return GPTM_INVALID_CONFIGURATION;
    }

    /* Configuring timer A */
    if (config->CHANNEL == GPTM_CHANNEL_A || config->CHANNEL == GPTM_CHANNEL_BOTH) {
        /* Clear TAMR */
        timer->GPTMTAMR = 0;

        switch (config->MODE)
        {
            /* Mentioned on page # 729 of tm4c123gh6pm.pdf.
               Register 2: GPTM Timer A Mode (GPTMTAMR), offset 0x004 */
            case GPTM_MODE_ONE_SHOT:
                timer->GPTMTAMR |= ONE_SHOT_MODE;
                break;
            case GPTM_MODE_PERIODIC:
                timer->GPTMTAMR |= PERIODIC_MODE;
                break;
            default:
                return GPTM_UNSUPPORTED_MODE;
        }

        /* Count direction */
        if (config->COUNT_DIR == GPTM_COUNT_UP) {
            timer->GPTMTAMR |= GPTM_TAMR_TACDIR;
        }
        else {
            timer->GPTMTAMR &= ~GPTM_TAMR_TACDIR;
        }

        /* Load interval */
        timer->GPTMTAILR = config->INTERVAL;

        /* Set prescaler (PRELOAD) depending on mode */
        /* On page # 760 of datasheet */
        /* Prescaler acts as clock divider in split down-count one-shot/periodic mode */
        /* Prescaler is 8 bits for 16-bit timers, 16 bits for 32-bit wide timers */
        /* Mask and assign appropriately */
        if (config->CONFIG == GPTM_SPLIT && config->COUNT_DIR == GPTM_COUNT_DOWN &&
            (config->MODE == GPTM_MODE_ONE_SHOT || config->MODE == GPTM_MODE_PERIODIC))
        {
            /* 8-bit */
            uint32_t maxPrescale = MAX_8_BIT;
            /* 16-bit prescaler for wide timers */
            if (config->TIMER >= GPTM_WTIMER0_32_64 && config->TIMER <= GPTM_WTIMER5_32_64) {
                maxPrescale = MAX_16_BIT;
            }
            if (config->PRESCALER > maxPrescale) {
                return GPTM_TOO_LARGE_PRESCALER;
            }
            /* Timer A prescaler register */
            timer->GPTMTAPR = config->PRESCALER;
        }
        else
        {
            /* In all other modes, prescaler acts as upper bits extension */
            timer->GPTMTAPR = config->PRESCALER;
        }

        /* Interrupt enable */
        if (config->GPTM_INTERRUPT == GPTM_INTERRUPT_ENABLE) {
            /* Clear interrupts */
            /* Section 11.4.1 (page # 746): The status flags are cleared by writing
            a 1 to the appropriate bit of the GPTM Interrupt Clear Register (GPTMICR) */
            timer->GPTMICR = GPTM_ICR_TATOCINT;
            timer->GPTMIMR |= GPTM_IMR_TATOIM;
            nvic_enbleTimerInterupt(config->TIMER, config->CHANNEL);
        }
        else {
            timer->GPTMIMR &= ~GPTM_IMR_TATOIM;
        }

    }

    /* Configuring timer B */
    if (config->CHANNEL == GPTM_CHANNEL_B || config->CHANNEL == GPTM_CHANNEL_BOTH) {
        /* Clear TBMR */
        timer->GPTMTBMR = 0;

        switch (config->MODE)
        {
            /* Mentioned on page # 733 of tm4c123gh6pm.pdf.
               Register 3: GPTM Timer B Mode (GPTMTBMR), offset 0x008 */
            case GPTM_MODE_ONE_SHOT:
                timer->GPTMTBMR |= ONE_SHOT_MODE;
                break;
            case GPTM_MODE_PERIODIC:
                timer->GPTMTBMR |= PERIODIC_MODE;
                break;
            default:
                return GPTM_UNSUPPORTED_MODE;
        }

        /* Count direction */
        if (config->COUNT_DIR == GPTM_COUNT_UP) {
            timer->GPTMTBMR |= GPTM_TBMR_TBCDIR;
        }
        else {
            timer->GPTMTBMR &= ~GPTM_TBMR_TBCDIR;
        }

        /* Load interval */
        timer->GPTMTBILR = config->INTERVAL;

        /* Set prescaler (PRELOAD) depending on mode */
        /* On page # 760 of datasheet */
        /* Prescaler acts as clock divider in split down-count one-shot/periodic mode */
        /* Prescaler is 8 bits for 16-bit timers, 16 bits for 32-bit wide timers */
        /* Mask and assign appropriately */
        if (config->CONFIG == GPTM_SPLIT && config->COUNT_DIR == GPTM_COUNT_DOWN &&
            (config->MODE == GPTM_MODE_ONE_SHOT || config->MODE == GPTM_MODE_PERIODIC))
        {
            /* 8-bit */
            uint32_t maxPrescale = MAX_8_BIT;
            /* 16-bit prescaler for wide timers */
            if (config->TIMER >= GPTM_WTIMER0_32_64 && config->TIMER <= GPTM_WTIMER5_32_64) {
                maxPrescale = MAX_16_BIT; 
            }
            if (config->PRESCALER > maxPrescale) {
                return GPTM_TOO_LARGE_PRESCALER;
            }
            /* Timer A prescaler register */
            timer->GPTMTBPR = config->PRESCALER;
        }
        else
        {
            /* In all other modes, prescaler acts as upper bits extension */
            timer->GPTMTBPR = config->PRESCALER;
        }

        /* Interrupt enable */
        if (config->GPTM_INTERRUPT == GPTM_INTERRUPT_ENABLE) {
            /* Clear interrupts */
            /* Section 11.4.1 (page # 746): The status flags are cleared by writing 
            a 1 to the appropriate bit of the GPTM Interrupt Clear Register (GPTMICR) */
            timer->GPTMICR = GPTM_ICR_TBTOCINT;
            timer->GPTMIMR |= GPTM_IMR_TBTOIM;
            nvic_enbleTimerInterupt(config->TIMER, config->CHANNEL);
        }
        else {
            timer->GPTMIMR &= ~GPTM_IMR_TBTOIM;
        }

    }

    return GPTM_SUCCESS;
}

/**
 * @brief gptm_reloadTimer : Reloads timer specified by the user.
 * 
 * @param config : Configuration specified by the user.
 * @param new_ticks  : Number of ticks to be loaded in load register.
 *
 * @return gptm_Error
 */
gptm_Error gptm_reloadTimer(gptm_Config* config, uint64_t new_ticks)
{
    if (config == 0) {
        return GPTM_NOT_INITIALIZED;
    }

    uint64_t maxLoad = 0;
    gptm_Regs* timer = gptm_getBase(config->TIMER);

    /* Determine max load based on configuration */
    if (config->CONFIG == GPTM_CONCATENATE) {
        /* Concatenated mode */
        if (config->TIMER >= GPTM_WTIMER0_32_64 && config->TIMER <= GPTM_WTIMER5_32_64) {
            maxLoad = MAX_64_BIT;
        }
        else {
            maxLoad = MAX_32_BIT;
        }
    }
    else if (config->CONFIG == GPTM_SPLIT) {
        /* Split mode */
        if (config->TIMER >= GPTM_WTIMER0_32_64 && config->TIMER <= GPTM_WTIMER5_32_64) {
            maxLoad = MAX_32_BIT;
        }
        else {
            maxLoad = MAX_16_BIT;
        }
    }
    else {
        return GPTM_INVALID_CONFIGURATION;
    }

    /* Validate load value */
    if (new_ticks > maxLoad) {
        return GPTM_TOO_LARGE_LOAD_VALUE;
    }

    /* Validate prescaler again, incase user reconfigures prescaler */
    if (config->CONFIG == GPTM_SPLIT && config->COUNT_DIR == GPTM_COUNT_DOWN &&
        (config->MODE == GPTM_MODE_ONE_SHOT || config->MODE == GPTM_MODE_PERIODIC)) {
        uint32_t maxPrescale = MAX_8_BIT;

        if (config->TIMER >= GPTM_WTIMER0_32_64 && config->TIMER <= GPTM_WTIMER5_32_64) {
            maxPrescale = MAX_16_BIT;
        }

        if (config->PRESCALER > maxPrescale) {
            return GPTM_TOO_LARGE_PRESCALER;
        }
    }

    /* Reload based on channel */
    switch (config->CHANNEL)
    {
        case GPTM_CHANNEL_A:
            timer->GPTMTAILR = new_ticks;
            break;
        case GPTM_CHANNEL_B:
            timer->GPTMTBILR = new_ticks;
            break;
        case GPTM_CHANNEL_BOTH:
            timer->GPTMTAILR = new_ticks;
            timer->GPTMTBILR = new_ticks;
            break;
        default:
            return GPTM_INVALID_CHANNEL;
    }

    return GPTM_SUCCESS;
}

/**
 * @brief gptm_startTimer : Starts the timer, after the timer has been initialized.
 * 
 * @param config : Configuration specified by the user.
 * 
 * @return gptm_Error
 */
gptm_Error gptm_startTimer(gptm_Config* config)
{
    if (config == 0) {
        return GPTM_NOT_INITIALIZED;
    }

    gptm_Regs* timer = gptm_getBase(config->TIMER);

    switch (config->CHANNEL)
    {
    case GPTM_CHANNEL_A:
        /* Clearing interrupt flags */
        timer->GPTMICR = GPTM_ICR_TATOCINT;
        /* Starting timer */
        timer->GPTMCTL |= GPTM_CTL_TAEN;
        break;
    case GPTM_CHANNEL_B:
        /* Clearing interrupt flags */
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
        /* Starting timer */
        timer->GPTMCTL |= GPTM_CTL_TBEN;
        break;
    case GPTM_CHANNEL_BOTH:
        /* Clearing interrupt flags */
        timer->GPTMICR = GPTM_ICR_TATOCINT;
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
        /* Starting timer */
        timer->GPTMCTL |= (GPTM_CTL_TAEN | GPTM_CTL_TBEN);
        break;
    default:
        return GPTM_INVALID_CHANNEL;
    }

    return GPTM_SUCCESS;
}

/**
 * @brief gptm_blockingDelay : Executes when the cpu waits for the timer to  finish.
 * 
 * @param config   : Configuration specified by the user.
 * @param load_val : Load value for timer.
 * 
 * @return gptm_Error
 */
gptm_Error gptm_blockingDelay(gptm_Config* config, uint32_t load_val)
{
    /* Configuration validation */
    if (config == 0) {
        return GPTM_INVALID_CONFIGURATION;
    }
    /* Channel validation */
    if (config->CHANNEL < GPTM_CHANNEL_A || config->CHANNEL > GPTM_CHANNEL_BOTH) {
        return GPTM_INVALID_CHANNEL;
    }
    /* Mode validation */
    if (config->MODE == GPTM_MODE_RTC) {
        return GPTM_UNSUPPORTED_MODE;
    }
    if (config->MODE < GPTM_MODE_ONE_SHOT || config->MODE > GPTM_MODE_PWM) {
        return GPTM_INVALID_MODE;
    }
    /* Count direction validation */
    if (config->COUNT_DIR < GPTM_COUNT_DOWN || config->COUNT_DIR > GPTM_COUNT_UP) {
        return GPTM_INVALID_COUNT_DIRECTION;
    }

    gptm_Regs* timer;
    timer = gptm_getBase(config->TIMER);

    /* Converting ms to ticks */
    /* Assuming 16 MHz system clock */
    uint32_t ticks = load_val * (SYSTEM_CLOCK);
    /* Timer A */
    if (config->CHANNEL == GPTM_CHANNEL_A) {
        timer->GPTMCTL &= ~GPTM_CTL_TAEN;
        timer->GPTMTAILR = ticks;
        timer->GPTMICR = GPTM_ICR_TATOCINT;
        timer->GPTMCTL |= GPTM_CTL_TAEN;

        /* Polling */
        while (!(timer->GPTMRIS & GPTM_RIS_TATORIS));

        /* Clearing interrupt */
        timer->GPTMICR = GPTM_ICR_TATOCINT;
    }

    /* Timer B */
    else if (config->CHANNEL == GPTM_CHANNEL_B) {
        timer->GPTMCTL &= ~GPTM_CTL_TBEN;
        timer->GPTMTBILR = ticks;
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
        timer->GPTMCTL |= GPTM_CTL_TBEN;

        /* Polling */
        while (!(timer->GPTMRIS & GPTM_RIS_TBTORIS));

        /* Clearing interrupt */
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
    }

    /* BOTH channels */
    else if (config->CHANNEL == GPTM_CHANNEL_BOTH) {
        /* A */
        timer->GPTMCTL &= ~GPTM_CTL_TAEN;
        timer->GPTMTAILR = ticks;
        timer->GPTMICR = GPTM_ICR_TATOCINT;
        timer->GPTMCTL |= GPTM_CTL_TAEN;

        /* B */
        timer->GPTMCTL &= ~GPTM_CTL_TBEN;
        timer->GPTMTBILR = ticks;
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
        timer->GPTMCTL |= GPTM_CTL_TBEN;

        /* Wait for both */
        while (!(timer->GPTMRIS & GPTM_RIS_TATORIS) || !(timer->GPTMRIS & GPTM_RIS_TBTORIS));

        /* Clearing interrupt */
        timer->GPTMICR = GPTM_ICR_TATOCINT;
        timer->GPTMICR = GPTM_ICR_TBTOCINT;
    }

    return GPTM_SUCCESS;
}

/**
 * @brief gptm_interruptCallback : Interrupt routine to execute when timer expires.
 * 
 * @param config   : Configuration specified by the user
 * 
 * @return
 */
void gptm_interruptCallback(gptm_Config* config)
{
    if (config == 0) {
        return;
    }

    /* Getting configured timer base */
    gptm_Regs* timer = gptm_getBase(config->TIMER);

    /* Clearing interrupt flags */
    timer->GPTMICR = GPTM_ICR_TATOCINT;
    Current_state = STATE_TURN_ON_GREEN_LED;
    /* Change lin app state to ack timeout */
    #ifdef LIN_MASTER
    Lin_master_state = LIN_MASTER_ACK_TIMEOUT;
    #endif
    return;
}

/**
 * @brief gptm_convertSecToTicks : Converts seconds to ticks.
 * 
 * @param seconds  : Seconds to be converted into ticks.
 * 
 * @return
 */
uint32_t gptm_convertSecToTicks(uint16_t seconds)
{
    return SYSTEM_CLOCK * seconds;
}

/**
 * @brief gptm_stopTimer : Stops the timer.
 * 
 * @param config  : Configuration specified byt the user.
 * 
 * @return
 */
gptm_Error gptm_stopTimer(gptm_Config* config)
{
    gptm_Regs* timer = gptm_getBase(config->TIMER);
    if (!timer) return GPTM_INVALID_TIMER;

    if (config->CHANNEL == GPTM_CHANNEL_A || config->CHANNEL == GPTM_CHANNEL_BOTH) {
         timer->GPTMCTL &= ~GPTM_CTL_TAEN;
    }
    if (config->CHANNEL == GPTM_CHANNEL_B || config->CHANNEL == GPTM_CHANNEL_BOTH) {
        timer->GPTMCTL &= ~GPTM_CTL_TBEN;
    }
    return GPTM_SUCCESS;
}