/***************************************************************************************************
* FILENAME : gptm.h
* DESCRIPTION : Defines macros, enums, structs and function prototypes for timer driver.
*
* NOTES :
*
* AUTHOR : Mariyam Shahid
***************************************************************************************************/

#ifndef __GPTM_H_
#define __GPTM_H_

/*************************************** Header Inclusion *****************************************/
#include "gpio.h"

/********************************************* Macros *********************************************/
/* Enabling clock for timer */
#define RCGCTIMER       (*((uint32_t *) 0x400FE604))
/* Enabling clock for wide timer */
#define RCGCWTIMER      (*((uint32_t *) 0x400FE65C))
#define SYSTEM_CLOCK    16000000
#define RCGC_EN_TIMER0_CLK  (1U << 0)
#define RCGC_EN_TIMER1_CLK  (1U << 1)
#define RCGC_EN_TIMER2_CLK  (1U << 2)
#define RCGC_EN_TIMER3_CLK  (1U << 3)
#define RCGC_EN_TIMER4_CLK  (1U << 4)
#define RCGC_EN_TIMER5_CLK  (1U << 5)
#define RCGC_EN_WTIMER0_CLK (1U << 0)
#define RCGC_EN_WTIMER1_CLK (1U << 1)
#define RCGC_EN_WTIMER2_CLK (1U << 2)
#define RCGC_EN_WTIMER3_CLK (1U << 3)
#define RCGC_EN_WTIMER4_CLK (1U << 4)
#define RCGC_EN_WTIMER5_CLK (1U << 5)
#define PRTIMER (*((volatile uint32_t *)0x400FEA04))
/* Max bit values */
#define MAX_8_BIT   0xFF
#define MAX_16_BIT  0xFFFF
#define MAX_32_BIT  0xFFFFFFFF
#define MAX_64_BIT  0xFFFFFFFFFFFFFFFFULL

/* Timers base addresses */
#define TIMER0_16_32_BASE     0x40030000
#define TIMER1_16_32_BASE     0x40031000
#define TIMER2_16_32_BASE     0x40032000
#define TIMER3_16_32_BASE     0x40033000
#define TIMER4_16_32_BASE     0x40034000
#define TIMER5_16_32_BASE     0x40035000
#define WTIMER0_32_64_BASE    0x40036000
#define WTIMER1_32_64_BASE    0x40037000
#define WTIMER2_32_64_BASE    0x4004C000
#define WTIMER3_32_64_BASE    0x4004D000
#define WTIMER4_32_64_BASE    0x4004E000
#define WTIMER5_32_64_BASE    0x4004F000

/* Defining timers */
#define TIMER0_16_32   ((gptm_Regs *)TIMER0_16_32_BASE)
#define TIMER1_16_32   ((gptm_Regs *)TIMER1_16_32_BASE)
#define TIMER2_16_32   ((gptm_Regs *)TIMER2_16_32_BASE)
#define TIMER3_16_32   ((gptm_Regs *)TIMER3_16_32_BASE)
#define TIMER4_16_32   ((gptm_Regs *)TIMER4_16_32_BASE)
#define TIMER5_16_32   ((gptm_Regs *)TIMER5_16_32_BASE)
#define WTIMER0_32_64  ((gptm_Regs *)WTIMER0_32_64_BASE)
#define WTIMER1_32_64  ((gptm_Regs *)WTIMER1_32_64_BASE)
#define WTIMER2_32_64  ((gptm_Regs *)WTIMER2_32_64_BASE)
#define WTIMER3_32_64  ((gptm_Regs *)WTIMER3_32_64_BASE)
#define WTIMER4_32_64  ((gptm_Regs *)WTIMER4_32_64_BASE)
#define WTIMER5_32_64  ((gptm_Regs *)WTIMER5_32_64_BASE)

/* Configurations */
#define ONE_SHOT_MODE 0x1
#define PERIODIC_MODE 0x2
/* Timer A enable bit — GPTMCTL */
#define GPTM_CTL_TAEN           (1U << 0)
/* Timer B enable bit — GPTMCTL */
#define GPTM_CTL_TBEN           (1U << 8)
/* Timer A count direction — GPTMTAMR */
#define GPTM_TAMR_TACDIR        (1U << 4)
/* Timer B count direction — GPTMTBMR */
#define GPTM_TBMR_TBCDIR        (1U << 4)
/* Timer A match interrupt enable — GPTMTAMR */
#define GPTM_TAMR_TAMIE         (1U << 5)
/* Timer B match interrupt enable — GPTMTBMR */
#define GPTM_TBMR_TBMIE         (1U << 5)
/* Timer A timeout interrupt mask — GPTMIMR */
#define GPTM_IMR_TATOIM         (1U << 0)
/* Timer B timeout interrupt mask — GPTMIMR */
#define GPTM_IMR_TBTOIM         (1U << 8)
/* Timer A timeout raw interrupt flag — GPTMRIS */
#define GPTM_RIS_TATORIS        (1U << 0)
/* Timer B timeout raw interrupt flag — GPTMRIS */
#define GPTM_RIS_TBTORIS        (1U << 8)
/* Timer A timeout interrupt clear — GPTMICR */
#define GPTM_ICR_TATOCINT       (1U << 0)
/* Timer B timeout interrupt clear — GPTMICR */
#define GPTM_ICR_TBTOCINT       (1U << 8)
/* Timer A PWM output level (invert) — GPTMCTL */
#define GPTM_CTL_TAPWML         (1U << 6)
/* Timer B PWM output level (invert) — GPTMCTL */
#define GPTM_CTL_TBPWML         (1U << 14)
/* Timer A alternate mode (PWM) — GPTMTAMR */
#define GPTM_TAMR_TAAMS         (1U << 3)
/* Timer A capture mode (PWM) — GPTMTAMR */
/* 0: edge-count mode, 1: edge-time mode */
#define GPTM_TAMR_TACMR         (1U << 2)
/* Timer B alternate mode (PWM) — GPTMTBMR */
#define GPTM_TBMR_TBAMS         (1U << 3)
/* Timer B capture mode (PWM) — GPTMTBMR */
/* 0: edge-count mode, 1: edge-time mode */
#define GPTM_TBMR_TBCMR         (1U << 2)
/* Timer A event mode bit position — GPTMCTL */
#define GPTM_CTL_TAEVENT_POS    (2U)
/* Timer B event mode bit position — GPTMCTL */
#define GPTM_CTL_TBEVENT_POS    (10U)

#define ONE_SHOT_MODE        0x1
#define PERIODIC_MODE        0x2
#define SPLIT_MODE           0x4
#define CONCATENATE_MODE     0x0

/****************************************** Enumerations ******************************************/
typedef enum {
    GPTM_TIMER0_16_32,
    GPTM_TIMER1_16_32,
    GPTM_TIMER2_16_32,
    GPTM_TIMER3_16_32,
    GPTM_TIMER4_16_32,
    GPTM_TIMER5_16_32,
    GPTM_WTIMER0_32_64,
    GPTM_WTIMER1_32_64,
    GPTM_WTIMER2_32_64,
    GPTM_WTIMER3_32_64,
    GPTM_WTIMER4_32_64,
    GPTM_WTIMER5_32_64
} gptm_SelectTimer;

typedef enum {
    GPTM_CHANNEL_A,
    GPTM_CHANNEL_B,
    GPTM_CHANNEL_BOTH
} gptm_Channel;

typedef enum {
    GPTM_CONCATENATE,
    GPTM_SPLIT
} gptm_Cfg;

typedef enum {
    GPTM_MODE_ONE_SHOT,
    GPTM_MODE_PERIODIC,
    /* Mentioned in secion 11.4.2: 
       To use the RTC mode, the timer must have a 32.768-KHz input signal 
       on an even CCP input. So, not dealing with RTC mode as it requires an input. */
    GPTM_MODE_RTC,
    /* Mentioned in secion 11.4.5:
       PWM requires a pin configured as output. */
    GPTM_MODE_PWM
} gptm_Mode;

typedef enum {
    GPTM_INTERRUPT_DISABLE = 0,
    GPTM_INTERRUPT_ENABLE
}gptm_InterruptEnable;

typedef enum {
    GPTM_COUNT_DOWN,
    GPTM_COUNT_UP
} gptm_CountDirection;

typedef enum {
    GPTM_SUCCESS = 16,
    GPTM_INVALID_TIMER,
    GPTM_INVALID_CONFIGURATION,
    GPTM_INVALID_CHANNEL,
    GPTM_INVALID_COUNT_DIRECTION,
    GPTM_UNSUPPORTED_MODE,
    GPTM_INVALID_MODE,
    GPTM_NOT_INITIALIZED,
    GPTM_TOO_LARGE_PRESCALER,
    GPTM_TOO_LARGE_LOAD_VALUE,
    __GPTM_RESERVED_START = 26,
    __GPTM_RESERVED_END = 31
} gptm_Error;

/******************************************* Data Types *******************************************/
/* General purpose timer registers */
typedef struct gptm_Regs {
    volatile uint32_t GPTMCFG;
    volatile uint32_t GPTMTAMR;
    volatile uint32_t GPTMTBMR;
    volatile uint32_t GPTMCTL;
    volatile uint32_t GPTMSYNC;
    uint32_t _reserved1;
    volatile uint32_t GPTMIMR;
    volatile uint32_t GPTMRIS;
    volatile uint32_t GPTMMIS;
    volatile uint32_t GPTMICR;
    volatile uint32_t GPTMTAILR;
    volatile uint32_t GPTMTBILR;
    volatile uint32_t GPTMTAMATCHR;
    volatile uint32_t GPTMTBMATCHR;
    volatile uint32_t GPTMTAPR;
    volatile uint32_t GPTMTBPR;
    volatile uint32_t GPTMTAPMR;
    volatile uint32_t GPTMTBPMR;
    volatile uint32_t GPTMTAR;
    volatile uint32_t GPTMTBR;
    volatile uint32_t GPTMTAV;
    volatile uint32_t GPTMTBV;
    volatile uint32_t GPTMRTCPD;
    volatile uint32_t GPTMTAPS;
    volatile uint32_t GPTMTBPS;
    volatile uint32_t GPTMTAPV;
    volatile uint32_t GPTMTBPV;
    uint32_t _reserved2[981];
    volatile uint32_t GPTMPP;
} gptm_Regs;

typedef struct gptm_Config {
    /* Timer module to use (TIMER0–TIMER5 or WTIMER0–WTIMER5) */
    gptm_SelectTimer TIMER;
    /* Channel A or Channel B or Both channels  */
    gptm_Channel CHANNEL;         
    /* One-shot, periodic */
    gptm_Mode MODE;
    /* Up-count or Down-count */
    gptm_CountDirection COUNT_DIR;
    /* Interval load value */
    uint32_t INTERVAL;
    /* Preload value */
    uint32_t PRESCALER;
    /* Interrupt enable/disable */
    gptm_InterruptEnable GPTM_INTERRUPT;
    /* Split or concatination */  
    gptm_Cfg CONFIG;
} gptm_Config;

/*************************************** Function Prototypes **************************************/
gptm_Error gptm_initTimer(gptm_Config* config);
gptm_Error gptm_startTimer(gptm_Config* config);
gptm_Error gptm_blockingDelay(gptm_Config* config, uint32_t delay);
void gptm_interruptCallback(gptm_Config* config);
uint32_t gptm_convertSecToTicks(uint16_t seconds);
static inline gptm_Error gptm_validateConfiguration(gptm_Config* config);
static inline gptm_Error gptm_enableClockForTimer(gptm_Config* config);
gptm_Error gptm_reloadTimer(gptm_Config* config, uint64_t new_ticks);
gptm_Error gptm_stopTimer(gptm_Config* config);

#endif /*__GPTM_H_*/