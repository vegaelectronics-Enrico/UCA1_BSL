//#############################################################################
//
//! \file   hal_system.c
//!
//! \brief  Hardware Abstraction Layer for MSP system including clocks,
//!         watchdog, and GPIOs
//!
//! This file is also used to enable/disable some test configurations
//!
//!  Group:          MSP Smart Metering Systems Applications
//!  Target Device:  MSP430FR6047
//!
//!  (C) Copyright 2019, Texas Instruments, Inc.
//#############################################################################

#ifndef HAL_SYSTEM_H_
#define HAL_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#if (__EVM430_ID__!=0x47)
#error "EVM not supported by HAL layer"
#endif


/*
 * TEST_TRIM_CONST:
 *  Uncomment and force TRIM to a default value (defined in TRIM_CONST_TYPE)
 *  Possible
 */
//#define TEST_TRIM_CONST

/*
 * TEST_ADC_MAX_AMPLITUDE:
 *  Report ADC Waveform MAX AMPLITUDE
 *  The value will be sent to GUI using STD.
 */
#define TEST_ADC_MAX_AMPLITUDE

#define USS_MCLK_FREQ_IN_HZ               8000000
#define USS_SMCLK_FREQ_IN_HZ               8000000
#define USS_LFXT_FREQ_IN_HZ                  32768


/* Clock system frequencies */
/*! DCO Frequency in Hz */
#define HAL_SYS_DCO_FREQ_HZ       (8000000)
/*! MCLK Frequency (derived from DCO) */
#define HAL_SYS_MCLK_FREQ_HZ      (USS_MCLK_FREQ_IN_HZ)
/*! SMCLK Frequency (derived from DCO) */
#define HAL_SYS_SMCLK_FREQ_HZ     (USS_SMCLK_FREQ_IN_HZ)
/*! LFXT Frequency in Hz */
#define HAL_SYS_LFXT_FREQ_HZ      (USS_LFXT_FREQ_IN_HZ)
/*! ACLK Frequency (derived from LFXT) */
#define HAL_SYS_ACLK_FREQ_HZ      (USS_LFXT_FREQ_IN_HZ)
/*! Delay in cycles using ACLK */
#define HAL_SYS_MSDELAY_ACLK(x)    (((uint32_t)x*HAL_SYS_ACLK_FREQ_HZ)/1000)

/*! Return value when no error is detected */
#define HAL_SYS_RETURN_OK               (0x00)
/*! Return value to indicate a watchdog reset */
#define HAL_SYS_ERROR_WATCHDOG_RESET    (0x01)
/*! Return value to indicate error in parameters */
#define HAL_SYS_ERROR_INITPARAMS        (0x02)

/* EVM LED and Button definition */
/* LED Definitions */
#define HAL_SYS_LED_0        0x01            /*! LED0 */
#define HAL_SYS_LED_1        0x02            /*! LED1 */
#define HAL_SYS_LED_2        0x04            /*! LED2 */
#define HAL_SYS_LED0_PORT    GPIO_PORT_PJ    /*! Port used for LED0 */
#define HAL_SYS_LED0_PIN     GPIO_PIN0       /*! Pin used for LED0  */
#define HAL_SYS_LED1_PORT    GPIO_PORT_PJ    /*! Port used for LED1 */
#define HAL_SYS_LED1_PIN     GPIO_PIN1       /*! Pin used for LED1  */
#define HAL_SYS_LED2_PORT    GPIO_PORT_PJ    /*! Port used for LED2 */
#define HAL_SYS_LED2_PIN     GPIO_PIN2       /*! Pin used for LED2  */
/* Buttons Definitions */
#define HAL_SYS_REED                0x01
#define HAL_SYS_REED_PORT           GPIO_PORT_P2
#define HAL_SYS_REED_PIN            GPIO_PIN4
#define REED_CLOSED                 !(P2IN & BIT4)

#ifdef FIRST_HW_1_0
#define SW1_S1  P7OUT &= ~BIT5
#define SW2_S1  P7OUT &= ~BIT4
#define SW3_S1  P8OUT &= ~BIT0
#define SW4_S1  P8OUT &= ~BIT1

#define SW1_S2  P7OUT |= BIT5
#define SW2_S2  P7OUT |= BIT4
#define SW3_S2  P8OUT |= BIT0
#define SW4_S2  P8OUT |= BIT1
#else
#define SW_S1   P7OUT &= ~BIT5
#define SW_S2   P7OUT |= BIT5

#endif

#ifdef FIRST_HW_1_0
#define SWITCH_CH1  SW1_S1; SW2_S1; SW3_S1; SW4_S1
#define SWITCH_CH2  SW1_S2; SW2_S2; SW3_S2; SW4_S2
#else
#define SWITCH_CH1  SW_S1
#define SWITCH_CH2  SW_S2
#endif

#define DAC161S_CS_UNSEL      P8OUT &= ~BIT7
#define DAC161S_CS_SEL        P8OUT |= BIT7

#define EXT_WDT_TOGGLE        PJOUT ^= BIT0

/* Function prototypes */
//*****************************************************************************
//
//! Initialize the System
//! Includes initialization of clocks, watchdog and GPIOs
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_Init(void);

//*****************************************************************************
//
//! Turn LED(s) on
//!
//! \param LEDs One or more LED(s) to turn on (HAL_SYS_LED_x)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_LEDOn(uint8_t LEDs);

//*****************************************************************************
//
//! Turn LED(s) off
//!
//! \param LEDs One or more LEDs to turn off (HAL_SYS_LED_x)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_LEDOff(uint8_t LEDs);

//*****************************************************************************
//
//! Toggle LED(s)
//!
//! \param LEDs One or more LEDs to toggle (HAL_SYS_LED_x)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_LEDToggle(uint8_t LEDs);

//*****************************************************************************
//
//! Initialize the buttons
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_ButtonsInit(void);

//*****************************************************************************
//
//! Clear flags for all buttons
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_ButtonsClearAll(void);

//*****************************************************************************
//
//! Clear interrupt flags for the specified buttons
//!
//! \param buttons One or more buttons to clear (HAL_SYS_BUTTON_xxx)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_ButtonsClearInt(uint16_t buttons);

//*****************************************************************************
//
//! Enable Interrupts for the specified buttons
//!
//! \param buttons One or more buttons to enable interrupts (HAL_SYS_BUTTON_xxx)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_ButtonsEnableInt(uint16_t buttons);

//*****************************************************************************
//
//! Disable interrupts for the specified buttons
//!
//! \param buttons One or more buttons to disable interrupts (HAL_SYS_BUTTON_xxx)
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_ButtonsDisableInt(uint16_t buttons);

//*****************************************************************************
//
//! Check if buttons were pressed
//!
//! \return pressed buttons (HAL_SYS_BUTTON_xxx)
//
// *****************************************************************************
extern uint16_t hal_system_ButtonsCheck(void);

//*****************************************************************************
//
//! Initialize Watchdog
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_WatchdogInit(void);

//*****************************************************************************
//
//! Feed Watchdog.
//! This function must be called periodically to avoid a reset.
//!
//! \return none
//
// *****************************************************************************
extern void hal_system_WatchdogFeed(void);

//*****************************************************************************
//
//! Get the reset source
//!
//! \return Value of SYSRSTIV register indicating reset source
//
// *****************************************************************************
extern uint16_t hal_system_GetResetSource(void);


#endif /* HAL_SYSTEM_H_ */
