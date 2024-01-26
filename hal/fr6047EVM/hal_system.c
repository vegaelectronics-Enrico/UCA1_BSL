#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal.h"


#include "UI\LCD_custom.h"
#include "main.h"
#include "System\SysStatus.h"
//
// Local function prototypes
//
static void hal_system_GPIOInit(void);
static void hal_system_ClockInit(void);

//
// Function declarations
//

void hal_system_Init(void)
{
    hal_system_WatchdogInit();
#ifndef __SVSH_ENABLE__
    PMM_disableSVSH();
#endif
    hal_system_GPIOInit();
    detectHwConfigurationPins();    //0.17
    SWITCH_CH1; //select the ultrasonic channel 1
    hal_system_ClockInit();
    hal_system_ButtonsInit();
#ifdef ENABLE_ADC
    hal_adc_init();
#endif
#ifdef ENABLE_LCD

    hal_lcd_Init();
    hal_lcd_turnoffLCD();
#endif
    LCD_init();
    //hal_system_LEDOff(HAL_SYS_LED_0);	/* Clear LED1 and LED2 if there are no errors	*/
    //hal_system_LEDOff(HAL_SYS_LED_1);
    //hal_system_LEDOff(HAL_SYS_LED_2);
}

void hal_system_LEDOff(uint8_t LEDs)
{
  if (LEDs & HAL_SYS_LED_0)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputHighOnPin(HAL_SYS_LED0_PORT, HAL_SYS_LED0_PIN);
#else
      GPIO_setOutputLowOnPin(HAL_SYS_LED0_PORT, HAL_SYS_LED0_PIN);
#endif
  }
  if (LEDs & HAL_SYS_LED_1)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputHighOnPin(HAL_SYS_LED1_PORT, HAL_SYS_LED1_PIN);
#else
      GPIO_setOutputLowOnPin(HAL_SYS_LED1_PORT, HAL_SYS_LED1_PIN);
#endif
    }
  if (LEDs & HAL_SYS_LED_2)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputHighOnPin(HAL_SYS_LED2_PORT, HAL_SYS_LED2_PIN);
#else
      GPIO_setOutputLowOnPin(HAL_SYS_LED2_PORT, HAL_SYS_LED2_PIN);
#endif
  }
}

void hal_system_LEDOn(uint8_t LEDs)
{
  if (LEDs & HAL_SYS_LED_0)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputLowOnPin(HAL_SYS_LED0_PORT, HAL_SYS_LED0_PIN);
#else
      GPIO_setOutputHighOnPin(HAL_SYS_LED0_PORT, HAL_SYS_LED0_PIN);
#endif
  }
  if (LEDs & HAL_SYS_LED_1)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputLowOnPin(HAL_SYS_LED1_PORT, HAL_SYS_LED1_PIN);
#else
      GPIO_setOutputHighOnPin(HAL_SYS_LED1_PORT, HAL_SYS_LED1_PIN);
#endif
    }
  if (LEDs & HAL_SYS_LED_2)
  {
#if (__EVM430_VER__==0x10)
      GPIO_setOutputLowOnPin(HAL_SYS_LED2_PORT, HAL_SYS_LED2_PIN);
#else
      GPIO_setOutputHighOnPin(HAL_SYS_LED2_PORT, HAL_SYS_LED2_PIN);
#endif
  }
}

void hal_system_LEDToggle(uint8_t LEDs)
{

  if (LEDs & HAL_SYS_LED_0)
  {
    GPIO_toggleOutputOnPin(HAL_SYS_LED0_PORT, HAL_SYS_LED0_PIN);
  }
  if (LEDs & HAL_SYS_LED_1)
  {
    GPIO_toggleOutputOnPin(HAL_SYS_LED1_PORT, HAL_SYS_LED1_PIN);
  }
  if (LEDs & HAL_SYS_LED_2)
  {
    GPIO_toggleOutputOnPin(HAL_SYS_LED2_PORT, HAL_SYS_LED2_PIN);
  }
}

void hal_system_ButtonsInit(void)
{
    GPIO_selectInterruptEdge(HAL_SYS_REED_PORT,
                             HAL_SYS_REED_PIN,
                             GPIO_HIGH_TO_LOW_TRANSITION);
    hal_system_ButtonsClearAll();
}

void hal_system_ButtonsClearAll(void)
{
    GPIO_clearInterrupt(HAL_SYS_REED_PORT, HAL_SYS_REED_PIN);
}


void hal_system_ButtonsClearInt(uint16_t buttons)
{
    if (buttons & HAL_SYS_REED)
    {
        GPIO_clearInterrupt(HAL_SYS_REED_PORT, HAL_SYS_REED_PIN);
    }
}

void hal_system_ButtonsEnableInt(uint16_t buttons)
{
    hal_system_ButtonsClearInt(buttons);

    if (buttons & HAL_SYS_REED)
    {
        GPIO_enableInterrupt(HAL_SYS_REED_PORT, HAL_SYS_REED_PIN);
    }
}


void hal_system_ButtonsDisableInt(uint16_t buttons)
{
    if (buttons & HAL_SYS_REED)
    {
        GPIO_disableInterrupt(HAL_SYS_REED_PORT, HAL_SYS_REED_PIN);
    }
}


uint16_t hal_system_ButtonsCheck(void)
{
    uint8_t ret_val = 0;

    if (GPIO_getInterruptStatus(HAL_SYS_REED_PORT, HAL_SYS_REED_PIN))
    {
        ret_val |= HAL_SYS_REED;
    }

    return ret_val;
}


void hal_system_WatchdogInit(void)
{
#ifdef __WATCHDOG_ENABLE__
    //Enable watchdog timer
    // Delay set to ~32K @ ~10K = 51sec
    WDT_A_initWatchdogTimer(__MSP430_BASEADDRESS_WDT_A__,
                            WDT_A_CLOCKSOURCE_VLOCLK,
                            WDT_A_CLOCKDIVIDER_512K);

#endif
}


void hal_system_WatchdogFeed(void)
{
#ifdef __WATCHDOG_ENABLE__
    // Feed watchdog timer
    WDT_A_resetTimer(__MSP430_BASEADDRESS_WDT_A__);
#endif
}


uint16_t hal_system_GetResetSource(void)
{
    uint16_t reset_source = SYSRSTIV;
    SYSRSTIV = 0x00;

    if (reset_source > SYSRSTIV__RSTNMI)
    {
        return reset_source;
    }
    else
    {
        return 0x00;
    }
}

//
// Local Function declarations
//
//*****************************************************************************
//
//! Initializes GPIOs
//!
//! \return none
//
// *****************************************************************************
static void hal_system_GPIOInit(void)
{
#ifdef FIRST_HW_1_0
    //P1
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0|GPIO_PIN4);
    /* P1.5: IRQ (OPEN DRAIN)*/
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN5);
    /* P1.6: UCB0 I2C SDA & P1.7: UCB0 I2C SCL  RS485_RX_UC_TX RS485_TX_UC_RX*/
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    //P2
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN4);   //reed
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);

    //P3
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN_ALL8);

    //P4
    GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN_ALL8);

    //P5
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN_ALL8);

    //P6
    GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN_ALL8);

    //P7
    GPIO_setOutputLowOnPin(GPIO_PORT_P7,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7,GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN_ALL8);

    //P8
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN1|GPIO_PIN3|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN3|GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //P9
    GPIO_setOutputLowOnPin(GPIO_PORT_P9,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P9,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);

    //PJ
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_PJ,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN6|GPIO_PIN7);
    /* Configure LFXT GPIO pins */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ,GPIO_PIN4 + GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
#else
    //P1
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN2 | GPIO_PIN4);
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN4);
    /* P1.5: IRQ (OPEN DRAIN)*/
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN3 | GPIO_PIN5);
    /* P1.6: UCB0 I2C SDA & P1.7: UCB0 I2C SCL  RS485_RX_UC_TX RS485_TX_UC_RX*/
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_SECONDARY_MODULE_FUNCTION);

    //P2
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN4);   //reed
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN4);

    //P3
    GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN_ALL8);

    //P4
    GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN_ALL8);

    //P5
    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN_ALL8);

    //P6
    GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN_ALL8);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN_ALL8);

    //P7
    GPIO_setOutputLowOnPin(GPIO_PORT_P7,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7,GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN_ALL8);

    //P8
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN1|GPIO_PIN3|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN3|GPIO_PIN7);
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    //P9
    GPIO_setOutputLowOnPin(GPIO_PORT_P9,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P9,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3);

    //PJ
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_PJ,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN6|GPIO_PIN7);
    /* Configure LFXT GPIO pins */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_PJ,GPIO_PIN4 + GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);
#endif
    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

//*****************************************************************************
//
//! Initializes the system clocks
//!
//! This function initializes crystals, DCO and ACLK,SMCLK,MCLK
//!
//! \return none
//
// *****************************************************************************
static void hal_system_ClockInit(void)
{

#if (HAL_SYS_MCLK_FREQ_HZ > 8000000)
    //Set wait state to 1
    FRAMCtl_A_configureWaitStateControl(FRAMCTL_A_ACCESS_TIME_CYCLES_1);
#endif

#if (HAL_SYS_DCO_FREQ_HZ == 8000000)
    // Set DCO frequency to default 8MHz
    CS_setDCOFreq(CS_DCORSEL_0, CS_DCOFSEL_6);
#elif (HAL_SYS_DCO_FREQ_HZ == 16000000)
    // Set DCO frequency to default 16MHz
    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_4);
#else
    #error "Clock configuration not supported"
#endif

#if (HAL_SYS_LFXT_FREQ_HZ == 32768)
    //Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768,0);
#else
    #error "Clock configuration not supported"
#endif


#if (HAL_SYS_MCLK_FREQ_HZ == (HAL_SYS_DCO_FREQ_HZ))
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
#elif (HAL_SYS_MCLK_FREQ_HZ == (HAL_SYS_DCO_FREQ_HZ/2))
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
#else
    #error "Clock configuration not supported"
#endif
#if (HAL_SYS_SMCLK_FREQ_HZ == (HAL_SYS_DCO_FREQ_HZ))
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
#elif (HAL_SYS_SMCLK_FREQ_HZ == (HAL_SYS_DCO_FREQ_HZ/2))
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_2);
#else
    #error "Clock configuration not supported"
#endif

#if (HAL_SYS_ACLK_FREQ_HZ == (HAL_SYS_LFXT_FREQ_HZ))
    CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
#else
    #error "Clock configuration not supported"
#endif

    // Intializes the XT1 crystal oscillator
    CS_turnOnLFXT(CS_LFXT_DRIVE_3);
}
