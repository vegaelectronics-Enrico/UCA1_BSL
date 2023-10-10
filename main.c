/*UCA1 ultrasonic water meter
 * based on MSP430FR6047
 */

#include "main.h"
#include <stdint.h>
#include "hal.h"
#include <stdio.h>
#include "Timers/Timers.h"
#include "EEmemory/25XXXXX.h"
#include "CommInt/CommInterfaces.h"
#include "UI/LCD_custom.h"
#include "Display_UI/Display_UI.h"
#include "CRC32/CRC32.h"
#include <intrinsics.h>


uint8_t TimeputToMainFw = 5;


#warning "correct settings: --code_model:large --data_model:large --near_data:none"
#warning "to avoid totalizers to be erased with the programming specify the memory erase range (RUN/DEBUG settings > UCA1 > target > MSP430 flash settings)"


#ifdef FIRST_HW_1_0
#warning "Attenzione! Versione HW 1_0"
#endif

/*
 * Timer modules usage
 * TIMER_A0 / CCP1 / CCP2 > pulses output handler
 * TIMER_A1 > USSSWLIB_TIMER_BASE_ADDRESS
 * TIMER_A2 > FUNCTIONTIMER__PERIPHERAL
 * TIMER_A3 > MODBUS TIMEOUT
 */

/*
 * Serial port usage
 * I2C HID      > EUSCI_B0
 * SPI_EEPROM   > EUSCI_B1
 */

/*** FUNCTIONS ***/

static inline void CallAddress(unsigned int addr)
{
    ((void (*)()) addr) ();               // type cast addr to function ptr, call
}


void main(void)
{
#warning "manca add / remove allarme impulsi sovrapposti nell'array"
    //uint16_t reset_source = 0x00;
    hal_system_Init();

    // Initializes the basic functionality of the system
    //TimersInit();
    //RTCC_Init();
    LCD_init();
    dispScreen = BSL_info;
    //__enable_interrupt();
    while(1)
    {
#ifdef __WATCHDOG_ENABLE__
        hal_system_WatchdogFeed();
#endif
        LcdUpdate();
        __delay_cycles(20000000);
        CallAddress( 0x7D00 );
    }
}

//--

// Port 1 interrupt service routine
/*#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    if(P2IFG & BIT4) //REED
    {
        /*if(dispScreen == disp_NewBatt)
        {
            InitBatteryEnergy(NOMINAL_BATTERY_CAPACITY_uAh);
            dispScreen = disp_AllBlack;
            dispSec = 3;
        }
        LongHoldMagnetReset();
        P2IFG &= ~BIT4;
    }
}*/

//--







