/*
 * CommInterfaces.c
 *
 *  Created on: 20 Sep 2021
 *      Author: Enrico
 */

#include "CommInterfaces.h"
#include <msp430.h>
#include "driverlib.h"
#include "gpio.h"
#include <eusci_b_spi.h>
#include <eusci_a_uart.h>
#include "hal.h"
#include <string.h>
#include <stdio.h>

/*
 *
 */

 const uint16_t RS485_BaudrateTable[RS485_BAUDARATE_SELECTIONS] = {1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600};

_dataBuffer_t RS485_Data = {0};
uint16_t BaudrateBps = 0;

void Init_UCB1_SPI_MasterMode(void)
{
    EUSCI_B_SPI_disable (EUSCI_B1_BASE);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    EUSCI_B_SPI_disable (EUSCI_B1_BASE);
    EUSCI_B_SPI_initMasterParam UCB1_SPI_par =
    {
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,
        USS_SMCLK_FREQ_IN_HZ,
        SPI_BRATE_EE,
        EUSCI_B_SPI_MSB_FIRST,
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW,
        EUSCI_B_SPI_3PIN
    };

    EUSCI_B_SPI_initMaster(EUSCI_B1_BASE, &UCB1_SPI_par);
    EUSCI_B_SPI_enable (EUSCI_B1_BASE);
}

//--



void UCB1_SPI_SendBytes(uint8_t* data, uint16_t nbytes)
{
    uint16_t txCount = 0;
#warning "mettere timeout"

    EUSCI_B_SPI_changeMasterClockParam UCB1_SPI_changeclock =
        {
            USS_SMCLK_FREQ_IN_HZ,
            SPI_BRATE_EE,
        };
    EUSCI_B_SPI_changeMasterClock (EUSCI_B1_BASE, &UCB1_SPI_changeclock);

    while(txCount < nbytes)
    {
        while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
        {
            ;
        }
        EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, *(data + txCount));
        txCount++;
    }
    while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
            {
                ;
            }
}

//--

void UCB1_SPI_ReceiveBytes(uint8_t* data, uint16_t nbytes)
{
    uint16_t txCount = 0;

    EUSCI_B_SPI_changeMasterClockParam UCB1_SPI_changeclock =
        {
            USS_SMCLK_FREQ_IN_HZ,
            SPI_BRATE_EE,
        };
    EUSCI_B_SPI_changeMasterClock (EUSCI_B1_BASE, &UCB1_SPI_changeclock);

#warning "mettere timeout"
    while(txCount < nbytes)
    {
        while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
        {
            ;
        }
        EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, 0);

        while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
        {
            ;
        }
        *(data + txCount) = EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);
        txCount++;
    }
}

//--

bool UCB1_SPI_SendReceiveBytes_420 (unsigned char *outData, unsigned char *inData, unsigned char length)
{
    uint16_t txCount = 0;
    uint32_t DelayCs = 0;

    DAC161S_CS_SEL;

    DelayCs = CS_DELAY_420;
    while(DelayCs > 0)
        DelayCs--;


    EUSCI_B_SPI_changeMasterClockParam UCB1_SPI_changeclock =
        {
            USS_SMCLK_FREQ_IN_HZ,
            SPI_BRATE_420,
        };
    EUSCI_B_SPI_changeMasterClock (EUSCI_B1_BASE, &UCB1_SPI_changeclock);

#warning "mettere timeout"
    while(txCount < length)
    {
        while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
        {
            ;
        }
        EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, *(outData + txCount));

        while (EUSCI_B_SPI_isBusy(EUSCI_B1_BASE))
        {
            ;
        }
        *(inData + txCount) = EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);
        txCount++;
    }

    DelayCs = CS_DELAY_420;
    while(DelayCs > 0)
        DelayCs--;
    DAC161S_CS_UNSEL;
    DelayCs = CS_DELAY_420;
    while(DelayCs > 0)
        DelayCs--;

#warning "verificare come gestire il return da funzione originale!!!"
    return true;
}

//RS485 serial port

void Init_UCA1_UART(void)
{
    _rs485Parity_en rs485ParitySet = noParity;
    uint8_t StopBitsSet = 1;
    BaudrateBps = 57600;

    EUSCI_A_SPI_disable (EUSCI_A1_BASE);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
     //EUSCI_A_SPI_disable (EUSCI_A1_BASE);
    EUSCI_A_UART_initParam UCA1_UART_par =
    {
        .selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK,
        .clockPrescalar = USS_SMCLK_FREQ_IN_HZ / BaudrateBps,
        .firstModReg = 0,
        .secondModReg = 0,
        .parity = rs485ParitySet,
        .msborLsbFirst = EUSCI_A_UART_LSB_FIRST,
        .numberofStopBits = StopBitsSet,
        .uartMode = EUSCI_A_UART_MODE,
        .overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
    };

    EUSCI_A_UART_init(EUSCI_A1_BASE, &UCA1_UART_par);
    EUSCI_A_UART_enable (EUSCI_A1_BASE);
}

//--

void Disable_UCA1_UART(void)
{
    EUSCI_A_UART_disable (EUSCI_A1_BASE);
}

void UCA1_UART_SendBytes(uint8_t* data, uint16_t nbytes)
{
    uint16_t txCount = 0;
    #warning "mettere timeout"
        while(txCount < nbytes)
        {
            while (EUSCI_A_UART_queryStatusFlags(EUSCI_A1_BASE, EUSCI_A_UART_BUSY))
            {
                ;
            }
            EUSCI_A_UART_transmitData(EUSCI_A1_BASE, *(data + txCount));
            txCount++;
        }
        while (EUSCI_A_UART_queryStatusFlags(EUSCI_A1_BASE, EUSCI_A_UART_BUSY))
        {
            ;
        }
}

//--
void RS485_StartReceive(void)
{
    RS485_DE_l;
    RS485_RE_l;
}

//--

void RS485_StopReceive(void)
{
    RS485_DE_l;
    RS485_RE_h;
}

//--

void RS485_StartTransmit(void)
{
    RS485_DE_h;
}

//--

void RS485_StopTransmit(void)
{
    RS485_DE_l;
}

//--

uint16_t GetBaudrateSelection(void)
{
    return BaudrateBps;
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch( __even_in_range( UCA1IV, USCI_UART_UCTXCPTIFG ) )
    {
    case USCI_NONE: // Vector 0: No interrupts
        break;
    case USCI_UART_UCRXIFG:  // Vector 2: Interrupt Source: Data received; Interrupt Flag: UCRXIFG; Interrupt Priority: Highest
        RS485_Data.Data[RS485_Data.NextPos] = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);
        RS485_Data.NextPos++;
        if(RS485_Data.NextPos >= SER_BUFFER)
        {
            RS485_Data.NextPos = 0;
            RS485_Data.OverflowFlag = true;
        }
        //Modbus_ResetTimer();
        LPM3_EXIT;
        break;
    case USCI_UART_UCTXIFG:  // Vector 4: Interrupt Source: Transmit buffer empty; Interrupt Flag: UCTXIFG; Interrupt Priority: Lowest
        break;
    case USCI_UART_UCSTTIFG:  // Vector 6:
        break;
    case USCI_UART_UCTXCPTIFG:  // Vector 8:
        break;
    }
}




