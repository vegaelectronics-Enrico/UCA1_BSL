/*UCA1 ultrasonic water meter
 * based on MSP430FR6047
 */

#include "main.h"
#include <stdint.h>
#include "hal.h"
#include <stdio.h>
#include <string.h>
#include "Timers/Timers.h"
#include "EEmemory/25XXXXX.h"
#include "CommInt/CommInterfaces.h"
#include "UI/LCD_custom.h"
#include "Display_UI/Display_UI.h"
#include "CRC32/CRC32.h"
#include "CRC16/CRC16.h"
#include <intrinsics.h>
#include <IntegrityCheck/IntegrityCheck.h>
#include <System/SysStatus.h>

bool HandleLine(void);

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

#define TIMEOUT_COUNT 20

#define ARRAY_SIZE  250
char RxArray[ARRAY_SIZE] = {0};
uint16_t DecodedFramData;
uint8_t RxCount = 0;
uint8_t DelimitersRec = 0;
uint8_t FirstDelimiterPos = 0;
uint8_t SecondDelimiterPos = 0;
uint16_t NextPacketSize = 0;
uint32_t FramAddress;
uint16_t ReceivedCRC16 = 0;
uint16_t *FramIntAddressPointer;
uint32_t BytesCounterWR = 0;
bool ValidAddressReceived = false;
bool OpStarted = false;
char AddressString[10];
char Crc16String[5];
char AddressHexString[10];
char SingleIntHex[6];
uint8_t TimeputToMainFw = 5;
uint16_t TimeoutsCount = TIMEOUT_COUNT;

typedef union
{
    uint16_t CRC16;
    struct
    {
        uint8_t CRC_L;
        uint8_t CRC_H;
    }bytes;
}CRCcalc_t;

CRCcalc_t CRCcalc = {0};

//--

/*** FUNCTIONS ***/

static inline void CallAddress(unsigned int addr)
{
    ((void (*)()) addr) ();               // type cast addr to function ptr, call
}


void main(void)
{
    char* pEnd;

#warning "manca add / remove allarme impulsi sovrapposti nell'array"
    //uint16_t reset_source = 0x00;
    hal_system_Init();
    /*
     * if the system booted after a modbus or HSPLL supervisor BOR jump immediately to the main program; they can't be both TRUE
     * the flags must be cleared by the main program
     */
    if((SharedCtrlStruct->RebootFromModbus || SharedCtrlStruct->HSPLL_resetBOR) && (SharedCtrlStruct->RebootFromModbus != SharedCtrlStruct->HSPLL_resetBOR))
        CallAddress( 0x7C80 );
    /*
     * if both the log completed and log started flags are set clear them because is not possible
     */
    if(SharedCtrlStruct->BSL_loadStarted && SharedCtrlStruct->BSL_loadCompleted)
    {
        SharedCtrlStruct->BSL_loadStarted = false;
        SharedCtrlStruct->BSL_loadCompleted = false;
    }

    //--
    // Initializes the basic functionality of the system
    //TimersInit();
    //RTCC_Init();
    LCD_init();
    dispScreen = BSL_info;
    LcdUpdate();
    Init_UCA1_UART();
    RS485_StartReceive();
    bool RxTimeout = false;
    bool DecLineRes = true;
    OpStarted = false;
    while((TimeoutsCount > 0) && DecLineRes && (RxCount < ARRAY_SIZE))
    {
        RxArray[RxCount] = EUSCI_A_UART_receiveData_timeout(EUSCI_A1_BASE, &RxTimeout);
        if((RxArray[RxCount] == '#') && (DelimitersRec < 2))
        {
            DelimitersRec++;
            if(DelimitersRec == 1)
            FirstDelimiterPos = RxCount;
            if(DelimitersRec == 2)
            {
                NextPacketSize = atoi(&RxArray[FirstDelimiterPos+1]);
                SecondDelimiterPos = RxCount;
            }
            RxCount++;
        }
        else
        {
            if(RxCount == (NextPacketSize + SecondDelimiterPos + 4))
            {
                //check the received lines(s) CRC16
                ReceivedCRC16 = 0;
                memset(Crc16String, 0, 5);
                strncpy(Crc16String, &RxArray[SecondDelimiterPos + NextPacketSize + 1], 4);
                ReceivedCRC16 = strtol(Crc16String, &pEnd, 16);
                GenerateCRC16((uint8_t*)RxArray, SecondDelimiterPos + NextPacketSize + 1, &CRCcalc.bytes.CRC_H, &CRCcalc.bytes.CRC_L);
                RS485_StopReceive();
                RS485_StartTransmit();
                __delay_cycles(5000);
                if(CRCcalc.CRC16 == ReceivedCRC16)
                {
                    if(!OpStarted)   //must be a dedicated RAM flag because the FRAM one will not be cleared if an attempt fails
                    {
                        LCD_row1_writeLoad();
                        OpStarted = true;
                    }

                    DecLineRes = HandleLine();   //CRC OK: handle the received line
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'O');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'K');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'\n');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'\r');
                    TimeoutsCount = TIMEOUT_COUNT;
                }
                else
                {
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'C');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'R');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'C');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'E');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'R');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'R');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'\n');
                    EUSCI_A_UART_transmitData(EUSCI_A1_BASE, (char)'\r');
                }
                RS485_StopTransmit();
                RS485_StartReceive();
                RxCount = 0;
                DelimitersRec = 0;
                memset(RxArray, 0, ARRAY_SIZE);
            }
            else
                if(!RxTimeout)
                    RxCount++;
        }
        if(RxTimeout)
        {
            TimeoutsCount--;
            LCD_progress_bar_row1(((float)TimeoutsCount * 100) /  TIMEOUT_COUNT);
        }

        EXT_WDT_TOGGLE;
        hal_system_WatchdogFeed();
    }
    if(SharedCtrlStruct->BSL_loadCompleted || !SharedCtrlStruct->BSL_loadStarted)
    {
        CallAddress( 0x7C80 );
    }
    PMM_trigBOR();
#warning "gestire limitazione di iterazioni e poi entrare in uno stato di basso consumo per evitare di scaricare batterie in strumenti che si bloccano dopo caricamento fallito"
}

/*
 * handle the received line.
 * First thing: check the CRC16 (last 2 bytes): if not matching > return ERR.
 * If is an address (@XXXXX): update the FRAM address and pointer
 * Otherwise: increment the FRAM address & pointer and write the received bytes
 *
 */
bool HandleLine(void)
{
    char* pEnd;
    if(RxArray[SecondDelimiterPos + 1] == '@')
    {
        //is an address
        strncpy(AddressString, &RxArray[SecondDelimiterPos + 2], NextPacketSize - 1);
        FramAddress = strtol(AddressString, &pEnd, 16);
        FramIntAddressPointer = (uint16_t*)FramAddress;
        ValidAddressReceived = true;
    }
    else if(RxArray[SecondDelimiterPos + 1] == 'q')
    {
        SharedCtrlStruct->BSL_loadStarted = false;
        SharedCtrlStruct->BSL_loadCompleted = true;
        TimeoutsCount = 0;
        return true;
    }
    else
    {
        //is data to be written
        if(ValidAddressReceived)
        {
            DecodedFramData = 0;
            uint8_t i;
            for(i = 0; i < NextPacketSize; i+=4)
            {
                //copy and exchange the bytes order in the word
                strncpy(&SingleIntHex[0], &RxArray[i + SecondDelimiterPos + 3], 2);
                strncpy(&SingleIntHex[2], &RxArray[i + SecondDelimiterPos + 1], 2);
                DecodedFramData = strtol(SingleIntHex, &pEnd, 16);
                if((((uint32_t)FramIntAddressPointer >= FW_FRAM_START) && ((uint32_t)FramIntAddressPointer <= FW_FRAM_STOP)) || (((uint32_t)FramIntAddressPointer >= FW_FRAM2_START) && ((uint32_t)FramIntAddressPointer <= FW_BOOT_WR_BLOCK2_STOP)))
                    *FramIntAddressPointer = DecodedFramData;
                FramIntAddressPointer++;
                if(!SharedCtrlStruct->BSL_loadStarted)
                {
                    SharedCtrlStruct->BSL_loadStarted = true;   //mark that the FRAM write operation has begun
                    SharedCtrlStruct->BSL_loadCompleted = false;
                }
            }
            __delay_cycles(1);
        }
        else
        {
            __delay_cycles(1);
        }
    }
    return true;
}


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







