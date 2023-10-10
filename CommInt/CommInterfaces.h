/*
 * CommInterfaces.h
 *
 *  Created on: 20 Sep 2021
 *      Author: Enrico
 */

#ifndef COMMINTERFACES_H_
#define COMMINTERFACES_H_


#include <stdint.h>
#include <stdbool.h>
//#include "Modbus.h"
#include <driverlib.h>


#define SPI_BRATE_EE        1000000
#define SPI_BRATE_420       500

#define SER_BUFFER          256

//#define USE_DMA_RX
#define RS485_BAUDARATE_SELECTIONS  8
#define RS485_DEFAULT_BAUDR_SEL     3   //3 = 9600bps

#define DATA_BUFFER_SIZE    MODBUS_BUFFER_BYTES

#define CS_DELAY_420    3000

typedef enum
{
    noParity = EUSCI_A_UART_NO_PARITY,
    oddParity = EUSCI_A_UART_ODD_PARITY,
    evenParity = EUSCI_A_UART_EVEN_PARITY,
}_rs485Parity_en;

typedef struct
{
    uint8_t ModbusAddr;
    uint8_t RS485_baudrate;
    _rs485Parity_en rs485Parity;
    uint8_t StopBits;
}_communicationSet_t;

/*
 * I/O defintiions
 */
#define RS485_RE_h      P5OUT |= BIT5
#define RS485_RE_l      P5OUT &= ~BIT5
#define RS485_DE_h      P4OUT |= BIT7
#define RS485_DE_l      P4OUT &= ~BIT7

/*
 * data types
 */

typedef struct
{
    //uint8_t Data [DATA_BUFFER_SIZE];
    uint8_t Data [SER_BUFFER];
    uint16_t NextPos;
#ifdef USE_DMA_RX
    uint16_t NextPosPrev;
#endif
    bool OverflowFlag;
}_dataBuffer_t;

extern _dataBuffer_t RS485_Data;

extern uint16_t BaudrateBps;
/*
 * functions prototypes
 */

void Init_UCB1_SPI_MasterMode(void);
void UCB1_SPI_SendBytes(uint8_t* data, uint16_t nbytes);
void UCB1_SPI_ReceiveBytes(uint8_t* data, uint16_t nbytes);
bool UCB1_SPI_SendReceiveBytes_420 (unsigned char *outData, unsigned char *inData, unsigned char length);
void Init_UCA1_UART(void);
void Disable_UCA1_UART(void);
void UCA1_UART_SendBytes(uint8_t* data, uint16_t nbytes);
void RS485_StartReceive(void);
void RS485_StopReceive(void);
void RS485_StartTransmit(void);
void RS485_StopTransmit(void);
uint16_t GetBaudrateSelection(void);


#endif /* COMMINTERFACES_H_ */
