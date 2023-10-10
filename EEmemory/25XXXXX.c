/*
 * 25XXXXX.c
 *
 *  Created on: 20 Sep 2021
 *      Author: Enrico
 */

#include "25XXXXX.h"
#include <msp430.h>
#include "CommInt/CommInterfaces.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "CRC16/CRC16.h"
#include <System/SysStatus.h>

/*
 * note sulla gestione dell'EEPROM con protezione
 *
 * EnableWP_EES(...) seleziona il range di memoria da proteggere; se diverso da "unprotect_all" anche il pin WP viene abilitato
 * Una scheda nuova non ha EEPROM protetta e WP è disabilitato
 * E' possibile quind accedere in scrittura senza jumper prima di attivare la protezione
 * In seguito sarà necessario inserire il jumper per disattivarla
 *
 */

_localPage_t localPageWR = {0};
_localPage_t localPageRD = {0};
_VarSeparate_t VarEE = {0};
_EES_sr_t EES_sr = {0};

//--
void Init_EES_EEPROM (void)
{
    //uint8_t SingleCommand;
    EE_ON;
    CS_EES_unsel;
    uint32_t delay = 50000;
    while(delay > 0)
    {
        delay--;
    }
    /*CS_EES_sel;
    SingleCommand = intrEE_WREN;
    UCB1_SPI_SendBytes(&SingleCommand, 1);
    CS_EES_unsel;
    delay = 100000;
    while(delay > 0)
    {
        delay--;
    }*/
    CS_EES_sel;
    ReadEES_SR();
    CS_EES_unsel;
    WP_EES_prot;
}

//--

void EEPROM_PageRead(uint16_t PageNumber)
{
    _eeAddrSeparate_t addrStartRead;
    addrStartRead.Addr16 = (PageNumber * EE_PAGE_SIZE);
    uint8_t sendBuffer_EE [3];
    memset(localPageRD.PageData, 0, EE_PAGE_SIZE);
    sendBuffer_EE[0] = instrEE_READ;
    sendBuffer_EE[1] = addrStartRead.bytes.msB;
    sendBuffer_EE[2] = addrStartRead.bytes.lsB;
    CS_EES_sel;
    UCB1_SPI_SendBytes(sendBuffer_EE, 3);
    UCB1_SPI_ReceiveBytes(localPageRD.PageData, EE_PAGE_SIZE);
    CS_EES_unsel;
}

//--

bool EEPROM_PageWrite(uint16_t PageNumber)
{
    /*if(sysAlarms.flags.turnOffVoltage)
        return false;
    if(PageNumber > EE_NPAGES - 1)
        return false;*/
    uint32_t delay = 15000;
    _eeAddrSeparate_t addrStartWrite;
    addrStartWrite.Addr16 = (PageNumber * EE_PAGE_SIZE);
    uint8_t sendBuffer_EE [3];
    sendBuffer_EE[0] = instrEE_WRITE;
    sendBuffer_EE[1] = addrStartWrite.bytes.msB;
    sendBuffer_EE[2] = addrStartWrite.bytes.lsB;

    EES_EnableLatch();

    CS_EES_sel;
    UCB1_SPI_SendBytes(sendBuffer_EE, 3);
    UCB1_SPI_SendBytes(localPageWR.PageData, EE_PAGE_SIZE);
    WP_EES_prot;
    CS_EES_unsel;

    while(delay > 0)
    {
        delay--;
    }

    memset(localPageRD.PageData, 0, EE_PAGE_SIZE);
    EEPROM_PageRead(PageNumber);
    if(memcmp(localPageWR.PageData, localPageRD.PageData, EE_PAGE_SIZE) == 0)
        return true;
    return false;

}

//--
//da chimaare prima di ogni scrittura (anche bit NV di status)
void EES_EnableLatch(void)
{
    uint8_t SingleCommand;

    WP_EES_unprot;
    CS_EES_sel;
    SingleCommand = intrEE_WREN;
    UCB1_SPI_SendBytes(&SingleCommand, 1);
    CS_EES_unsel;
}



//scrive N bytes suddividendo la scrittura in pagine; TRUE = write OK
bool EEPROM_Write_Par(uint16_t PageStart, uint8_t* Data, uint16_t Size)
{
    uint16_t BytesToWrite;
    uint16_t PageN = PageStart;
    uint8_t* DataPtr = Data;
    BytesToWrite = Size;
    uint16_t ErrCnt = 0;


    while(BytesToWrite >= EE_PAGE_SIZE)
    {
        memcpy(localPageWR.PageData, DataPtr, EE_PAGE_SIZE);
        if(!EEPROM_PageWrite(PageN))
            ErrCnt++;
        DataPtr += EE_PAGE_SIZE;
        BytesToWrite -= EE_PAGE_SIZE;
        PageN++;
    }
    if(BytesToWrite > 0)
    {
        memcpy(localPageWR.PageData, DataPtr, BytesToWrite);
        if(!EEPROM_PageWrite(PageN))
            ErrCnt++;
        BytesToWrite = 0;
    }
    if(ErrCnt == 0)
        return true;
    return false;
}

//--

void EEPROM_Read_Par(uint16_t PageStart, uint8_t *Dest, uint16_t Size)
{
    uint16_t BytesToRead = Size;
    uint16_t PageN = PageStart;
    uint8_t* DestPtr = Dest;

    while(BytesToRead >= EE_PAGE_SIZE)
    {
        EEPROM_PageRead(PageN);
        memcpy(DestPtr, localPageRD.PageData, EE_PAGE_SIZE);
        DestPtr += EE_PAGE_SIZE;
        BytesToRead -= EE_PAGE_SIZE;
        PageN++;
    }
    if(BytesToRead > 0)
    {
        EEPROM_PageRead(PageN);
        memcpy(DestPtr, localPageRD.PageData, BytesToRead);
        BytesToRead = 0;
    }
}

void TestPageRead(uint16_t PageNumber)
{
    memset(localPageRD.PageData, 0, EE_PAGE_SIZE);
    EEPROM_PageRead(PageNumber);
}

//--
//reads a used page: if all the values are equal to 0x00 or 0xFF returns TRUE (EEPROM not working), TRUE othrwise
bool EEpromCheckErrFunctionality(void)
{
    TestPageRead(METER_PAR_PAGE_START);
    bool Err00 = true;
    bool ErrFF = true;
    uint8_t ByteCnt;
    for(ByteCnt=0; ByteCnt<EE_PAGE_SIZE; ByteCnt++)
    {
        if(localPageRD.PageData[ByteCnt] != 0x00)
            Err00 = false;
        if(localPageRD.PageData[ByteCnt] != 0xFF)
            ErrFF = false;
    }
    return (Err00 | ErrFF);
}

//--

void TestPageWrite(uint16_t PageNumber)
{
    memset(localPageWR.PageData, 0, EE_PAGE_SIZE);
    localPageWR.PageData[0] = 0xCD;
    localPageWR.PageData[1] = 0xEF;
    localPageWR.PageData[30] = 0x55;
    localPageWR.PageData[31] = 0x66;
    EEPROM_PageWrite(PageNumber);
}

//--

void ReadEES_SR(void)
{
    uint8_t SingleCommand = instrEE_RDSR;
    CS_EES_sel;
    uint16_t delay = 100;
    while(delay > 0)
    {
        delay--;
    }
    UCB1_SPI_SendBytes(&SingleCommand, 1);
    UCB1_SPI_ReceiveBytes(&EES_sr.val, 1);
    CS_EES_unsel;
    delay = 100;
    while(delay > 0)
    {
        delay--;
    }
}

//--

void WriteEES_SR(void)
{
    uint8_t SingleCommand = instrEE_WRSR;
    CS_EES_sel;
    WP_EES_unprot;
    uint32_t delay = 100;
    while(delay > 0)
    {
        delay--;
    }
    UCB1_SPI_SendBytes(&SingleCommand, 1);
    UCB1_SPI_SendBytes(&EES_sr.val, 1);
    CS_EES_unsel;
    delay = 10000;
    while(delay > 0)
    {
        delay--;
    }
    WP_EES_prot;

    delay = 100;
    while(delay > 0)
    {
        delay--;
    }
}

/*
 * sets the protection range and if different from "unprotected" enables the WP pin
 */
bool EnableWP_EES(_EES_protection_range ProtectionRange)
{

    ReadEES_SR();
    switch(ProtectionRange)
    {
    case protect_none:
        EES_sr.bits.bp1 = false;
        EES_sr.bits.bp0 = false;
        EES_sr.bits.wpen = false;
        break;
    case protect_upper_1_4:
        EES_sr.bits.bp1 = false;
        EES_sr.bits.bp0 = true;
        EES_sr.bits.wpen = true;
        break;
    case protect_upper_1_2:
        EES_sr.bits.bp1 = true;
        EES_sr.bits.bp0 = false;
        EES_sr.bits.wpen = true;
        break;
    case protect_all:
        EES_sr.bits.bp1 = true;
        EES_sr.bits.bp0 = true;
        EES_sr.bits.wpen = true;
        break;
    }

    EES_EnableLatch();
    WriteEES_SR();
    ReadEES_SR();
    return EES_sr.bits.wpen;
}

//--

_EES_protection_range CheckEES_protection(void)
{
    if(EES_sr.bits.bp1 && EES_sr.bits.bp0)
        return protect_all;
    else if(!EES_sr.bits.bp1 && EES_sr.bits.bp0)
        return protect_upper_1_4;
    else if(EES_sr.bits.bp1 && !EES_sr.bits.bp0)
        return protect_upper_1_2;
    return protect_none;
}

