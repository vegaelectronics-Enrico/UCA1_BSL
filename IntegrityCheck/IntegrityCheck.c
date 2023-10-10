/*
 * IntegrityCheck.c
 *
 *  Created on: 7 Mar 2022
 *      Author: Enrico
 */

#include "IntegrityCheck.h"
#include "System/SysStatus.h"
#include "EEmemory/25XXXXX.h"

_CRCresult_t CRCresult = {0};
bool CheckOngoing = false;
uint32_t* FwCrcStored = (uint32_t*)FW_CRC32_LOCATION;
uint32_t FwCRC32_calc = 0;
uint32_t NextAddrCRC32 = 0;
uint32_t ByteFirmwareCount = 0;
uint8_t* ByteFwPtr;

#warning "impostare valori definitivi"
uint16_t FwCrc_autoCnt = 60;         //starts 60s after reset
uint16_t ParCrc_autoCnt = 60;       //starts 60s after reset
uint16_t EEprom_autoCnt = 30;

bool StartFwCheck = false;
bool StartParCheck = false;
bool StartEEpromCheck = false;

#warning "sostituire conteggio secondi con evento orologio?"

void FwCRC32_check_autostart_tick(void)
{
    if(FwCrc_autoCnt > 0)
        FwCrc_autoCnt--;
    else
    {
        FwCrc_autoCnt = FW_CHECK_SECONDS;
        StartFwCheck = true;
    }
}

//--

void ParCRC16_check_autostart_tick(void)
{
    if(ParCrc_autoCnt > 0)
        ParCrc_autoCnt--;
    else
    {
        ParCrc_autoCnt = PAR_CHECK_SECONDS;
        StartParCheck = true;
    }
}

//--

void EEprom_check_autostart_tick(void)
{
    if(EEprom_autoCnt > 0)
        EEprom_autoCnt--;
    else
    {
        EEprom_autoCnt = EE_CHECK_SECONDS;
        StartEEpromCheck = true;
    }
}

//--

void HandleIntergrityCheck(void)
{
    if(CheckOngoing)
    {
        FirmwareCRC32_handle();
        return;
    }
    if(StartFwCheck)
    {
        FirmwareCRC32_start();
        StartFwCheck = false;
    }
    if(StartParCheck)
    {
        //il controllo setta i flag in caso di errore CRC in EEPROM o FRAM e controlla che la configurazione presente in memoria attiva corrisponda ad EEPROM
        //ReplaceUSS_UserCfg(true);
        //Replace_MeterPar(true);
        StartParCheck = false;
    }
    if(StartEEpromCheck)
    {
        StartEEpromCheck = false;
        if(EEpromCheckErrFunctionality())
        {
            //sysAlarms.flags.eepromErr = true;
            AddActiveError(err_EEPROM);
        }
        else
        {
            //sysAlarms.flags.eepromErr = false;
            RemoveActiveError(err_EEPROM);
        }
    }
}

crc_check_en FirmwareCRC32_handle(void)
{
    uint32_t ResidualStep = 0;

    if(CheckOngoing)
    {
        if((NextAddrCRC32 >= FW_FRAM_START) && (NextAddrCRC32 <= FW_FRAM_STOP))
        {
            if((NextAddrCRC32 + STEP_SIZE - 1) <= FW_FRAM_STOP)
            {
                FwCRC32_calc = crc32_engine( FwCRC32_calc, ByteFwPtr, STEP_SIZE );
                NextAddrCRC32 += STEP_SIZE;
                ByteFwPtr = (uint8_t*)NextAddrCRC32;
            }
            else
            {
                ResidualStep = FW_FRAM_STOP - NextAddrCRC32 + 1;
                FwCRC32_calc = crc32_engine( FwCRC32_calc, ByteFwPtr, ResidualStep );
                NextAddrCRC32 = FW_FRAM2_START;
                ByteFwPtr = (uint8_t*)NextAddrCRC32;
            }
        }

        else if((NextAddrCRC32 >= FW_FRAM2_START) && (NextAddrCRC32 <= FW_FRAM2_STOP))
        {
            if((NextAddrCRC32 + STEP_SIZE - 1) <= FW_FRAM2_STOP)
            {
                FwCRC32_calc = crc32_engine( FwCRC32_calc, ByteFwPtr, STEP_SIZE );
                NextAddrCRC32 += STEP_SIZE;
                ByteFwPtr = (uint8_t*)NextAddrCRC32;
            }
            else
            {
                ResidualStep = FW_FRAM2_STOP - NextAddrCRC32 + 1;
                FwCRC32_calc = crc32_engine( FwCRC32_calc, ByteFwPtr, ResidualStep );
            }
        }
        else if(NextAddrCRC32 > FW_FRAM2_STOP)
        {
            CheckOngoing = false;

            if(*FwCrcStored != FwCRC32_calc)
            {
                CRCresult.Fw_CRC32_err = true;
                return check_err;
            }
            else
            {
                CRCresult.Fw_CRC32_err = false;
                return check_ok;
            }
        }
        return check_ongoing;
    }
    return check_not_running;
}

//--

void FirmwareCRC32_start(void)
{
    FwCRC32_calc = crc32_reset();
    ByteFirmwareCount = 0;
    NextAddrCRC32 = FW_FRAM_START;
    ByteFwPtr = (uint8_t*)NextAddrCRC32;
    CheckOngoing = true;
}

//--

_integrity_en CheckDataIntegrity(void)
{
    if(!CRCresult.Fw_CRC32_err && !CRCresult.ParCfg_CRC16_EEPROM_err && !CRCresult.ParCfg_CRC16_FRAMcpy_err && !CRCresult.ParCfg_differ && !CRCresult.ParMtr_CRC16_EEPROM_err && !CRCresult.ParMtr_CRC16_FRAM_err)
    {
        //sysAlarms.flags.fwCrcErr = false;
        //sysAlarms.flags.parCrcErr = false;
        return integrityOk;
    }
    else if(CRCresult.Fw_CRC32_err)
    {
        //sysAlarms.flags.fwCrcErr = true;
        return fwError;
    }
    else
    {
        //sysAlarms.flags.parCrcErr = true;
        return parError;
    }
}

