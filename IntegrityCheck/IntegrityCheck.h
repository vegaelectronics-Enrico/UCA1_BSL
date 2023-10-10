/*
 * IntefrityCheck.h
 *
 *  Created on: 7 Mar 2022
 *      Author: Enrico
 */

#ifndef INTEGRITYCHECK_INTEGRITYCHECK_H_
#define INTEGRITYCHECK_INTEGRITYCHECK_H_

#include "CRC16/CRC16.h"
#include "CRC32/CRC32.h"
#include <stdbool.h>
#include <stdint.h>
#include "UI/LCD_custom.h"
#include "Display_UI/Display_UI.h"

#define FW_CRC32_LOCATION      0x4000

#define FW_FRAM_START          0x4004
#define FW_FRAM_LENGTH         0xBF7C
#define FW_FRAM_STOP           FW_FRAM_START + FW_FRAM_LENGTH - 1

#define FW_FRAM2_START         0x10000
#define FW_FRAM2_LENGTH        0x30000
#define FW_FRAM2_STOP          FW_FRAM2_START + FW_FRAM2_LENGTH - 1

#define STEP_SIZE 1024

#warning "impostare valori definitivi"
#define FW_CHECK_SECONDS       36000
#define PAR_CHECK_SECONDS      600
#define EE_CHECK_SECONDS       600


typedef enum
{
    check_not_running,
    check_ongoing,
    check_ok,
    check_err
}crc_check_en;

typedef enum
{
    integrityOk,
    fwError,
    parError
}_integrity_en;

typedef struct
{
    bool Fw_CRC32_err;
    bool ParCfg_CRC16_EEPROM_err;
    bool ParCfg_CRC16_FRAMcpy_err;
    bool ParCfg_differ;
    bool ParMtr_CRC16_EEPROM_err;
    bool ParMtr_CRC16_FRAM_err;
}_CRCresult_t;

extern _CRCresult_t CRCresult;

//--

extern uint32_t* FwCrcStored;
extern uint32_t FwCRC32_calc;

//--

void FwCRC32_check_autostart_tick(void);
void ParCRC16_check_autostart_tick(void);
void EEprom_check_autostart_tick(void);
crc_check_en FirmwareCRC32_handle(void);
void FirmwareCRC32_start(void);
void HandleIntergrityCheck(void);
_integrity_en CheckDataIntegrity(void);

#endif /* INTEGRITYCHECK_INTEGRITYCHECK_H_ */
