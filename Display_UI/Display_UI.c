/*
 * Display_UI.c
 *
 *  Created on: 18 Oct 2021
 *      Author: Enrico
 */

#include "Display_UI.h"
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
#include <masterIncludes.h>
#include "Timers/Timers.h"
#include "IntegrityCheck/IntegrityCheck.h"
//--

bool FlashIcon = false;
uint16_t HoldMagnetSec = 0;
displayScreens dispScreen = BSL_info;
uint16_t dispSec = 11;

#define TPOS_TIMEOUT_S  240
uint8_t BackToTPos_timeout = TPOS_TIMEOUT_S;

_errView_t errViewArray[10] = {error_none};

#pragma DATA_SECTION (ResetAfterReed, ".fram_var")
bool ResetAfterReed;    //set to TRUE after a parameter modification via modbus

void LcdScreenTick(void)
{
    if(dispSec > 0)
        dispSec--;
}

//--

bool CheckResetReed(void)
{
    if(ResetAfterReed)
    {
        ResetAfterReed = false;
        return true;
    }
    return false;
}

//--

void LcdScreenHandle(void)
{
    switch(dispScreen)
    {
    case disp_AllBlack:
        LCD_all_on();
        if(dispSec == 0)
        {
            dispScreen = disp_AllClear;
            dispSec = 2;
        }
        break;
    case disp_AllClear:
        LCD_all_off();
        if(dispSec == 0)
        {
            dispScreen = BSL_info;
            dispSec = 5;
        }
        break;
    case BSL_info:
        /*if(dispSec == 0)
        {
            dispScreen = dispRtcc;
            dispSec = 3;
        }*/
        break;
    }
}

//--


void LcdUpdate(void)
{
    switch (dispScreen)
    {
    case BSL_info:
        LCD_bsl_info_view(FW_VER, FW_REV, 0xFFFFFFFF, finalFwId);
        break;
    /*case live_measure:

        ICON_clock_OFF;
        ICON_calendar_OFF;

        FlashIcon ^= 1;

        LCD_clear_row1();
        if(totVisualize == visualizeTP)
        {
            LCD_acc_write(GetScaledTotalizer(totLive.val.TPl), 3, 0, 0);
            Sign1(0);
        }
        else if(totVisualize == visualizeTN)
        {
            LCD_acc_write(GetScaledTotalizer(totLive.val.TNl), 3, 0, 0);
            Sign1(1);
        }
        else if(totVisualize == visualizeLC)
        {
            LCD_acc_write(GetScaledTotalizer(totLive.val.LCl), 3, 0, 0);
            Sign1(2);
        }

        LCD_clear_row2();
        LCD_flow_write(GetScaledFlowrate(MeterFlowrateLPH), 3);

        LcdAlarmIconsHandle();
        LcdFlowIconsHandle();
        LcdTotIconsHandle();
        LcdBatteryIconHandle();
        break;

    case FW_info:
        LCD_fw_info_view(FW_VER, FW_REV, *FwCrcStored, finalFwId);
        break;

    case dispRtcc:
        LCD_rtcc_view();
        break;

    case dispModbusCfg:
        LCD_modbus_cfg(MeterParData.str.communicationSet.ModbusAddr, BaudrateBps, MeterParData.str.communicationSet.StopBits, MeterParData.str.communicationSet.rs485Parity);
        break;

    case errCheckView:
        break;

    case EEPROM_err:
        LCD_clear();
        LCD_row2_writeErr(err_EEPROM);
        break;

    case FW_CRC_err:
        LCD_checksum_error(FwCRC32_calc);
        break;

    case PAR_CRC_err:
        LCD_parameters_error();
        break;

    case LowVolt_err:
        LCD_lowVoltage();
        break;*/
    }

}

void LcdAlarmIconsHandle(void)
{
    //gestione icone allarme su schermata principale
   /* if((sysAlarms.flags.measInvalid ||
            sysAlarms.flags.PulsPOverlap ||
            sysAlarms.flags.PulsNOverlap ||
            sysAlarms.flags.lowVoltage ||
            sysAlarms.flags.highTemp ||
            sysAlarms.flags.lowTemp ||
            sysAlarms.flags.CH1_disabled ||
            sysAlarms.flags.CH2_disabled ||
            sysAlarms.flags.highDrain ||
            sysAlarms.flags.HSPLL_unlocked ||
            sysAlarms.flags.HSPLL_otherAlarms) && FlashIcon)
        ICON_alarm_ON;
    else
        ICON_alarm_OFF;
    //--
    if(sysAlarms.flags.measNoSignal)
        ICON_us_signal_ON;
    else
        ICON_us_signal_OFF;
    //--
    if(sysAlarms.flags.PulsPOverlap)
        ICON_P1_ON;
    else
        ICON_P1_OFF;
    //--
    if(sysAlarms.flags.PulsNOverlap)
        ICON_P2_ON;
    else
        ICON_P2_OFF;*/
}

void LcdBatteryIconHandle(void)
{
    /*if(batteryStatus.batteryLev == battLev_4)
        IconBattery(lcdBatt_full);
    else if(batteryStatus.batteryLev == battLev_3)
        IconBattery(lcdBatt_L2_3);
    else if(batteryStatus.batteryLev == battLev_2)
        IconBattery(lcdBatt_L1_3);
    else if(batteryStatus.batteryLev == battLev_1)
        IconBattery(lcdBatt_empty);
    else if(batteryStatus.batteryLev == battLev_empty)
        {
        if(FlashIcon)
            IconBattery(lcdBatt_empty);
        else
            IconBattery(lcdBatt_off);
        }
    else
        IconBattery(lcdBatt_off);*/
}

void LcdFlowIconsHandle(void)
{
    /*switch(MeterParData.str.measSet.scaleFactor)
    {
    case ScaleLPH_LPH:
        IconInstUT_TB(lcdFlow_L_h);
        break;
    case ScaleLPH_M3h:
        IconInstUT_TB(lcdFlow_m3_h);
        break;
    case ScaleLPH_M3s:
        IconInstUT_TB(lcdFlow_m3_s);
        break;
    case ScaleLPH_Ls:
        IconInstUT_TB(lcdFlow_L_s);
        break;
    case ScaleLPH_gpm:
        IconInstUT_TB(lcdFlow_gpm);
        break;
    }*/
}

void LcdTotIconsHandle(void)
{
    /*switch(MeterParData.str.totSet.scaleFactor_L_tot)
    {
    case ScaleL_L:
        IconAccUT(lcdTot_L);
        break;
    case ScaleL_M3:
        IconAccUT(lcdTot_m3);
        break;
    case ScaleL_gal:
        IconAccUT(lcdTot_gal);
        break;
    case ScaleL_ft3:
        IconAccUT(lcdTot_ft3);
        break;
    case ScaleL_bbl:
        IconAccUT(lcdTot_bbl);
        break;
    }*/
}

//--

void NextTotView(void)
{
    /*if(totVisualize < (visualizeCount - 1))
        totVisualize++;
    else
        totVisualize = visualizeTP;*/
}

//--
void LongHoldMagnetInc(void)
{
    if(!(P2IN & BIT4))
    {
        HoldMagnetSec++;
    }
    else
    {
        if(HoldMagnetSec >= 10)
        {
            HoldMagnetSec = 0;
            ResetAfterReed = true;
            PMMCTL0 |= (PMMPW|PMMSWBOR);    //Changed to trigger a BOR instead of a POR since 0.19
        }
        /*if(HoldMagnetSec >= 5)
        {
            HoldMagnetSec = 0;
            dispScreen = errCheckView;
            if(ViewActiveErrors(true))
                dispSec+=2;
        }*/
        HoldMagnetSec = 0;
    }
}

//--

void LongHoldMagnetReset(void)
{
    HoldMagnetSec = 0;
}

//--
//aggiunge l'errore all'array se non è già presente
void AddActiveError(_errView_t Error)
{
    if(Error == error_none)
        return;
    uint8_t CurrentPos = 0;
    uint8_t EmptyPos;
    bool ErrPresent = false;
    while(CurrentPos < (sizeof(errViewArray) / sizeof(errViewArray[0])))
    {
        if(errViewArray[CurrentPos] == Error)
            ErrPresent = true;
        if(errViewArray[CurrentPos] == error_none)
            EmptyPos = CurrentPos;
        CurrentPos++;
    }
    if(!ErrPresent)
    {
        errViewArray[EmptyPos] = Error;
    }
}

//--
//rimuove l'errore dall'array se è presente
void RemoveActiveError(_errView_t Error)
{
    if(Error == error_none)
        return;
    uint8_t CurrentPos = 0;
    bool ErrPresent = false;
    while(CurrentPos < (sizeof(errViewArray) / sizeof(errViewArray[0])))
    {
        if(errViewArray[CurrentPos] == Error)
        {
            ErrPresent = true;
            break;
        }
        CurrentPos++;
    }
    if(ErrPresent)
    {
        errViewArray[CurrentPos] = error_none;   //canvella l'erorre
    }
}

//--
//se presente, visualizza un errore dell'array (return TRUE). Se FirstError == TRUE la ricerca viene resettata, altrimenti prosegue nell'array
bool ViewActiveErrors(bool FirstError)
{
    static uint8_t CurrentPosition;

    if(FirstError)
    {
        CurrentPosition = 0;
    }
    while(CurrentPosition < (sizeof(errViewArray) / sizeof(errViewArray[0])))
    {
        if(errViewArray[CurrentPosition] != error_none)
        {
            LCD_clear();
            LCD_row2_writeErr(errViewArray[CurrentPosition]);
            CurrentPosition++;
            return true;    //an error is currently visualized
        }
        CurrentPosition++;
    }
    return false;       //no errors are visualized
}

//--

void BackToPosTotalizer_tick(void)
{
    /*if(totVisualize != visualizeTP)
    {
        if(BackToTPos_timeout > 0)
            BackToTPos_timeout--;
        else
        {
        totVisualize = visualizeTP;
        BackToTPos_timeout = TPOS_TIMEOUT_S;
        }
    }*/
}

//--



