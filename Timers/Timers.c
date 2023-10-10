/*
 * Timers.c
 *
 *  Created on: 6 Sep 2021
 *      Author: Enrico
 */
#include "Timers.h"
#include <msp430.h>
#include "driverlib.h"
#include "gpio.h"
#include "Display_UI/Display_UI.h"
#include "IntegrityCheck/IntegrityCheck.h"
#include <System/SysStatus.h>
#include "CommInt/DAC161S.h"
#include "main.h"
//--

#pragma DATA_SECTION (RTCC_Calendar, ".other_fram")
Calendar RTCC_Calendar;

_oneSecFlags_t oneSecFlags = {0};
bool MidnightExecuted = false;
RTCCstatus_t RTCCstatus = {0};
//--

void TimersInit(void)
{
    RTC_C_holdClock(RTC_C_BASE);
    RTC_C_initCalendar(RTC_C_BASE, &RTCC_Calendar, RTC_C_FORMAT_BINARY);
    RTC_C_enableInterrupt(RTC_C_BASE, RTC_C_CLOCK_READ_READY_INTERRUPT);
    RTC_C_startClock(RTC_C_BASE);
}

//--

void OneSecondEvents(void)
{
    oneSecFlags.LCD_update = true;
    //oneSecFlags.RefreshVoltTemp = true;
    //PulseHandler(MeterFlowrateLPH / 3.6, 1000.0);
    //TotalizerIncrememnt(MeterFlowrateLPH, 1.0);
    LcdScreenTick();
    LongHoldMagnetInc();
    //FwCRC32_check_autostart_tick();
    //ParCRC16_check_autostart_tick();
    //EEprom_check_autostart_tick();
    //Modbus_disable_wr_tick();
    //BattEnergy_tick();
    //loopTick();
    //AutoChannelReArm_tick();
    //TimeoutUSSreinit_tick();
    //BackToPosTotalizer_tick();
    //AutostartPeakSearch_tick();
    if(TimeputToMainFw > 0)
        TimeputToMainFw--;
}

//--

void RTCC_Init(void)
{
    //check the RTCC data to be coherent
    if(!CheckValidCalendar(RTCC_Calendar))
    {
        RTCC_Calendar.Hours=0;
        RTCC_Calendar.Minutes=0;
        RTCC_Calendar.Seconds=0;
        RTCC_Calendar.DayOfMonth=1;
        RTCC_Calendar.Month=1;
        RTCC_Calendar.Year=FW_YEAR + 2000;
        RTCC_Calendar.DayOfWeek = dayofweek((int)RTCC_Calendar.DayOfMonth, (int)RTCC_Calendar.Month, (int)RTCC_Calendar.Year);
    }
    else
    {
        RTCCstatus.bits.ValidInFram = true;
    }

    RTC_C_initCalendar(RTC_C_BASE,&RTCC_Calendar,RTC_C_FORMAT_BINARY);
    RTC_C_enableInterrupt(RTC_C_BASE,RTC_C_CLOCK_READ_READY_INTERRUPT);
    RTC_C_startClock(RTC_C_BASE);
}

void RTCC_Get(void)
{
    RTCC_Calendar=RTC_C_getCalendarTime(RTC_C_BASE);
}


bool RTCCisValid(void)
{
    if (RTCCstatus.bits.ManuallyUpdated || RTCCstatus.bits.ValidInFram)
        return true;
    else
        return false;
}

unsigned char CheckValidCalendar(Calendar CheckCalendar)
{
    if(CheckCalendar.Hours>23 ||
                CheckCalendar.Minutes>59 ||
                CheckCalendar.Seconds>59 ||
                CheckCalendar.Month>12 ||
                CheckCalendar.Month<1 ||
                CheckCalendar.DayOfMonth>31 ||
                CheckCalendar.DayOfMonth<1 ||
                CheckCalendar.DayOfWeek>6 ||
                CheckCalendar.Year>2099 ||
                CheckCalendar.Year<2000)
        return 0;
    else
        return 1;
}


//return TRUE if A>=B
bool CompareCalendars (Calendar A, Calendar B)
{
    if(A.Year>B.Year)
        return true;
    if(A.Year<B.Year)
        return false;
    //same year:
    if(A.Month > B.Month)
        return true;
    if(A.Month < B.Month)
        return false;
    //same month
    if(A.DayOfMonth > B.DayOfMonth)
        return true;
    if(A.DayOfMonth < B.DayOfMonth)
        return false;
    //same day of month
    if(A.Hours > B.Hours)
        return true;
    if(A.Hours < B.Hours)
        return false;
    //same hour
    if(A.Minutes > B.Minutes)
        return true;
    if(A.Minutes < B.Minutes)
        return false;
    //same minute
    if(A.Seconds > B.Seconds)
        return true;
    return false;
}

//--
// returns the day of month corrected depending on the month and year; if == 0 or > 31 returns 0 to indicate an error
unsigned char AdjustDayOfMmonth(unsigned char Day, unsigned char Month, unsigned int Year)
{
    unsigned char RetDay = Day;

    if(RetDay == 0 || RetDay > 31)
        return 0;

    if(Month == 2)  //feb
    {
        if(Year % 4 == 0)   //bisestile
        {
            if(RetDay > 29)
                RetDay = 29;
        }
        else
        {
            if(RetDay > 28)
                RetDay = 28;
        }
    }

    else if (Month == 1 || Month == 3 || Month == 5 || Month == 7 || Month == 8 || Month == 10 || Month == 12)
    {
        if(RetDay > 31)
            RetDay = 31;
    }

    else if (Month == 4 || Month == 6 || Month == 9 || Month == 11)
    {
        if(RetDay > 30)
            RetDay = 30;
    }

    else
        return 0;
    return RetDay;
}

//--

unsigned char dayofweek(int d, int m, int y)
{
    static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return (unsigned char)( y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}


//--

void HandleMidnightEvents(void)
{

    if(RTCC_Calendar.Hours != 0)
        return;
    if(RTCC_Calendar.Minutes != 0)
        return;
    if(RTCC_Calendar.Seconds != 0)
    {
        MidnightExecuted = false;
        return;
    }
    if(!MidnightExecuted)
    {
        /*
         * events to serve here
         */
                                      //refresh all the FLASH parameters
        MidnightExecuted = true;
    }

}

//--

/*
 * 3.19
 */

void PushRTCC(Calendar NewCalendar)
{
    RTC_C_initCalendar(RTC_C_BASE, &NewCalendar,
                           RTC_C_FORMAT_BINARY);
                           RTC_C_enableInterrupt(RTC_C_BASE,
                           RTC_C_CLOCK_READ_READY_INTERRUPT);
                           RTC_C_startClock(RTC_C_BASE);
   RTCC_Get();
}

//




#pragma vector=RTC_VECTOR
__interrupt void rtc_isr(void)
{
    switch ( __even_in_range( RTCIV, RTCIV_RT1PSIFG ) )
    {
        case RTCIV_NONE:                    // No interrupts
            break;
        case RTCIV_RTCOFIFG:                // RTCOFIFG
            break;
        case RTCIV_RTCRDYIFG:               // RTCRDYIFG
            RTCC_Get();
            OneSecondEvents();
            RTC_C_enableInterrupt(RTC_C_BASE, RTC_C_CLOCK_READ_READY_INTERRUPT);
            break;
        case RTCIV_RTCTEVIFG:               // RTCEVIFG
            break;
        case RTCIV_RTCAIFG:                 // RTCAIFG
            break;
        case RTCIV_RT0PSIFG:                // RT0PSIFG
            break;
        case RTCIV_RT1PSIFG:                // RT1PSIFG
            break;
        default: break;
    }
    LPM3_EXIT;
}



