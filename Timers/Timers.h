/*
 * Timers.h
 *
 *  Created on: 6 Sep 2021
 *      Author: Enrico
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "rtc_c.h"
#include <stdlib.h>
#include <stdbool.h>

void OneSecondEvents(void);
void TimersInit(void);

extern Calendar RTCC_Calendar;

typedef struct
{
    bool LCD_update;
    bool RefreshVoltTemp;
}_oneSecFlags_t;

//--

extern _oneSecFlags_t oneSecFlags;

//--

typedef union
{
    struct
    {
        unsigned ValidInFram       :1;
        unsigned ManuallyUpdated   :1;
        unsigned NU                :6;
    }bits;
    unsigned char val;
}RTCCstatus_t;

extern RTCCstatus_t RTCCstatus;

void RTCC_Init(void);
void RTCC_Get(void);
unsigned char CheckValidCalendar(Calendar CheckCalendar);
bool CompareCalendars (Calendar A, Calendar B);
unsigned char AdjustDayOfMmonth(unsigned char Day, unsigned char Month, unsigned int Year);
unsigned char dayofweek(int d, int m, int y);
bool RTCCisValid(void);
void HandleMidnightEvents(void);
void PushRTCC(Calendar NewCalendar);

//--


#endif /* TIMERS_H_ */
