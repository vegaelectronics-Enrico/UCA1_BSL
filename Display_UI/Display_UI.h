/*
 * Display_UI.h
 *
 *  Created on: 18 Oct 2021
 *      Author: Enrico
 */

#ifndef DISPLAY_UI_DISPLAY_UI_H_
#define DISPLAY_UI_DISPLAY_UI_H_

#include <stdbool.h>
#include <stdint.h>
#include <System/SysStatus.h>
#include "UI/LCD_custom.h"

//--

typedef enum
{
    disp_AllBlack,
    disp_AllClear,
    BSL_info
}displayScreens;

//--

extern displayScreens dispScreen;
extern uint16_t dispSec;


void LcdUpdate(void);
void LcdAlarmIconsHandle(void);
void LcdBatteryIconHandle(void);
void LcdFlowIconsHandle(void);
void LcdTotIconsHandle(void);
void NextTotView(void);
void LongHoldMagnetInc(void);
void LongHoldMagnetReset(void);
void LcdScreenTick(void);
bool CheckResetReed(void);
void LcdScreenHandle(void);
void AddActiveError(_errView_t Error);
void RemoveActiveError(_errView_t Error);
bool ViewActiveErrors(bool FirstError);
void BackToPosTotalizer_tick(void);

#endif /* DISPLAY_UI_DISPLAY_UI_H_ */
