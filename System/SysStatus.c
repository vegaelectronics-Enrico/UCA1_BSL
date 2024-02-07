/*
 * SysStatus.c
 *
 *  Created on: 15 Nov 2021
 *      Author: Enrico
 */

#include "SysStatus.h"
#include "Display_UI/Display_UI.h"
#include "UI/LCD_custom.h"
#include <math.h>

_sysConfig_t sysConfig = {0};

_SharedCtrlStruct_t* SharedCtrlStruct = 0x43EFC;    //point to the shared FRAM data control struct


//--
void detectHwConfigurationPins(void)
{
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P6,GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN1);

    sysConfig.A_BatteryCurrentSense = !GPIO_getInputPinValue(GPIO_PORT_P6,GPIO_PIN3);
    sysConfig.B_notAssigned = !GPIO_getInputPinValue(GPIO_PORT_P8,GPIO_PIN0);
    sysConfig.C_notAssigned = !GPIO_getInputPinValue(GPIO_PORT_P8,GPIO_PIN1);

    GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P8,GPIO_PIN0|GPIO_PIN1);
}


