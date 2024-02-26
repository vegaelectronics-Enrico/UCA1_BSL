/*
 * SysStatus.h
 *
 *  Created on: 15 Nov 2021
 *      Author: Enrico
 */

#ifndef SYSTEM_SYSSTATUS_H_
#define SYSTEM_SYSSTATUS_H_

#include <stdint.h>
#include <stdbool.h>

//--

#define LOW_VOLTAGE_TH          3100.0
#define TURN_OFF_TH             2700.0
#define HIGH_TEMP_TH            60.0
#define LOW_TEMP_TH             -25.0
#define EXTERNAL_POWER_TH       3900.0

#define CHANNEL_INVALID_COUNT   20  //number of consecutives channel invalid events to disable it
#define CHANNEL_REARM_SECONDS   60  //seconds count to re-enable the channel

#define CURR_MEAS_CIRCUIT_uA    15  //current meas circuit self-consumption current

#define HSPLL_CONS_ERR_BOR      100 //is more than N samples cause every time the USS is re-initialized

typedef struct
{
    unsigned A_BatteryCurrentSense :1;
    unsigned B_notAssigned :1;
    unsigned C_notAssigned :1;
    unsigned NU :5;
}_sysConfig_t;

extern _sysConfig_t sysConfig;

//--

typedef struct
{
    unsigned RebootFromModbus :1;
    unsigned HSPLL_resetBOR :1;
    unsigned REED_resetBOR :1;
    unsigned BSL_loadStarted :1;
    unsigned BSL_loadCompleted :1;
    unsigned NU: 11;
    uint8_t LastExecutedFwVer;
    uint8_t LastExecutedFwRev;
    uint16_t LastReceivedPassword;
    uint8_t UnusedBytes[158];
}_SharedCtrlStruct_t;


extern _SharedCtrlStruct_t* SharedCtrlStruct;

//--

void detectHwConfigurationPins(void);

#endif /* SYSTEM_SYSSTATUS_H_ */
