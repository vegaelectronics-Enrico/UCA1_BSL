/*
 * LCD_custom.h
 *
 *  Created on: 17/set/2012
 *      Author: Enrico
 */

#ifndef LCD_CUSTOM_H_
#define LCD_CUSTOM_H_


#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>
#include "main.h"
#include "CommInt/CommInterfaces.h"


//#define LCD_USE_RES_LADDER
#define FIXED_3DEC

//#include "DataTypes/DataTypes.h"
//#include "system/rtc.h"
//#include "logger/update_logger.h"

//icone unità tecniche accumulatori

#define LCD_COM1    BIT0
#define LCD_COM2    BIT1
#define LCD_COM3    BIT2
#define LCD_COM4    BIT3
#define LCD_COM5    BIT4
#define LCD_COM6    BIT5
#define LCD_COM7    BIT6
#define LCD_COM8    BIT7


#define M_PIN1  LCDM1
#define M_PIN2  LCDM2
#define M_PIN3  LCDM3
#define M_PIN4  LCDM11
#define M_PIN5  LCDM12
#define M_PIN6  LCDM13
#define M_PIN7  LCDM14
#define M_PIN8  LCDM15
#define M_PIN9  LCDM16
#define M_PIN10 LCDM17
#define M_PIN11 LCDM18
#define M_PIN12 LCDM19
#define M_PIN13 LCDM20
#define M_PIN14 LCDM21
#define M_PIN15 LCDM22
#define M_PIN16 LCDM23
#define M_PIN17 LCDM24
#define M_PIN18 LCDM25
#define M_PIN19 LCDM26
#define M_PIN20 LCDM27
#define M_PIN21 LCDM28
#define M_PIN22 LCDM29
#define M_PIN23 LCDM30
#define M_PIN24 LCDM31
#define M_PIN25 LCDM32



#ifdef LCD_100550
#define TOT_CLEAR_ALL       M_PIN24 &= ~(LCD_COM7 | LCD_COM8); M_PIN25 &= ~(LCD_COM5 | LCD_COM6 | LCD_COM7)
#define TOT_GAL				M_PIN25 |= LCD_COM6 | LCD_COM7
#define TOT_BBL             M_PIN25 |= LCD_COM5
#define TOT_FT3             M_PIN24 |= LCD_COM8
#define TOT_M3              M_PIN24 |= LCD_COM7
#define TOT_L               M_PIN25 |= LCD_COM7
#elif defined (LCD_110147)
#define TOT_CLEAR_ALL       M_PIN23 &= ~(LCD_COM5 |LCD_COM6 | LCD_COM7 | LCD_COM8); M_PIN22 &= ~LCD_COM5
#define TOT_GAL             M_PIN23 |= LCD_COM6 | LCD_COM7
#define TOT_BBL             M_PIN23 |= LCD_COM8
#define TOT_FT3             M_PIN23 |= LCD_COM5
#define TOT_M3              M_PIN22 |= LCD_COM5
#define TOT_L               M_PIN23 |= LCD_COM6
#endif


//--

//icone unità tecniche portata istantanea
#ifdef LCD_100550
#define INST_CLEAR_ALL      M_PIN18 &= ~(LCD_COM8); M_PIN19 = 0x00; M_PIN20 &= ~LCD_COM1
#define INST_M3_h           M_PIN19 |= (LCD_COM2 | LCD_COM3 | LCD_COM4); M_PIN20 |= LCD_COM1
#define INST_M3_s           M_PIN19 |= (LCD_COM1 | LCD_COM2 | LCD_COM3 | LCD_COM4)
#define INST_L_h            M_PIN19 |= (LCD_COM6 | LCD_COM8)
#define INST_L_s            M_PIN19 |= (LCD_COM7 | LCD_COM8)
#define INST_gpm            M_PIN19 |= (LCD_COM4 | LCD_COM5)
#define INST_bar            M_PIN18 |= LCD_COM8
#elif defined(LCD_110147)
#define INST_CLEAR_ALL      M_PIN17 = 0x00; M_PIN20 &= ~LCD_COM8; M_PIN22 &= ~LCD_COM8;
#define INST_M3_h           M_PIN17 |= (LCD_COM1 | LCD_COM3 | LCD_COM4 | LCD_COM5)
#define INST_M3_s           M_PIN17 |= (LCD_COM2 | LCD_COM3 | LCD_COM4 | LCD_COM5)
#define INST_L_h            M_PIN17 |= LCD_COM7; M_PIN20 |= LCD_COM8
#define INST_L_s            M_PIN17 |= LCD_COM8; M_PIN20 |= LCD_COM8
#define INST_gpm            M_PIN17 |= (LCD_COM5 | LCD_COM6)
#define INST_bar            M_PIN22 |= LCD_COM8
#endif

//--

//altre icone
#ifdef LCD_100550
#define ICON_alarm_ON          M_PIN4 |= LCD_COM6
#define ICON_us_signal_ON      M_PIN4 |= LCD_COM7
#define ICON_alarm_OFF         M_PIN4 &= ~LCD_COM6
#define ICON_us_signal_OFF     M_PIN4 &= ~LCD_COM7
#define ICON_P1_ON             M_PIN2 |= LCD_COM5
#define ICON_P1_OFF            M_PIN2 &= ~LCD_COM5
#define ICON_P2_ON             M_PIN3 |= LCD_COM5
#define ICON_P2_OFF            M_PIN3 &= ~LCD_COM5
#define ICON_clock_ON          M_PIN17 |= LCD_COM8
#define ICON_clock_OFF         M_PIN17 &= ~LCD_COM8
#define ICON_calendar_ON       M_PIN4 |= LCD_COM5
#define ICON_calendar_OFF      M_PIN4 &= ~LCD_COM5
#elif defined(LCD_110147)
#define ICON_alarm_ON          M_PIN4 |= LCD_COM6
#define ICON_us_signal_ON      M_PIN4 |= LCD_COM7
#define ICON_alarm_OFF         M_PIN4 &= ~LCD_COM6
#define ICON_us_signal_OFF     M_PIN4 &= ~LCD_COM7
#define ICON_P1_ON             M_PIN2 |= LCD_COM5
#define ICON_P1_OFF            M_PIN2 &= ~LCD_COM5
#define ICON_P2_ON             M_PIN3 |= LCD_COM5
#define ICON_P2_OFF            M_PIN3 &= ~LCD_COM5
#define ICON_clock_ON          M_PIN24 |= LCD_COM8
#define ICON_clock_OFF         M_PIN24 &= ~LCD_COM8
#define ICON_calendar_ON       M_PIN4 |= LCD_COM5
#define ICON_calendar_OFF      M_PIN4 &= ~LCD_COM5
#endif


typedef enum
{
    lcdTot_none,
    lcdTot_m3,
    lcdTot_L,
    lcdTot_gal,
    lcdTot_bbl,
    lcdTot_ft3
}_dispTot_en;

//--

typedef enum
{
    lcdFlow_none,
    lcdFlow_m3_h,
    lcdFlow_m3_s,
    lcdFlow_L_h,
    lcdFlow_L_s,
    lcdFlow_gpm
}_dispFlow_en;

//--

typedef enum
{
    lcdBatt_off,
    lcdBatt_empty,
    lcdBatt_L1_3,
    lcdBatt_L2_3,
    lcdBatt_full
}_dispBatt_en;

//--

typedef enum _lcdRtcsetFields {
	fieldNone = 0,
	fieldYear,
	fieldMonth,
	fieldDay,
	fieldHour,
	fieldMinutes
}lcdRtcsetField;

typedef enum _firmwareId_t_
{
	bootloaderFwId = 0,
	finalFwId
}firmwareId_t;

typedef enum _labelId_t_
{
	labelSaveUserPars = 0,
	labelLoadUserPars,
	labelLoadFactoryPars
}labelId_t;

typedef enum
{
    //enumerated errors
    error_none = 0,
    err_FW_CRC = 1,
    err_PAR_CRC = 2,
    err_EEPROM = 3,
    err_no_signal = 4,
    err_pulses_out = 5,
    err_invalid_signal = 6,
    err_invalid_flowrate = 7,
    err_low_voltage = 8,        //just alarm
    err_insuff_voltage = 9,     //alarm and measurment / EEPROM writes disabled
    err_high_temp = 10,
    err_low_temp = 11,
    err_high_battery_drain = 12,
    err_dual_path_ch1 = 13,
    err_dual_path_ch2 = 14,
    err_hspll_unlocked = 15,
    err_hspll_other = 16,
    //not usable code
    err_not_valid = 99
}_errView_t;

//--

void LCD_init(void);
void LCD_disable (void);
void LCD_clear(void);
void LCD_fill_row1(unsigned char DecimalDigits);
void LCD_fill_row2(unsigned char DecimalDigits);
void LCD_acc_write(double value,unsigned char Ndec,unsigned char Nzero,bool ViewOverrun);
void LCD_flow_write(float value,unsigned char Ndec);
void Sign1(unsigned char sign);
void Sign2(unsigned char sign);
void ShowDot(unsigned char row, unsigned char position);
void IconGSM(unsigned char show, unsigned char level);
void IconAlert(unsigned char OnOff);
void IconBattery(_dispBatt_en  level);
void IconAccUT(_dispTot_en selection);
void IconInstUT_TB(_dispFlow_en UTTB);
void LCD_clear_row1(void);
void LCD_clear_row2(void);
void LCD_all_on(void);
void LCD_all_off(void);
void LCD_row1_write_hex( uint32_t val );
void LCD_checksum_error(uint32_t CRCdisp);
void LCD_new_batt(void);
void LCD_modbus_cfg(uint8_t Address, uint16_t Baudrate, uint8_t StopBits, _rs485Parity_en Parity);
void LCD_parameters_error(void);
void LCD_lowVoltage(void);
void LCD_bsl_info_view(unsigned char FwVer,unsigned char FwRev, unsigned long FwCRC32, firmwareId_t fwId);
void LCD_row2_writeErr(_errView_t ErrorNumber);
void LCD_rtcc_view (void);
#endif /* LCD_CUSTOM_H_ */
