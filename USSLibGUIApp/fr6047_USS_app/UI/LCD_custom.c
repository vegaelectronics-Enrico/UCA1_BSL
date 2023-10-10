/*
 * LCD_custom.c
 *
 *  Created on: 17/set/2012
 *      Author: Enrico Sorgato
 *
 *      ////////////////////////////////////////
 *
 */

#include "LCD_custom.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Display_UI/Display_UI.h"
#include "Timers/Timers.h"


//#include "board/CpuPinConfig.h"
//#include "system/clock.h"
//#include "DataTypes/DataTypes.h"

static const unsigned long long PowersOfTen[]={1,10,1e2,1e3,1e4,1e5,1e6,1e7,1e8,1e9,1e10,1e11,1e12,1e13,1e14,1e15,1e16,1e17};

#define CHAR_0		0
#define CHAR_1		1
#define CHAR_2		2
#define CHAR_3		3
#define CHAR_4		4
#define CHAR_5		5
#define CHAR_6		6
#define CHAR_7		7
#define CHAR_8		8
#define CHAR_9		9
#define CHAR_A		10
#define CHAR_b		11
#define CHAR_B		CHAR_8
#define CHAR_C		12
#define CHAR_d		13
#define CHAR_E		14
#define CHAR_F		15
#define CHAR_BLANK	16
#define CHAR_L		17
#define CHAR_DASH	18
#define CHAR_USCORE	19
#define CHAR_t		20
#define CHAR_J		21
#define CHAR_P		22
#define CHAR_U		23
#define CHAR_n		24
#define CHAR_o		25
#define CHAR_r		26
#define CHAR_Y		27
#define CHAR_I		CHAR_1
#define CHAR_O		CHAR_0
#define CHAR_Z		CHAR_2
#define CHAR_S		CHAR_5
#define CHAR_g		CHAR_9
#define CHAR_M      28

const unsigned char LCD_Char_Map[] =
{
    0xF5,                //0: '0' or 'O'
    0x05,                //1: '1' or 'I'
    0xD3,		         //2: '2' or 'Z'
    0x97,                //3: '3'
    0x27,                //4: '4'
    0xB6,                //5: '5' or 'S'
    0xF6,			     //6: '6'
    0x15,                //7: '7'
    0xF7,				 //8: '8' or 'B'
    0xB7,      			 //9: '9' or 'g'
    0x77,				 //10: A
    0xE6,				 //11: b
    0xF0,				 //12: C
    0xC7,				 //13: d
    0xF2,				 //14: E
    0x72,				 //15: F
    0x00,			     //16: (blank)
    0x70,				 //17: L
    0x04,				 //18: -
    0x10,				 //19: _
    0xE2,			     //20: t
    0x3A,			     //21: J
    0x73,			     //22: P
    0x7A,			     //23: U
    0x46,			     //24: n
    0xC6,				 //25: o
    0x42,				 //26: r
    0xA7,				 //27: Y
    0x37                 //28:
};

#define NDIGIT_ROW1			12
#define NDIGIT_ROW2			9


//parte intera riga 1
unsigned char LCD_digit_row1[NDIGIT_ROW1]=
{
	CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK
};
//parte decimale riga 1
unsigned char LCD_digit_row2[NDIGIT_ROW2]=
{
	CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK
};


//inizializzazione controller integrato MCU
void LCD_init(void)
{


	LCD_C_initParam initParams = {0};
    initParams.clockSource = LCD_C_CLOCKSOURCE_ACLK;
    initParams.clockDivider = LCD_C_CLOCKDIVIDER_6;
    initParams.clockPrescalar = LCD_C_CLOCKPRESCALAR_16;
    initParams.muxRate = LCD_C_8_MUX;
    initParams.waveforms = LCD_C_LOW_POWER_WAVEFORMS;
    initParams.segments = LCD_C_SEGMENTS_ENABLED;

    LCD_C_init(LCD_C_BASE, &initParams);
    // LCD Operation - VLCD generated internally, V2-V4 generated internally, v5 to ground

    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_0, LCD_C_SEGMENT_LINE_2);
    LCD_C_setPinAsLCDFunctionEx(LCD_C_BASE, LCD_C_SEGMENT_LINE_10, LCD_C_SEGMENT_LINE_31);

//#ifdef LCD_USE_RES_LADDER
    LCD_C_setVLCDSource(LCD_C_BASE,
                        LCD_C_VLCD_GENERATED_INTERNALLY,
                        LCD_C_V2V3V4_GENERATED_INTERNALLY_NOT_SWITCHED_TO_PINS,
                        LCD_C_V5_VSS);

    LCD_C_setVLCDVoltage(LCD_C_BASE,
                        LCD_C_CHARGEPUMP_DISABLED);
    LCD_C_disableChargePump(LCD_C_BASE);

    // Clear LCD memory
    LCD_C_clearMemory(LCD_C_BASE);

    //Turn LCD on
    LCD_C_on(LCD_C_BASE);

}
//disables the LCD controller
void LCD_disable (void)
{
	LCDCCTL0 = 0;
}

//////////////////////////////////////////

void LCD_clear(void)
{
	LCDCMEMCTL |= LCDCLRM;
	int idx;
	for (idx = 0; idx < NDIGIT_ROW1; ++idx)
	{
		LCD_digit_row1[idx] = CHAR_BLANK;
	}
	for (idx = 0; idx < NDIGIT_ROW2; ++idx)
	{
		LCD_digit_row2[idx] = CHAR_BLANK;
	}
}

void LCD_row1_write_hex( uint32_t val )
{
    LCD_clear_row1();
	uint8_t idx = 3;
	for ( ; idx < 3+8; idx++ )
	{
		uint8_t digit = (uint8_t)(val & 0x0000000F);
		val >>= 4;
		LCD_digit_row1[(NDIGIT_ROW1-1) - idx ] = digit;
	}
	LCD_fill_row1(0);
}


/////////////////////////////////////////
//converte il valore ricevuto nei singoli digit di LCD_digit_row1

void LCD_acc_write(double value,unsigned char Ndec,unsigned char Nzero,bool ViewOverrun)
{
#ifdef FIXED_3DEC
    unsigned char NdecLocal = 3;
#else
    unsigned char NdecLocal = Ndec;
#endif
    unsigned char NzeroLocal = Nzero;
    unsigned long OverFlowCount = 0;

	LCD_clear_row1();
	//Sign1( (unsigned char)(value < 0) );

	unsigned long long IntPart = llabs( (signed long long)( value * PowersOfTen[Ndec] ) );
	if(IntPart>999999999999)
	{
	    OverFlowCount = (unsigned long)(IntPart / 1e12);
	    IntPart -= (1e12 * (unsigned long long)OverFlowCount);
	}
	else
	    OverFlowCount = 0;
    if(ViewOverrun)
    {
        IntPart = (unsigned long long)OverFlowCount;
        NdecLocal = 0;
        NzeroLocal = 4;
        LCD_digit_row1[0] = CHAR_o;
        LCD_digit_row1[1] = CHAR_F;
        LCD_digit_row1[2] = CHAR_BLANK;
    }
    else if(OverFlowCount>0)
        NzeroLocal = 12;

	signed char i=0;
	bool forceZero = false;
	unsigned long long ULL1=0;
	unsigned char IntDigit=0;
	unsigned char nDigit = NdecLocal + NzeroLocal;
	unsigned const char lastDigit = (NDIGIT_ROW1 - 1);



	for( i = lastDigit ; i > -1; i-- )
	{
		ULL1=PowersOfTen[i];
		if(ULL1<=IntPart)
		{
			forceZero = true;
			IntDigit=(unsigned char)(IntPart/ULL1);
			LCD_digit_row1[lastDigit-i]=IntDigit;
			IntPart-=(IntDigit * ULL1);
		}
		else if ( ( i <= nDigit ) || forceZero )
		{
			LCD_digit_row1[lastDigit-i] = CHAR_0;
		}

	}
#ifdef FIXED_3DEC
    LCD_fill_row1(1);
#else
    LCD_fill_row1(Ndec);
#endif
}

//////////////////////////////////////////


//visualizza versione e revisione FW ed il suo CRC32
void LCD_bsl_info_view(unsigned char FwVer,unsigned char FwRev, unsigned long FwCRC32, firmwareId_t fwId)
{
    LCD_all_off();

	//scrivo il CRC32 nella riga 1
#if LEGAL_FW
	LCD_row1_write_hex(FwCRC32);
#else
	if(FwCRC32 != 0xFFFFFFFF)
	    LCD_row1_write_hex(FwCRC32);
#endif
	//ottengo digit, inserisco il decimale e scrivo ver/rev fw nella riga 2
    LCD_clear_row2();
    LCD_digit_row2[0]=CHAR_b;
	LCD_digit_row2[2]=FwVer/10;
	LCD_digit_row2[3]=FwVer%10;
	LCD_digit_row2[4]=FwRev/10;
	LCD_digit_row2[5]=FwRev%10;
	LCD_fill_row2(3);

}

//--

void LCD_rtcc_view (void)
{
    LCD_clear_row1();
    LCD_clear_row2();
    LCD_digit_row1[1] = RTCC_Calendar.DayOfMonth / 10;
    LCD_digit_row1[2] = RTCC_Calendar.DayOfMonth % 10;
    LCD_digit_row1[3] = RTCC_Calendar.Month / 10;
    LCD_digit_row1[4] = RTCC_Calendar.Month % 10;
    LCD_digit_row1[5] = CHAR_2;
    LCD_digit_row1[6] = CHAR_0;
    LCD_digit_row1[7] = (RTCC_Calendar.Year - 2000) / 10;
    LCD_digit_row1[8] = (RTCC_Calendar.Year - 2000) % 10;
    ShowDot(1, 5);
    ShowDot(1, 7);
    LCD_digit_row2[0] = RTCC_Calendar.Hours / 10;
    LCD_digit_row2[1] = RTCC_Calendar.Hours % 10;
    LCD_digit_row2[2] = RTCC_Calendar.Minutes / 10;
    LCD_digit_row2[3] = RTCC_Calendar.Minutes % 10;
    LCD_digit_row2[4] = RTCC_Calendar.Seconds / 10;
    LCD_digit_row2[5] = RTCC_Calendar.Seconds % 10;
    LCD_fill_row1(0);
    LCD_fill_row2(0);
    ShowDot(2, 3);
    ShowDot(2, 5);
    ShowDot(2, 7);
    ICON_clock_ON;
    ICON_calendar_ON;

}

//--

void LCD_all_on(void)
{
    M_PIN1=0xFF;
    M_PIN2=0xFF;
    M_PIN3=0xFF;
    M_PIN4=0xFF;
    M_PIN5=0xFF;
    M_PIN6=0xFF;
    M_PIN7=0xFF;
    M_PIN8=0xFF;
    M_PIN9=0xFF;
    M_PIN10=0xFF;
    M_PIN11=0xFF;
    M_PIN12=0xFF;
    M_PIN13=0xFF;
    M_PIN14=0xFF;
    M_PIN15=0xFF;
    M_PIN16=0xFF;
    M_PIN17=0xFF;
    M_PIN18=0xFF;
    M_PIN19=0xFF;
    M_PIN20=0xFF;
    M_PIN21=0xFF;
    M_PIN22=0xFF;
    M_PIN23=0xFF;
    M_PIN24=0xFF;
    M_PIN25=0xFF;
}

void LCD_all_off(void)
{
    M_PIN1=0x00;
    M_PIN2=0x00;
    M_PIN3=0x00;
    M_PIN4=0x00;
    M_PIN5=0x00;
    M_PIN6=0x00;
    M_PIN7=0x00;
    M_PIN8=0x00;
    M_PIN9=0x00;
    M_PIN10=0x00;
    M_PIN11=0x00;
    M_PIN12=0x00;
    M_PIN13=0x00;
    M_PIN14=0x00;
    M_PIN15=0x00;
    M_PIN16=0x00;
    M_PIN17=0x00;
    M_PIN18=0x00;
    M_PIN19=0x00;
    M_PIN20=0x00;
    M_PIN21=0x00;
    M_PIN22=0x00;
    M_PIN23=0x00;
    M_PIN24=0x00;
    M_PIN25=0x00;
}




/////////////////////////////////////////
void LCD_row2_writeErr(_errView_t ErrorNumber)
{
    uint8_t ErrorNumberNormalized = ErrorNumber;
    if(ErrorNumberNormalized > 99)
        ErrorNumberNormalized = 99;
    LCD_digit_row2[0] = CHAR_E;
    LCD_digit_row2[1] = CHAR_r;
    LCD_digit_row2[2] = CHAR_r;
    LCD_digit_row2[3] = CHAR_BLANK;
    LCD_digit_row2[4] = ErrorNumberNormalized / 10;
    LCD_digit_row2[5] = ErrorNumberNormalized % 10;
    LCD_fill_row2( 0 );
}


////////////////////////////////////////////////
//visualizza schermata fw corrotti
void LCD_checksum_error(uint32_t CRCdisp)
{
	LCD_clear();
	LCD_row1_write_hex(CRCdisp);
	LCD_row2_writeErr(err_FW_CRC);
	IconAlert(1);
}


//--

void LCD_new_batt(void)
{
    LCD_clear();

    LCD_digit_row1[0] = CHAR_b;
    LCD_digit_row1[1] = CHAR_A;
    LCD_digit_row1[2] = CHAR_t;
    LCD_digit_row1[3] = CHAR_t;
    LCD_digit_row1[4] = CHAR_E;
    LCD_digit_row1[5] = CHAR_r;
    LCD_digit_row1[6] = CHAR_Y;
    LCD_fill_row1( 0 );

    LCD_clear_row2();
    LCD_digit_row2[4]=dispSec / 10;
    LCD_digit_row2[5]=dispSec % 10;
    LCD_fill_row2( 0 );

}
//--

void LCD_modbus_cfg(uint8_t Address, uint16_t Baudrate, uint8_t StopBits, _rs485Parity_en Parity)
{
    LCD_clear();

    LCD_digit_row1[0] = CHAR_A;
    LCD_digit_row1[1] = CHAR_d;
    LCD_digit_row1[2] = CHAR_d;
    LCD_digit_row1[3] = CHAR_BLANK;
    LCD_digit_row1[4] = Address / 100;
    LCD_digit_row1[5] = (Address / 10) - (LCD_digit_row1[4] * 10);
    LCD_digit_row1[6] = Address % 10;

    LCD_digit_row1[9] = 8;
    if(Parity == EUSCI_A_UART_NO_PARITY)
        LCD_digit_row1[10] = CHAR_n;
    else if(Parity == EUSCI_A_UART_ODD_PARITY)
        LCD_digit_row1[10] = CHAR_o;
    else if(Parity == EUSCI_A_UART_EVEN_PARITY)
        LCD_digit_row1[10] = CHAR_E;
    LCD_digit_row1[11] = StopBits;

    LCD_fill_row1( 0 );

    LCD_clear_row2();

    LCD_flow_write((float)Baudrate, 0);

}

//--
//errore parametri (CRC o diversi dopo confronto
void LCD_parameters_error(void)
{
    LCD_clear();
    LCD_row2_writeErr(err_PAR_CRC);
    IconAlert(1);
}

//--
//tensione troppo bassa
void LCD_lowVoltage(void)
{
    LCD_clear();
    LCD_row2_writeErr(err_insuff_voltage);
    IconAlert(1);
}

/////////////////////////////////////////
//converte il valore ricevuto nei singoli digit di LCD_digit_row1
void LCD_flow_write(float value,unsigned char Ndec)
{
#ifdef FIXED_3DEC
    unsigned char NdecLocal = 3;
#else
    unsigned char NdecLocal = Ndec;
#endif
	LCD_clear_row2();
	Sign2( (unsigned char)(value < 0) );

	unsigned long long IntPart = labs( (signed long long)( value * PowersOfTen[NdecLocal] ) );
	if(IntPart>999999999)
		IntPart=999999999;
	
	signed char i=0;
	bool forceZero = false;
	unsigned long long UL1=0;
	unsigned char IntDigit=0;
	const unsigned char lastDigit = NDIGIT_ROW2 - 1;
	
	for( i = lastDigit; i > -1; i--)
	{
		UL1=PowersOfTen[i];
		if(UL1<=IntPart)
		{
			forceZero = true;
			IntDigit=(unsigned char)(IntPart/UL1);
			LCD_digit_row2[lastDigit-i]=IntDigit;
			IntPart-=(IntDigit * UL1);
		}
		else if( ( i <= NdecLocal ) || forceZero )
		{
			LCD_digit_row2[lastDigit-i] = CHAR_0;
		}
	}
#ifdef FIXED_3DEC
	LCD_fill_row2(1);
#else
	LCD_fill_row2(Ndec);
#endif
}

/////////////////////////////////////////
//scrive il contenuto di LCD_digit_row1 sulla prima riga del display
void LCD_fill_row1(unsigned char DecimalDigits)
{
#ifdef LCD_100550
    M_PIN1 |= (LCD_Char_Map[LCD_digit_row1[0]] & 0xF0)>>4;        //pin1
    M_PIN2 |= (LCD_Char_Map[LCD_digit_row1[0]] & 0x0F);   //pin2

    M_PIN3 |= (LCD_Char_Map[LCD_digit_row1[1]] & 0xF0)>>4;        //pin3
    M_PIN4 |= (LCD_Char_Map[LCD_digit_row1[1]] & 0x0F);  //pin4

    M_PIN5 |= (LCD_Char_Map[LCD_digit_row1[2]] & 0xF0)>>4;       //pin5
    M_PIN6|= (LCD_Char_Map[LCD_digit_row1[2]] & 0x0F);   //pin6

    M_PIN7 |= (LCD_Char_Map[LCD_digit_row1[3]] & 0xF0)>>4;       //pin7
    M_PIN8 |= (LCD_Char_Map[LCD_digit_row1[3]] & 0x0F);  //pin8

    M_PIN9 |= (LCD_Char_Map[LCD_digit_row1[4]] & 0xF0)>>4;       //pin9
    M_PIN10 |= (LCD_Char_Map[LCD_digit_row1[4]] & 0x0F);  //pin10

    M_PIN11 |= (LCD_Char_Map[LCD_digit_row1[5]] & 0xF0)>>4;       //pin11
    M_PIN12 |= (LCD_Char_Map[LCD_digit_row1[5]] & 0x0F);  //pin12

    M_PIN13 |= (LCD_Char_Map[LCD_digit_row1[6]] & 0xF0)>>4;       //pin13
    M_PIN14 |= (LCD_Char_Map[LCD_digit_row1[6]] & 0x0F);  //pin14

    M_PIN15 |= (LCD_Char_Map[LCD_digit_row1[7]] & 0xF0)>>4;       //pin15
    M_PIN16 |= (LCD_Char_Map[LCD_digit_row1[7]] & 0x0F);  //pin16

    M_PIN25 |= (LCD_Char_Map[LCD_digit_row1[8]] & 0xF0)>>4;       //pin25
    M_PIN24 |= (LCD_Char_Map[LCD_digit_row1[8]] & 0x0F);      //pin24

    M_PIN23 |= (LCD_Char_Map[LCD_digit_row1[9]] & 0xF0)>>4;       //pin23
    M_PIN17 |= (LCD_Char_Map[LCD_digit_row1[9]] & 0x0F)<<1;  //pin17

    M_PIN22 |= (LCD_Char_Map[LCD_digit_row1[10]] & 0xF0)>>4;    //pin22
    M_PIN18 |= (LCD_Char_Map[LCD_digit_row1[10]] & 0x0F)<<1; //pin18

    M_PIN21 |= (LCD_Char_Map[LCD_digit_row1[11]] & 0xF0)>>4;      //pin21
    M_PIN20 |= (LCD_Char_Map[LCD_digit_row1[11]] & 0x0F)<<1; //pin20
#elif (defined LCD_110147)
    M_PIN1 |= (LCD_Char_Map[LCD_digit_row1[0]] & 0xF0)>>4;        //pin1
    M_PIN2 |= (LCD_Char_Map[LCD_digit_row1[0]] & 0x0F);   //pin2

    M_PIN3 |= (LCD_Char_Map[LCD_digit_row1[1]] & 0xF0)>>4;        //pin3
    M_PIN4 |= (LCD_Char_Map[LCD_digit_row1[1]] & 0x0F);  //pin4

    M_PIN5 |= (LCD_Char_Map[LCD_digit_row1[2]] & 0xF0)>>4;       //pin5
    M_PIN6|= (LCD_Char_Map[LCD_digit_row1[2]] & 0x0F);   //pin6

    M_PIN7 |= (LCD_Char_Map[LCD_digit_row1[3]] & 0xF0)>>4;       //pin7
    M_PIN8 |= (LCD_Char_Map[LCD_digit_row1[3]] & 0x0F);  //pin8

    M_PIN9 |= (LCD_Char_Map[LCD_digit_row1[4]] & 0xF0)>>4;       //pin9
    M_PIN10 |= (LCD_Char_Map[LCD_digit_row1[4]] & 0x0F);  //pin10

    M_PIN11 |= (LCD_Char_Map[LCD_digit_row1[5]] & 0xF0)>>4;       //pin11
    M_PIN12 |= (LCD_Char_Map[LCD_digit_row1[5]] & 0x0F);  //pin12

    M_PIN13 |= (LCD_Char_Map[LCD_digit_row1[6]] & 0xF0)>>4;       //pin13
    M_PIN14 |= (LCD_Char_Map[LCD_digit_row1[6]] & 0x0F);  //pin14

    M_PIN15 |= (LCD_Char_Map[LCD_digit_row1[7]] & 0xF0)>>4;       //pin15
    M_PIN16 |= (LCD_Char_Map[LCD_digit_row1[7]] & 0x0F);  //pin16

    M_PIN25 |= (LCD_Char_Map[LCD_digit_row1[8]] & 0xF0)>>4;       //pin25
    M_PIN24 |= (LCD_Char_Map[LCD_digit_row1[8]] & 0x0F);      //pin24

    M_PIN23 |= (LCD_Char_Map[LCD_digit_row1[9]] & 0xF0)>>4;       //pin23
    M_PIN22 |= (LCD_Char_Map[LCD_digit_row1[9]] & 0x0F)<<1;  //pin17

    M_PIN21 |= (LCD_Char_Map[LCD_digit_row1[10]] & 0xF0)>>4;    //pin21
    M_PIN20 |= (LCD_Char_Map[LCD_digit_row1[10]] & 0x0F)<<1; //pin20

    M_PIN19 |= (LCD_Char_Map[LCD_digit_row1[11]] & 0xF0)>>4;      //pin19
    M_PIN18 |= (LCD_Char_Map[LCD_digit_row1[11]] & 0x0F)<<1; //pin18
#endif

    ShowDot( 1, DecimalDigits );

}

////////////////////////////////////////////////////////////
//cancella la prima riga
void LCD_clear_row1(void)
{
	unsigned char i=0;

	//Sign1(0);
	for(i = 0; i < NDIGIT_ROW1; i++)
		LCD_digit_row1[i] = CHAR_BLANK;
#ifdef LCD_100550
	M_PIN1 &= ~0x0F;
	M_PIN2 &= ~0x0F;
	M_PIN3 &= ~0x0F;
	M_PIN4 &= ~0x0F;
	M_PIN5 &= ~0x0F;
	M_PIN6 &= ~0x0F;
	M_PIN7 &= ~0x0F;
	M_PIN8 &= ~0x0F;
	M_PIN9 &= ~0x0F;
	M_PIN10 &= ~0x0F;
	M_PIN11 &= ~0x0F;
	M_PIN12 &= ~0x0F;
	M_PIN13 &= ~0x0F;
	M_PIN14 &= ~0x0F;
	M_PIN15 &= ~0x0F;
	M_PIN16 &= ~0x0F;
	M_PIN17 &= ~0x0E;
	M_PIN18 &= ~0x0E;
	M_PIN20 &= ~0x0E;
	M_PIN21 &= ~0x0F;
	M_PIN22 &= ~0x0F;
	M_PIN23 &= ~0x0F;
	M_PIN24 &= ~0x0F;
    M_PIN25 &= ~0x0F;
#elif defined (LCD_110147)
    M_PIN1 &= ~0x0F;
    M_PIN2 &= ~0x0F;
    M_PIN3 &= ~0x0F;
    M_PIN4 &= ~0x0F;
    M_PIN5 &= ~0x0F;
    M_PIN6 &= ~0x0F;
    M_PIN7 &= ~0x0F;
    M_PIN8 &= ~0x0F;
    M_PIN9 &= ~0x0F;
    M_PIN10 &= ~0x0F;
    M_PIN11 &= ~0x0F;
    M_PIN12 &= ~0x0F;
    M_PIN13 &= ~0x0F;
    M_PIN14 &= ~0x0F;
    M_PIN15 &= ~0x0F;
    M_PIN16 &= ~0x0F;
    M_PIN18 &= ~0x0E;
    M_PIN19 &= ~0x0F;
    M_PIN20 &= ~0x0E;
    M_PIN21 &= ~0x0F;
    M_PIN22 &= ~0x0E;
    M_PIN23 &= ~0x0F;
    M_PIN24 &= ~0x0F;
    M_PIN25 &= ~0x0F;
#endif

}
////////////////////////////////////////////////////////////
//cancella la seconda riga del display
void LCD_clear_row2(void)
{
	unsigned char i=0;

	//Sign2(0);
	for(i=0; i<NDIGIT_ROW2; i++)
		LCD_digit_row2[i] = CHAR_BLANK;
#ifdef LCD_100550
	M_PIN5 &= ~0xF0;
	M_PIN6 &= ~0xF0;
	M_PIN7 &= ~0xF0;
	M_PIN8 &= ~0xF0;
	M_PIN9 &= ~0xF0;
	M_PIN10 &= ~0xF0;
	M_PIN11 &= ~0xF0;
	M_PIN12 &= ~0xF0;
	M_PIN13 &= ~0xF0;
	M_PIN14 &= ~0xF0;
	M_PIN15 &= ~0xF0;
	M_PIN16 &= ~0xF0;
    M_PIN17 &= ~0x70;
    M_PIN18 &= ~0x70;
    M_PIN20 &= ~0x70;
    M_PIN21 &= ~0xF0;
    M_PIN22 &= ~0xF0;
    M_PIN23 &= ~0xF0;
#elif defined (LCD_110147)
    M_PIN5 &= ~0xF0;
    M_PIN6 &= ~0xF0;
    M_PIN7 &= ~0xF0;
    M_PIN8 &= ~0xF0;
    M_PIN9 &= ~0xF0;
    M_PIN10 &= ~0xF0;
    M_PIN11 &= ~0xF0;
    M_PIN12 &= ~0xF0;
    M_PIN13 &= ~0xF0;
    M_PIN14 &= ~0xF0;
    M_PIN15 &= ~0xF0;
    M_PIN16 &= ~0xF0;
    M_PIN18 &= ~0x70;
    M_PIN19 &= ~0xF0;
    M_PIN20 &= ~0x70;
    M_PIN21 &= ~0xF0;
    M_PIN24 &= ~0x70;
    M_PIN25 &= ~0xF0;
#endif

}
////////////////////////////////////////////////////////////
////scrive il contenuto di LCD_digit_row2 sulla seconda riga del display
void LCD_fill_row2(unsigned char DecimalDigits)
{
#ifdef LCD_100550
    M_PIN5 |= (LCD_Char_Map[LCD_digit_row2[0]] & 0xF0);
    M_PIN6 |= (LCD_Char_Map[LCD_digit_row2[0]] & 0x0F)<<4;

    M_PIN7 |= (LCD_Char_Map[LCD_digit_row2[1]] & 0xF0);
    M_PIN8 |= (LCD_Char_Map[LCD_digit_row2[1]] & 0x0F)<<4;

    M_PIN9 |= (LCD_Char_Map[LCD_digit_row2[2]] & 0xF0);
    M_PIN10 |= (LCD_Char_Map[LCD_digit_row2[2]] & 0x0F)<<4;

    M_PIN11 |= (LCD_Char_Map[LCD_digit_row2[3]] & 0xF0);
    M_PIN12 |= (LCD_Char_Map[LCD_digit_row2[3]] & 0x0F)<<4;

    M_PIN13 |= (LCD_Char_Map[LCD_digit_row2[4]] & 0xF0);
    M_PIN14 |= (LCD_Char_Map[LCD_digit_row2[4]] & 0x0F)<<4;

    M_PIN15 |= (LCD_Char_Map[LCD_digit_row2[5]] & 0xF0);
    M_PIN16 |= (LCD_Char_Map[LCD_digit_row2[5]] & 0x0F)<<4;

    M_PIN23 |= (LCD_Char_Map[LCD_digit_row2[6]] & 0xF0);
    M_PIN17 |= (LCD_Char_Map[LCD_digit_row2[6]] & 0x0F)<<4;

    M_PIN22 |= (LCD_Char_Map[LCD_digit_row2[7]] & 0xF0);
    M_PIN18 |= (LCD_Char_Map[LCD_digit_row2[7]] & 0x0F)<<4;

    M_PIN21 |= (LCD_Char_Map[LCD_digit_row2[8]] & 0xF0);
    M_PIN20 |= (LCD_Char_Map[LCD_digit_row2[8]] & 0x0F)<<4;
#elif defined(LCD_110147)
    M_PIN5 |= (LCD_Char_Map[LCD_digit_row2[0]] & 0xF0);
    M_PIN6 |= (LCD_Char_Map[LCD_digit_row2[0]] & 0x0F)<<4;

    M_PIN7 |= (LCD_Char_Map[LCD_digit_row2[1]] & 0xF0);
    M_PIN8 |= (LCD_Char_Map[LCD_digit_row2[1]] & 0x0F)<<4;

    M_PIN9 |= (LCD_Char_Map[LCD_digit_row2[2]] & 0xF0);
    M_PIN10 |= (LCD_Char_Map[LCD_digit_row2[2]] & 0x0F)<<4;

    M_PIN11 |= (LCD_Char_Map[LCD_digit_row2[3]] & 0xF0);
    M_PIN12 |= (LCD_Char_Map[LCD_digit_row2[3]] & 0x0F)<<4;

    M_PIN13 |= (LCD_Char_Map[LCD_digit_row2[4]] & 0xF0);
    M_PIN14 |= (LCD_Char_Map[LCD_digit_row2[4]] & 0x0F)<<4;

    M_PIN15 |= (LCD_Char_Map[LCD_digit_row2[5]] & 0xF0);
    M_PIN16 |= (LCD_Char_Map[LCD_digit_row2[5]] & 0x0F)<<4;

    M_PIN25 |= (LCD_Char_Map[LCD_digit_row2[6]] & 0xF0);
    M_PIN24 |= (LCD_Char_Map[LCD_digit_row2[6]] & 0x0F)<<4;

    M_PIN21 |= (LCD_Char_Map[LCD_digit_row2[7]] & 0xF0);
    M_PIN20 |= (LCD_Char_Map[LCD_digit_row2[7]] & 0x0F)<<4;

    M_PIN19 |= (LCD_Char_Map[LCD_digit_row2[8]] & 0xF0);
    M_PIN18 |= (LCD_Char_Map[LCD_digit_row2[8]] & 0x0F)<<4;
#endif

    ShowDot( 2, DecimalDigits );
}

//////////////////////////////////////////////////////////////
//segno "-" prima riga: 0=positivo 1=negativo 2=differenza altro=nessuno
void Sign1(unsigned char sign)
{
#ifdef LCD_100550
	if(sign == 0)
	{
	    M_PIN18 |= LCD_COM1;
	    M_PIN17 &= ~LCD_COM1;
	}
	else if (sign == 1)
	{
        M_PIN17 |= LCD_COM1;
        M_PIN18 &= ~LCD_COM1;
	}
	else if(sign == 2)
	{
	    M_PIN18 |= LCD_COM1;
	    M_PIN17 |= LCD_COM1;
	}
	else
	{
        M_PIN17 &= ~LCD_COM1;
        M_PIN18 &= ~LCD_COM1;
	}
#elif defined(LCD_110147)
    if(sign == 0)
    {
        M_PIN18 |= LCD_COM1;
        M_PIN20 &= ~LCD_COM1;
        M_PIN22 &= ~LCD_COM1;
    }
    else if (sign == 1)
    {
        M_PIN20 |= LCD_COM1;
        M_PIN18 &= ~LCD_COM1;
        M_PIN22 &= ~LCD_COM1;
    }
    else if(sign == 2)
    {
        M_PIN22 |= LCD_COM1;
        M_PIN18 &= ~LCD_COM1;
        M_PIN20 &= ~LCD_COM1;
    }
    else
    {
        M_PIN18 &= ~LCD_COM1;
        M_PIN20 &= ~LCD_COM1;
        M_PIN22 &= ~LCD_COM1;
    }
#endif
}
//////////////////////////////////////////////////////////////
//segno "-" seconda riga: 0=positivo 1=negativo
void Sign2(unsigned char sign)
{
#ifdef LCD_100550
    if(!sign)
    {
        M_PIN24 |= LCD_COM5;
        M_PIN24 &= ~LCD_COM6;
    }
    else
    {
        M_PIN24 |= LCD_COM6;
        M_PIN24 &= ~LCD_COM5;
    }
#elif defined(LCD_110147)
    if(!sign)
    {
        M_PIN22 |= LCD_COM6;
        M_PIN22 &= ~LCD_COM7;
    }
    else
    {
        M_PIN22 |= LCD_COM7;
        M_PIN22 &= ~LCD_COM6;
    }
#endif
}

/////////////////////////////////////////////////////////////
//accensione punto decimale nella posizione indicata
//da chiamare dopo il riempimento della mappa LCDMx con i valori numerici
void ShowDot(unsigned char row,unsigned char position)
{
	if(row!=1 && row!=2)
		return;
	if(row==1)
	{
		switch(position)
		{
			case 1:
				M_PIN24 |= BIT3;
				break;
			case 2:
				M_PIN16 |= BIT3;
				break;
			case 3:
				M_PIN14 |= BIT3;
				break;
			case 4:
				M_PIN12 |= BIT3;
				break;
			case 5:
				M_PIN10 |= BIT3;
				break;
			case 6:
				M_PIN8 |= BIT3;
				break;
			case 7:
				M_PIN6 |= BIT3;
				break;
			case 8:
			    M_PIN4 |= BIT3;
			    break;
			case 9:
			    M_PIN2 |= BIT3;
			    break;

			default:
				break;
		}
	}
	if(row==2)
	{
		switch(position)
		{
			case 1:
                M_PIN16 |= LCD_COM8;
				break;
			case 2:
                M_PIN14 |= LCD_COM8;
				break;
			case 3:
                M_PIN12 |= LCD_COM8;
				break;
			case 4:
                M_PIN10 |= LCD_COM8;
				break;
			case 5:
                M_PIN8 |= LCD_COM8;
				break;
            case 6:
                M_PIN6 |= LCD_COM8;
                break;
			default:
				break;
		}
	}

}

//////////////////////////////////////////////
//visualizzazione icona GSM: show=1 attiva; level 0,1,2,3
void IconGSM(unsigned char show, unsigned char level)
{
	if(!show)
	{
		LCDM1 &= ~0xF0;
		return;
	}
	else
	{
		LCDM1 |= BIT4;
		LCDM1 &= ~0xE0;
		switch(level)
		{
		case 1:
			LCDM1 |= BIT5;
			break;
		case 2:
			LCDM1 |= BIT5 | BIT6;
			break;
		case 3:
			LCDM1 |= BIT5 | BIT6 | BIT7;
			break;
		default:
			break;
		}
	}
}


///////////////////////////////////////////////////
//visualizzazione icona allarme
void IconAlert(unsigned char OnOff)
{
	if(!OnOff)
		M_PIN4 &= ~LCD_COM6;
	else
	    M_PIN4 |= LCD_COM6;
}

//////////////////////////////////////////////
//visualizzazione icona batteria: level 0=non attivo; 1=vuoto; 2=1 tacca; 3= 2 tacche; 4=3 tacche
void IconBattery(_dispBatt_en level)
{
    M_PIN1 &= ~(LCD_COM6 | LCD_COM7);
    M_PIN2 &= ~(LCD_COM6 | LCD_COM7);

    switch(level)
    {
    case lcdBatt_off:
        break;
    case lcdBatt_empty:
        M_PIN2 |= LCD_COM6;
        break;
    case lcdBatt_L1_3:
        M_PIN2 |= LCD_COM6;
        M_PIN1 |= LCD_COM7;
        break;
    case lcdBatt_L2_3:
        M_PIN2 |= LCD_COM6;
        M_PIN1 |= LCD_COM7;
        M_PIN1 |= LCD_COM6;
        break;
    case lcdBatt_full:
        M_PIN2 |= LCD_COM6;
        M_PIN1 |= LCD_COM7;
        M_PIN1 |= LCD_COM6;
        M_PIN2 |= LCD_COM7;
        break;
    }

}

/////////////////////////////////////////////
//visualizzazione icone unità tecnica accumulatori
void IconAccUT(_dispTot_en selection)
{
	TOT_CLEAR_ALL;
	switch(selection)
	{
	case lcdTot_none:
		TOT_CLEAR_ALL;
		break;
	case lcdTot_m3:
		TOT_M3;
		break;
	case lcdTot_L:
		TOT_L;
		break;
	case lcdTot_gal:
		TOT_GAL;
		break;
	case lcdTot_bbl:
		TOT_BBL;
		break;
	case lcdTot_ft3:
	    TOT_FT3;
		break;

	default:
		break;

	}
}

///////////////////////////////////////////////////////////////////////////////
//impostazione icone unità istantanee + base dei tempi
void IconInstUT_TB(_dispFlow_en UTTB)
{
    INST_CLEAR_ALL;
	switch (UTTB)
	{
	case lcdFlow_none:
		INST_CLEAR_ALL;
		break;
	case lcdFlow_m3_h:
	    INST_M3_h;
		break;
	case lcdFlow_m3_s:
	    INST_M3_s;
		break;
	case lcdFlow_L_h:
	    INST_L_h;
		break;
	case lcdFlow_L_s:
	    INST_L_s;
		break;
	case lcdFlow_gpm:
	    INST_gpm;
		break;

	default:
		break;
	}

}








