/******************************************************************************
*
* Copyright (C) 2012 - 2020 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Default linker command file for Texas Instruments MSP430FR6047
*
*****************************************************************************/

/******************************************************************************/
/*                                                                            */
/*   Usage:  lnk430 <obj files...>    -o <out file> -m <map file> lnk.cmd     */
/*           cl430  <src files...> -z -o <out file> -m <map file> lnk.cmd     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* These linker options are for command line linking only.  For IDE linking,  */
/* you should set your linker options in Project Properties                   */
/* -c                                               LINK USING C CONVENTIONS  */
/* -stack  0x0100                                   SOFTWARE STACK SIZE       */
/* -heap   0x0100                                   HEAP AREA SIZE            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* 1.209 */
/*----------------------------------------------------------------------------*/
-stack 0x0400

//#define __USS_RUN_ALG_FROM_RAM__

/****************************************************************************/
/* SPECIFY THE SYSTEM MEMORY MAP                                            */
/****************************************************************************/
//UNIFLASH SETTINGS: erase option by address range 0x4000 / 0x43EFF
MEMORY
{
    TINYRAM                 : origin = 0xA, length = 0x16
    BSL                     : origin = 0x1000, length = 0x800
    CAL_CONF                : origin = 0x1900, length = 0x100
    TLVMEM                  : origin = 0x1A00, length = 0x100
    BOOTROM                 : origin = 0x1B00, length = 0x100
    RAM                     : origin = 0x1C00, length = 0x1000
    BOOTLOADER				: origin = 0x4000, length = 0x3C00
    //RELOC_INT				: origin = 0x7C00, length = 0x80	//re-defined interrupt table
    FREE_MEM_1				: origin = 0x7C6E, length = 0x12
    MAIN_PROGRAM            : origin = 0x7C80, length = 0x817E
    DELIMITER_1				: origin = 0xFDFE, length = 2	//delimiters are necessary to have the code sections starting with @..... and ending before another @...... on the .txt file
    DELIMITER_2				: origin = 0x10000, length = 2
    MAIN_PROGRAM2         	: origin = 0x10002,length = 0x2F7FC
    DELIMITER_3				: origin = 0x3F7FE, length = 2
    FW_CRC32				: origin = 0x3F800, length = 0x4
    FW_OTHER_INFO			: origin = 0x3F804, length = 0x3FC

    FRAM_PAR_LOG			: origin = 0x3FC00,length = 0x0400	//parameters log modify attempt
    FRAM_USS_PARAM			: origin = 0x40000,length = 0x400	//data section reserved to USS FRAM variables / parameters
    FRAM_VAR				: origin = 0x40400,length = 0x3B00	//data section used as RAM for variables
    OTHER_FRAM_NO_ERASE		: origin = 0x43F00,length = 0xAC
    TOTALIZERS				: origin = 0x43FAC,length = 0x34	//data section reserved to store the totalizers struct and a backup copy
    VERSION                 : origin = 0x43FE0,length = 0x0008
    LIB_VERSION             : origin = 0x43FE8,length = 0x0008
    BATTERY_DATA    		: origin = 0x43FF0,length = 0x0010

    BRINTVEC				: origin = 0xFE00, length = 0x180, fill = 0xFFFF	//interrupt branch section: contains jump instructions that point to the PROXY_VECTORS table


    JTAGSIGNATURE           : origin = 0xFF80, length = 0x0004, fill = 0xFFFF
    BSLSIGNATURE            : origin = 0xFF84, length = 0x0004, fill = 0xFFFF
    IPESIGNATURE            : origin = 0xFF88, length = 0x0008, fill = 0xFFFF

    PROXY_VECTORS			: origin = 0xFF90, length = 0x6E
    RESET                   : origin = 0xFFFE, length = 0x0002

	/*
	relocated interrupt table
	*/
    /*INT00                   : origin = 0x7C00, length = 0x0002, fill = 0xFFFF
    INT01                   : origin = 0x7C02, length = 0x0002, fill = 0xFFFF
    INT02                   : origin = 0x7C04, length = 0x0002, fill = 0xFFFF
    INT03                   : origin = 0x7C06, length = 0x0002, fill = 0xFFFF
    INT04                   : origin = 0x7C08, length = 0x0002, fill = 0xFFFF
    INT05                   : origin = 0x7C0A, length = 0x0002, fill = 0xFFFF
    INT06                   : origin = 0x7C0C, length = 0x0002, fill = 0xFFFF
    INT07                   : origin = 0x7C0E, length = 0x0002, fill = 0xFFFF
    INT08                   : origin = 0x7C10, length = 0x0002, fill = 0xFFFF
    INT09                   : origin = 0x7C12, length = 0x0002, fill = 0xFFFF
    INT10                   : origin = 0x7C14, length = 0x0002, fill = 0xFFFF
    INT11                   : origin = 0x7C16, length = 0x0002, fill = 0xFFFF
    INT12                   : origin = 0x7C18, length = 0x0002, fill = 0xFFFF
    INT13                   : origin = 0x7C1A, length = 0x0002, fill = 0xFFFF
    INT14                   : origin = 0x7C1C, length = 0x0002, fill = 0xFFFF
    INT15                   : origin = 0x7C1E, length = 0x0002, fill = 0xFFFF
    INT16                   : origin = 0x7C20, length = 0x0002, fill = 0xFFFF
    INT17                   : origin = 0x7C22, length = 0x0002, fill = 0xFFFF
    INT18                   : origin = 0x7C24, length = 0x0002, fill = 0xFFFF
    INT19                   : origin = 0x7C26, length = 0x0002, fill = 0xFFFF
    INT20                   : origin = 0x7C28, length = 0x0002, fill = 0xFFFF
    INT21                   : origin = 0x7C2A, length = 0x0002, fill = 0xFFFF
    INT22                   : origin = 0x7C2C, length = 0x0002, fill = 0xFFFF
    INT23                   : origin = 0x7C2E, length = 0x0002, fill = 0xFFFF
    INT24                   : origin = 0x7C30, length = 0x0002, fill = 0xFFFF
    INT25                   : origin = 0x7C32, length = 0x0002, fill = 0xFFFF
    INT26                   : origin = 0x7C34, length = 0x0002, fill = 0xFFFF
    INT27                   : origin = 0x7C36, length = 0x0002, fill = 0xFFFF
    INT28                   : origin = 0x7C38, length = 0x0002, fill = 0xFFFF
    INT29                   : origin = 0x7C3A, length = 0x0002, fill = 0xFFFF
    INT30                   : origin = 0x7C3C, length = 0x0002, fill = 0xFFFF
    INT31                   : origin = 0x7C3E, length = 0x0002, fill = 0xFFFF
    INT32                   : origin = 0x7C40, length = 0x0002, fill = 0xFFFF
    INT33                   : origin = 0x7C42, length = 0x0002, fill = 0xFFFF
    INT34                   : origin = 0x7C44, length = 0x0002, fill = 0xFFFF
    INT35                   : origin = 0x7C46, length = 0x0002, fill = 0xFFFF
    INT36                   : origin = 0x7C48, length = 0x0002, fill = 0xFFFF
    INT37                   : origin = 0x7C4A, length = 0x0002, fill = 0xFFFF
    INT38                   : origin = 0x7C4C, length = 0x0002, fill = 0xFFFF
    INT39                   : origin = 0x7C4E, length = 0x0002, fill = 0xFFFF
    INT40                   : origin = 0x7C50, length = 0x0002, fill = 0xFFFF
    INT41                   : origin = 0x7C52, length = 0x0002, fill = 0xFFFF
    INT42                   : origin = 0x7C54, length = 0x0002, fill = 0xFFFF
    INT43                   : origin = 0x7C56, length = 0x0002, fill = 0xFFFF
    INT44                   : origin = 0x7C58, length = 0x0002, fill = 0xFFFF
    INT45                   : origin = 0x7C5A, length = 0x0002, fill = 0xFFFF
    INT46                   : origin = 0x7C5C, length = 0x0002, fill = 0xFFFF
    INT47                   : origin = 0x7C5E, length = 0x0002, fill = 0xFFFF
    INT48                   : origin = 0x7C60, length = 0x0002, fill = 0xFFFF
    INT49                   : origin = 0x7C62, length = 0x0002, fill = 0xFFFF
    INT50                   : origin = 0x7C64, length = 0x0002, fill = 0xFFFF
    INT51                   : origin = 0x7C66, length = 0x0002, fill = 0xFFFF
    INT52                   : origin = 0x7C68, length = 0x0002, fill = 0xFFFF
    INT53                   : origin = 0x7C6A, length = 0x0002, fill = 0xFFFF
    INT54                   : origin = 0x7C6C, length = 0x0002, fill = 0xFFFF
*/

    /*INT00                   : origin = 0xFF90, length = 0x0002
    INT01                   : origin = 0xFF92, length = 0x0002
    INT02                   : origin = 0xFF94, length = 0x0002
    INT03                   : origin = 0xFF96, length = 0x0002
    INT04                   : origin = 0xFF98, length = 0x0002
    INT05                   : origin = 0xFF9A, length = 0x0002
    INT06                   : origin = 0xFF9C, length = 0x0002
    INT07                   : origin = 0xFF9E, length = 0x0002
    INT08                   : origin = 0xFFA0, length = 0x0002
    INT09                   : origin = 0xFFA2, length = 0x0002
    INT10                   : origin = 0xFFA4, length = 0x0002
    INT11                   : origin = 0xFFA6, length = 0x0002
    INT12                   : origin = 0xFFA8, length = 0x0002
    INT13                   : origin = 0xFFAA, length = 0x0002
    INT14                   : origin = 0xFFAC, length = 0x0002
    INT15                   : origin = 0xFFAE, length = 0x0002
    INT16                   : origin = 0xFFB0, length = 0x0002
    INT17                   : origin = 0xFFB2, length = 0x0002
    INT18                   : origin = 0xFFB4, length = 0x0002
    INT19                   : origin = 0xFFB6, length = 0x0002
    INT20                   : origin = 0xFFB8, length = 0x0002
    INT21                   : origin = 0xFFBA, length = 0x0002
    INT22                   : origin = 0xFFBC, length = 0x0002
    INT23                   : origin = 0xFFBE, length = 0x0002
    INT24                   : origin = 0xFFC0, length = 0x0002
    INT25                   : origin = 0xFFC2, length = 0x0002
    INT26                   : origin = 0xFFC4, length = 0x0002
    INT27                   : origin = 0xFFC6, length = 0x0002
    INT28                   : origin = 0xFFC8, length = 0x0002
    INT29                   : origin = 0xFFCA, length = 0x0002
    INT30                   : origin = 0xFFCC, length = 0x0002
    INT31                   : origin = 0xFFCE, length = 0x0002
    INT32                   : origin = 0xFFD0, length = 0x0002
    INT33                   : origin = 0xFFD2, length = 0x0002
    INT34                   : origin = 0xFFD4, length = 0x0002
    INT35                   : origin = 0xFFD6, length = 0x0002
    INT36                   : origin = 0xFFD8, length = 0x0002
    INT37                   : origin = 0xFFDA, length = 0x0002
    INT38                   : origin = 0xFFDC, length = 0x0002
    INT39                   : origin = 0xFFDE, length = 0x0002
    INT40                   : origin = 0xFFE0, length = 0x0002
    INT41                   : origin = 0xFFE2, length = 0x0002
    INT42                   : origin = 0xFFE4, length = 0x0002
    INT43                   : origin = 0xFFE6, length = 0x0002
    INT44                   : origin = 0xFFE8, length = 0x0002
    INT45                   : origin = 0xFFEA, length = 0x0002
    INT46                   : origin = 0xFFEC, length = 0x0002
    INT47                   : origin = 0xFFEE, length = 0x0002
    INT48                   : origin = 0xFFF0, length = 0x0002
    INT49                   : origin = 0xFFF2, length = 0x0002
    INT50                   : origin = 0xFFF4, length = 0x0002
    INT51                   : origin = 0xFFF6, length = 0x0002
    INT52                   : origin = 0xFFF8, length = 0x0002
    INT53                   : origin = 0xFFFA, length = 0x0002
    INT54                   : origin = 0xFFFC, length = 0x0002*/

}

/****************************************************************************/
/* Specify the LEA memory map                                               */
/****************************************************************************/

#define LEASTACK_SIZE   0x138

MEMORY
{
    LEARAM                  : origin = 0x2C00, length = 0x1000 - LEASTACK_SIZE
    LEASTACK                : origin = 0x3C00 - LEASTACK_SIZE, length = LEASTACK_SIZE
}

/****************************************************************************/
/* SPECIFY THE SECTIONS ALLOCATION INTO MEMORY                              */
/****************************************************************************/
//--preferred_order=_c_int00_noargs	>>>>necessary?

SECTIONS
{
    GROUP(RW_IPE)
    {

        GROUP(READ_WRITE_MEMORY)
        {
            .TI.persistent : {}              /* For #pragma persistent            */
            .cio           : {}              /* C I/O Buffer                      */
            .sysmem        : {}              /* Dynamic memory allocation area    */
        } PALIGN(0x0400) RUN_START(fram_rw_start)

        GROUP(IPENCAPSULATED_MEMORY)
        {

            .ipestruct     : {}              /* IPE Data structure             */
            .ipe           : {}              /* IPE                            */
            .ipe_const     : {}              /* IPE Protected constants        */
            .ipe:_isr      : {}              /* IPE ISRs                       */
        } PALIGN(0x0400), RUN_START(fram_ipe_start) RUN_END(fram_ipe_end) RUN_END(fram_rx_start)

    } >0x4000


    .cinit            : {}  > BOOTLOADER          /* Initialization tables             */
    .binit            : {}  > BOOTLOADER          /* Boot-time Initialization tables   */
    .pinit            : {}  > BOOTLOADER          /* C++ Constructor tables            */
    .init_array       : {}  > BOOTLOADER          /* C++ Constructor tables            */
    .mspabi.exidx     : {}  > BOOTLOADER          /* C++ Constructor tables            */
    .mspabi.extab     : {}  > BOOTLOADER          /* C++ Constructor tables            */
    .text:_isr:_c_int00_noargs_mpu : {} > _c_int00_address
    //.freeMem1		  : {}  > FREE_MEM_1
    .text:_isr        : {}  > BOOTLOADER          /* Code ISRs   						 */

    //.TI.persistent    : {}  > FRAM_VAR                 /* For #pragma persistent            */
    .brintvec 		  : {} > BRINTVEC            /* BRANCH INST PROGRAM INT VECTOR TABLE */
	.proxy_vectors    : {} > PROXY_VECTORS
#ifndef __USS_RUN_ALG_FROM_RAM__
	.USS_ramfunc      : {}  > BOOTLOADER
#endif

//#ifndef __LARGE_DATA_MODEL__
    .const            : {} > BOOTLOADER           /* Constant data                     */
//#else
//    .const            : {} >> BOOTLOADER | BOOTLOADER2  /* Constant data                     */
//#endif

//#ifndef __LARGE_CODE_MODEL__
    .text             : {} > BOOTLOADER           /* Code                              */
//#else
//    .text             : {} >> BOOTLOADER2 | BOOTLOADER  /* Code                              */
//#endif

    #ifdef __TI_COMPILER_VERSION__
        #if __TI_COMPILER_VERSION__ >= 15009000
            //#ifndef __LARGE_CODE_MODEL__
                .TI.ramfunc : {} load=BOOTLOADER, run=RAM, table(BINIT)
            //#else
            //    .TI.ramfunc : {} load=BOOTLOADER | BOOTLOADER2, run=RAM, table(BINIT)
            //#endif
        #endif
    #endif

	#ifdef __USS_RUN_ALG_FROM_RAM__
	    #ifdef __TI_COMPILER_VERSION__
	        #if __TI_COMPILER_VERSION__ >= 15009000
				//#ifndef __LARGE_DATA_MODEL__
	                .USS_ramfunc : {} load=BOOTLOADER, run=RAM, table(BINIT)
				//#else
	            //    .USS_ramfunc : {} load=BOOTLOADER | BOOTLOADER2, run=RAM, table(BINIT)
	        	//#endif
		#endif
	#endif
#endif

    .jtagsignature      : {} > JTAGSIGNATURE
    .bslsignature       : {} > BSLSIGNATURE

    GROUP(SIGNATURE_SHAREDMEMORY)
    {
        .ipesignature       : {}            /* IPE Signature                     */
        .jtagpassword       : {}            /* JTAG Password                     */
    } > IPESIGNATURE

    #ifdef __USS_RUN_ALG_FROM_RAM__
        .bss        : {} > RAM | BOOTLOADER                 /* Global & static vars              */
        .data       : {} > RAM | BOOTLOADER                 /* Global & static vars              */
        .TI.noinit  : {} > RAM | BOOTLOADER                 /* For #pragma noinit                */
    #else
        .bss        : {} > RAM                 /* Global & static vars              */
        .data       : {} > RAM 			       /* Global & static vars              */
        .TI.noinit  : {} > RAM                 /* For #pragma noinit                */
    #endif
    .stack      : {} > RAM (HIGH)           /* Software system stack             */

    .tinyram    : {} > TINYRAM              /* Tiny RAM                          */

    .leaRAM      : {} > LEARAM               /* LEA RAM                           */
    .leaStack    : {} > LEASTACK (HIGH)      /* LEA STACK                         */

    /* MSP430 interrupt vectors */
/*
    .int00       : {}               > INT00
    .int01       : {}               > INT01
    .int02       : {}               > INT02
    .int03       : {}               > INT03
    .int04       : {}               > INT04
    .int05       : {}               > INT05
    .int06       : {}               > INT06
    .int07       : {}               > INT07
    .int08       : {}               > INT08
    .int09       : {}               > INT09
    .int10       : {}               > INT10
    .int11       : {}               > INT11
    .int12       : {}               > INT12
    .int13       : {}               > INT13
    SDHS         : { * ( .int14 ) } > INT14 type = VECT_INIT
    SAPH         : { * ( .int15 ) } > INT15 type = VECT_INIT
    HSPLL        : { * ( .int16 ) } > INT16 type = VECT_INIT
    UUPS         : { * ( .int17 ) } > INT17 type = VECT_INIT
    LEA          : { * ( .int18 ) } > INT18 type = VECT_INIT
    PORT9        : { * ( .int19 ) } > INT19 type = VECT_INIT
    PORT8        : { * ( .int20 ) } > INT20 type = VECT_INIT
    PORT7        : { * ( .int21 ) } > INT21 type = VECT_INIT
    EUSCI_B1     : { * ( .int22 ) } > INT22 type = VECT_INIT
    EUSCI_A3     : { * ( .int23 ) } > INT23 type = VECT_INIT
    EUSCI_A2     : { * ( .int24 ) } > INT24 type = VECT_INIT
    PORT6        : { * ( .int25 ) } > INT25 type = VECT_INIT
    PORT5        : { * ( .int26 ) } > INT26 type = VECT_INIT
    TIMER4_A1    : { * ( .int27 ) } > INT27 type = VECT_INIT
    TIMER4_A0    : { * ( .int28 ) } > INT28 type = VECT_INIT
    AES256       : { * ( .int29 ) } > INT29 type = VECT_INIT
    RTC_C        : { * ( .int30 ) } > INT30 type = VECT_INIT
    LCD_C        : { * ( .int31 ) } > INT31 type = VECT_INIT
    PORT4        : { * ( .int32 ) } > INT32 type = VECT_INIT
    PORT3        : { * ( .int33 ) } > INT33 type = VECT_INIT
    TIMER3_A1    : { * ( .int34 ) } > INT34 type = VECT_INIT
    TIMER3_A0    : { * ( .int35 ) } > INT35 type = VECT_INIT
    PORT2        : { * ( .int36 ) } > INT36 type = VECT_INIT
    TIMER2_A1    : { * ( .int37 ) } > INT37 type = VECT_INIT
    TIMER2_A0    : { * ( .int38 ) } > INT38 type = VECT_INIT
    PORT1        : { * ( .int39 ) } > INT39 type = VECT_INIT
    TIMER1_A1    : { * ( .int40 ) } > INT40 type = VECT_INIT
    TIMER1_A0    : { * ( .int41 ) } > INT41 type = VECT_INIT
    DMA          : { * ( .int42 ) } > INT42 type = VECT_INIT
    EUSCI_A1     : { * ( .int43 ) } > INT43 type = VECT_INIT
    TIMER0_A1    : { * ( .int44 ) } > INT44 type = VECT_INIT
    TIMER0_A0    : { * ( .int45 ) } > INT45 type = VECT_INIT
    ADC12_B      : { * ( .int46 ) } > INT46 type = VECT_INIT
    EUSCI_B0     : { * ( .int47 ) } > INT47 type = VECT_INIT
    EUSCI_A0     : { * ( .int48 ) } > INT48 type = VECT_INIT
    WDT          : { * ( .int49 ) } > INT49 type = VECT_INIT
    TIMER0_B1    : { * ( .int50 ) } > INT50 type = VECT_INIT
    TIMER0_B0    : { * ( .int51 ) } > INT51 type = VECT_INIT
    COMP_E       : { * ( .int52 ) } > INT52 type = VECT_INIT
    UNMI         : { * ( .int53 ) } > INT53 type = VECT_INIT
    SYSNMI       : { * ( .int54 ) } > INT54 type = VECT_INIT*/
    .reset       : {}               > RESET  /* MSP430 reset vector
*/
}
/****************************************************************************/
/* MPU/IPE SPECIFIC MEMORY SEGMENT DEFINITONS                               */
/****************************************************************************/

#ifdef _IPE_ENABLE
    #define IPE_MPUIPLOCK 0x0080
    #define IPE_MPUIPENA 0x0040
    #define IPE_MPUIPPUC 0x0020

    // Evaluate settings for the control setting of IP Encapsulation
    #if defined(_IPE_ASSERTPUC1)
        #if defined(_IPE_LOCK ) && (_IPE_ASSERTPUC1 == 0x08))
            fram_ipe_enable_value = (IPE_MPUIPENA | IPE_MPUIPPUC |IPE_MPUIPLOCK);
        #elif defined(_IPE_LOCK )
            fram_ipe_enable_value = (IPE_MPUIPENA | IPE_MPUIPLOCK);
        #elif (_IPE_ASSERTPUC1 == 0x08)
            fram_ipe_enable_value = (IPE_MPUIPENA | IPE_MPUIPPUC);
        #else
            fram_ipe_enable_value = (IPE_MPUIPENA);
        #endif
    #else
        #if defined(_IPE_LOCK )
            fram_ipe_enable_value = (IPE_MPUIPENA | IPE_MPUIPLOCK);
        #else
            fram_ipe_enable_value = (IPE_MPUIPENA);
        #endif
    #endif

    // Segment definitions
    #ifdef _IPE_MANUAL                  // For custom sizes selected in the GUI
        fram_ipe_border1 = (_IPE_SEGB1>>4);
        fram_ipe_border2 = (_IPE_SEGB2>>4);
    #else                           // Automated sizes generated by the Linker
        fram_ipe_border2 = fram_ipe_end >> 4;
        fram_ipe_border1 = fram_ipe_start >> 4;
    #endif

    fram_ipe_settings_struct_address = Ipe_settingsStruct >> 4;
    fram_ipe_checksum = ~((fram_ipe_enable_value & fram_ipe_border2 & fram_ipe_border1) | (fram_ipe_enable_value & ~fram_ipe_border2 & ~fram_ipe_border1) | (~fram_ipe_enable_value & fram_ipe_border2 & ~fram_ipe_border1) | (~fram_ipe_enable_value & ~fram_ipe_border2 & fram_ipe_border1));
#endif

#ifdef _MPU_ENABLE
    #define MPUPW (0xA500)    /* MPU Access Password */
    #define MPUENA (0x0001)   /* MPU Enable */
    #define MPULOCK (0x0002)  /* MPU Lock */
    #define MPUSEGIE (0x0010) /* MPU Enable NMI on Segment violation */

    __mpu_enable = 1;
    // Segment definitions
    #ifdef _MPU_MANUAL // For custom sizes selected in the GUI
        mpu_segment_border1 = _MPU_SEGB1 >> 4;
        mpu_segment_border2 = _MPU_SEGB2 >> 4;
        mpu_sam_value = (_MPU_SAM0 << 12) | (_MPU_SAM3 << 8) | (_MPU_SAM2 << 4) | _MPU_SAM1;
    #else // Automated sizes generated by Linker
        #ifdef _IPE_ENABLE //if IPE is used in project too
        //seg1 = any read + write persistent variables
        //seg2 = ipe = read + write + execute access
        //seg3 = code, read + execute only
    	       mpu_segment_border1 = fram_ipe_start >> 4;
    	       mpu_segment_border2 = fram_rx_start >> 4;
    	       mpu_sam_value = 0x1573; // Info R, Seg3 RX, Seg2 RWX, Seg1 RW
        #else
    	       mpu_segment_border1 = fram_rx_start >> 4;
    	       mpu_segment_border2 = fram_rx_start >> 4;
    	       mpu_sam_value = 0x1513; // Info R, Seg3 RX, Seg2 R, Seg1 RW
        #endif
    #endif
    #ifdef _MPU_LOCK
        #ifdef _MPU_ENABLE_NMI
            mpu_ctl0_value = MPUPW | MPUENA | MPULOCK | MPUSEGIE;
        #else
            mpu_ctl0_value = MPUPW | MPUENA | MPULOCK;
        #endif
    #else
        #ifdef _MPU_ENABLE_NMI
            mpu_ctl0_value = MPUPW | MPUENA | MPUSEGIE;
        #else
            mpu_ctl0_value = MPUPW | MPUENA;
        #endif
    #endif
#endif

/****************************************************************************/
/* INCLUDE PERIPHERALS MEMORY MAP                                           */
/****************************************************************************/

-l msp430fr60471.cmd
