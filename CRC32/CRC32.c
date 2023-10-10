#include "crc32.h"
#include "IntegrityCheck/IntegrityCheck.h"
#include "Display_UI/Display_UI.h"
/*
 * File:   crc32.h
 * Author: examples of TI
 *g(x) slaa221    MSP430 CRC alogrithm slaa221.pdf
 *Created on 25 giugno 2014, 9.54
 */
/*
 FRAM                    : origin = 0x4000, length = 0xBF80
 FRAM2                   : origin = 0x10000,length = 0x30000
*/



#define CRC32_POLY          0x04C11DB7
#define CRC32_INIT_REM      0xA0019999

#define CRC32_FINAL_XOR     0xFFFFFFFF


uint32_t crc32_engine( uint32_t currCrc, uint8_t* bytePtr, uint32_t size )
{
    uint32_t count = 0;
    for ( ; count < size; ++count )
    {
        uint32_t msg = *bytePtr++;
        msg <<= 24;

        uint8_t j;
        for(j = 0 ; j < 8 ; j++)
        {
            if ((msg ^ currCrc) >> 31)
                currCrc = (currCrc << 1) ^ CRC32_POLY;
            else
                currCrc <<= 1;
            msg <<= 1;
        }
    }

    return currCrc;

}

uint32_t crc32_reset()
{
    return CRC32_INIT_REM;
}

uint32_t crc32_result( uint32_t currCrc )
{
    return(currCrc ^ CRC32_FINAL_XOR);
}

//--






