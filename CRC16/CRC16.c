
#include "CRC16.h"
#include <msp430.h>


/////////////////////////////////////////////////////////////

void GenerateCRC16(unsigned char *ByteStart, unsigned short dataLenght, unsigned char *DestinationHigh, unsigned char *DestinationLow)
{
    unsigned short CheckSum;
    unsigned short j;
    unsigned char lowCRC;
    unsigned char highCRC;
    unsigned short i;


    CheckSum = 0xCBFE;
    for (j = 0; j < dataLenght; j++)
    {
        CheckSum = CheckSum^(unsigned short) (*ByteStart); // XOR
        for (i = 8; i > 0; i--)
        {
            if((CheckSum & 0x0001) != 0)
            {
                CheckSum >>= 1;
                CheckSum ^= 0xA001;
            }
            else
            {
                CheckSum >>= 1;
            }
        }
        ByteStart++;
    }
    highCRC = CheckSum >> 8;
    CheckSum <<= 8;
    lowCRC = CheckSum >> 8;
    *DestinationHigh = highCRC;
    *DestinationLow = lowCRC;

    return;

}

////////////////////////////////////////////////////////////////

bool CheckCRC16(unsigned char *ByteStart, unsigned short dataLenght, unsigned char CRChighToCheck, unsigned char CRClowToCheck)
{
    unsigned short CheckSum;
    unsigned short j;
    volatile unsigned char lowCRC;
    volatile unsigned char highCRC;
    unsigned short i;


    CheckSum = 0xCBFE;
    for (j = 0; j < dataLenght; j++)
    {
        CheckSum = CheckSum^(unsigned short) (*ByteStart); // XOR
        for (i = 8; i > 0; i--)
            if ((CheckSum)&0x0001)
                CheckSum = (CheckSum >> 1)^0xa001;
            else
                CheckSum >>= 1;
        ByteStart++;
    }
    highCRC = CheckSum >> 8;
    CheckSum <<= 8;
    lowCRC = CheckSum >> 8;
    if ((lowCRC == CRClowToCheck) && (highCRC == CRChighToCheck))
        return true;
    else
        return false;

}

/////////////////////////////////////////////////////////////
