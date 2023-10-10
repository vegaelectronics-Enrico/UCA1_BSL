#ifndef CRC16_H_
#define CRC16_H_

#include <stdbool.h>

void GenerateCRC16(unsigned char *ByteStart,unsigned short dataLenght, unsigned char *DestinationHigh, unsigned char *DestinationLow);
bool CheckCRC16(unsigned char *ByteStart,unsigned short dataLenght,unsigned char CRChighToCheck,unsigned char CRClowToCheck);

#endif
