/*
 * File:   crc32.h
 * Author: examples of TI
 *g(x) slaa221    MSP430 CRC alogrithm slaa221.pdf
 *Created on 25 giugno 2014, 9.54
 */

#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//! \brief Calculate and returns the CRC-32 of the given byte array
//!
//! This function computes and returns the value of the CRC-32 of the given array
//! skipping the portion starting at index StartSkipPosition and ending at index
//! StopSkipPosition.
//! The algorith uses the following parameters
//! - initial seed 0xffffffff
//! - polynomial 0x04C11DB7
//! - no reversed input
//! - no reversed output
//!
//! \param ByteStart is the pointer at the array start
//! \param dataLenght is the length of data to compute the CRC i.e. not including the size of the gap (must be <= of array length)
//! \param StartSkipPosition is the index of the first byte to be skipped
//! \param StopSkipPosition is the index of the last byte to be skipped
//!
//! \return the calculated CRC
//
//*****************************************************************************


uint32_t crc32_reset();

uint32_t crc32_engine( uint32_t currCrc, uint8_t* bytePtr, uint32_t size );

uint32_t crc32_result( uint32_t currCrc );



#endif /* CRC32_H_ */
