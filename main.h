/*
 * main.h
 *
 *  Created on: 13 Oct 2021
 *      Author: Enrico
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

/*
 * OLD hardware and display compatibility definitions
 */
//#define FIRST_HW_1_0        //define to compile the firmware for the first samples 1_0
//#define LCD_100550
#define LCD_110147

//

#define FW_VER  1
#define FW_REV  2

#define FW_DAY      3
#define FW_MONTH    4
#define FW_YEAR     24


extern uint8_t TimeputToMainFw;


#endif /* MAIN_H_ */
