/******************************************************************************
 *
 * Author: Gabriel LoDolce
 * Author of last change: Matthew Paley
 * Date of last change: 7/28/2014
 * Revision: 1.0
 *
 *******************************************************************************
 * 
 * FileName:        LCDroutines.c
 * Dependencies:    Delays.h and p18cxxx.h
 * Processor:       PIC18F
 * Compiler:        C18
 *
 *******************************************************************************
 *
 * File Description: Implementation file for the LCD routines library
 *
 ******************************************************************************/

#include <delays.h>
#include <string.h>
#include "LCDroutines.h"

/*------------------------------------------------------------------------------
 * Static variables for internal use (not visible outside of this file)
 -----------------------------------------------------------------------------*/

/* The following instruction delays assume 10MHz clock. These can be changed for
** other oscillator frequencies but note the valid range is only 0-255
*/
static const unsigned char InitDelayCount_ = 25; // Instruction cycles/10000 for 0.1 second delay
static const unsigned char CharDelayCount_ = 30; // Instruction cycles/10 for 40usec delay

static const rom far char LCDInitStr_[] = "\x33\x32\x28\x01\x0C\x06\x00"; // LCD initialization

/*------------------------------------------------------------------------------
 * Public functions intended for the user
 -----------------------------------------------------------------------------*/

/********************************************************************
 *     Function Name:    
 *     Parameters:       
 *     Description:      
 *
 ********************************************************************/
void InitLCD(void) {
	unsigned char count = 0;

	// Delay 0.1 second for the LCD controller to come out of reset
	Delay10KTCYx( InitDelayCount_ );

	// Drive RS low for command
	LCD_RS_LAT = 0;

	// Send Each Byte one nibble at a time until we see the null character
	while( LCDInitStr_[count] != 0x00 ) {
		LCD_E_LAT = 1;                          // Drive E high
		LCD_DATA_LAT = LCDInitStr_[count];	// Send the upper nibble
		LCD_E_LAT = 0;                          // Drive E low so LCD will process input
		Delay10KTCYx( InitDelayCount_/10 );

		LCD_E_LAT = 1;                          // Drive E high
		LCD_DATA_LAT = LCDInitStr_[count] << 4;	// Send the lower nibble
		LCD_E_LAT = 0;                          // Drive E low so LCD will process input
		Delay10KTCYx( InitDelayCount_/10 );

		count++;                                // Increment the counter
	}

	LCD_RS_LAT = 1;                                 // Drive RS high to exit command mode
}

/******************************************************************************
 *     Function Name:	DisplayC
 *     Parameters:      Pointer to a character array in program memory
 *     Description:		This function sends a character array in program memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayC( const rom far char * LCDStr ) {
    char temp[10];   // Temporary buffer to store input in
    strcpypgm2ram(temp,LCDStr); // Creating proper data input for DisplayV
    DisplayV(temp);             // Calling DisplayV function
}

/******************************************************************************
 *     Function Name:	DisplayV
 *     Parameters:      Pointer to a character array in data memory
 *     Description:		This function sends a character array in data memory
 *						to the LCD display. Note the first character of the
 *						string is the positioning commmand. The string must
 *						also be terminated by the null character
 *
 *						This function generates a 40us delay using the c18
 *						Delay library
 *
 ******************************************************************************/
void DisplayV( const char * LCDStr ) {
    unsigned char count = 0;
    LCD_RS_LAT = 0;                 // Telling the LDC we are about to transmit
    while(LCDStr[count] != 0x00) {
        // Transmitting the upper nibble
        LCD_E_LAT = 1;
        LCD_DATA_LAT = LCDStr[count];
        LCD_E_LAT = 0;

        // Transmitting the lower nibble
        LCD_E_LAT = 1;
        LCD_DATA_LAT = (LCDStr[count] << 4);
        LCD_E_LAT = 0;

        Delay10TCYx( CharDelayCount_ ); // Delay to allow LCD to display char

        count++;
        LCD_RS_LAT = 1;
    }
}
