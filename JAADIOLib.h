/* 
 * File:   JAADSM.h
 * Author: Daniel Lan
 *
 * Created on July 22, 2019, 4:58 PM
 */

#ifndef JAADIOLIB_H
#define	JAADIOLIB_H

#define SCL_TRIS TRISBbits.TRISB11
//Serial Clock Pin ID
#define SCL_BIT PORTBbits.RB11

#define SCL_LAT LATBbits.LATB11


#define SDA_TRIS TRISBbits.TRISB12
// Serial Data Pin ID
#define SDA_BIT PORTBbits.RB12

#define SDA_LAT LATBbits.LATB12

//Define true and false because C is dumb
#define true 1
#define false 0

//this enum stores the two constants for the IO_setPortDirection function
enum {
	INPUT,
	OUTPUT
};

//this enum stores the two constants for the IO_setPort function
enum {
	HIGH,
	LOW
};

//this enum stores the two constants for the potential ports
enum {
	SCL,
	SDA
};


int IO_setPortDirection(int pin, int direction);
// setting the port's value to input or output
/* example use
setPortDirection(SCL_TRIS, 0);
*/


int IO_setPort(int pin, int newValue);
// set the value to either 1 (high voltage) or 0 (low voltage)


int IO_readPort(int pin);
// read from the input value

#endif	/* JAADSM_H */


