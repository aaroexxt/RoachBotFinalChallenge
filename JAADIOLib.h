/* 
 * File:   JAADSM.h
 * Author: Aaron
 *
 * Created on July 22, 2019, 4:58 PM
 */

#ifndef JAADIOLIB_H
#define	JAADIOLIB_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SCL_TRIS TRISBbits.TRISB9
//Serial Clock Pin ID

#define SCL_BIT PORTBbits.RB9

#define SCL_LAT LATBbits.LATB9


#define SDA_TRIS TRISBbits.TRISB8
// Serial Data Pin ID
#define SDA_BIT PORTBbits.RB8

#define SDA_LAT LATBbits.LATB8


int setPortDirection(short pinNumber, char direction);
// setting the port's value to input or output
/* example use
setPortDirection(SCL_TRIS, 0);
*/


int setPort(short pinNumber, char newValue);
// set the value to either 1 (high voltage) or 0 (low voltage)


char readPort(short pinNumber);
// read from the input value


char checkInputValue(short pinNumber);
// check if the input value is an input or output







#ifdef	__cplusplus
}
#endif

#endif	/* JAADSM_H */


