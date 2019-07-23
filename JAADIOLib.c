/* 
 * File:   JAADSM
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */

//this enum stores the two constants for the IO_setPortDirection function




/** @param[in] port ID number(pinNumber) and INPUT or OUTPUT assignment (direction). 
pinNumber equals to 1 if direction equals to 1, otherwise pinNumber equals to 0.
setting the port's value to input or output
*/

#include "JAADIOLib.h"
int IO_setPortDirection(int pin, int direction){
	if (pin == SDA) {
		if(direction == INPUT){
			SDA_TRIS = 1;
		} else {
			SDA_TRIS = 0;
		}
	}
	else{
		if (direction == INPUT){

			SCL_TRIS = 1;
		}
		else{

			SCL_TRIS = 0;
		}
		
	}


}

/** @param[in] port ID number(pinNumber) and electricity value 1 or 0 (newValue). 
pinNumber equals to 1 if newValue equals to HIGH, otherwise pinNumber equals to 0.
set the value to either 1 (high voltage) or 0 (low voltage)
*/

int IO_setPort(int pin, int newValue){

	if (pin == SDA) {
		
		SDA_LAT = newValue;
		
	}
	else{
		
		SCL_LAT = newValue;
		
		
	}

}

/** @param[in] port ID number(pinNumber), read from the input value. 
Return (1) if pinNumber equals to 1, otherwise returns 0.
check if the input value is an input or output

*/

int IO_readPort(int pin){

	if(pin == SDA){

		return SDA_BIT;
	}
	else{

		return SCL_BIT;
	}

}









