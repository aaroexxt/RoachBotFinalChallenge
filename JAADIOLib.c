/* 
 * File:   JAADSM
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */

//this enum stores the two constants for the IO_setPortDirection function
enum {
	INPUT,
	OUTPUT
}

enum{
	HIGH,
	LOW
}


/** @param[in] port ID number(pinNumber) and INPUT or OUTPUT assignment (direction). 
pinNumber equals to 1 if direction equals to 1, otherwise pinNumber equals to 0.
setting the port's value to input or output
*/
int IO_setPortDirection(short pinNumber, int direction){

	if (direction == INPUT){

		pinNumber = 1;
	}
	else{

		pinNumber = 0;
	}

}

/** @param[in] port ID number(pinNumber) and electricity value 1 or 0 (newValue). 
pinNumber equals to 1 if newValue equals to HIGH, otherwise pinNumber equals to 0.
set the value to either 1 (high voltage) or 0 (low voltage)
*/

int IO_setPort(short pinNumber, int newValue){

	if (newValue == HIGH){

		pinNumber = 1;
	}
	else{

		pinNumber = 0;
	}
}

/** @param[in] port ID number(pinNumber), read from the input value. 
Return (1) if pinNumber equals to 1, otherwise returns 0.
*/

int IO_readPort(short pinNumber){

	if(pinNumber == 1){

		return 1;
	}
	else{

		return 0;
	}

}

/** @param[in] port ID number(pinNumber).
Return (1) if pinNumber equals to 1, otherwise returns 0.
check if the input value is an input or output
*/

char IO_checkInputValue(short pinNumber){

	if (pinNumber == 1){

		return 1;

	}
	else{

		return 0;
	}
}


