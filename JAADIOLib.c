/* 
 * File:   JAADSM
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */


enum {
	INPUT,
	OUTPUT
}

enum{
	HIGH,
	LOW
}

int setPortDirection(short pinNumber, int direction){

	if (direction == INPUT){

		pinNumber = 1;
	}
	else{

		pinNumber = 0;
	}

}

// setting the port's value to input or output


int setPort(short pinNumber, int newValue){
// set the value to either 1 (high voltage) or 0 (low voltage)
	if (newValue == HIGH){

		pinNumber = 1;
	}
	else{

		pinNumber = 0;
	}
}

int readPort(short pinNumber){

	if(pinNumber == 1){

		return 1;
	}
	else{

		return 0;
	}

}
// read from the input value


char checkInputValue(short pinNumber){

	if (pinNumber == 1){

		return 1;

	}
	else{

		return 0;
	}
}
// check if the input value is an input or output


