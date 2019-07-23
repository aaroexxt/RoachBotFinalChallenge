/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */

#include "JAADI2CLib.h"
#include "JAADIOLib.h"

char I2C_initted = FALSE;

int I2C_Init(unsigned int speed) {
	if (!I2C_initted) {
		I2C_initted = true;
		//Set SCL/SDA to OUTPUTS
		//Set SCL/SDA to HIGH
		if (IO_checkLAT(SCLLAT))

	}
}
void internal_startSeq(char I2Caddress) {

}