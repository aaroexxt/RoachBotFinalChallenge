/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */

//SOmeone please help i am writing this completely custom lol


/*
I2C COMMUNICATION LETS GET IT


START:
SDA goes high -> low before SCL high -> low
STOP:
SDA goes low -> high after SCL low -> high
ADDRESS:
7 or 10 bit sequence (will be 7) that identifies the slave

REQUIRES PULL UP RESISTORS



*/

#include "JAADI2CLib.h"
#include "JAADIOLib.h"

/** Start condition delay time: 4.0 us (100 kHz) */
const int T1 = 4;

/** Basic clock delay time: 4.7 us (100 kHz) */
const int T2 = 5;

/** Maximum number of clock stretching retries: 100 us */
const int CLOCK_STRETCHING_RETRY_MAX = 25;

/** Transaction state; start or repeated start condition. */
char m_start;

/** CPU CLOCK */
#define CPU_FREQUENCY (80000000UL)                  /* Fcy = 80MHz (unsigned long) */
#define CORE_TIMER_FREQUENCY (CPU_FREQUENCY/2) //core timer only triggeres after 2 cpu cycles
#define CORE_TIMER_MILLISECONDS (CORE_TIMER_FREQUENCY/1000) //millisecond count time (cpu cycles)
#define CORE_TIMER_MICROSECONDS (CORE_TIMER_FREQUENCY/1000000) //microsecond count time (cpu cycles)







char I2C_initted = FALSE;

//Top level abstractions
int I2C_Init(unsigned int speed) {
	if (!I2C_initted) {
		I2C_initted = true;
		//Set SCL/SDA to OUTPUTS
		//Set SCL/SDA to HIGH
		if (IO_checkLAT(SCLLAT))

	}
}

/********
* ALL LOW LEVEL FRAME CONDITIONS/ROUTINES
* all frame conditions return 1 for success 0 for fail
*******/

//Start condition, beginning of frame
char startCondition() {
	IO_setPortDirection(SDA, INPUT); //set sda to input
	if (IO_readPort(SDA) == 0) { //NACK recieved which is a rip
		return false;
	}
	SDALOW(); //set sda low
	delayUS(T1); //delay 4uS
	SCLLOW(); //set scl low
	return true;
}

//Repeated start condition
char repeated_start_condition() {
	delayMicroseconds(T1);
	IO_setPortDirection(SDA, INPUT);
	if (IO_readPort(SDA) == 0) { //NACK recieved which is a rip
		return false;
	}
	IO_setPortDirection(SCL, INPUT);
	delayMicroseconds(T2);
	SDALOW();
	delayMicroseconds(T1);
	SCLLOW();
	return (true);
}

//Allows clock stretching detection using pins
char clock_stretching() {
	for (int retry = 0; retry < CLOCK_STRETCHING_RETRY_MAX; retry++) {
		if (IO_readPort(SCL)) { //wait for ACK
			return true;
		}
		delayMicroseconds(T1);
	}
	return (false);
}

//End of frame condition
char stop_condition() {
    delayMicroseconds(T1);
    SDALOW();
    IO_setPortDirection(SCL, INPUT);
    delayMicroseconds(T1);
    if (!clock_stretching()) return (false); //check for clock stretching
    IO_setPortDirection(SDA, INPUT);

    return (IO_readPort(SDA) == 0); //check if sda is being pulled low
}

//Write a single bit to device
char write_bit(char value) { //should write a single bit but can't because C doesn't have bool type lol
    if (value == 1) {
    	IO_setPortDirection(SDA, INPUT);
    } else {
    	SDALOW();
    }
    delayMicroseconds(T2);
    IO_setPortDirection(SCL, INPUT);
    delayMicroseconds(T1);
    if (!clock_stretching()) {
    	return false;
    }
    SCLLOW();
    return (true);
}

//Read a single bit from device
char read_bit(char& value) { //it's pointer time bois
    IO_setPortDirection(SDA, INPUT);
    delayMicroseconds(T2);
    IO_setPortDirection(SCL, INPUT);
    delayMicroseconds(T1);
    if (!clock_stretching()) {
    	return false;
    }
    value = IO_readPort(SDA);
    SCLLOW();
    return true;
}


/******
MEDIUM LOW LEVEL FUNCTIONS
******/
char write_byte(unsigned char byte, char& nack) {
	for (int i = 0; i < 8; i++) { //for each bit in the byte
	  if (!write_bit(byte & 0x80)) return (false);
	  byte <<= 1;
	}
	return (read_bit(nack));
}




/******
LOWEST LEVEL FUNCTIONS
******/

void SDAHIGH() {} //Pullup resistor should take care of this
void SCLHIGH() {} //same with this

void SDALOW() {
	IO_setPortDirection(SDA, OUTPUT);
	IO_setPort(SDA, LOW);
}

void SCLLOW() {
	IO_setPortDirection(SCL, OUTPUT);
	IO_setPort(SCL, LOW);
}

void delayUS(UINT32 delay_us) {
   UINT32 DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_us * CORE_TIMER_MICROSECONDS));
}

void delayMS(UINT32 delay_ms) {
   UINT32 DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_ms * CORE_TIMER_MILLISECONDS));
}
 