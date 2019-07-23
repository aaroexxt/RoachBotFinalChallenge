/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 * Big credit to the Arduino TWI and Wire libraries for their reference code :)
 */



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

//require the i2c lib
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
* MEDIUM LOW LEVEL FUNCTIONS
* yo these functions basically use the lowest level to build up to bytes (which will then be used in the frame)
******/

//Write a single byte (will return NACK bit so we'll know if it returned successfully)
char write_byte(unsigned char byte, char& nack) {
	for (int i = 0; i < 8; i++) { //for each bit in the byte
		if (!write_bit(byte & 0x80)) { //only take the MSB via bitwise and
			return false;
		}
	  	byte <<= 1; //bitshift it left (will wrap, but that's ok)
	}
	return (read_bit(nack));
}

char read_byte(unsigned char& byte, char ack) {
    unsigned char value;
    byte = 0;
    for (int i = 0; i < 8; i++) {
		if (!read_bit(value)) {
			return (false); //if failed to read just return false
		}
		byte = (byte << 1) | value; //bitshift it to the proper place and bitwise OR with value to return current full byte (LSB to MSB)
    }
    return (write_bit(!ack)); //return if the device will accept a ACK bit
}

/******
* HIGH LOW LEVEL FUNCTIONS
* Uses medium low level functions to write long strings of data, or read similarly long strings
******/

int read(unsigned int addr, void* buf, int count) { //count buffer in BYTES
    // Check if repeated start condition should be generated
    if (!m_start && !repeated_start_condition()) { //if start is false and repeated start condition is false then i2c is not initted so kill
    	return (-1);
    }
    m_start = false;

    // Address device with read request and check that it acknowledges
    char nack;
    if (!write_byte(addr | 1, nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    // Read bytes and acknowledge until required size
    unsigned char* bp = (unsigned char*) buf;
    int size = count;
    while (size--) { //subtract a single bit from size
      char ack = (size != 0); //if size is not 0, then it is 1 (which is the goal)
      unsigned char data;
      if (!read_byte(data, ack)) { //if reading byte returns an error then exit
      	return (-1);
      }
      *bp++ = data; //add to the buffer
    }
    return count; //return the amount of bytes read
}

int write(uint8_t addr, void* buf, int count) { //count in BYTES
    // Check if repeated start condition should be generated
    if (!m_start && !repeated_start_condition()) { //if start is false and repeated start condition is false then i2c is not initted so kill
    	return (-1);
    }
    m_start = false;

    // Address device with read request and check that it acknowledges
    char nack;
    if (!write_byte(addr | 1, nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    // Sanity check the size of count and buf to write
    if (count == 0 || sizeof(buf) == 0) {
    	return (0);
    }

    // Write given buffer to device
    int count = 0;
    int i = 0;
    for(i=0; i<count; i++) {

    	///THIS IS VERY NOT DONE PLS FINISH
      const uint8_t* bp = (const uint8_t*) vp->buf;
      size_t size = vp->size;
      count += size;
      while (size--) {
	uint8_t data = *bp++;
	if (!write_byte(data, nack) || nack) return (-1);
      }
    }
    return (count);
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
 