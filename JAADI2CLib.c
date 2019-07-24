/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 * Big credit to the Arduino TWI and Wire libraries for their reference code :)
 * 
 * Good links:
 * https://github.com/mikaelpatel/Arduino-TWI/blob/master/src/Software/TWI.h
 * https://github.com/mdunne/ANIMA/blob/master/code/Small_scale/I2C_Driver.X/I2C_Driver_test.c
 * http://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/
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

//float to record accel lsb constants
int _accel_mg_lsb;
int _mag_mgauss_lsb;
int _gyro_dps_digit;

/** CPU CLOCK */
#define CPU_FREQUENCY (80000000UL)                  /* Fcy = 80MHz (unsigned long) */
#define CORE_TIMER_FREQUENCY (CPU_FREQUENCY/2) //core timer only triggeres after 2 cpu cycles
#define CORE_TIMER_MILLISECONDS (CORE_TIMER_FREQUENCY/1000) //millisecond count time (cpu cycles)
#define CORE_TIMER_MICROSECONDS (CORE_TIMER_FREQUENCY/1000000) //microsecond count time (cpu cycles)

/** DEBUG MODE */
char debugMode = false;

/*****
* DEBUG FUNCTIONS
*****/

void debugPrint(char str[]) {
	puts(str);
	printf("\r\n");
}

void debugPrintArray(char array[], int len) {
	int i = 0;
	for (i = 0; i<len; i++) {
		printf("i=%d, e=%d \r\n", i, array[i]);
	}
}

void I2C_setDebugOn(void) {
	debugMode = true;
}

void I2C_setDebugOff(void) {
	debugMode = false;
}

void I2C_printAccel(AccelData data) {
	printf("(AccReading) x: %d, y: %d, z: %d \r\n", data.x, data.y, data.z);
}

void I2C_printGyro(GyroData data) {
	printf("(GyroReading) x: %d, y: %d, z: %d \r\n", data.x, data.y, data.z);
}

void I2C_printMag(MagData data) {
	printf("(MagReading) x: %d, y: %d, z: %d \r\n", data.x, data.y, data.z);
}
/******
LOWEST LEVEL FUNCTIONS
******/

void SDAHIGH(void) {} //Pullup resistor should take care of this
void SCLHIGH(void) {} //same with this

void SDALOW(void) {
	IO_setPortDirection(SDA, OUTPUT);
	IO_setPort(SDA, LOW);
}

void SCLLOW(void) {
	IO_setPortDirection(SCL, OUTPUT);
	IO_setPort(SCL, LOW);
}

void delayUS(unsigned long delay_us) {
   unsigned long DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_us * CORE_TIMER_MICROSECONDS));

   return;
}

void delayMS(unsigned long delay_ms) {
   unsigned long DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_ms * CORE_TIMER_MILLISECONDS));
   
   return;
}

/********
* ALL LOW LEVEL FRAME CONDITIONS/ROUTINES
* all frame conditions return 1 for success 0 for fail
*******/

//Start condition, beginning of frame
char startCondition() {
	debugPrint("startCondition");
	IO_setPortDirection(SDA, INPUT); //set sda to input so that it gets pulled high
	if (IO_readPort(SDA) == 0) { //Other device is pulling it low for some reason?
		return false;
	}
	SDALOW(); //set sda low
	delayUS(T1); //delay 4uS
	SCLLOW(); //set scl low
	return true;
}


//Allows clock stretching detection using pins
char clock_stretching() {
	debugPrint("clkStretching");
	int retry;
	for (retry = 0; retry < CLOCK_STRETCHING_RETRY_MAX; retry++) {
		if (IO_readPort(SCL)) { //wait for ACK
			return true;
		}
		delayUS(T1);
	}
	return (false);
}

//End of frame condition
char stop_condition() {
	debugPrint("stopCondition");
    delayUS(T1);
    SDALOW();
    IO_setPortDirection(SCL, INPUT);
    delayUS(T1);
    if (!clock_stretching()) return (false); //check for clock stretching
    IO_setPortDirection(SDA, INPUT); //allow sda to go high

    return (IO_readPort(SDA) == 0); //check if sda is being pulled low
}

//Write a single bit to device
char write_bit(char value) { //should write a single bit but can't because C doesn't have bool type lol
    debugPrint("writeBit:");
    debugPrint(&value);

    if (value == 1) {
    	IO_setPortDirection(SDA, INPUT); //sda goes high
    } else {
    	SDALOW(); //sda goes low
    }
    delayUS(T2);
    IO_setPortDirection(SCL, INPUT); //pulse scl high
    delayUS(T1);
    if (!clock_stretching()) { //check for clock stretching
    	return false;
    }
    SCLLOW(); //and then pulse scl low
    return (true);
}

//Read a single bit from device
char read_bit(char *value) { //it's pointer time bois
    debugPrint("readBit");

    IO_setPortDirection(SDA, INPUT);
    delayUS(T2);
    IO_setPortDirection(SCL, INPUT);
    delayUS(T1);
    if (!clock_stretching()) {
    	return false;
    }
    char portVal = IO_readPort(SDA);
    value = &portVal;
    SCLLOW();
    return true;
}


/******
* MEDIUM LOW LEVEL FUNCTIONS
* yo these functions basically use the lowest level to build up to bytes (which will then be used in the frame)
******/

//Write a single byte (will return NACK bit so we'll know if it returned successfully)
char write_byte(unsigned char byte, unsigned char *nack) {
	int i = 0;

	for (i = 0; i < 8; i++) { //for each bit in the byte
		if (!write_bit(byte & 0x80)) { //only take the MSB via bitwise and
			return false;
		}
	  	byte <<= 1; //bitshift it left (will wrap, but that's ok)
	}
	return (read_bit(nack));
}

char read_byte(unsigned char *finalByte, char ack) {
    unsigned char value;
    unsigned char byte = 0;
    int i = 0;

    for (i = 0; i < 8; i++) {
		if (!read_bit(&value)) {
			return (false); //if failed to read just return false
		}
		byte = (byte << 1) | value; //bitshift it to the proper place and bitwise OR with value to return current full byte (LSB to MSB)
    }
    finalByte = &byte;
    return (write_bit(!ack)); //return if the device will accept a ACK bit
}

char read_bits(unsigned char *finalByte, char ack, int len) {
    unsigned char value;
    unsigned char byte = 0;
    int i = 0;

    for (i = 0; i < len; i++) {
		if (!read_bit(&value)) {
			return (false); //if failed to read just return false
		}
		byte = (byte << 1) | value; //bitshift it to the proper place and bitwise OR with value to return current full byte (LSB to MSB)
    }
    finalByte = &byte;
    return (write_bit(!ack)); //return if the device will accept a ACK bit
}

char read_bitsBuffer(unsigned char *buffer, char ack, int len) {
    unsigned char value;
    int i = 0;

    for (i = 0; i < len; i++) {
		if (!read_bit(&value)) {
			return (false); //if failed to read just return false
		}
		buffer[i] = value; //write it to the proper portion of the buffer
    }
    return (write_bit(!ack)); //return if the device will accept a ACK bit
}

/******
* HIGH LOW LEVEL FUNCTIONS
* Uses medium low level functions to write long strings of data, or read similarly long strings
******/

int writeRegister(unsigned int addr, unsigned char reg, unsigned char value) {
	/*// Check if repeated start condition should be generated
    if (!m_start && !repeated_start_condition()) { //if start is false and repeated start condition is false then i2c is not initted so kill
    	return (-1);
    }
    m_start = false;*/

    startCondition();

    // Address device with read request and check that it acknowledges
    char nack;
    if (!write_byte(addr & 0b11111110, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    //Write to register
    if (!write_byte(reg, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    //Write value
    if (!write_byte(value, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    stop_condition();

    return true;
}

unsigned char readRegister(unsigned int addr, unsigned char reg, int bits) {
	/*// Check if repeated start condition should be generated
    if (!m_start && !repeated_start_condition()) { //if start is false and repeated start condition is false then i2c is not initted so kill
    	return (-1);
    }
    m_start = false;*/

	startCondition();

	// Address device with read request and check that it acknowledges
    char nack;
    if (!write_byte(addr & 0b11111110, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1); //make sure R/W bit is 0
    }

    //Read from register
    if (!write_byte(reg, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    startCondition(); //repeat start condition

    if (!write_byte(addr | 1, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1); //^^ make sure addr bit 0 is 1 to make RW bit high for reading
    }

    //Read from register
    unsigned char value;

    if (!read_bits(&value, nack, bits) || nack) { //if nack returns, then no device found or other issue with protocol
    	return -1; //^ use pointer
    }

    stop_condition();

    return value; //return pointerized value
}

unsigned char readRegisterBuffer(unsigned int addr, unsigned char reg, unsigned char *buffer, int bits) {
	/*// Check if repeated start condition should be generated
    if (!m_start && !repeated_start_condition()) { //if start is false and repeated start condition is false then i2c is not initted so kill
    	return (-1);
    }
    m_start = false;*/

	startCondition();

	// Address device with read request and check that it acknowledges
    char nack;
    if (!write_byte(addr & 0b11111110, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1); //make sure R/W bit is 0
    }

    //Read from register
    if (!write_byte(reg, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1);
    }

    startCondition(); //repeat start condition

    if (!write_byte(addr | 1, &nack) || nack) { //if nack returns, then no device found or other issue with protocol
    	return (-1); //^^ make sure addr bit 0 is 1 to make RW bit high for reading
    }

    //Read from register
    if (!read_bitsBuffer(buffer, nack, bits) || nack) { //if nack returns, then no device found or other issue with protocol
    	return -1; //^ use pointer
    }

    stop_condition();

    return true; //return pointerized value
}





/********
* HIGHEST LEVEL OF ABSTRACTION (I2C Gyro/Accel/Mag Driver)
*******/
 
char I2C_initted = false;

//Top level abstractions
char I2C_Init() {
	if (!I2C_initted) {
		I2C_initted = true;
		printf("JAADi2c outerINIT recieved");
		IO_setPortDirection(SDA, INPUT); //allow both to be pulled high
		IO_setPortDirection(SCL, INPUT);

		return I2C_InitSensors(); //Initialize the sensors
	} else {
		return false;
	}
}

char I2C_InitSensors() {
	debugPrint("I2C_INITSENSORS BEGIN");
	// soft reset & reboot accel/gyro
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG8, 0x05);

	// soft reset & reboot magnetometer
  	writeRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);

  	debugPrint("accel and mag soft reset OK");
  	delayMS(10); //wait 10ms

  	unsigned char accelId = readRegister(ACCELADDR, LSM9DS1_REGISTER_WHO_AM_I_XG, 8);
  	printf("ACCEL whoami: %x",accelId);
	if (accelId != LSM9DS1_XG_ID) { //reeee accelId check failed
		return false;
	}

	unsigned char magId = readRegister(MAGADDR, LSM9DS1_REGISTER_WHO_AM_I_M, 8);
	printf("MAG whoami: %x",magId);
	if (magId != LSM9DS1_MAG_ID) {
		return false;
	}

	// enable gyro continuous
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

	// Enable the accelerometer continous
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing


	// enable mag continuous
	//write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG1_M, 0xFC); // high perf XY, 80 Hz ODR
	writeRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00); // continuous mode
	//write8(MAGTYPE, LSM9DS1_REGISTER_CTRL_REG4_M, 0x0C); // high perf Z mode
	debugPrint("Written basic configs to mag and accel");

	// Set default ranges for the various sensors  
	setupAccel(LSM9DS1_ACCELRANGE_2G);
	setupMag(LSM9DS1_MAGGAIN_4GAUSS);
	setupGyro(LSM9DS1_GYROSCALE_245DPS);
	debugPrint("Default ranging done for sensors");

	return true;
}

void setupAccel(accelRange_t range) {
	debugPrint("setupAccel called");
	unsigned char reg = readRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG6_XL, 8);
	reg &= ~(0b00011000);
	reg |= range;
	printf("Setting range");
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG6_XL, reg);

	switch (range) {
		case LSM9DS1_ACCELRANGE_2G:
			_accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
			break;
		case LSM9DS1_ACCELRANGE_4G:
			_accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
			break;
		case LSM9DS1_ACCELRANGE_8G:
			_accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
			break;    
		case LSM9DS1_ACCELRANGE_16G:
			_accel_mg_lsb =LSM9DS1_ACCEL_MG_LSB_16G;
			break;
	}

	debugPrint("setupAccel done");
	return;
}

void setupMag(magGain_t gain) {
	debugPrint("setupMag called");
	unsigned char reg = readRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG2_M, 8);
	reg &= ~(0b01100000);
	reg |= gain;
	writeRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG2_M, reg);

	switch(gain) {
		case LSM9DS1_MAGGAIN_4GAUSS:
			_mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_4GAUSS;
			break;
		case LSM9DS1_MAGGAIN_8GAUSS:
			_mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_8GAUSS;
			break;
		case LSM9DS1_MAGGAIN_12GAUSS:
			_mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_12GAUSS;
			break;
		case LSM9DS1_MAGGAIN_16GAUSS:
			_mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_16GAUSS;
			break;
	}

	debugPrint("setupMag done");
	return;
}

void setupGyro(gyroScale_t scale) {
	debugPrint("setupGyro called");
	unsigned char reg = readRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG1_G, 8);
	reg &= ~(0b00011000);
	reg |= scale;
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG1_G, reg);

	switch(scale) {
		case LSM9DS1_GYROSCALE_245DPS:
			_gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
			break;
		case LSM9DS1_GYROSCALE_500DPS:
			_gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
			break;
		case LSM9DS1_GYROSCALE_2000DPS:
			_gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
			break;
	}

	debugPrint("setupGyro done");
	return;
}

AccelData getAccelData() {
	debugPrint("getAccelData called"); //debug print
	char buffer[6];
	readRegisterBuffer(ACCELADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];
	int32_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int32_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int32_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	xhi *= _accel_mg_lsb;
	xhi /= 1000;
	xhi *= SENSORS_GRAVITY_STANDARD;

	yhi *= _accel_mg_lsb;
	yhi /= 1000;
	yhi *= SENSORS_GRAVITY_STANDARD;

	zhi *= _accel_mg_lsb;
	zhi /= 1000;
	zhi *= SENSORS_GRAVITY_STANDARD;
	
	//Create accelData struct
	AccelData data = {xhi, yhi, zhi};
	return data;
}

MagData getMagData() {
	debugPrint("getMagData called"); //debug print
	// Read the magnetometer
	char buffer[6];
	readRegisterBuffer(MAGADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_M, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];
	int32_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int32_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int32_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;
	
	xhi *= _mag_mgauss_lsb;
	xhi /= 1000;
	yhi *= _mag_mgauss_lsb;
	yhi /= 1000;
	zhi *= _mag_mgauss_lsb;
	zhi /= 1000;

	//Create magData struct
	MagData data = {xhi, yhi, zhi};
	return data;
}

GyroData getGyroData() {
	debugPrint("getMagData called"); //debug print
	// Read gyro
	char buffer[6];
	readRegisterBuffer(ACCELADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_G, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];
	int32_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int32_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int32_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	xhi *= _gyro_dps_digit;
	yhi *= _gyro_dps_digit;
	zhi *= _gyro_dps_digit;

	//Create gyroData struct
	GyroData data = {xhi, yhi, zhi};
	return data;
}

