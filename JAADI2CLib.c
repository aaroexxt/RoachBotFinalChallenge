/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 * 
 * Good links that I used as references when writing this:
 * https://github.com/mikaelpatel/Arduino-TWI/blob/master/src/Software/TWI.h
 * https://github.com/bitbank2/Multi_BitBang/blob/40bb8210b8423d5daf0cd90062e5b8e4404a7a96/src/Multi_BitBang.cpp
 * http://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/
 * https://calcium3000.wordpress.com/2016/08/19/i2c-bit-banging-tutorial-part-i/
 * https://robot-electronics.co.uk/i2c-tutorial
 * https://gzalo.com/i2c_en/
 */

//Hey anyone who uses this in the future, thanks for tuning in. This took a while to make & debug so I hope that you use it well :)

//require the i2c lib
#include "JAADI2CLib.h"
#include "JAADIOLib.h"

/** Start condition delay time: 4.0 us (100 kHz) */
const int T1 = 4;

/** Basic clock delay time: 4.7 us (100 kHz) */
const int T2 = 5;

//float to record various scaling constants (setup later)
float _accel_mg_lsb;
float _mag_mgauss_lsb;
float _gyro_dps_digit;

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
    if (debugMode == true) {
        puts(str);
        printf("\r\n");
    }
}

void debugPrintArray(char array[], int len) {
	if (debugMode == true) {
        int i = 0;
        for (i = 0; i<len; i++) {
            printf("i=%d, e=%d \r\n", i, array[i]);
        }
    }
}

//Debug functions - enable/disable
void I2C_setDebugOn(void) {
	debugMode = true;
}

void I2C_setDebugOff(void) {
	debugMode = false;
}

//Debug print functions - will print data reading
void I2C_printAccel(AccelData data) {
	printf("(AccReading) x: %.3f, y: %.3f, z: %.3f \r\n", data.x, data.y, data.z);
}

void I2C_printGyro(GyroData data) {
	printf("(GyroReading) x: %.3f, y: %.3f, z: %.3f \r\n", data.x, data.y, data.z);
}

void I2C_printMag(MagData data) {
	printf("(MagReading) x: %.3f, y: %.3f, z: %.3f \r\n", data.x, data.y, data.z);
}
/******
LOWEST LEVEL FUNCTIONS
******/

void SDAHIGH(void) { //Allow SDA to be pulled high by pullup resistor by setting it as input
	IO_setPortDirection(SDA, INPUT);
}
void SCLHIGH(void) { //Allow SCL to be pulled high by pullup resistor by setting it as input
	IO_setPortDirection(SCL, INPUT);
}

void SDALOW(void) { //Make SDA low by setting it as output and then setting it to low
	IO_setPortDirection(SDA, OUTPUT);
	IO_setPort(SDA, LOW);
}

void SCLLOW(void) { //Make SCL low by setting it as output and then setting it to low
	IO_setPortDirection(SCL, OUTPUT);
	IO_setPort(SCL, LOW);
}

void delayUS(unsigned long delay_us) {
    return; //no delay required
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
	SDAHIGH();             // i2c start bit sequence
	SCLHIGH();
	delayUS(T1);
	SDALOW();
	delayUS(T1);
	SCLLOW();
	delayUS(T1);
	return true;
}

//End of frame condition
char stopCondition() {
	SDALOW();             // i2c stop bit sequence
	delayUS(T1);
  	SCLHIGH();
  	delayUS(T1);
  	SDAHIGH();
  	delayUS(T1);
  	return true;
}

unsigned char rxByte(char ack) {
	debugPrint("rxByte");
	int bitCounter = 0;
	unsigned char byte = 0; //

	SDAHIGH(); //set data line as input in preparation to read
	for (bitCounter = 0; bitCounter < 8; bitCounter++) {
		delayUS(T1); //wait for data to settle
		SCLHIGH(); //clock high
		byte <<= 1;

		while (IO_readPort(SCL) == 0) {};    // wait for any SCL clock stretching

		if (IO_readPort(SDA) == HIGH) { //check for data byte
			byte |= 1;
		}
		SCLLOW();
	} 

	if (ack) {
		SDALOW();
	} else {
		SDAHIGH();
	}

	SCLHIGH();
	delayUS(T1);             // send (N)ACK bit
	SCLLOW();
	delayUS(T1);
	SDALOW();

	//printf("rxByte: 0x%x\r\n",byte);
	return byte;
}

unsigned char txByte(unsigned char byte) {
	debugPrint("txByte");
	//printf("Byte: 0x%x\r\n",byte);

	int bitCounter = 0;
	static unsigned char ack;

	for (bitCounter = 0; bitCounter < 8; bitCounter++) {
		if (byte & 0x80) { //only get the high order bit
			SDAHIGH();
		} else {
			SDALOW();
		}
		SCLHIGH();
		delayUS(T1);
		SCLLOW();
		byte <<= 1; //shift byte by 1
	}

	//read ACK bit
	SDAHIGH();
	SCLHIGH();
	delayUS(T1);

	ack = (IO_readPort(SDA) == HIGH) ? 1 : 0;          // possible ACK bit
	//printf("txACK: %d\r\n",ack);

	SCLLOW();
	//DONT KNOW WHETHER THE FOLLOWING 2 LINES ARE REQUIRED
	delayUS(T1);
	SDALOW();

	return ((ack == 0) ? true : false); //low ACK bit indicates success
}

/******
* HIGH LOW LEVEL FUNCTIONS
* Uses medium low level functions to write long strings of data, or read similarly long strings
******/

int writeRegister(unsigned int addr, unsigned char reg, unsigned char value) {

    startCondition();

    // Address device with read request and check that it acknowledges
    addr <<= 1; //shift address to proper position
    txByte(addr);

    //Write to register
    txByte(reg);

    //Write value
   	txByte(value);

    stopCondition();

    return true;
}

int readRegister(unsigned int addr, unsigned char reg) {

	startCondition();

	// Address device with read request and check that it acknowledges
    addr <<= 1; //shift address to proper position
    txByte(addr);

    //Write register byte
    txByte(reg);

    startCondition(); //repeat start condition

    txByte(addr | 0b00000001);

    //Read from register
    unsigned char byte = rxByte(0); //ACK 0

    stopCondition();

    return byte; //return pointerized value
}

unsigned char readRegisterBuffer(unsigned int addr, unsigned char reg, unsigned char *buffer, int bytesToRead) {

	startCondition();

	// Address device with read request and check that it acknowledges
	addr <<= 1; //shift address to proper position
    txByte(addr);

    //Read from register
    txByte(reg);

    startCondition(); //repeat start condition

    txByte(addr | 0b00000001);

    //Read from register
    int i = 0;
    for (i = 0; i < bytesToRead; i++) {
    	buffer[i] = rxByte((i != bytesToRead-1) ? 1 : 0); //ack should be 0 on last byte; otherwise 1
    }

    stopCondition();

    return true; //return pointerized value
}





/********
* HIGHEST LEVEL OF ABSTRACTION (I2C Gyro/Accel/Mag Driver)
*******/
 
char I2C_initted = false;

//Top level abstractions
char I2C_Init() {
	if (I2C_initted == false) {
		I2C_initted = true;
		IO_setPortDirection(SDA, INPUT); //allow both to be pulled high
		IO_setPortDirection(SCL, INPUT);

		return I2C_InitSensors(); //Initialize the sensors
	} else {
		return false;
	}
}

char I2C_InitSensors() { //do full init sequence
	debugPrint("I2C_INITSENSORS BEGIN");
	// soft reset & reboot accel/gyro
	writeRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
   
	// soft reset & reboot magnetometer
	writeRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);

  	debugPrint("accel and mag soft reset OK");
  	delayMS(10); //wait 10ms

  	//Check if the accelerometer is present - check internal ID
  	unsigned char accelId = readRegister(ACCELADDR, LSM9DS1_REGISTER_WHO_AM_I_XG);
  	printf("ACC/GYRO whoami: %x\r\n",accelId);
	if (accelId != LSM9DS1_XG_ID) { //reeee accelId check failed
		return false;
	}
    
   	//Check if the magnetometer is present - check internal ID
	unsigned char magId = readRegister(MAGADDR, LSM9DS1_REGISTER_WHO_AM_I_M);
	printf("MAG whoami: %x\r\n",magId);
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
	setupAccel(LSM9DS1_ACCELRANGE_2G); //because the roach is small, just use the smallest possible values for scaling
	setupMag(LSM9DS1_MAGGAIN_4GAUSS);
	setupGyro(LSM9DS1_GYROSCALE_245DPS);
	debugPrint("Default ranging done for sensors");

	return true;
}

/*********
* SETUP FUNCTIONS
*********/

void setupAccel(accelRange_t range) {
	debugPrint("setupAccel called");
	unsigned char reg = readRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG6_XL);
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
	unsigned char reg = readRegister(MAGADDR, LSM9DS1_REGISTER_CTRL_REG2_M);
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
	unsigned char reg = readRegister(ACCELADDR, LSM9DS1_REGISTER_CTRL_REG1_G);
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

/*********
* DATA CONVERSION
*********/

AccelData I2C_getAccelData() {
	debugPrint("getAccelData called"); //debug print
	unsigned char buffer[6];
	readRegisterBuffer(ACCELADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_XL, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];//each xhi/xlo is one half of the full 16-bit number
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	// Fix two's complement
	int16_t intermediateX = (signed short)xhi;
	int16_t intermediateY = (signed short)yhi;
	int16_t intermediateZ = (signed short)zhi;

	//Float conversion
	float realX = (float)intermediateX;
	float realY = (float)intermediateY;
	float realZ = (float)intermediateZ;

	// Unit conversion
	realX *= _accel_mg_lsb;
	realX /= 1000.0;
	realX *= SENSORS_GRAVITY_STANDARD;

	realY *= _accel_mg_lsb;
	realY /= 1000.0;
	realY *= SENSORS_GRAVITY_STANDARD;

	realZ *= _accel_mg_lsb;
	realZ /= 1000.0;
	realZ *= SENSORS_GRAVITY_STANDARD;
	
	//Create accelData struct
	AccelData data = {realX, realY, realZ};
	return data;
}

MagData I2C_getMagData() {
	debugPrint("getMagData called"); //debug print
	// Read the magnetometer
	unsigned char buffer[6];
	readRegisterBuffer(MAGADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_M, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	// Fix two's complement
	int16_t intermediateX = (signed short)xhi;
	int16_t intermediateY = (signed short)yhi;
	int16_t intermediateZ = (signed short)zhi;

	//Float conversion
	float realX = (float)intermediateX;
	float realY = (float)intermediateY;
	float realZ = (float)intermediateZ;
	
	// Unit conversion
	realX *= _mag_mgauss_lsb;
	realX /= 1000.0;
	realY *= _mag_mgauss_lsb;
	realY /= 1000.0;
	realZ *= _mag_mgauss_lsb;
	realZ /= 1000.0;

	//Create magData struct
	MagData data = {realX, realY, realZ};
	return data;
}

GyroData I2C_getGyroData() {
	debugPrint("getMagData called"); //debug print
	// Read gyro
	unsigned char buffer[6];
	readRegisterBuffer(ACCELADDR, 0x80 | LSM9DS1_REGISTER_OUT_X_L_G, buffer, 6);

	debugPrintArray(buffer, 6); //debug print array
	uint8_t xlo = buffer[0];
	int16_t xhi = buffer[1];
	uint8_t ylo = buffer[2];
	int16_t yhi = buffer[3];
	uint8_t zlo = buffer[4];
	int16_t zhi = buffer[5];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8; xhi |= xlo;
	yhi <<= 8; yhi |= ylo;
	zhi <<= 8; zhi |= zlo;

	// Fix two's complement
	int16_t intermediateX = (signed short)xhi;
	int16_t intermediateY = (signed short)yhi;
	int16_t intermediateZ = (signed short)zhi;

	//Float conversion
	float realX = (float)intermediateX;
	float realY = (float)intermediateY;
	float realZ = (float)intermediateZ;

	// Unit conversion
	realX *= _gyro_dps_digit;
	realY *= _gyro_dps_digit;
	realZ *= _gyro_dps_digit;

	//Create gyroData struct
	GyroData data = {realX, realY, realZ};
	return data;
}

