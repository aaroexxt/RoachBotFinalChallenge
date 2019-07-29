/* 
 * File:   JAADI2CLib.h
 * Author: Aaron
 *
 * Created on July 22, 2019, 4:58 PM
 */
#include "GenericTypeDefs.h"
#ifndef JAADI2CLIB_H
#define	JAADI2CLIB_H

#ifndef true
#define true 1
#define false 0
#endif

//Address and internal address of sensors
#define ACCELADDR          (0x6B)
#define MAGADDR                (0x1E)
#define LSM9DS1_XG_ID                      (0b01101000)
#define LSM9DS1_MAG_ID                     (0b00111101)

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F) 

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS      (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS      (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS     (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS     (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS     (0.07000F)

#define SENSORS_GRAVITY_STANDARD             (9.80665F)              /**< Earth's gravity in m/s^2 */
//Various register IDs for use in driver
#define	LSM9DS1_REGISTER_WHO_AM_I_XG          0x0F
#define	LSM9DS1_REGISTER_CTRL_REG1_G          0x10
#define	LSM9DS1_REGISTER_CTRL_REG2_G          0x11
#define	LSM9DS1_REGISTER_CTRL_REG3_G          0x12
#define	LSM9DS1_REGISTER_TEMP_OUT_L           0x15
#define	LSM9DS1_REGISTER_TEMP_OUT_H           0x16
#define	LSM9DS1_REGISTER_STATUS_REG           0x17
#define	LSM9DS1_REGISTER_OUT_X_L_G            0x18
#define	LSM9DS1_REGISTER_OUT_X_H_G            0x19
#define	LSM9DS1_REGISTER_OUT_Y_L_G            0x1A
#define	LSM9DS1_REGISTER_OUT_Y_H_G            0x1B
#define	LSM9DS1_REGISTER_OUT_Z_L_G            0x1C
#define	LSM9DS1_REGISTER_OUT_Z_H_G            0x1D
#define	LSM9DS1_REGISTER_CTRL_REG4            0x1E
#define	LSM9DS1_REGISTER_CTRL_REG5_XL         0x1F
#define	LSM9DS1_REGISTER_CTRL_REG6_XL         0x20
#define	LSM9DS1_REGISTER_CTRL_REG7_XL         0x21
#define	LSM9DS1_REGISTER_CTRL_REG8            0x22
#define	LSM9DS1_REGISTER_CTRL_REG9            0x23
#define	LSM9DS1_REGISTER_CTRL_REG10           0x24

#define	LSM9DS1_REGISTER_OUT_X_L_XL           0x28
#define	LSM9DS1_REGISTER_OUT_X_H_XL           0x29
#define	LSM9DS1_REGISTER_OUT_Y_L_XL           0x2A
#define	LSM9DS1_REGISTER_OUT_Y_H_XL           0x2B
#define	LSM9DS1_REGISTER_OUT_Z_L_XL           0x2C
#define	LSM9DS1_REGISTER_OUT_Z_H_XL           0x2D


#define	LSM9DS1_REGISTER_WHO_AM_I_M          0x0F
#define	LSM9DS1_REGISTER_CTRL_REG1_M         0x20
#define	LSM9DS1_REGISTER_CTRL_REG2_M         0x21
#define	LSM9DS1_REGISTER_CTRL_REG3_M         0x22
#define	LSM9DS1_REGISTER_CTRL_REG4_M         0x23
#define	LSM9DS1_REGISTER_CTRL_REG5_M         0x24
#define	LSM9DS1_REGISTER_STATUS_REG_M        0x27
#define	LSM9DS1_REGISTER_OUT_X_L_M           0x28
#define	LSM9DS1_REGISTER_OUT_X_H_M           0x29
#define	LSM9DS1_REGISTER_OUT_Y_L_M           0x2A
#define	LSM9DS1_REGISTER_OUT_Y_H_M           0x2B
#define	LSM9DS1_REGISTER_OUT_Z_L_M           0x2C
#define	LSM9DS1_REGISTER_OUT_Z_H_M           0x2D
#define	LSM9DS1_REGISTER_CFG_M               0x30
#define	LSM9DS1_REGISTER_INT_SRC_M           0x31

#define	LSM9DS1_MAGDATARATE_3_125HZ           (0b000 << 2)
#define	LSM9DS1_MAGDATARATE_6_25HZ            (0b001 << 2)
#define	LSM9DS1_MAGDATARATE_12_5HZ            (0b010 << 2)
#define	LSM9DS1_MAGDATARATE_25HZ              (0b011 << 2)
#define	LSM9DS1_MAGDATARATE_50HZ              (0b100 << 2)
#define	LSM9DS1_MAGDATARATE_100HZ             (0b101 << 2)

//Sensor range enums
typedef enum
{
	LSM9DS1_ACCELRANGE_2G                = (0b00 << 3),
	LSM9DS1_ACCELRANGE_16G               = (0b01 << 3),
	LSM9DS1_ACCELRANGE_4G                = (0b10 << 3),
	LSM9DS1_ACCELRANGE_8G                = (0b11 << 3)
} accelRange_t;

typedef enum
{
	LSM9DS1_MAGGAIN_4GAUSS               = (0b00 << 5),  // +/- 4 gauss
	LSM9DS1_MAGGAIN_8GAUSS               = (0b01 << 5),  // +/- 8 gauss
	LSM9DS1_MAGGAIN_12GAUSS              = (0b10 << 5),  // +/- 12 gauss
	LSM9DS1_MAGGAIN_16GAUSS              = (0b11 << 5)   // +/- 16 gauss
} magGain_t;

typedef enum {
  LSM9DS1_GYROSCALE_245DPS             = (0b00 << 3),  // +/- 245 degrees per second rotation
  LSM9DS1_GYROSCALE_500DPS             = (0b01 << 3),  // +/- 500 degrees per second rotation
  LSM9DS1_GYROSCALE_2000DPS            = (0b11 << 3)   // +/- 2000 degrees per second rotation
} gyroScale_t;

#define	LSM9DS1_GYROSCALE_245DPS              (0b00 << 3)  // +/- 245 degrees per second rotation
#define	LSM9DS1_GYROSCALE_500DPS              (0b01 << 3)  // +/- 500 degrees per second rotation
#define	LSM9DS1_GYROSCALE_2000DPS             (0b11 << 3)   // +/- 2000 degrees per second rotation

//Structs for data transfer

typedef struct {
	float x;
    float y;
    float z;
} AccelData;

typedef struct {
	float x;
    float y;
    float z;
} MagData;

typedef struct {
	float x;
    float y;
    float z;
} GyroData;


//Function definitions
char I2C_Init(); //init function taking speed as a value in Hz
char I2C_InitSensors();

//Debug functions
void I2C_setDebugOn();
void I2C_setDebugOff();
void I2C_printAccel(AccelData data);
void I2C_printGyro(GyroData data);
void I2C_printMag(MagData data);

//Internal functions that are exposed for some reason
void setupAccel();
void setupMag();
void setupGyro();

//Data structures
AccelData I2C_getAccelData();
MagData I2C_getMagData();
GyroData I2C_getGyroData();

#endif	/* JAADSM_H */

