/* 
 * File:   JAADI2CLib.h
 * Author: Aaron
 *
 * Created on July 22, 2019, 4:58 PM
 */

#ifndef JAADSM_H
#define	JAADSM_H

#ifdef	__cplusplus
extern "C" {
#endif

int I2C_Init(unsigned int speed); //init function taking speed as a value in Hz
int I2C_ReadInt(char I2Caddress, char deviceRegister); //read a single int from address & register
unsigned char I2C_ReadReg(char I2Caddress, char deviceRegister);
unsigned char I2C_WriteReg(char I2Caddress, char deviceRegister, char data);
void I2C_ReadMultiple(char I2Caddress, char deviceRegister, char *outArray, int numBytes)

void internal_startSeq(char I2Caddress);
void internal_stopSeq(char I2Caddress);


#ifdef	__cplusplus
}
#endif

#endif	/* JAADSM_H */

