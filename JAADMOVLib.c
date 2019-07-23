/* 
 * File:   JAADSM
 * Author: Ashvin
 *
 * Created on July 5, 2019, 1:32 PM
 */

#include "JAADMOVLib.h"
#define p 0.2
int currentPos = 0;
int gyroAngle = 0;
int setpoint = 0;
char isFinished = 0;
int count = 0;

void initTurn(int degrees){
    count = 0;
    currentPos = 0;
    setpoint = degrees;
    isFinished = 0;
}
int turn(void){
    gyroAngle = 0; //will change to update gyroAngle
    currentPos += gyroAngle;
    if(abs(setpoint - currentPos) < 10){
        count++;
    } else {
        count = 0;
    }
    if(count > 10){
        isFinished = 1;
        return 0;
    } else{
        return p * (setpoint - currentPos);
    }
}

char isTurnFinished(void){
    return isFinished;
}