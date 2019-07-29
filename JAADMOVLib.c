/* 
 * File:   JAADSM
 * Author: Ashvin
 *
 * Created on July 5, 2019, 1:32 PM
 */

#include "JAADMOVLib.h"
#include "timers.h"
#include "JAADI2CLib.h"
#define timerId 5
#define checkTime 1500

/*******
* TURN CODE
*******/

//Control variables
#define turnP 1.3
#define driveP 0.2
#define minPower 55
#define maxPower 65
//Error thresholds
#define turnThreshold 5
#define driveThreshold 10

//Variables to track state of turn
float turnCurrentPos = 0;
float gyroAngle = 0;
int turnSetpoint = 0;
char isFinishedTurn = 0;
int turnCount = 0;
char turnTimeCheck = 0;

int prevClockTime = 0;
int currentClockTime = 0;
GyroData data;
AccelData fwdData;

void MOV_initTurn(int degrees) {
    turnCount = 0;
    turnCurrentPos = 0;
    turnSetpoint = degrees;
    isFinishedTurn = 0;
    turnTimeCheck = 0;
    prevClockTime = TIMERS_GetTime();
}

int MOV_updateTurn(void) {
    data = I2C_getGyroData();
    gyroAngle = (360.0 / 380.0) * data.z;
    currentClockTime = TIMERS_GetTime();
    turnCurrentPos += gyroAngle * (currentClockTime - prevClockTime) / 1000;
    float deltaSetpoint = turnSetpoint - turnCurrentPos;

    if (deltaSetpoint < turnThreshold && deltaSetpoint > - turnThreshold && !turnTimeCheck){
        TIMERS_InitTimer(timerId, checkTime);
        turnTimeCheck = 1;
    }
    
    if ((deltaSetpoint > turnThreshold || deltaSetpoint < - turnThreshold) && TIMERS_IsTimerExpired(timerId) && turnTimeCheck) {
        turnTimeCheck = 0;
    } else if((deltaSetpoint < turnThreshold && deltaSetpoint > - turnThreshold) && TIMERS_IsTimerExpired(timerId) && turnTimeCheck){
        isFinishedTurn = 1;
        return 0;
    }
    printf("%d\r\n", deltaSetpoint);
    printf("%d\r\n", turnTimeCheck);
    prevClockTime = currentClockTime;
    if(deltaSetpoint < turnThreshold && deltaSetpoint > -turnThreshold){
        return 0;
    }

    char newPower = turnP * deltaSetpoint;
    if(newPower > maxPower){
        return maxPower;
    } else if(newPower < -maxPower){
        return -maxPower;
    }
    else if(newPower < minPower && newPower > 0){
        return minPower;
    } else if(newPower > -minPower){
        return -minPower;
    }
    return newPower;
}

char MOV_isTurnFinished(void){
    return isFinishedTurn;
}

/*******
* DRIVE CODE
*******/

//Variables to track state of drive
float driveCurrentPos = 0;
float acc = 0;
float vel = 0;
int driveSetpoint = 0;
char isFinishedDrive = 0;
int driveCount = 0;
char driveTimeCheck = 0;

void MOV_initFwd(int distance){
    vel = 0;
    driveSetpoint = distance;
    driveCount = 0;
    driveCurrentPos = 0;
    isFinishedDrive = 0;
    driveTimeCheck = 0;
    prevClockTime = TIMERS_GetTime();
}

int MOV_updateFwd(void){
    fwdData = I2C_getAccelData();
    acc = fwdData.y;
    if(fwdData.y < 0.4 && fwdData.y > -0.4) {
        acc = 0;
    }
    currentClockTime = TIMERS_GetTime();
    vel += acc * (currentClockTime - prevClockTime) / 1000.0;
    driveCurrentPos += vel * (currentClockTime - prevClockTime) / 1000.0;
    float deltaSetpoint = driveSetpoint - driveCurrentPos;
    printf("dSet %.3f\r\n", deltaSetpoint);
    printf("vel %.3f\r\n", vel);
    if (deltaSetpoint < driveThreshold && deltaSetpoint > -driveThreshold && !driveTimeCheck){
        TIMERS_InitTimer(timerId, checkTime);
        driveTimeCheck = 1;
    }
    
    if ((deltaSetpoint > driveThreshold || deltaSetpoint < -driveThreshold) && TIMERS_IsTimerExpired(timerId) && driveTimeCheck) {
        driveTimeCheck = 0;
    } else if((deltaSetpoint < driveThreshold && deltaSetpoint > -driveThreshold) && TIMERS_IsTimerExpired(timerId) && driveTimeCheck){
        isFinishedDrive = 1;
        return 0;
    }

    prevClockTime = currentClockTime;

    char newPower = driveP * deltaSetpoint;
    if(newPower > maxPower){
        return maxPower;
    } else if(newPower < -maxPower){
        return -maxPower;
    }
    else if(newPower < minPower && newPower > 0){
        return minPower;
    } else if(newPower > -minPower){
        return -minPower;
    }
    return newPower;
}

char MOV_isFwdFinished(){
    return isFinishedDrive;
}