/* 
 * File:   JAADSM
 * Author: Ashvin
 *
 * Created on July 5, 2019, 1:32 PM
 */

#include "JAADMOVLib.h"
#define turnP 0.2
#define driveP 0.2
#define timerId 0
#define checkTime 1000

/*******
* TURN CODE
*******/

//Variables to track state of turn
int turnCurrentPos = 0;
int gyroAngle = 0;
int turnSetpoint = 0;
char isFinishedTurn = 0;
int turnCount = 0;
char turnTimeCheck = 0;

void MOV_initTurn(int degrees) {
    turnCount = 0;
    turnCurrentPos = 0;
    turnSetpoint = degrees;
    isFinishedTurn = 0;
    turnTimeCheck = 0;
}

int MOV_updateTurn(void) {
    gyroAngle = 0; //will change to update gyroAngle
    turnCurrentPos += gyroAngle;
    int deltaSetpoint = turnSetpoint - turnCurrentPos;

    if (deltaSetpoint < 10 && deltaSetpoint > - 10 && TIMERS_IsTimerExpired(timerId) && !turnTimeCheck){
        TIMERS_InitTimer(timerId, checkTime);
        turnTimeCheck = 1;
    }
    
    if ((deltaSetpoint > 10 || deltaSetpoint < - 10) && TIMERS_IsTimerExpired(timerId) && turnTimeCheck) {
        turnTimeCheck = 0;
    } else if((deltaSetpoint < 10 && deltaSetpoint > - 10) && TIMERS_IsTimerExpired(timerId) && turnTimeCheck){
        return 0;
        isFinishedTurn = 1;
    }
    return turnP * deltaSetpoint;
}

char MOV_isTurnFinished(void){
    return isFinishedTurn;
}

/*******
* DRIVE CODE
*******/

//Variables to track state of drive
int driveCurrentPos = 0;
int acc = 0;
int vel = 0;
int driveSetpoint = 0;
char isFinishedDrive = 0;
int driveCount = 0;
char driveTimeCheck

void MOV_initFwd(int distance){
    vel = 0;
    driveSetpoint = distance;
    driveCount = 0;
    driveCurrentPos = 0;
    isFinishedDrive = 0;
    driveTimeCheck = 0;
}

int MOV_updateFwd(void){
    acc = 0; //change to accelerometer value
    vel += acc;
    driveCurrentPos += vel;
    int deltaSetpoint = driveSetpoint - driveCurrentPos;

    if (deltaSetpoint < 10 && deltaSetpoint > - 10 && TIMERS_IsTimerExpired(timerId) && !driveTimeCheck){
        TIMERS_InitTimer(timerId, checkTime);
        driveTimeCheck = 1;
    }
    
    if ((deltaSetpoint > 10 || deltaSetpoint < - 10) && TIMERS_IsTimerExpired(timerId) && driveTimeCheck) {
        driveTimeCheck = 0;
    } else if((deltaSetpoint < 10 && deltaSetpoint > - 10) && TIMERS_IsTimerExpired(timerId) && driveTimeCheck){
        return 0;
        isFinishedDrive = 1;
    }
    return driveP * deltaSetpoint;
}

char MOV_isFwdFinished(){
    return isFinishedDrive;
}