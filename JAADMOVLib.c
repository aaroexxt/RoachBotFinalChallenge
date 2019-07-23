/* 
 * File:   JAADSM
 * Author: Ashvin
 *
 * Created on July 5, 2019, 1:32 PM
 */

#include "JAADMOVLib.h"
#define turnP 0.2
#define driveP 0.2
int turnCurrentPos = 0;
int gyroAngle = 0;
int turnSetpoint = 0;
char isFinishedTurn = 0;
int turnCount = 0;

void initTurn(int degrees){
    turnCount = 0;
    turnCurrentPos = 0;
    turnSetpoint = degrees;
    isFinishedTurn = 0;
}
int turn(void){
    gyroAngle = 0; //will change to update gyroAngle
    turnCurrentPos += gyroAngle;
    if(turnSetpoint - turnCurrentPos < 10 && turnSetpoint - turnCurrentPos > - 10){
        turnCount++;
    } else {
        turnCount = 0;
    }
    if(turnCount > 10){
        isFinishedTurn = 1;
        return 0;
    } else{
        return turnP * (turnSetpoint - turnCurrentPos);
    }
}

char isTurnFinished(void){
    return isFinishedTurn;
}

int driveCurrentPos = 0;
int acc = 0;
int vel = 0;
int driveSetpoint = 0;
char isFinishedDrive = 0;
int driveCount = 0;

void initFwd(int distance){
    vel = 0;
    driveSetpoint = distance;
    driveCount = 0;
    driveCurrentPos = 0;
    isFinishedDrive = 0;
}

int fwd(void){
    acc = 0; //change to accelerometer value
    vel += acc;
    driveCurrentPos += vel;
    if(driveSetpoint - driveCurrentPos < 10 && driveSetpoint - driveCurrentPos > -10){
        driveCount++;
    } else {
        driveCount = 0;
    }
    if(driveCount > 10){
        isFinishedDrive = 1;
        return 0;
    } else {
        return driveP * (driveSetpoint - driveCurrentPos);
    }
}

char isFwdFinished(){
    return isFinishedDrive;
}