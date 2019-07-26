//File: LocateExtractionPoint.c

//Author: Joshua Pan


#include <stdio.h>
#include "JAADI2CLib.h"
#include "JAADMOVLib.h"
#include "Roach_Events.h"
#include "roach.h"
#include "Roach_Top_Level_SM.h"
#include "timers.h"

//a list of states that this SM uses:

//Enum for Main States
enum {
    Aligning,
    Orienting,
    Driving,
    Finish
};

//Enum for Aligning Substates
enum {
    TurnZero,
    DriveForward,
    Reverse,
    TurnNinety,
    DriveToCorner
};

static int current_state;
static int substate_state;

AccelData currentAccel;
GyroData currentGyro;
MagData currentMag;

/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_LocateExtractionPoint_StateMachine(void)
{
    printf("init locateextractionpoint");
    /*I2C_Init(); //init I2C library
    I2C_setDebugOn(); //set debug mode to be on*/
    
//    current_state = Aligning;
    //^ removed because I don't want switch-case to run
};

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event*/
Event Run_Roach_LocateExtractionPoint_StateMachine(Event event) {
    //Update the current gyro values
    currentAccel = I2C_getAccelData();
    currentGyro = I2C_getGyroData();
    currentMag = I2C_getMagData();
    
    //Print the values for debugging
    I2C_printAccel(currentAccel);
    I2C_printGyro(currentGyro);
    I2C_printMag(currentMag);
    
    switch (current_state) {
        case Aligning:
        printf("Aligning");
            switch (substate_state) {
                case TurnZero:
                    printf("TurnZero");
                    if (MOV_isTurnFinished()) {
                       substate_state = DriveForward;
                       Roach_LeftMtrSpeed(100);
                       Roach_RightMtrSpeed(100);
                    } else{
                        int newMotorSpeed = MOV_updateTurn();
                        Roach_LeftMtrSpeed(newMotorSpeed);
                        Roach_RightMtrSpeed(newMotorSpeed);
                    }
                    break;
                case DriveForward:
                    printf("DriveForward");
                    if (event == BOTH_BUMPER_PRESSED) {
                        current_state = Reverse;
                        MOV_initFwd(-5);
                    }
                    
                    break;
                case Reverse:
                    printf("Reverse");
                     if (MOV_isFwdFinished()) {
                        substate_state = TurnNinety;
                        MOV_initTurn(90);
                    }else{
                        int newMotorSpeed =  MOV_updateFwd();
                        Roach_LeftMtrSpeed(newMotorSpeed);
                        Roach_RightMtrSpeed(newMotorSpeed);
                    }
                    break;
                case TurnNinety:
                    printf("TurnNinety");
                    if (MOV_isTurnFinished()) {
                        substate_state = DriveToCorner;
                        Roach_LeftMtrSpeed(100);
                        Roach_RightMtrSpeed(100);
                    }else{
                        int newMotorSpeed = MOV_updateTurn();
                        Roach_LeftMtrSpeed(newMotorSpeed);
                        Roach_RightMtrSpeed(newMotorSpeed);
                    }
                    break;
                case DriveToCorner:
                    printf("DriveToCorner");
                    if(event == BOTH_BUMPER_PRESSED){
                        current_state = Orienting;
                        MOV_initTurn(135);
                    }
            }
            break;
        case Orienting:
            printf("Orienting");
            if (MOV_isTurnFinished()) {
                current_state = Driving;
                MOV_initFwd(20);
            } else{
                int newMotorSpeed = MOV_updateTurn();
                Roach_LeftMtrSpeed(newMotorSpeed);
                Roach_RightMtrSpeed(newMotorSpeed);
            }
            break;
        case Driving:
            printf("Driving");
            if (MOV_isFwdFinished()){
                current_state= Finish;
                Roach_LeftMtrSpeed(0);
                Roach_RightMtrSpeed(0);
            }else{
                int newMotorSpeed =  MOV_updateFwd();
                Roach_LeftMtrSpeed(newMotorSpeed);
                Roach_RightMtrSpeed(newMotorSpeed);
            }
            break;
        case Finish:
            break;

    }        
    return event;
};
