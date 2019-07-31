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
    DriveToCorner,
    TurnToWall1,
    TurnToWall2
};


static int current_state;
static int substate_state;
char leftPressedFirst = 0;
//sets the speed of the roach after one of the front bumpers are pressed 
const int RoachTurnSpeed = 100;



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
    
   current_state = Aligning;
   substate_state = DriveForward;
   Roach_LeftMtrSpeed(70);
   Roach_RightMtrSpeed(-70);
//   MOV_initTurn(-45);
//     MOV_initFwd(-20);
};

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event*/
Event Run_Roach_LocateExtractionPoint_StateMachine(Event event) {
    
    switch (current_state) {
        case Aligning:
        printf("Aligning");
            switch (substate_state) {
                // Roach_LeftMtrSpeed(-100);
                // Roach_RightMtrSpeed(-100);
                // substate_state = Reverse;
                
                case TurnZero:
                    printf("TurnZero");
                    if (MOV_isTurnFinished()) {
                       substate_state = DriveForward;
                       Roach_LeftMtrSpeed(70);
                       Roach_RightMtrSpeed(-70);
                    } else{
                        int newMotorSpeed = MOV_updateTurn();
                        Roach_LeftMtrSpeed(-newMotorSpeed);
                        Roach_RightMtrSpeed(-newMotorSpeed);
                    }
                    break;
                case DriveForward:
//                    printf("DriveForward");
                    if (event == FRONT_LEFT_BUMP_PRESSED) {
                        Roach_LeftMtrSpeed(0);
                        Roach_RightMtrSpeed(-100);
                        leftPressedFirst = 1;
                        substate_state = TurnToWall1;
                    } else if(event == FRONT_RIGHT_BUMP_PRESSED){
                        Roach_LeftMtrSpeed(100);
                        Roach_RightMtrSpeed(0);
                        leftPressedFirst = 0;
                        substate_state = TurnToWall1;
                    }

                    
                    break;

                case TurnToWall1:
                    printf("TurnToWall1");
                    if (!leftPressedFirst && event == FRONT_LEFT_BUMP_PRESSED){
//                        printf("HERE");
//                        MOV_initFwd(-20);
//                        substate_state = Reverse;
                        substate_state = TurnNinety;
                        MOV_initTurn(90);
                    }
                    else if (leftPressedFirst && event == FRONT_RIGHT_BUMP_PRESSED)
                    {
//                        printf("HERE");
//                        MOV_initFwd(-20);
//                        substate_state = Reverse;
                    substate_state = TurnNinety;
                        MOV_initTurn(90);
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
                        Roach_RightMtrSpeed(-newMotorSpeed);
                    }
                    break;
                case TurnNinety:
                    printf("TurnNinety");
                    if (MOV_isTurnFinished()) {
                        substate_state = DriveToCorner;
                        Roach_LeftMtrSpeed(70);
                        Roach_RightMtrSpeed(-70);
                    }else{
                        int newMotorSpeed = MOV_updateTurn();
                        Roach_LeftMtrSpeed(-newMotorSpeed);
                        Roach_RightMtrSpeed(-newMotorSpeed);
                    }
                    break;
                case DriveToCorner:
                    printf("DriveToCorner");

                    if (event == FRONT_LEFT_BUMP_PRESSED) {
                        Roach_LeftMtrSpeed(0);
                        Roach_RightMtrSpeed(-100);
                        leftPressedFirst = 1;
                        substate_state = TurnToWall2;
                    } else if(event == FRONT_RIGHT_BUMP_PRESSED){
                        Roach_LeftMtrSpeed(100);
                        Roach_RightMtrSpeed(0);
                        leftPressedFirst = 0;
                        substate_state = TurnToWall2;
                    }
                    break;

                case TurnToWall2:
                    printf("TurnToWall2");
                     if (!leftPressedFirst && event == FRONT_LEFT_BUMP_PRESSED){
//                        printf("HERE");
//                        MOV_initFwd(-20);
//                        substate_state = Reverse;
                        current_state = Orienting;
                        MOV_initTurn(145);
                    }
                    else if (leftPressedFirst && event == FRONT_RIGHT_BUMP_PRESSED)
                    {
//                        printf("HERE");
//                        MOV_initFwd(-20);
//                        substate_state = Reverse;
                        current_state = Orienting;
                        MOV_initTurn(145);
                    }                   

                    break;
            }

            break;


        case Orienting:
            printf("Orienting");
            if (MOV_isTurnFinished()) {
                current_state = Driving;
                MOV_initFwd(18);
            } else{
                int newMotorSpeed = MOV_updateTurn();
                Roach_LeftMtrSpeed(-newMotorSpeed);
                Roach_RightMtrSpeed(-newMotorSpeed);
            }
            break;
        case Driving:
            printf("Driving");
            if (MOV_isFwdFinished()){
                Roach_LeftMtrSpeed(0);
                Roach_RightMtrSpeed(0);
                current_state= Finish;
            }else{
                int newMotorSpeed =  MOV_updateFwd();
                Roach_LeftMtrSpeed(newMotorSpeed);
                Roach_RightMtrSpeed(-newMotorSpeed);
            }
            break;
        case Finish:
            break;

    }        
    return event;
};



