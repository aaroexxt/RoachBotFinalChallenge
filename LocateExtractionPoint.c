//File: LocateExtractionPoint.c


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
    //your states here
    Aligning,
    Orienting,
    Driving
};

//Enum for Aligning Substates
enum {
    TurnZero,
    DriveForward,
    Reverse,
    TurnNinety
};

int current_state;
int substate_state;

AccelData currentAccel;
GyroData currentGyro;
MagData currentMag;

/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_LocateExtractionPoint_StateMachine(void)
{
    I2C_Init(); //init I2C library
    I2C_setDebugOn(); //set debug mode to be on
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
            switch (substate_state) {
                case TurnZero:
                    printf("Turn Zero");
                    break;
                case DriveForward:
                    printf("DriveForward");
                    break;
                case Reverse:
                    printf("Reverse");
                    break;
                case TurnNinety:
                    printf("TurnNinety");
                    break;
            }
            break;
        case Orienting:
            break;
        case Driving:
            break;
    }        
    return event;
};
