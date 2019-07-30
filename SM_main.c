/* 
 * File:   Hello_World_main.c
 * Author: AutonomousSystemsLab
 *
 * Created on July 5, 2019, 1:32 PM
 */

//Standard C libraries:
#include <stdio.h>
#include <stdlib.h>

//Microchip's specialized library
#include <xc.h>

//A library to use the uc32 board
#include "BOARD.h"
#include "roach.h"
#include "timers.h"
#include "Roach_Top_Level_SM.h"
#include "JAADMain.h"
#include "JAADI2CLib.c"
#include "JAADMOVLib.c"
#include "JAADIOLib.c"
#include "Roach_Events.h"


// <editor-fold defaultstate="collapsed" desc="//PrintEvent() definiton">
void PrintEvent(Event event)
{
    switch (event) {

    case NO_EVENT:
        printf("Event:  NO_EVENT\r\n");
        break;
    case STATE_TRANSITION:
        printf("Event: STATE_TRANSITION\r\n");
        break;
    case FRONT_RIGHT_BUMP_PRESSED:
        printf("Event:  FRONT_RIGHT_BUMP_PRESSED\r\n");
        break;
    case FRONT_LEFT_BUMP_PRESSED:
        printf("Event:  FRONT_LEFT_BUMP_PRESSED\r\n");
        break;
    case REAR_RIGHT_BUMP_PRESSED:
        printf("Event:  REAR_RIGHT_BUMP_PRESSED\r\n");
        break;
    case REAR_LEFT_BUMP_PRESSED:
        printf("Event:  REAR_LEFT_BUMP_PRESSED\r\n");
        break;
    case FRONT_RIGHT_BUMP_RELEASED:
        printf("Event:  FRONT_RIGHT_BUMP_RELEASED\r\n");
        break;
    case FRONT_LEFT_BUMP_RELEASED:
        printf("Event:  FRONT_LEFT_BUMP_RELEASED\r\n");
        break;
    case REAR_RIGHT_BUMP_RELEASED:
        printf("Event:  REAR_RIGHT_BUMP_RELEASED\r\n");
        break;
    case REAR_LEFT_BUMP_RELEASED:
        printf("Event:  REAR_LEFT_BUMP_RELEASED\r\n");
        break;
    case ENTERED_DARK:
        printf("Event:  ENTERED_DARK\r\n");
        break;
    case ENTERED_LIGHT:
        printf("Event:  ENTERED_LIGHT\r\n");
        break;
    case NAV_TIMER_EXPIRED:
        printf("Event:  NAV_TIMER_EXPIRED\r\n");
        break;
    default:
        printf("Please add your event to SM_main -> PrintEvent()\r\n");

    }

}
    // </editor-fold>
AccelData currentAccel;
GyroData currentGyro;
MagData currentMag;
int mov;
int main(void)
{
    
    int state = 0;
    //These calls configure the Pic32 so that it can interact with the Roach hardware
    BOARD_Init();
    Roach_Init();
    TIMERS_Init();
    
    //Initialization code here:
    printf("Welcome to COSMOS final project framework, compiled on %s %s\r\n", __TIME__, __DATE__);
    
    printf("\r\n---------\r\nI2CINIT %d\r\n---------\r\n",I2C_Init());
    I2C_setDebugOff();
    delayMS(2000);
//    MOV_initTurn(180);

    //first, setup the framework:
    Initialize_LocateExtractionPoint_StateMachine();

    while (1) {
//        currentMag = I2C_getMagData();
//        currentGyro = I2C_getGyroData();
//        if(!MOV_isTurnFinished()){
//            mov = MOV_updateTurn();
//            Roach_LeftMtrSpeed(-mov);
//            Roach_RightMtrSpeed(-mov);
//        } else {
//            printf("DONE");
//            Roach_LeftMtrSpeed(0);
//            Roach_RightMtrSpeed(0);
//        }
        //Print the values for debugging
//        I2C_printAccel(currentAccel);
//        I2C_printGyro(currentGyro);
//            delayMS(100);

        //I2C_printMag(currentMag);
//        delayMS(100);
        //continuous services (event checkers):
        Event this_event = CheckForAllEvents();
        Run_Roach_LocateExtractionPoint_StateMachine(this_event);
//        if (this_event != NO_EVENT) {
//            //seed rand (leverage unpredictable event timing):
//            srand(TIMERS_GetTime() ^ rand());
//            
//            //handle event:
//            PrintEvent(this_event);
//            //run service:
//            
//            //clear event:
//            this_event = NO_EVENT;
//
//        }

        

    }



    return (EXIT_SUCCESS);
}

