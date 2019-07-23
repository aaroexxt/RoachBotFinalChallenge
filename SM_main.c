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
#include "JAADSM.h"
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

int main(void)
{
    


    //These calls configure the Pic32 so that it can interact with the Roach hardware
    BOARD_Init();
    Roach_Init();
    TIMERS_Init();

    //Initialization code here:
    printf("Welcome to COSMOS final project framework, compiled on %s %s\r\n", __TIME__, __DATE__);


    //first, setup the framework:
    Initialize_RoachStateMachine();

    while (1) {
        //continuous services (event checkers):
        Event this_event = CheckForAllEvents();

        if (this_event != NO_EVENT) {
            //seed rand (leverage unpredictable event timing):
            srand(TIMERS_GetTime() ^ rand());
            
            //handle event:
            PrintEvent(this_event);
            //run service:
            Run_Roach_TopLevel_StateMachine(this_event);
            //clear event:
            this_event = NO_EVENT;

        }

    }

    return (EXIT_SUCCESS);
}

