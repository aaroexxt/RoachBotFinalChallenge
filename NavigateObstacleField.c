#include <stdio.h>

#include "Roach_Events.h"
#include "roach.h"
#include "Roach_Top_Level_SM.h"
#include "timers.h"

//a list of states that this SM uses:

enum {
    //your states here
    Forward,
    Reverse,
    Idle,
};

int current_state;

/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_NavigateObstacleField_StateMachine(void)
{
    //set current state:
};

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event*/
Event Run_Roach_NavigateObstacleField_StateMachine(Event event)
{
    switch (current_state) {
    case Forward:
        printf("Current state = FORWARD\r\n");
        if(event == FRONT_LEFT_BUMP_PRESSED || event == FRONT_RIGHT_BUMP_PRESSED){
            TRANSITION_TO(Reverse);
        }
    case Reverse:
        printf("Current state = REVERSE\r\n");
        if(event == STATE_TRANSITION){
            Roach_LeftMtrSpeed(-100);
            Roach_RightMtrSpeed(-100);
        }
    }        
    return event;
};
