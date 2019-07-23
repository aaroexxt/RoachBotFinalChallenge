#include <stdio.h>

#include "Roach_Events.h"
#include "roach.h"
#include "Roach_Top_Level_SM.h"
#include "timers.h"

#include "NavigateObstacleField.h"
#include "CrossNarrowBridge.h"
#include "LocateExtractionPoint.h"


//a list of states that this SM uses:

enum {
    NavigateObstacleField,
    CrossNarrowBridge,
    LocateExtractionPoint,
    Idle
};

int current_state;

/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_RoachStateMachine(void)
{
    current_state = NavigateObstacleField;
    ThrowEvent(STATE_TRANSITION);
    
    Initialize_NavigateObstacleField_StateMachine();
    Initialize_CrossNarrowBridge_StateMachine();
    Initialize_LocateExtractionPoint_StateMachine();
};

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event*/
void Run_Roach_TopLevel_StateMachine(Event event)
{
    switch (current_state) {
    case NavigateObstacleField:
        printf("Top level state:  NavigateObstacleField\r\n");
        event = Run_Roach_NavigateObstacleField_StateMachine(event);
        if (event == ENTERED_DARK) {
            TRANSITION_TO(CrossNarrowBridge);
        }
        break;
        
    case CrossNarrowBridge:
        printf("Top level state:  CrossNarrowBridge\r\n");
        event = Run_Roach_CrossNarrowBridge_StateMachine(event);
        if (event == FRONT_LEFT_BUMP_PRESSED || event == FRONT_RIGHT_BUMP_PRESSED) {
            TRANSITION_TO(LocateExtractionPoint);
        }
        break;
        
    case LocateExtractionPoint:
        printf("Top level state:  LocateExtractionPoint\r\n");
        if (event == FRONT_LEFT_BUMP_PRESSED){
            ThrowEvent(DONE);
        }
        event = Run_Roach_LocateExtractionPoint_StateMachine(event);
        if (event == DONE) {
            TRANSITION_TO(Idle);
        }
        break;

    case Idle:
        printf("Top level state:  Idle\r\n");
        if (event == STATE_TRANSITION){
            TIMERS_InitTimer(NAV_TIMER, 500);
            Roach_LEDSSet(0xFFF);
        }
        if (event == NAV_TIMER_EXPIRED) {
            TIMERS_InitTimer(NAV_TIMER, 500);
            Roach_LEDSSet(~Roach_LEDSGet());
        }
        break;

        
    };
}