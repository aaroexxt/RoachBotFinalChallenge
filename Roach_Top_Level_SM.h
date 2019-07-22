/* 
 * File:   Roach_State_Machine.h
 * Author: AutonomousSystemsLab
 *
 * Created on July 15, 2019, 10:37 AM
 */

#ifndef ROACH_STATE_MACHINE_H
#define	ROACH_STATE_MACHINE_H

#include "Roach_Events.h"


/* This macro allows you to cleanly transition between states, and trigger
 * a STATE_TRANSITION event as well */
#define TRANSITION_TO(x) { \
    printf("Transitioning to state " #x); \
    current_state = x; \
    ThrowEvent(STATE_TRANSITION); }



/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_RoachStateMachine(void);

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event */
void Run_Roach_TopLevel_StateMachine(Event event);

#endif	/* ROACH_STATE_MACHINE_H */

