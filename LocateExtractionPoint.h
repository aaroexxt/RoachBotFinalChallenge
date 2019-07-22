/* 
 * File:   Roach_State_Machine.h
 * Author: AutonomousSystemsLab
 *
 * Created on July 15, 2019, 10:37 AM
 */

#ifndef LOCATE_EXTRACTION_POINT_H
#define	LOCATE_EXTRACTION_POINT_H

#include "Roach_Events.h"

/* This function initializes the roach state machine.
 * At a minimum, this requires setting the first state.
 * Also, execute any actions associated with initializing the SM 
 * (that is, the actions on the arrow from the black dot in the SM diagram)*/
void Initialize_LocateExtractionPoint_StateMachine(void);

/* 
 * @briefThis function feeds newly detected events to the roach state machine.
 * @param event:  The most recently detected event */
Event Run_Roach_LocateExtractionPoint_StateMachine(Event event);

#endif	/* ROACH_STATE_MACHINE_H */

