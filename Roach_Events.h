/* 
 * File:   Roach_Events.h
 * Author: AutonomousSystemsLab
 *
 * Created on July 15, 2019, 10:49 AM
 */

#ifndef ROACH_EVENTS_H
#define	ROACH_EVENTS_H


/* This timer is reserved as the navigation timer, which produces NAV_TIMER_EXPIRED events.
 * Use it as an input to TIMERS_* functions, eg TIMERS_InitTimer(NAV_TIMER, 1000);
 * */
#define NAV_TIMER 14

//A list of events in this system:

typedef enum {
    NO_EVENT,
    STATE_TRANSITION,
    DONE,
    FRONT_RIGHT_BUMP_PRESSED,
    FRONT_LEFT_BUMP_PRESSED,
    REAR_RIGHT_BUMP_PRESSED,
    REAR_LEFT_BUMP_PRESSED,
    FRONT_RIGHT_BUMP_RELEASED,
    FRONT_LEFT_BUMP_RELEASED,
    REAR_RIGHT_BUMP_RELEASED,
    REAR_LEFT_BUMP_RELEASED,
    ENTERED_DARK,
    ENTERED_LIGHT,
    NAV_TIMER_EXPIRED,
    BOTH_BUMPER_PRESSED,
} Event;

void ThrowEvent(Event event_to_throw);

Event CheckForAllEvents(void);

#endif	/* ROACH_EVENTS_H */

