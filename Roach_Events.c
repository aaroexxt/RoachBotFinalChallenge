#include "Roach_Events.h"
#include "roach.h"
#include "timers.h"
#include "serial.h"

#include <stdio.h>


/* Conrigure the */
#define DEBOUNCE_TIMER 15 //select a timer to be the debounce timer
#define DEBOUNCE_PERIOD 30 //in milliseconds, the duration of the debounce period

#define LIGHT_THRESHOLD 800 //The threshold between light and darkness - you may need to change this!


/* ThrowEvent and CheckForThrownEvent are a pair,
 * ThrowEvent passes an event to CheckForThrownEvent, which
 * CheckForThrownEvent then reports to the ES framework and then clears*/
char thrown_event = TRUE;
void ThrowEvent(Event event_to_throw){
    thrown_event = event_to_throw;
}

Event CheckForThrownEvent(void){
    Event event_to_throw = thrown_event;
    thrown_event = NO_EVENT;
    return event_to_throw;
}


Event CheckForTimerEvents(void)
{
    static char previous_timer_state = TIMER_NOT_ACTIVE;
    char current_timer_state = TIMERS_IsTimerActive(NAV_TIMER);

    if (previous_timer_state != current_timer_state) {
        previous_timer_state = current_timer_state;

        if (current_timer_state == TIMER_NOT_ACTIVE) return NAV_TIMER_EXPIRED;
    }

    //if we got this far, there was no event
    return NO_EVENT;
}

Event CheckBothBumperPressed(void){
    static char leftFirst = 0;
    static char firstCheck = 0;
    static char previous_fr_bumper_state = BUMPER_NOT_TRIPPED;
    static char previous_fl_bumper_state = BUMPER_NOT_TRIPPED;
    char current_fr_bumper_state = Roach_ReadFrontRightBumper();    
    char current_fl_bumper_state = Roach_ReadFrontLeftBumper();
    if((previous_fl_bumper_state == BUMPER_NOT_TRIPPED && current_fr_bumper_state == BUMPER_TRIPPED) || (previous_fr_bumper_state == BUMPER_NOT_TRIPPED && current_fr_bumper_state == BUMPER_TRIPPED)){
        TIMERS_InitTimer(0, 50);
        firstCheck = 1;
    }
    if(firstCheck){
        firstCheck = 0;
        if(TIMERS_IsTimerExpired(0)){
                if(current_fl_bumper_state == BUMPER_TRIPPED && current_fr_bumper_state == BUMPER_TRIPPED){
                    return BOTH_BUMPER_PRESSED;
                }
        }
    }
    previous_fr_bumper_state = current_fr_bumper_state;
    previous_fl_bumper_state = current_fl_bumper_state;
    return NO_EVENT;
}
Event CheckForBumperEvents(void)
{
    static char previous_fl_bumper_state = BUMPER_NOT_TRIPPED;
    static char previous_fr_bumper_state = BUMPER_NOT_TRIPPED;

    //a rough but effective way to handle debouncing:
    if (TIMERS_IsTimerActive(DEBOUNCE_TIMER)) return NO_EVENT;

    //now check for events:
    char current_fl_bumper_state = Roach_ReadFrontLeftBumper();
    if (current_fl_bumper_state != previous_fl_bumper_state) {
        previous_fl_bumper_state = current_fl_bumper_state;
        //start debounce period:
        TIMERS_InitTimer(DEBOUNCE_TIMER, DEBOUNCE_PERIOD);
        if (previous_fl_bumper_state != 0) {
            return FRONT_LEFT_BUMP_PRESSED;
        } else return FRONT_LEFT_BUMP_RELEASED;
    }

    char current_fr_bumper_state = Roach_ReadFrontRightBumper();
    if (current_fr_bumper_state != previous_fr_bumper_state) {
        previous_fr_bumper_state = current_fr_bumper_state;
        //start debounce period:
        TIMERS_InitTimer(DEBOUNCE_TIMER, DEBOUNCE_PERIOD);
        if (current_fr_bumper_state != 0) {
            return FRONT_RIGHT_BUMP_PRESSED;
        } else return FRONT_RIGHT_BUMP_RELEASED;
    }

    //if we got this far, there was no event
    return NO_EVENT;
}

enum {
    DARK, LIGHT
};

Event CheckForLightEvents(void)
{
    static char previousLightState = LIGHT;

    char currentLightState;
    int current_light_reading = Roach_LightLevel();
    if (current_light_reading >= LIGHT_THRESHOLD) {
        currentLightState = DARK;
    } else currentLightState = LIGHT;
    if (currentLightState != previousLightState) {
        previousLightState = currentLightState;
        if (currentLightState == DARK) {
            return ENTERED_DARK;
        } else return ENTERED_LIGHT;
    }

    //if we got this far, there was no event
    return NO_EVENT;
}

Event CheckForAllEvents(void)
{
    Event returnEvent = NO_EVENT;

    returnEvent = CheckForThrownEvent();
    if (returnEvent != NO_EVENT) return returnEvent;
    
    returnEvent = CheckForTimerEvents();
    if (returnEvent != NO_EVENT) return returnEvent;

    returnEvent = CheckForBumperEvents();
    if (returnEvent != NO_EVENT) return returnEvent;

    returnEvent = CheckForLightEvents();
    if (returnEvent != NO_EVENT) return returnEvent;

    return NO_EVENT;
}
