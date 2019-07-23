/* 
 * File:   JAADI2CLib.c
 * Author: Aaron Becker
 *
 * Created on July 5, 2019, 1:32 PM
 */

//SOmeone please help i am writing this completely custom lol


/*
I2C COMMUNICATION LETS GET IT


START:
SDA goes high -> low before SCL high -> low
STOP:
SDA goes low -> high after SCL low -> high
ADDRESS:
7 or 10 bit sequence (will be 7) that identifies the slave

REQUIRES PULL UP RESISTORS



*/

#include "JAADI2CLib.h"
#include "JAADIOLib.h"

/** Start condition delay time: 4.0 us (100 kHz) */
const int T1 = 4;

/** Basic clock delay time: 4.7 us (100 kHz) */
const int T2 = 5;

/** Maximum number of clock stretching retries: 100 us */
const int CLOCK_STRETCHING_RETRY_MAX = 25;

/** Transaction state; start or repeated start condition. */
char m_start;

/** CPU CLOCK */
#define CPU_FREQUENCY (80000000UL)                  /* Fcy = 80MHz (unsigned long) */
#define CORE_TIMER_FREQUENCY (CPU_FREQUENCY/2) //core timer only triggeres after 2 cpu cycles
#define CORE_TIMER_MILLISECONDS (CORE_TIMER_FREQUENCY/1000) //millisecond count time (cpu cycles)
#define CORE_TIMER_MICROSECONDS (CORE_TIMER_FREQUENCY/1000000) //microsecond count time (cpu cycles)







char I2C_initted = FALSE;

//Top level abstractions
int I2C_Init(unsigned int speed) {
	if (!I2C_initted) {
		I2C_initted = true;
		//Set SCL/SDA to OUTPUTS
		//Set SCL/SDA to HIGH
		if (IO_checkLAT(SCLLAT))

	}
}

//Basic conditions for frames

//return 1 for success 0 for fail
char startCondition() {
	SDAHIGH(); //pulse sda
	if (checkLat(SDAPIN)) {
		return false;
	}
	SDALOW();
	delayUS(T1);
	SCLHIGH(); //pulse scl
	SCLLOW();


}

//Low level functions
void delayUS(UINT32 delay_us) {
   UINT32 DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_us * CORE_TIMER_MICROSECONDS));
}

void delayMS(UINT32 delay_ms) {
   UINT32 DelayStartTime;
   
   DelayStartTime = ReadCoreTimer(); //read core timer into delay start time
   while((ReadCoreTimer() - DelayStartTime) < (delay_ms * CORE_TIMER_MILLISECONDS));
}
 