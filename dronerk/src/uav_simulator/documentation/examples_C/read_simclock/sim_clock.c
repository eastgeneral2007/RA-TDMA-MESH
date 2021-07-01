/* example of interface with world simulator CLOCK */

#include "libclock.h"  // prototypes 

#define PREFIX	"MAIN"

#include <stdio.h>	// printf
#include <inttypes.h>	// int16_t etc
#include <sys/mman.h>	/* shm_open */
#include <sys/stat.h>	/* For mode constants */
#include <fcntl.h>	/* For O_* constants */
#include <semaphore.h>	/* semaphore */
#include <unistd.h> /* close */





/***********************
 * Static evil globals 
 ***********************/
/* NONE HURRAY ! */

/***************************+*
 * nonstatic Evil globals 
 **********************/
/* NONE HURRAY ! */



/**
 * What: Initialize Simulator clock 
 * How: it creates a sharedmemory an semaphore for other processes to read/control the clock  
 * */
int clock_init(void)
{
	printf("Initiating the clock\n");
	int ret = SimClock_init();
	return ret ;
}

/* printf clock time */
void clock_print10times(void)
{
	for (int i = 0;i<5;i++)
	{
		char s[5];
		sprintf(s, "[x%d]", i);
		SimClock_print(s);
		sleep(1);
	}
}




int main()
{
	clock_init();
	
	clock_print10times();
	
	/* this can deadlock the simulator. 
	 * it STOPs the simulator clock.
	 * any call to get time or print time will wait forever.
	 theres no "watchdog" that resumes the clock after a timeout */
	SimClock_pause(); 
	
	sleep(2);
	SimClock_resume();
	
	clock_print10times(); /* time is basically still the same before pausing */
			
		
	return 0;
}


