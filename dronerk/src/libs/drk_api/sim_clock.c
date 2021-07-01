/* if we want to test drone code in a simulation
 * this module needs to be compiled too
 * compilation of drklib*.so demands -D SIMULATOR flag
 * this module allow the drone-app to read clocktime. 
 * all drklib or app code should use this module to read time or to operate in time 
 * utils.h already uses this module for most of its functions:
 * ex.: getClockTime(), microsleep(), etc 
 * at init time, a sim-clock gets a random offset and drift regarding hostmachine clocktime 
 * time at each virtual drone is therefore different 
 * Important:
 * drones are able to PAUSE and RESUME simtime */

#include <fcntl.h>	/* For O_* constants */ 
#include <inttypes.h>	/* int64_t etc */
#include <semaphore.h>	/* semaphore */
#include <stdlib.h> /* rand */
#include <stdio.h>	// printf
#include <sys/mman.h>	/* shm_open */
#include <sys/stat.h>	/* For mode constants */
#include <sys/timeb.h>
#include <unistd.h> /* close */


#include "drk/sim_clock.h"  /* public functions */
#include "drk/utils.h"

#define MAX_OFFSET 			500000 // microseconds - 500ms is realistic
#define PREFIX "SIMCLK"


typedef enum
{
	OKAY=1 ,
	NOTOKAY=-1
}
return_codes ;

#define IF_INIT_RETURN(x)\
if (!init_flag)\
{\
	PRINTF_FL_ERR("Not init'ed. Call SimClock_init()\n");\
	return x;\
}


/***********************
 * Static evil globals 
 ***********************/
static volatile int init_flag = 0 ;
static simclock_t 	*microseconds_ptr ; // shared mem space - 8 bytes to store "microseconds"
static sem_t 		*clock_semaphore ;

/* each instance of uav-simulated-app has a unique drift and offset */
static float	drift = 0 ; /* random init*/
static int 		offset = 0 ; /* random init too */
	



simclock_t driftme( simclock_t tmp ) 
{
	return tmp + drift*tmp + offset;
}


/* "init" clock by opening a shared memory space and semaphore
 *  that world simulator has already created */
int SimClock_init(void)
{

	const int oflag = O_RDONLY ; /* create . Read access only */
	const mode_t mode = S_IRUSR ;
	int fd = shm_open(CMEM_NAME, oflag, mode );
	if (fd == -1)
	{
		PRINTF_FL_ERR("Clock Memory Space does not exist :(. run ./drone_simulator ?\n");
		//perror("shm_open");
		return NOTOKAY ;
	}
	

	microseconds_ptr = (simclock_t*)mmap( NULL , sizeof(simclock_t), PROT_READ, MAP_SHARED, fd, 0 ) ;
	if (MAP_FAILED == microseconds_ptr)
	{
		PRINTF_FL_ERR("mmap failed!") ;
		return NOTOKAY ;
	}
	
	close(fd) ; /* not needed anymore */ 
	
	clock_semaphore = sem_open(CSEM_NAME, O_CREAT, 0444, 0) ;
	if (SEM_FAILED == clock_semaphore)
	{
		PRINTF_FL_ERR("clock Semaphore does not exist!\n");
		return NOTOKAY ;
	}
	
	/* create some offset and drift from the simulator clock when providing readings */
	struct timeb rawtime ;
	ftime( &rawtime ) ;
	srand( rawtime.millitm ) ;
	
	drift = (rand()%5) / 4.0 * 0.000001 ;
	offset = (rand() % (2*MAX_OFFSET)) - MAX_OFFSET ;
	PRINTF_FL("Clock init Done -- offset %dus, drift %.4f%%\n", 
		offset, 
		100.0*(1.0+drift)) ;
		
	
	init_flag = 1 ; /* set module loaded */
	return OKAY ;
}


/* return sim clock ( microseconds ) */
simclock_t SimClock_get(void)
{
	IF_INIT_RETURN(0);
	sem_wait(clock_semaphore);  
	simclock_t tmp_clock = *microseconds_ptr ;
	sem_post(clock_semaphore); 
	
	return driftme(tmp_clock);
}


/* pause clock time, 
 * WARNING: possible deadlocks
 * no one can access it until calling Resume! 
 * it returns current time on the clock */
simclock_t SimClock_pause(void)
{
	IF_INIT_RETURN(0);
	sem_wait(clock_semaphore);  
	simclock_t tmp_clock = *microseconds_ptr ;
	PRINTF_FL("[CLOCK %" PRIu64 "us] Paused\n", driftme(tmp_clock));
	return driftme(tmp_clock);
}

/* resume clock time */
void SimClock_resume(void)
{
	IF_INIT_RETURN();
	sem_post(clock_semaphore); 
	SimClock_print("Resumed");
}

/* printf clock time */
#ifdef VERBOSE
void SimClock_print(char *string)
{
	IF_INIT_RETURN();
	PRINTF_FL("[CLOCK %.2fs = %"PRIu64"us] %s\n", 
		SimClock_get()/1e6, SimClock_get(), string) ;
}
#endif

