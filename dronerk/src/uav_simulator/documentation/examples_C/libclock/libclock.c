/* interface with world simulator CLOCK */

//#include "world_simulator_api.h"  // get port numbers, read clock, etc


#include "generic.h"

#include "utils.h"
#include "libclock.h"
#define PREFIX "libclock"


#include <stdio.h>		/* printf */
#include <inttypes.h>	/* int64_t etc */
#include <sys/mman.h>	/* shm_open */
#include <sys/stat.h>	/* For mode constants */
#include <fcntl.h>		/* For O_* constants */
#include <semaphore.h>	/* semaphore */
#include <unistd.h>		/* ftruncate */


#define IF_INIT_RETURN(x)\
if (!init_flag)\
{\
	PRINTF_FL_ERR("Not init'ed. Call SimClock_init()\n");\
	return x;\
}


/***********************
 * Static evil globals 
 ***********************/
static simclock_t 		*microseconds_ptr; // shared mem space - 8 bytes to store "microseconds"
static sem_t 			*clock_semaphore ;
static volatile int 	init_flag = 0 ;


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
	
	init_flag = 1 ; /* set module loaded */
	return OKAY ;
}


/* return sim clock ( microseconds ) */
simclock_t SimClock_get(void)
{
	IF_INIT_RETURN(0);
	simclock_t tmp_clock = 0; 
	sem_wait(clock_semaphore);  
	tmp_clock = *microseconds_ptr ;
	sem_post(clock_semaphore); 
	return tmp_clock ;
}

/* pause clock time, no one can access it until calling Resume */
simclock_t SimClock_pause(void)
{
	IF_INIT_RETURN(0);
	sem_wait(clock_semaphore);  
	simclock_t tmp_clock = *microseconds_ptr ;
	PRINTF_FL("[CLOCK %" PRIu64 "us] Paused\n" , *microseconds_ptr ) ;
	return tmp_clock;
}

/* resume clock time */
void SimClock_resume(void)
{
	IF_INIT_RETURN();
	sem_post(clock_semaphore); 
	SimClock_print("Resumed");
}

/* printf clock time */
void SimClock_print(char * string)
{
	IF_INIT_RETURN();
	sem_wait(clock_semaphore);  
	PRINTF_FL("[CLOCK %"PRIu64"us] %s\n", *microseconds_ptr, string) ;
	sem_post(clock_semaphore); 
}
