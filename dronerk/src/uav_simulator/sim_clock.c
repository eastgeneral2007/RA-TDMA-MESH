/* CLOCK of simulator  */
#include "sim_clock.h"
#define PREFIX	"SimClock"
#include "utils.h"


#include <stdio.h>		/* printf */
#include <inttypes.h>	/* int64_t etc */
#include <sys/mman.h>	/* shm_open */
#include <sys/stat.h>	/* For mode constants */
#include <fcntl.h>		/* For O_* constants */
#include <semaphore.h>	/* semaphore */
#include <unistd.h>		/* ftruncate */
#include "return_codes.h"


#define IF_INIT_RETURN(x)\
if (!init_flag)\
{\
	PRINTF_FL_ERR("Not init'ed. Call SimClock_init()\n");\
	return x;\
}


/***********************
 * Static evil globals 
 ***********************/
static simclock_t *microseconds_ptr ; /* save sim time here - its a shared 8B memory space  */ 
static volatile int init_flag = 0 ;
static sem_t 	*clock_semaphore ; /* to control the access to microseconds_ptr */

/***************************+*
 * nonstatic Evil globals 
 *********************++*/
/* NONE HURRAY ! */


/**
 * What: Initialize Simulator clock 
 * How: it creates a sharedmemory an semaphore for other processes to read/control the clock  
 * */
int8_t SimClock_init(void)
{
	PRINTF_FL("Initiating \n");
	/* create and it mustbe **NEW**, 
	 * otherwise we maybe reading some old instance(?). 
	 * R+W access */
	const int oflag = O_CREAT | O_EXCL | O_RDWR ; 
	const mode_t mode = S_IRUSR | S_IWUSR ;
	int fd;
	do
	{
		fd = shm_open(CMEM_NAME, oflag, mode );
		if (fd!=-1)break;
		
		PRINTF_FL_WARN("Clock SHM already exists..trying to unlink..\n");
		if ( shm_unlink(CMEM_NAME) == -1 )
		{
			PRINTF_FL_ERR("..Clock SHM fail to unlink :(\n");
			return NOTOKAY;
		}
		//else
		PRINTF_FL("..Clock SHM - unlinked :)\n");
	} while(1);

	/* set memory space to 64 bits. that's all we need */
	if (-1 == ftruncate(fd, sizeof(simclock_t))) 
	{
		PRINTF_FL_ERR("ftruncate - failed!\n");
		return NOTOKAY;
	}
	microseconds_ptr = (simclock_t*)
		mmap(NULL, sizeof(simclock_t), 
		PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0 ) ;
	if (MAP_FAILED == microseconds_ptr)
	{
		PRINTF_FL_ERR("mmap - failed!\n") ;
		return NOTOKAY ;
	}
	close(fd) ; /* file not needed anymore */ 
	
	
	/* semaphore now to control mem space: */ 
	do
	{
		clock_semaphore = sem_open(CSEM_NAME, O_CREAT|O_EXCL, 0777, 1 ) ;
		if (SEM_FAILED != clock_semaphore){
			PRINTF_FL("sem ok %p\n",(void*)clock_semaphore);
			break;}
		PRINTF_FL("Clock Semaphore already exists!\n");
		if (-1 == sem_unlink(CSEM_NAME))
		{
			PRINTF_FL_ERR("Clock Semaphore - fail to unlink!\n");
			return NOTOKAY ;
		}
		//else
		PRINTF_FL("Clock Semaphore - unlinked!\n");
		
	} while (1) ;
	
	*microseconds_ptr = 0 ; /* init clock */
	init_flag = 1 ; /* set module loaded */
	return OKAY ;
}


/* returns time of the clock
 * returns 0 in case clock wasnt loaded */
simclock_t SimClock_get(void)
{
	IF_INIT_RETURN(0);
	simclock_t tmp_clock = 0 ; 
	sem_wait(clock_semaphore);  
	tmp_clock = *microseconds_ptr ;
	sem_post(clock_semaphore); 
	return tmp_clock;
}

void SimClock_move(simclock_t delta)
{
	IF_INIT_RETURN();
	sem_wait( clock_semaphore ); 
	*microseconds_ptr += delta ;
	sem_post( clock_semaphore ); 
}

/* set clock to <x> us */
void SimClock_set( simclock_t time )
{
	IF_INIT_RETURN();
	sem_wait( clock_semaphore ); 
	*microseconds_ptr = time ;
	sem_post( clock_semaphore ); 
}

/* printf clock time */
void SimClock_print(void)
{
	IF_INIT_RETURN();
	sem_wait( clock_semaphore );  
	PRINTF_FL("[CLOCK %.2fs = %"PRIu64"us]\n", 
		*microseconds_ptr/1e6, *microseconds_ptr) ;
	sem_post( clock_semaphore ); 
}

/* pause clock time */
uint64_t SimClock_pause(void)
{
	IF_INIT_RETURN(0);
	sem_wait( clock_semaphore );  
	PRINTF_FL("[CLOCK %.2fs = %"PRIu64"us] paused\n", 
		*microseconds_ptr/1e6, *microseconds_ptr) ;
	return *microseconds_ptr;
}

/* resume clock time */
void SimClock_resume(void)
{
	IF_INIT_RETURN();
	sem_post( clock_semaphore ); 
	PRINTF_FL("[CLOCK %.2fs = %"PRIu64"us] Resumed\n", 
		*microseconds_ptr/1e6, *microseconds_ptr) ;
}
