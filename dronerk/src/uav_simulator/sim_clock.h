#ifndef SIM_CLOCK_H_
#define SIM_CLOCK_H_





#define CMEM_NAME	"/clock_memspace"
#define CSEM_NAME	"/clock_semaphore"


#include <inttypes.h>
typedef uint64_t simclock_t;

/* read the current time */
simclock_t SimClock_get(void);

/* move time by <delta>us current time */
void SimClock_move( uint64_t delta );

/* set a given <time>us in the clock */
void SimClock_set( uint64_t time ) ;

/* printf clock time */
void SimClock_print(void);

/* init clock by creating a shared memory space for other processes to read clock time */
int8_t SimClock_init(void);

/* pause clock time */
uint64_t SimClock_pause(void);

/* resume clock time */
void SimClock_resume(void);
#endif
