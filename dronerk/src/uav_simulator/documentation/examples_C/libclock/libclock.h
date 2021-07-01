#ifndef SIM_CLOCK_H_
#define SIM_CLOCK_H_


#define CMEM_NAME	"/clock_memspace"
#define CSEM_NAME	"/clock_semaphore"

#include <inttypes.h>
typedef uint64_t simclock_t;

/* read the current time */
simclock_t SimClock_get(void);

/* printf clock time */
void SimClock_print(char *string);

/* init clock by opening a shared memory and semaphore space 
 * to read clock time */
int SimClock_init(void);

/* pause clock time */
simclock_t SimClock_pause(void);

/* resume clock time */
void SimClock_resume(void);


#endif
