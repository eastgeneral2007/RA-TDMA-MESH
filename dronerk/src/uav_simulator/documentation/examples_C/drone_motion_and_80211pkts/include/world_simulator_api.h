#ifndef WORLD_SIMULATOR_API_H_
#define WORLD_SIMULATOR_API_H_



/* Simulator expects to receive stuff FROM these ports */
#define UDP_PORT_BASE_SRC_80211	40000U /* drone Bport from where 80211pkt data comes  */
#define UDP_PORT_BASE_SRC_ACT	41000U /* drone Bport from where actuation data comes  */

/* Simulator receives data sent TO this port */
#define SIM_RX_ALL				50000U /* SIM is listening to EVERYHING at this port */


/* Drones sends data from these ports */
#define PORT_ISENSOR				61000U /* SIM sends intsensor data to uav . uavs should listen to this */ 
#define PORT_ESENSOR				62000U /* SIM sends extsensor data to uav . uavs should listen to this */
#define PORT_UAV_RX80211			60000U /* SIM sends data to drone to this port.uavs should listen to this */

/* drone payloads to/from world_simulator */
#define MAX_80211_SIZE			(1500)
#define MAX_SENSOR_SIZE			(30)


#define MILLION			1000000.0

#include <inttypes.h>	// int16_t etc



// this should be a include ! ../libclock/ has this include 
/* get clock time */
uint64_t SimClock_get();

/* printf clock time */
void SimClock_print(void) ;

/* init clock by creating a shared memory space for other processes to read clock time */
int8_t SimClock_init();

/* pause clock time */
void SimClock_pause();

/* resume clock time */
void SimClock_resume();


#endif

