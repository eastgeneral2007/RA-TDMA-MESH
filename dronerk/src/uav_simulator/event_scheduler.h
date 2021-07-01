#ifndef EVENT_SCHEDULER_H_
#define EVENT_SCHEDULER_H_

#include <inttypes.h> /* int16_t etc */
#include <string.h> /* memcpy */
#include <stdlib.h> /* malloc */
#include <stdio.h> /* printf */
#include <netinet/in.h> /* udp packets */
#include <ifaddrs.h> /* udp packets */
#include <arpa/inet.h> /* udp packets */
#include <pthread.h> /* i have threads :D */
#include <unistd.h>
#include <sys/socket.h>


#include "return_codes.h"
#include "utils.h"

#define MAX_NUAVS 	10 
#define ISENSOR_READINGS_PERIOD 300000 /* us (simulation) */
#define ESENSOR_READINGS_PERIOD 300000 /* us (simulation) */
#define DEBUG_ACT 		0 // 1 or 0 to activate/disable debug of actuation messages 
// #define DEBUG 1
#define DEBUG_80211 	1// 1 or 0 to activate/disable debug
#define DEBUG_ESENSOR 	0// 1 or 0 to activate/disable debug
#define DEBUG_ISENSOR	0// 1 or 0 to activate/disable debug
#define CSMA 			1 // 1 or 0 to activate/disable csma

#define PLOT_PERIOD 	800000 //800ms
#define READER_PERIOD 	5 /* scheduler-event-reader cpu-clock period - us*/
#define LAYOUT_UPD_PERIOD 50000


#define LIST_SIZE					(3000) /* event queue size */



#if (LIST_SIZE < 128)
	typedef int8_t qidx_t ;
	#define QIDX_FMT PRId8
#elif (LIST_SIZE < 32768)
	typedef int16_t qidx_t ;
	#define QIDX_FMT PRId16
//#elif if LIST_SIZE < 256*256*256*256
//	typedef uint32_t qidx_t ;
#else
	typedef int64_t qidx_t ;
	#define QIDX_FMT PRId64
#endif /* LIST_SIZE < 256 */

/* typedefs */


/*
 * An event might be 80211 packet actuation orders and sensing readiings
 * ----
 * all have unique ID,  a type, when were added to the list of events 
 * and when they should get triggerd,
 * some have backoff already
 * all have src and dst
 * and obvisouly some payload   */
typedef struct {
	uint32_t	id; // unique id
	uint8_t		type; // 80211, sensor, actuat
	uint64_t 	added_timestamp; // us
	uint64_t 	when_to_trigger; // us
	double		backoff; //csma - time to wait till next try. each packet will backoff n times, backoff is a random time = random_n() * SLOTTIME 
	int 		CW; //csma - current contention window
	uint8_t 	dst; // routing of this event
	uint8_t 	src; // routing of this event
	ssize_t 	length; // payload of event
	void 		*pkt_ptr; // payload of event
} event_t ;



 /* 
  * between UAV simulator and other processes 
  * we use sockets and send packets of type packet_t
  * */
typedef struct {
	int 	src ; //id
	int 	dst ; //id
	void 	*data ;  
	size_t 	lendata ;
} packet_t ;

/** used to save the limits of our stats_total **/
typedef struct {
	uint16_t init_ms ; 
	uint16_t end_ms ;
} slot_limits_t ;

/**  outmost header crossing the wireless network **/
typedef struct {
	double 			timestamp_ms ; // offsettime when packet entered the **NIC outbox** (when we call SEND() )
	slot_limits_t 	slot ; // 0--65000ms
	uint32_t 		bitmask_slots ; /* n-th bit == 1 ? then n-th node is active . max 32 nodes */
	uint8_t 		slot_id ; // 0--254 (one slot per host)
} tdma_header_t ;
   
/**********************
 *  Prototypes
 * *****************/
int 	eventScheduler_init(tinfo_t *tinfo, int ndrones) ; /* main calls this to start scheduler threads */
void 	event_dumpall(void); /* print *all* events on the screen */










#endif
