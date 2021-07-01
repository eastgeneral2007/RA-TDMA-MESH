/* Layer PM - packet manager */
#ifndef _PACKET_MANAGER_H
#define _PACKET_MANAGER_H

#include <inttypes.h>
#include "utils.h"
#include "tdma_types.h"


/* queue stuff */
//#define PM_Q_NUMELEMENTS		10500 /* size of the queue where wifi packets will be saved  (network buffer) */
//#define PM_Q_HEADER				24 /* size od the header of an element in the queue */





/* PM packet types */
// todo enum 
//#define STATUS_TYPE 1 /* UAVs use this to share their status with other uavs: position, and other state conditions , for network mgt  */
//#define PDR_TYPE 2 /* sharing pdr matrix , for network mgt  */
//#define MISSION 3 /* mission related packets */


#define FILENAME_ROUTE		"route.conf"
#define MAX_APP_PACKET_SIZE	1500
#ifndef MAX_IP
#define MAX_IP				25
#endif
#define PM_BLOCKING			1
#define PM_NONBLOCKING		0

#define STRING 	3 /* type - text */
typedef struct {
		uint8_t type;
		/*----*/
		char string[MAX_APP_PACKET_SIZE];
}  __attribute__ ((__packed__)) string_pkt_t ;


#ifdef PM_IMPORT

/* flags */
#define DUMP
#define PM_BUFFULL_DISCARD

/* size of the PM queues */
#define PM_TX_QUEUE_SIZE		50
#define PM_RX_QUEUE_SIZE		50


#define MAX_ROUTING_LINES	4
#define ROUTE_STRING_SIZE	15 
#define MAX_SEQ_NUM			999999 /* plus a prefix of the IP */

#define MAX_PM_PACKET_SIZE	(MAX_APP_PACKET_SIZE + sizeof(pm_header_t)) 

#define PREFIX				"PM"



#ifndef PM_LOADED_CONDITION
#define PM_LOADED_CONDITION \
	if (PM_initiated == 0) return 
#endif
#endif


/************ 
 * PUBLIC structs 
 * 
 * *********************/
typedef struct { /* This is sent through the network */
	uint8_t sink_IP;
	uint8_t dst_IP;
}  __attribute__ ((__packed__)) pair_ips_t ;

/* struct for PM_receive() */
typedef struct {
	void		*payload_ptr;
	uint16_t 	size ;
	uint8_t 	or_src ;
	uint8_t 	dst ;
} pm_pkt_t ;


typedef enum pm_types {
	APP 	= 1,
	STATUS 	= 2,
	ROUTING = 3,
	PM_LAST_TYPE = 260 /* force 2bytes */
} __attribute__ ((__packed__)) pm_type_t;

/** PM packet type - network packets leaving or entering the PM have this header **/
typedef struct {
	uint32_t 	seq_num ; /* each packet entering this layer from an UPPER layer has a diff seqnum */
	uint8_t 	or_src ; /* original src - flow beginning */
	uint8_t 	sink ; /* last destination - flow end */
	uint8_t 	try ; /* try counter */
	uint8_t 	total ; /* total number of tries expected */
	uint8_t 	need_ack ; /* flag TRUE or FALSE: this packet needs an ack */
	uint16_t 	type ; // 1- MGT or 2-APP
} __attribute__ ((__packed__)) pm_header_t ; /* 9+x bytes */


/*********************** *************
 * *****PRIVATE  *******structs*************** 
 * ***********************************/
#ifdef PM_IMPORT
 
/** used to save PM-module statistics **/
typedef struct  {
	double 		bytes_sent;
	double 		clock_seconds;
	uint64_t 	cou;
} pm_stats_t ;





typedef struct {
	pm_header_t header ;
	char data[MAX_APP_PACKET_SIZE] ;
}  __attribute__ ((__packed__)) pm_gen_pkt ;


		
///** PM packet type - network packets leaving or entering the PM have this header **/
//typedef struct {
	//pm_header_t header ;
	//char rest[1400] ;
//} pm_packet_t ;

/**  bookkeeping an OUTgoing Pm packet **/
typedef struct {
	double		timestamp ; /* epoch time when the packet entered the queue */
	uint8_t		prev_ip ; /* save where this pckt come from (prev hop) */
	uint8_t		next_ip ; /* save where this pckt will go next (next hop) */
	uint16_t	pkt_len ; /*  (header+payload) 0--65000 bytes */
	void		*pkt_ptr ; /* location of PM-packet (header+payload) */
} pm_tx_queue_item_t ;

/**  bookkeeping an incoming Pm packet **/
typedef struct {
	//double timestamp ; /* epoch time when the packet entered the queue */
	uint16_t	pkt_len ; /*  (header+payload) 0--65000 bytes */
	void* 		pkt_ptr ; /* location of PM-packet (header+payload) */
} pm_rx_queue_item_t ;

typedef struct {
	//char 		sink_string[MAX_ROUTING_LINES][ROUTE_STRING_SIZE] ;
	//char 		dest_string[MAX_ROUTING_LINES][ROUTE_STRING_SIZE] ;
	uint8_t		sink_IP[MAX_ROUTING_LINES] ;
	uint8_t		dest_IP[MAX_ROUTING_LINES] ;
	uint8_t		node_ip ;
} Param_pm_t ;

#endif /* PM_IMPORT - structs */





/** print out received string from other node **/
#if defined(VERBOSE) || defined(LOGDATA)
error_t parserString( pm_pkt_t pkt ) ;
#endif
void sendString( char *instring, uint8_t dst );


/*******************************
 * *Public* Function Prototypes 
 * ****************************/
 /** 
 * @brief  init PM module 
 * 
 * Displays the packets received from the mouse
 * Exits after receiving the number of packets specified in argument
 * 
 * @param my_ip Choose what ip we want to use. 0 for default
 * 
 * @return Return E_SUCCESS upon success and negative error_t code otherwise
 */
error_t		drk_PM_init( uint8_t my_ip ) ; 

/** close the module properly **/
error_t		drk_PM_close( void ) ;

/** send a packet to PM **/
error_t		PM_send( uint8_t sink_IP, uint16_t pkt_len , 
	const void* const pkt_ptr, int hi_prio ) ; 

/** print current routing table **/
error_t 	PM_printRoutingSettings( void ) ; 

/** for upper layers to read a packet in our Rx queue. or_src is the IP of the original source **/
error_t		PM_receive( pm_pkt_t *pkt, uint8_t blocking ) ; 

/** **/
float		PM_getTxBufferUse( void ) ;

/** **/
float		PM_getRxBufferUse( void ) ;

/** **/
error_t		PM_registerFlow( uint8_t or_src , uint8_t sink );

/** **/
error_t		PM_newTopology( pair_ips_t const topology[] , 
	int entries ) ;

/** get next hop, given the local ip and the sink ip **/
uint8_t 	PM_getNextHop( uint8_t sink_ip ) ;


/************************************************************* *
 * ************ (Private) PROTOTYPES **************
 * *******************************************/
#ifdef PM_IMPORT
 
//#pragma message("importing <PM> private functions")
 /** route stuff **/
static error_t 	loadRouteSettings( void ) ;
static int 		isThisMe( uint8_t ip ) ; /* check if this ip refers to me .*/
static uint32_t getSeqNum( uint8_t sink_IP ) ;

static error_t 	initThreads( void ); /* init rx and tx threads */
static error_t	loadSettings( const uint8_t my_ip ) ; /* load routing stuff */
static void 	*rxThread();
static void 	*txThread();

/* Queue/Buffer related stuff */
static error_t 	initQueue( void ) ; /* erase queues */
static error_t	deleteTxElement( uint16_t r_ptr ) ;
static error_t 	deleteRxElement( void ) ;
static uint16_t getRxBufferUse();/* number of packets to be read */
static uint16_t getTxBufferUse();/* number of packets to be sent */
/* move a write pointer to next available index in the queue.
 * return -1 in case queue is full 
 * return 1 - ok */
static error_t 	moveWPtr( 
	volatile uint16_t* const 	w_ptr, 
	volatile uint16_t* const 	r_ptr , 
	uint16_t 					queue_size , 
	uint8_t 					overwrite ) ;

static error_t	getOldestPMPkt( 
	void *payload_p,
	uint16_t * const len, 
	uint8_t * const prev_hop, 
	uint8_t * const next_hop, 
	double * const timestamp ) ;

/* call TDMA_send periodically until packet gets through */
static void 	persistentSend( uint8_t dest_ip , 
	uint16_t pkt_len , const void * const pkt_ptr, int hi_prio );

static void semaphoreDebugger( int ret ) ;


#ifdef LOGDATA
static void 	logPacket( 
	FILE *logfile , 
	pm_header_t header , 
	double timestamp , 
	uint8_t prev_hop , uint8_t next_hop ,
	uint16_t pm_pkt_len , 
	uint16_t buf_use , 
	bandwidth_t bandwidth ) ; /* tdma_types.h */
#endif

#ifdef DUMP
static void 	dumpTxQueue( void ) ;
static void 	dumpRxQueue( void ) ;
#endif

#ifdef VERBOSE
static void	 	dumpTxQueueItem( pm_tx_queue_item_t item ) ;
#endif


#endif /* PM_IMPORT */











#endif /* _PACKET_MANAGER_H*/
