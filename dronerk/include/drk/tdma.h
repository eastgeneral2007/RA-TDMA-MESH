/*! \file tdma.h
    \brief A Documented file.
    
    TDMA syncronizer stuff
    * Created by : Luis Pinto
 * date: 25 aug 2015 15:20 GMT
*/

/********************
TDMA syncronizer stuff
*******************/
/**

 */
#ifndef _TDMA_H_
#define _TDMA_H_


#define TDMA_DEBUG 1

#include <inttypes.h> /* int16,etc */
#include "tdma_types.h" /*  */
#include "utils.h" /* error_t */
/// todo : clients will have access to all inner workings



#ifndef TDMA_IMPORT
	/* externs for all */
	extern volatile uint8_t TExit_threads ; /* flag when it is time to close the module */
	extern const uint8_t 	DRONE_LIST[4] ;
	
	
#endif

#if defined(TDMA_IMPORT) || \
defined(TDMA_SLOT_IMPORT) || \
defined(TDMA_UTILS_IMPORT)

#ifndef TDMA_LOADED_CONDITION
#define TDMA_LOADED_CONDITION \
	if (Tdma_initiated == 0) return 
#endif

#endif


/* PUBLIC Function prototypes here. */
/***********************************************************************

 **********************************************************************/
/** init module: threads, slots, etc **/
error_t 	drk_TDMA_init( uint8_t my_id );

/** close and cleanup module **/
error_t 	drk_TDMA_close( void ) ;

void drk_TDMA_setSpanningTree(int **tree, int numDrones, int flag);

/** 
 * @brief  Get newly rcvd packets. 
 * 
 * <long explanation>
 * blocking == true
 * 
 * @param pkt_ptr pointer to packet payload
 * @param other_ip : gets source of the packet, 
 * @return size of packet received
 */ 
ssize_t	TDMA_receive( void *pkt_ptr , uint8_t *other_ip ) ; 
	

/** 
 * @brief send a packet of any chosen type  
 * 
 * @param dest_ip 
 * @param pkt_len
 * @param type
 * @param pkt_ptr 
 * @return error code 
 */
error_t TDMA_sendAnyPacket( 
	uint8_t dest_ip, 
	uint16_t type ,  /* :) */
	uint16_t pkt_len , 
	const void * const pkt_ptr );


/** 
 * @brief send packet of type PM
 * 
 * it adds packet to the tx queue to be sent by TDMA module 
 * 
 * @param dest_ip 
 * @param pkt_len
 * @param pkt_ptr 
 * @return error code 
 */
error_t	TDMA_send(
	uint8_t dest_ip, 
	uint16_t pkt_len , 
	const void * const pkt_ptr, int hi_prio ) ; 
	

/** 
 * @brief change between TDMA modes: CSMA/ rigidTDMA slots / dynamicTDMA slots (VSDP) 
 */
void 	TDMA_off(void); /* aka CSMA */
void 	TDMA_rigid(void); /* */
void 	TDMA_dynamic(void); /* */
void 	TDMA_nosync(void);



/** check if TDMA buffer is full. TDMA_send fails if full**/
int 	TDMA_isRxBufferFull( void ) ; 

/** **/
void	TDMA_dumpTxQueue( void );


/************
 *  FUNCTIONS TO BE USED ONLY BY TDMA* MODULES 
 * semi-private
 * ***********/
#if defined(TDMA_IMPORT) ||\
 defined(TDMA_SLOT_IMPORT) ||\
 defined(TDMA_UTILS_IMPORT)
uint16_t 	tdma_getRxBufferUse( void ) ; /* number of packets to be read */
uint16_t 	tdma_getTxBufferUse( void ) ; /* number of packets to be sent in Q */
#endif /* impoted some of tdma* modules */
/*************************************************************************************** 
 * Prototypes - Internal Local Functions - to be used only by this unit
 *************************************************************************************/
/** only for TDMA.c 
 * PRIVATE **/
#ifdef TDMA_IMPORT

/** thread inits etc **/
static error_t 		initThreads( void ); /* init rx and tx threads */
static error_t 		initQueue( void ); /* erase queues */

/*********************************
 * Regarding Incoming pkts
 * ******************************/
static error_t 		parserPmPkt( void *rx_tdmapkt_ptr, uint16_t num_bytes_read, uint8_t other_IP );
static error_t 		parserPurePkt( void *rx_tdmapkt_ptr, uint16_t num_bytes_read, uint8_t other_IP );
error_t 			rx_queue_waitForFreeSpots(void);
static void 		*rxThread() ; /* thread that receive packet from the TX_queue */


/*********************************
 * Regarding TRANSMISSIONS and SLOT
 * ******************************/
static void 		*txThread() ; /* thread that sends packets from the TX_queue */
static error_t 		tx_thread_main_handler(void);
static int 			build_send_log(tdma_tx_queue_item_t * const item_packet);
static int 			buildTDMAPacket(
	tdma_tx_queue_item_t * const item_packet, 
	tdma_gen_packet_t * const tdma_pkt_output) ;
static error_t 		actuallySend( 
	const tdma_gen_packet_t * const tdma_packet_ptr , /* packet built, with tdma header and payload  */
	size_t tdma_packet_len, /* total size  */
	uint8_t dest_IP , /* where to send this  */
	int g_tx_sock_fd /* socket fd to use */
	);	

	
/* actions to do after finishing tx slot: */
static error_t 		end_of_slot_operations(void);

/* actions to do at beginning my txslot: */
static error_t 		begin_of_slot_operations(void);

//static int 			inject( uint8_t dest_ip, uint16_t pkt_len , void *pkt_ptr ) ;

/* get the oldest (FIFO) TDMA-packet from the queue: */
static error_t 		getOldestTDMAPkt( tdma_tx_queue_item_t *packet )  ; 

/* rgenerate and return the new seq number for the flow [MYSELF]->[SINK NODE]: */
static uint32_t 	getTDMASeqNum( uint8_t dest_IP );
/* generate and return the new seq number for the flow [MYSELF]->[wherever]*/
static uint32_t 	getTDMASeqNumG(void);

void sendBeacons(void);


#endif /* TDMA private stuff */





#endif /* _TDMA_H */

