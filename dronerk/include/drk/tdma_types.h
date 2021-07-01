#ifndef _TDMA_TYPES_H_
#define _TDMA_TYPES_H_
/******** 
 * tdma types 
 * ****************
 * Created by : Luis Pinto
 * date: 9 feb 2018 11:32 GMT
 */

#include <inttypes.h>
#include <pthread.h>

/*******************************************************************************
 * Constants
 ******************************************************************************/
 #define VSP 				0 //use or do not use VSP Variable Slot-width Protocol
 
 
 /* print tx'ed data every <PRINT_KB>kB sent */
#define PRINT_KB			1000 

#define PERIOD_SHARE_SLOT 	1 /* seconds */

#define MAX_PACKET_SIZE		1500 /* ultimate maximum for sendto() packet */

#ifndef MAX_IP
	#define MAX_IP 			255
#endif

#define MAX_N_SLOTS			5




#define OUT_BUFF_THRESHOLD	100 /* < 1 pkt +-*/
#define GUARD_INTERVAL_MS	2 /* used by tdma_getOwnSlot_guarded() */

/* size of the tdma queues */
#define TDMA_TX_QUEUE_SIZE	100 // 50
#define TDMA_RX_QUEUE_SIZE	100 // 50

#define PKTDELAY_ARRAY_SIZE	1000


/** network constants **/
#define RX_LOCAL_PORT		60000 /* udp packets are expected to be received at this port*/
#define TX_LOCAL_PORT		40000 /* udp packets are sent FROM this port*/
//#define xPORT_SIM_80211		50000 /* simulator is listening to this port to receive (802.11) pkts */

/** configuration filesnames **/
#define FILENAME_NET 	"net.conf"
#define FILENAME_TDMA 	"tdma.conf"

/** queue related **/
/* compile tweeks into the system */
#define TDMA_BUFFULL_DISCARD
#define ALWAYS_ACCEPT 1 /* 1 - accept incom. packets even if in-buffer is full. override */



/*******************************************************************************
 * Structs and typedefs
 ******************************************************************************/
/* Constants declarations here. */
#if 0
#if TX_QUEUE_SIZE < 256
typedef uint8_t queue_idx_t ;
#elif TX_QUEUE_SIZE < 256*256
typedef uint16_t queue_idx_t ;
#elif TX_QUEUE_SIZE < 256*256*256*256
typedef uint32_t queue_idx_t ;
#else
typedef uint64_t queue_idx_t ;
#endif /* TX_QUEUE_SIZE < 256 */
#endif 
 
/* used to save TDMA-module statistics */
typedef struct  {
	/* */
	double 		total_bytes_sent; /* number of bytes sent since reference (module init) */
	double 		total_clock_seconds; /* elapsed time since reference ( module init) */
	uint64_t 	total_pktsent ; /* number of packets sent since reference ( module init) */


	double 		slot_clock_seconds; /* elapsed time since reference (slot begin ) */
	uint64_t 	slot_round_counter ; /* number of packets sent since reference (slot begin ) */
	double 		slot_bytes_sent[MAX_IP+1] ; /* number of bytes sent to each IP since reference (slot begin )*/
	//double 		slot_bytes_sent_any; /* <-check index 0 of bytes_sent */
	
	double 		round_bytes_rcvd[MAX_IP+1]; /* number of bytes rcvd from each IP in the last round (count ends at BEGIN of my TX slot) */

	uint16_t	last_rx_pkt_pos_ms[MAX_IP+1];
	uint16_t 	last_tx_pkt_ms ;

	/* set once ! - time references */
	double 		epoch_init_slot ; /* ref time for begin of slot */
	double 		time_beginning ; /* ref time for begin of module */
} tdma_stats_t ;



/* tdma packet types */
typedef enum tdma_types {
	PURE	= 4U, /* simple TDMA packet */
	PM 		= 1U,
	PDR 	= 2U,
	REQUEST = 3U,
	//etc
} __attribute__ ((__packed__))  tdma_type_t; /* 2B?*/


/** used to save the boundaries of our SLOT [tB,tE] **/
typedef struct {
	uint16_t begin_ms ; 
	uint16_t end_ms ;
}  __attribute__ ((__packed__)) slot_limits_t ;

/**  outmost header crossing the wireless network **/
/* example:
 * [038][000] - timestamp
 * [199] - partms
 * [002] - slotid
 * [000][000]
 * [150][000]
 * [000][000]
 * [003][000][000][000]
 * [001][000]
 * --payload: [072][101][108][108][111][032][102][114][111][109][032][035][050][032][116][111][032][035][049][058][032][050][048]
 */
typedef struct {
	uint16_t 		timestamp_ms ; // round period offset time when packet entered the **NIC outbox** ( when we call sendto() )
	uint8_t 		timestamp_partms ; // # of parts in 255 - fraction of timestamp 
	uint8_t 		slot_id ; /* 0--254 (one slot per host) */
	slot_limits_t 	slot ; /* [init end] ms */
	#if VSP
	uint16_t 		request_slot_width_ms ; /* transmitters use this to request new receiver's slotwidth  */
	#endif
	uint32_t		seq_num ;
	uint32_t		seq_num_gen ;
	uint16_t		type ; /* Hi byte*/
}  __attribute__ ((__packed__)) tdma_header_t ; /* 14 + x bytes */

/* all packets on the air are UDP tdma_packets (tdma header + somepayload) */
typedef struct {
	tdma_header_t 	header ;
	char 			payload[MAX_PACKET_SIZE] ;
}  __attribute__ ((__packed__)) tdma_gen_packet_t ;

/**  bookkeeping outgoing packets. TDMA headers arebuilt at send time  **/
typedef struct {
	double 		timestamp ; /* epoch time (s) when the packet entered the queue */
	uint8_t 	dest_ip ; /* where this will go next (0--254) */
	uint16_t 	type ;
	uint16_t 	payloadpkt_len ; /*  packet's payload length 0--65000 bytes */
	void 		*payloadpkt_ptr ; /* location of packet's payload */
} tdma_tx_queue_item_t ;

/** book-keeping incomming packets. TDMA headers are not saved  **/
typedef struct {
	//double timestamp ; /* epoch time when the packet entered the queue */
	uint8_t 	src_ip ; /* where this came from: prev_hop IP (0--254)  */
	uint16_t 	payloadpkt_len ; /* packet's payload length : 0--65000 bytes */
	void 		*payloadpkt_ptr ; /* location of packet's payload */
} tdma_rx_queue_item_t ;

typedef struct {
	volatile uint16_t 	w ;
	volatile uint16_t 	r ;
	pthread_mutex_t		mutex ;
} queue_ptr_t ;




typedef struct {
	float 		phy_datarate_MBps ; /* Mbps */
	float 		throughput_target_MBps ; /* Mbps */
	uint16_t 	round_period_ms ; /* TDMA round period, in ms . range: [0,65000]ms */
	uint16_t 	standard_width_ms ; /* roundperiod / slot_total */
	uint8_t 	slot_total ; /* number of slots in the network - max 254 */
	uint8_t 	slot_id ; /* my slot id \in [1 , ..., slot_total ] */
	uint8_t 	sync_flag ; /* true or false (1 or 0), syncronize my slots based on the other nodes */
	uint8_t 	node_ip ; /* save my ip => max 254 devices */
	#if VSP
	uint8_t 	vsp ; 
	#endif
} param_tdma_t ;


/* bandwidth table */
typedef struct {
	float in[MAX_IP+1] ; 
	float out[MAX_IP+1] ;
} bandwidth_t ;

#endif /* TDMA_TYPES_H */
