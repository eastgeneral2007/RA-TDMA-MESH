/*******************************************
 * ***************TDMA MODULE *************
*******************************************/
#define TDMA_IMPORT /* this unhides private functions, from my self-header */
#include "drk/tdma.h" /* self header - public functions go here */

/* sibling tdma module components */
#include "drk/tdma_utils.h" /* utils related with TDMA */
#include "drk/tdma_slot.h" /* related with slot management */
#include "drk/tdma_types.h" /* all types and defines tdma */

/* other modules headers */
#include "drk/drk.h" /* drk_exit */
#include "drk/utils.h" /* functions & data types used across DRK */
#ifdef SIMULATOR
#include "drk/sim_clock.h"
#include "drk/world_simulator_api.h"
#endif
#include "drk/pdr.h"
#include "drk/packet_manager.h" /* PM header for hi-prio pkt hijack */

/* system headers */
#include <arpa/inet.h> // udp packets
#include <errno.h> // strerror
#include <inttypes.h> /* int8_t etc */
#include <ifaddrs.h> // udp packets
#include <linux/sockios.h> // SIOCOUTQ
#include <math.h> /* some math at some point */
#include <netinet/in.h> // udp packets
#include <pthread.h> // we have threads
#include <stdio.h> // printf
#include <stdlib.h> /* atoi*/
#include <string.h> // memset, memcpy
#include <sys/time.h>
#include <sys/ioctl.h> // ioctls
#include <sys/wait.h> // waitpid
#include <unistd.h> // sleep

/* Drones always receive tdma-packets at this port no matter what.
 * in simulator mode:
 * drones should send to SIM_RX port, SIMULATOR then sends packates to RX80211
 * in real mode:
 *  drones send data to other drones directly to this port */
#define PORT_UAV_RX80211		60100U


/*******************************************************************************
 * Defines
 ******************************************************************************/
/* module name and macro to check if module has been initiated */
#define PREFIX				"TDMA"


#define BS_IP		22 /* where to send user-informative strings */



/**********************************************************************************************
 * evil externable Globals
 **************************************************************************************************/
/* flag when it is time to close the TDMA module: */
volatile int	 			TExit_threads = 0; 

#define NIPS			10,11,12,13,14,15,16,17,18,19,20,21,22,23,255 
/* ip list: */
const uint8_t 				DRONE_LIST[] = {NIPS} ; 
#undef NIPS

/**********************************************************************************************
 * Evil STATIC Globals - "externs should not pass!"
 **************************************************************************************************/
#ifdef LOGDATA
static FILE 				*tdma_log_tx = NULL;
#endif

/* flags if this module has been load  */
volatile int		Tdma_initiated = 0; 

/* thread ids : tx and rx */
static pthread_t 			Thread_tids[2]; 

/* socket for tx */
static int 					g_tx_sock_fd;

/* mutex to control when to enter transmission phase */
static sem_t				sem_inside_slot  ; 


/** tx queue stuff  @todo use struct */
static volatile tdma_tx_queue_item_t 	TDMA_tx_queue[TDMA_TX_QUEUE_SIZE] ; /* the queue */ 
static pthread_mutex_t 					TDMA_tx_queue_mutex = PTHREAD_MUTEX_INITIALIZER ; /* mutex to read/write to the queue , multiple threads need access to it */
static volatile uint16_t 				TDMA_tx_queue_w_ptr, TDMA_tx_queue_r_ptr ; /* pointers to r+w */ 
static sem_t							TDMA_Transmit_sem ; /* to communicate to txthread there's a packet to transmit */
static sem_t							tx_queue_freespots_sem ; /* to communicate to TDMA_send ther's space to add a new packet */
/** rx queue stuff **/
volatile tdma_rx_queue_item_t 			TDMA_rx_queue[TDMA_RX_QUEUE_SIZE] ; /* the queue */ 
static queue_ptr_t 						TDMA_rx_queue_ptr ;
static sem_t 							Receive_sem ; /* to sleep TDMA_receive caller. inform there's packets to read */
static sem_t 							rx_queue_freespots_sem ;

uint8_t **spanning_tree;

void sugusr_hdlr()
{
	/* do nothing */
	//puts("k\n");
}


/***********************************************************************
 * INTERFACE TO UPPER LAYERS : TDMA_xxxx()
 * @todo add "drk" prefix to all 
 **********************************************************************/

void drk_TDMA_setSpanningTree(uint8_t **tree, int numDrones, int flag){

	
	printf("Func TDMA_setSpanningTree Started\n");
	int a,b;

	if(flag == 0){
		spanning_tree = (uint8_t **) malloc(numDrones * sizeof(uint8_t *));

		for(int row = 0; row<numDrones; row++){
			spanning_tree[row] = (uint8_t *) malloc(numDrones * sizeof(uint8_t));
		}
	}

	for(a=0; a<numDrones; a++){
		for(b=0; b<numDrones; b++) spanning_tree[a][b] = tree[a][b];
	}
	
	printf("Func TDMA_setSpanningTree done\n\n");
}

 
/** INIT MODULE **/
error_t drk_TDMA_init( uint8_t my_id )
{
	//block_all_signals() ;
	unblock_one_signal(SIGUSR1);
	sig_set_action(sugusr_hdlr, SIGUSR1);
	
	
	int i = 1 ;
	if ( 0 > tdma_loadSettings( my_id ) ) 
	{
		PRINTF_FL_ERR("Failed to load settings\n" ) ;
		return E_OTHER	;
	}
	i++;
	if ( 0 > initQueue() ) /* init tx queue */
	{
		PRINTF_FL_ERR("failed to init tx queue \n" ) ;
		return E_OTHER ;
	}
	PRINTF_FL_WARN("init threads\n");
	i++;
	if ( 0 > initThreads() ) 
	{
		PRINTF_FL_ERR( "failed to init threads\n" ) ;
		return E_PTHREAD;
	}
	/* make this a robut mutex */
	if (0 != sem_init( &sem_inside_slot, 0, 0 )) /* starts at zero - locked . SigHnd increments 1 */
	{ 
		PRINTF_FL_ERR("Sem init faild (%s)\n", strerror(errno));
		return E_SEMAPHORE ;
	}
	usleep(10000); /* let threads start their while cycles */
	
	Tdma_initiated = 1 ;
	return E_SUCCESS ; /* all good */
}


/** turn off tdma:
 * slots are now 100% of T; 
 * no synch ; no DVSP ;
 * headers are still in place 
 * keyboard 7 */
void TDMA_off( void )
{
	char strin[50];
	snprintf( strin , sizeof(strin), "CSMA/CA");
	PRINTF_FL_WARN("%s\n", strin );
	//sendString( strin, BS_IP);
	//Param_tdma.slot_id = 0 ;
	Param_tdma.standard_width_ms = Param_tdma.round_period_ms ;
	Param_tdma.sync_flag = 0 ;
	#if VSP
	Param_tdma.vsp = 0 ; /* no dyn-slots */
	#endif 
	tdma_clearStuff(); /* Tx slot is the whole thing */
}

/** make all nodes have equal sized slot ;
 * synch - ok 
 * DVSP - off 
 * keyboard 8 **/
void TDMA_rigid( void )
{
	char strin[50];
	snprintf( strin , sizeof(strin), 
		"TDMA rigidslots, MAX_NUAVSwith adapt-synch");
	PRINTF_FL_WARN("%s\n", strin );
	//sendString(strin, BS_IP);
	Param_tdma.sync_flag = 1 ; /* when syncronizing, slot becomes default size T/n */
	#if VSP
	Param_tdma.vsp = 0 ; /* no dyn-slots */
	#endif
	tdma_clearStuff(); /* Tx slot is the whole thing */
}


/** turn DVSP on **/
void TDMA_dynamic(void)
{
	#if VSP
	"VSP is  active ! remove this line if you want to really compile it"
	char strin[50];
	snprintf( strin , sizeof(strin), 
		"TDMA DVSP, with adapt-synch");
	PRINTF_FL_WARN("%s\n", strin );
	//sendString(strin, BS_IP);
	Param_tdma.sync_flag = 1 ;
	Param_tdma.vsp = 1; /* dyn-slots */
	tdma_clearStuff(); /* Tx slot is the whole thing */
	#endif
}

/* keyboard 9 */
void TDMA_nosync(void)
{
	char strin[50];
	snprintf(strin, sizeof(strin), 
		"TDMA NO-adapt-synch, no dvsp");
	PRINTF_FL_WARN("%s\n", strin );
	//sendString(strin, BS_IP);
	Param_tdma.sync_flag = 0 ; /* avoid sync */
	#if VSP
	Param_tdma.vsp = 0 ; /* no dyn-slots */
	#endif
	tdma_slot_setstd();
}


/** close this module **/
error_t drk_TDMA_close( void )
{
	TDMA_LOADED_CONDITION E_NO_INIT; 
	
	TExit_threads = 1 ; /* trigger closure */
	Tdma_initiated = 0 ;
	
	PRINTF_FL("[TDMA] sending signals \n");
	while (0==pthread_kill( Thread_tids[0] , SIGUSR1 ) ){ microsleep(1000);}
	while (0==pthread_kill( Thread_tids[1] , SIGUSR1 ) ){ microsleep(1000);}
	//PRINTF_FL("[TDMA] joining \n");
	pthread_join( Thread_tids[0] , NULL ) ; 
	pthread_join( Thread_tids[1] , NULL ) ; 
	
	PRINTF_FL("[TDMA] Closed\n");
	
	//PRINTF_FL("[TDMA] Closed\n");
	
	return E_SUCCESS;
}







/** Give a pure pkt to the TDMA module 
 * it is added to the tx queue to be sent later , 
 * and is received at the PM layer on the other side */
error_t TDMA_send( 
	uint8_t dest_ip, 
	uint16_t pkt_len, 
	const void * const pkt_ptr,
	int hi_prio )
{
	//PRINTF_FL_WARN(
		//"sending %s pkt\n" , 
		//hi_prio?"hiprio":" normal" );
	return TDMA_sendAnyPacket( dest_ip, (hi_prio<<8) | PURE, pkt_len, pkt_ptr ) ;
}



/** send a TDMA packet of any <type> **/
error_t TDMA_sendAnyPacket( 
	uint8_t dest_ip, 
	uint16_t type ,  
	uint16_t pkt_len , 
	const void * const pkt_ptr )
{
	TDMA_LOADED_CONDITION	E_NO_INIT; 
	
	//if ( dest_ip == 0 ) /* default destination is the upstream node */
		//dest_ip = TDMA_upStream() ;

	
	/* wait here till tx_queue_freespots_sem has space to add a packet (pos. value)*/	
	/* sleep thread until free spot */
	int ret ;
	ret = sem_wait_safe(
		&tx_queue_freespots_sem, 
		"tx_queue_freespots_sem",
		&TExit_threads); 
	if (0 > ret) 
		return ret ;

	
	
	
		
	/* error handling */
	if ( 
	(0 == 1) || //( dest_ip == (uint8_t)getMyIP() ) || 
	(0 == dest_ip) )
	{
		PRINTF_FL_ERR( "Can't send pkts to myself or #0\n ") ;
		return E_INVALID_INPUT ;
	}
	
	
	
	/* ready to write in the queue . lock */
	pthread_mutex_lock( &TDMA_tx_queue_mutex ) ;
	
	/* can we update the write-pointer? */
	if ( (TDMA_tx_queue_w_ptr+1) % TDMA_TX_QUEUE_SIZE == TDMA_tx_queue_r_ptr) 
	{
		/* Nope ! */
		int val ; 
		sem_getvalue( &tx_queue_freespots_sem , &val ); 
		PRINTF_FL(
			"Queue is full. (w_ptr=%d | r_ptr=%d) . "
			"Semaphore %d\n" ,
			TDMA_tx_queue_w_ptr, TDMA_tx_queue_r_ptr ,
			val ) ;
		TDMA_dumpTxQueue();
		#ifndef X86
		drk_exit();
		#endif
		pthread_mutex_unlock(&TDMA_tx_queue_mutex) ;
		return E_FULL_QUEUE ; /* full queue */
	}
	
	/* buld a new item */
	tdma_tx_queue_item_t new_item ; 
	new_item.timestamp 		= getEpoch() ;
	new_item.dest_ip 		= dest_ip ;
	new_item.type 			= type ;
	new_item.payloadpkt_len = pkt_len ;
	new_item.payloadpkt_ptr = malloc( (size_t)pkt_len ) ;
	if (NULL == new_item.payloadpkt_ptr)
	{
		pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ;
		return E_NO_MEM; /* failed to add pkt to queue. no memory available */
	}
	memcpy( new_item.payloadpkt_ptr, pkt_ptr, (size_t)pkt_len ) ;
	
	/* add it to the queue */
	TDMA_tx_queue[TDMA_tx_queue_w_ptr] = new_item ; /* add new pkt at the current Write ptr */
	TDMA_tx_queue_w_ptr = (TDMA_tx_queue_w_ptr+1) % TDMA_TX_QUEUE_SIZE ; /* update Write ptr */
	
	// printf("TDMA:send@ TDMA_tx_queue_w_ptr=%d (TDMA_tx_queue_r_ptr=%d)\n" , TDMA_tx_queue_w_ptr, TDMA_tx_queue_r_ptr ) ; fflush( stdout ) ;
	pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ;
	
	/* inform getOldestTDMAPkt that there is a new packet to transmit*/
	sem_post(&TDMA_Transmit_sem); 
	
	int val=-1;
	sem_getvalue(&TDMA_Transmit_sem, &val);
	//PRINTF_FL_WARN("External pkt added [TDMA_Transmit_sem: %d]\n",val);
	return E_SUCCESS ; /* ok to return */
}




/********************************
* Retrieve oldest recvd packet - "peeled" version - remove tdma header
* return:	-1		 : error
* 			0 		 : no packets
* 		    otherwise: pkt_len
* ssize_t is SIGNED.
*****************************************************/
ssize_t TDMA_receive( void *pkt_ptr, uint8_t *other_ip ) 
{
	
	TDMA_LOADED_CONDITION E_NO_INIT; 
	
	/* number of packets to be read is in Receive_sem */
	//PRINTF_FL_WARN("waiting for pkts\n");
	int ret ;
	ret = sem_wait_safe(&Receive_sem, "TDMA Receive_sem", &TExit_threads); 
	if (0 > ret) 
		return ret ;
		
	pthread_mutex_lock( &(TDMA_rx_queue_ptr.mutex) );
	if ( TDMA_rx_queue_ptr.w == TDMA_rx_queue_ptr.r ) 
	{	/* queue is empty?! */
		pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex )) ;
		PRINTF_FL_ERR("ERROR semaphore is broken\n");
		return E_SEMAPHORE ;
	}

	/* grab oldest packet from queue : */
	//PRINTF_FL("PM is reading me R%" PRIu16 "\n", TDMA_rx_queue_ptr.r ) ;
	uint16_t pkt_len = TDMA_rx_queue[TDMA_rx_queue_ptr.r].payloadpkt_len ;
	memcpy( pkt_ptr , TDMA_rx_queue[TDMA_rx_queue_ptr.r].payloadpkt_ptr ,
		(size_t)pkt_len ) ;
	free( TDMA_rx_queue[TDMA_rx_queue_ptr.r].payloadpkt_ptr ) ; /* delete item from txqueue */
	*other_ip = TDMA_rx_queue[TDMA_rx_queue_ptr.r].src_ip ; 
	memset( (void*)&TDMA_rx_queue[TDMA_rx_queue_ptr.r] , 0 , 
		sizeof( tdma_rx_queue_item_t ) ) ; /* zero item from rx queue */
	TDMA_rx_queue_ptr.r = ( TDMA_rx_queue_ptr.r + 1 ) % TDMA_RX_QUEUE_SIZE ; /* update reader pointer */
	pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex) ) ;
	
	sem_post( &rx_queue_freespots_sem ); /* post another empty on the queue */
	return pkt_len ; /* leave */
		
		
		/* if we are here, it means caller wants a blocking call - we only leave when a new packet is present  */
		//RINTF_FL("Waiting for packets...\n");
		
		//microsleep(10000);
		
	
	

}

/*  mutexed */
int TDMA_isRxBufferFull(void)
{
	//PRINTF_FL("waiting....\n");
	pthread_mutex_lock( &(TDMA_rx_queue_ptr.mutex) ) ;
	if ( ( (TDMA_rx_queue_ptr.w+1) % TDMA_RX_QUEUE_SIZE) == TDMA_rx_queue_ptr.r) /* if next writting point is the cur reading point */
	{
		//PRINTF_FL_ERR("FULL rx: W%" PRIu16 ", R%" PRIu16 "....\n",
			//TDMA_rx_queue_ptr.w , TDMA_rx_queue_ptr.r );
		//PRINTF_FL("unlocked....\n");
		pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex) ) ;
		return 1 ; /* true - im full */
	}


	pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex)  ) ;
	//PRINTF_FL("unlocked....\n");
	return 0 ; /* false - im not full */
}

/* not protected - dump anytime */
void TDMA_dumpTxQueue( void )
{
	for ( uint16_t i = 0; i < TDMA_TX_QUEUE_SIZE; i++ )
		PRINTF_FL(
			"Tx_queue[%" PRIu16 "] = "
			"{ ptr %p , dst %"PRIu8" , len %"PRIu16"B }\n" ,
			i,
			TDMA_tx_queue[i].payloadpkt_ptr , 
			TDMA_tx_queue[i].dest_ip ,
			TDMA_tx_queue[i].payloadpkt_len ) ;
}






/*******************************************************************
 * Static functions ( local usage only ) 
 * ******************************************************************/


/** init rx and tx threads - TDMA layer init **/
error_t initThreads( void )
{
	/* create thread attributes : stack size */
	size_t desired_stack_size = 400000 ; // thread stack size 400KB
	pthread_attr_t thread_attr;
	pthread_attr_init( &thread_attr );
	pthread_attr_setstacksize( &thread_attr, desired_stack_size ) ;

	/* Create thread that will continuously send messages from other nodes */
	#if 1
	if ( pthread_create( &Thread_tids[0] , &thread_attr, &txThread , NULL) != 0 )
	{
		PRINTF_FL_ERR("[Tx Thread] - Error spawning (%s)\n", 
			strerror( errno ) );
		exit( EXIT_FAILURE );
		TExit_threads = 1 ;
		return -1 ;
	}
	PRINTF_FL("[Tx Thread] created with success\n");
    #endif 
    
    /* Create thread that will continuously receive messages from other nodes */
	if ( pthread_create( &Thread_tids[1] , &thread_attr, &rxThread , NULL) != 0 )
	{
		PRINTF_FL_ERR("[RX Thread] - Error spawning (%s)\n" , 
			strerror( errno ));
		TExit_threads = 1 ;
		return -2 ;

	}
	PRINTF_FL("[RX Thread] created with success\n");
	
	if ( 0 != pthread_attr_destroy( &thread_attr ) )
	{
		TExit_threads = 1 ;
		return -3;
	}

	/* wait for the threads to enter their cycles */
	microsleep(200000) ;
	
	/* if threads are already dead, lets join and return -1 */
	if ( TExit_threads ) 
	{
		PRINTF_FL_ERR("Joinning...\n");
		/* wait here until all app threads terminate! */
		for ( int j = 0 ; j <= 1 ; j++ )
			pthread_join( Thread_tids[j] , NULL ) ;
		return -1 ;
	}
	return E_SUCCESS ;
}


/** init the queue with zeros 
 * and respective semaphores **/
error_t initQueue( void )
{

	CLEAR( TDMA_tx_queue ); /*erase packets queue*/
	TDMA_tx_queue_r_ptr = 0 ; TDMA_tx_queue_w_ptr = 0 ;
	
	/* initiate tranmit sem */
	if ( sem_init(&TDMA_Transmit_sem,0,0) < 0 ) /* used to communicate when there pkts to send in TDMA_tx_queue */
		PRINTF_FL_ERR("TraSem init %s ", strerror( errno ) ) ;
	if ( sem_init(&tx_queue_freespots_sem,0,TDMA_TX_QUEUE_SIZE-2) < 0 ) /* used to communicate when TDMA_tx_queue has space */
		PRINTF_FL_ERR("tx_qSem init %s ", strerror( errno ) ) ;
	
	CLEAR( TDMA_rx_queue );
	TDMA_rx_queue_ptr.r = 0 ; TDMA_rx_queue_ptr.w = 0 ;
	pthread_mutex_init( &(TDMA_rx_queue_ptr.mutex) , NULL ) ;
	if ( sem_init(&Receive_sem, 0, 0) < 0 )
		PRINTF_FL_ERR("Receive_sem init %s ", strerror( errno ) ) ;
	if ( sem_init(&rx_queue_freespots_sem, 0, TDMA_RX_QUEUE_SIZE-1) < 0 )
		PRINTF_FL_ERR("rx_queue_freespots_sem init %s ", strerror( errno ) ) ;
	
		
	PDR_init();
	//TDMA_rx_queue_ptr.mutex = PTHREAD_MUTEX_INITIALIZER ; /* mutex to read/write to the queue */
	//TODO: malloc queue instead ? we could have dynamic buffers size. ..
	return 1; /* ok */
}



/** call socket SENDTO() **/
error_t actuallySend( 
	const tdma_gen_packet_t * const tdma_packet_ptr, /*packet built, w/ tdma header and payload */
	size_t tdma_packet_len, /* total size  */
	uint8_t dest_IP , /* where to send this  */
	int tx_sk_fd /* socket fd to use */
	)
{
	/* select dest_id */
	struct sockaddr_in send_addr; 
#ifdef SIMULATOR
	prepare_sockaddr(dest_IP, SIM_RX_ALL, &send_addr);
#else
	prepare_sockaddr(dest_IP, PORT_UAV_RX80211, &send_addr);
#endif
	
	//dumpData((uint8_t*)(&tdma_packet_ptr),tdma_packet_len ) ;

	/* typical send to */
	int ret2 = sendto( 
		tx_sk_fd,
		(void*)tdma_packet_ptr, 
		tdma_packet_len  , 
		0 , /* flags */ 
		(const struct sockaddr *)&send_addr , 
		(socklen_t)sizeof(send_addr) ) ;
	
	if ( 0 > ret2 )
	{
		PRINTF_FL_ERR("Error sending to {%s:%d} (%s)\n", 
			inet_ntoa(send_addr.sin_addr), 
			ntohs(send_addr.sin_port),
			strerror(errno)) ; 
		if (EINVAL == errno)
			PRINTF_FL_ERR("EINVAL\n");
		return E_OTHER ;
	}
	
	
	//PRINTF_FL_WARN("Sent to %s:%d\n" , 
		//inet_ntoa( send_addr.sin_addr ), 
		//ntohs(send_addr.sin_port) );
		
	//PRINTF_FL_WARN("Sent\n");
	return E_SUCCESS ;
}




#if 0
/** send a micro TDMA packet 
 * upstream 
 * guarantees DVSP gets requests **/
void sendBeacons(void)
{
	static unsigned counter = 0;
	if ( 10 != counter ) 
	{
		counter++;
		return;
	}
	
	counter = 0 ;
	
	//tdma_printBin( 0 ) ;
	//tdma_printBout( 0 ) ;
	
	if ( 0 != TDMA_upStream() )
	{
		
		tdma_tx_queue_item_t new_item ;  
		new_item.timestamp = getEpoch() ;
		new_item.dest_ip = TDMA_upStream() ;
		new_item.payloadpkt_len = 1 ;
		new_item.payloadpkt_ptr = malloc(1) ; /* buildsendlog frees payload at the end */
		new_item.type = 3U ; /* low prio */
		
		char *payload = "X" ;
		memcpy( new_item.payloadpkt_ptr, payload , 1 ) ;
		build_send_log( &new_item );
		
		PRINTF_FL(
			"Injecting pkt upstream (to #%" PRIu8 ")\n" , 
			TDMA_upStream() ) ;
		
		
	}
}
#endif

	
/**********************************************************************
 * this thread takes care of tansmitting stuff inside the tdma-tx-queue 
 **********************************************************************
 * - sends packts only during a bounded tdma slot
 * - trottles data to respect a given throughput cap
 * - logs sent tdma packets 
 *********************************************************************/
void *txThread()
{
	error_t status = E_SUCCESS ; 
	
	tdma_clearStats(); /* clear statistics metrics */

	/* ### SOCKET TO SEND PACKETS ### */
	g_tx_sock_fd = socket( PF_INET, SOCK_DGRAM , IPPROTO_UDP ) ; 
	if (0 > g_tx_sock_fd) 
	{
		PRINTF_FL_ERR( "sendsock error\n") ;
		status = E_SOCKET ; 
		goto error_socket1 ;
	}
	PRINTF_FL_WARN("SOCKET OK\n");
	
	#if 1/* allow broadcast */
		const int broadcastEnable=1;
		#ifdef VERBOSE
		int ret=
		#endif
		setsockopt(
			g_tx_sock_fd, SOL_SOCKET, SO_BROADCAST, 
			&broadcastEnable, sizeof(broadcastEnable));
		
		#ifdef VERBOSE
		if (ret<0) 
		{
			PRINTF_FL_ERR("failed seting broadcast\n");
			return NULL;
		}
		#endif
	#endif
	/* Bind socket */
	/* link port to the socket, we need to set source port  */
	struct sockaddr_in si_me ; 
	si_me.sin_family 		= AF_INET ; /*address format (ip4)*/
	si_me.sin_port 			= htons( TX_LOCAL_PORT + Param_tdma.node_ip ) ; 
	//si_me.sin_addr.s_addr		= inet_addr("192.168.2.3");
	
	//si_me.sin_addr.s_addr 	= htonl( INADDR_ANY ) ; //this socket is not reading packets, so not relevant 
	PRINTF_FL_WARN("preparing to bind %s:%hu\n", 
		inet_ntoa(si_me.sin_addr),
		ntohs(si_me.sin_port));
	if (-1 == bind( g_tx_sock_fd, 
			(struct sockaddr*)&si_me, 
			sizeof(si_me))) 
	{
		PRINTF_FL_ERR( 
			"Error binding - srcport %s:%hu (%s)\n", 
				inet_ntoa(si_me.sin_addr),
				ntohs(si_me.sin_port), 
				strerror (errno) );
		status = E_BIND ;
		goto error_bind2 ;
	}
	PRINTF_FL_WARN( 
			"TDMA will send packets from srcport %hu (%s)\n", 
			ntohs(si_me.sin_port), 
			strerror (errno) );
		
	/* open tx log file */
	#ifdef LOGDATA
	{
		char prefix[10];
		snprintf( prefix, sizeof(prefix)-1,
			"TDMA#%02d", Param_tdma.node_ip ) ;
		status = open_log_file( prefix , "TX" , &tdma_log_tx );
		if ( 0 > status )
			goto error_file4 ; 
	}
	#endif 
	//char file_buf[40] ; /* string to write a line into log-file */
	
	/* init slot boundaries - begin & end */
	tdma_initSlotLimits();


	
	//const uint8_t src_ip = Param_tdma.node_ip ; /* my ip */

	sleep(1);
	
	
	/* timestamp begin of the while loop  */
	double time_beginning2 = getEpoch();
	if ( 0 > time_beginning2 )
	{
		status = (int)time_beginning2;
		goto error_time5 ;
	}	
	Tdma_statistics.time_beginning = time_beginning2;
	 
	slot_limits_t myslot = TDMA_getOwnSlot();
	//uint16_t cur_time_ms = (uint16_t)tdma_getCurrentRoundTime();
	double cur_time_ms_d = tdma_getCurrentRoundTimeD();

	/* how long we need to wait to *BEGIN* slot? */
	suseconds_t t_us ;
	t_us = 
		(cur_time_ms_d < myslot.begin_ms) ? 
		1000.0*(myslot.begin_ms - cur_time_ms_d) :
		1000.0*(( Param_tdma.round_period_ms - cur_time_ms_d ) + myslot.begin_ms) ;
	usleep( t_us ) ;
	
	PRINTF_FL(
		"We are at %ldus from starting the slot. "
		"t=%.3fms , [%"PRIu16",%"PRIu16"]\n",
		t_us,
		cur_time_ms_d,		
		myslot.begin_ms,
		myslot.end_ms);
	 
	
	
	
	status = E_TERMINATING ; 
	

	/* start cycle */
	while ( !TExit_threads )
	{
		
		if ( tx_thread_main_handler() < 0 )
			break;		
	
		////////////////////////
	} //end of while()
	
	/* leaving the while(), make full cleanup: */
	//PRINTF_FL_WARN("left the txTDMA while(1)\n");
	error_time5 :
	#ifdef LOGDATA
	fclose( tdma_log_tx ) ;
	error_file4 :
	#endif
	//free( tmp_txpkt ) ;
	//error_mem3:
	error_bind2:
		shutdown(g_tx_sock_fd,2) ;
	error_socket1:
	
	
	
	TExit_threads = 1 ;
	Tdma_initiated = 0 ;
	if (status < E_TERMINATING )
	{
		printError( status ) ;
		PRINTF_FL_ERR( 
			"[TDMA TX] Thread terminated with Errors: %d \t[%08x] *******\n",  status ,
			(unsigned int)pthread_self() );  
			return NULL ;
	}
	
	PRINTF_FL( 
		"[TDMA TX] Thread terminated: normally\t[%08x] *******\n", 
		(unsigned int)pthread_self() );  
	
	return NULL ;

}

















/**********************************************************************
 * this thread takes care of tansmitting stuff inside the tdma-rx-queue 
 **********************************************************************
 * - reads from socket
 * - logs rcvd tdma packets 
 * - adds it to the RX queue
 *********************************************************************/
void *rxThread()
{

	error_t status = E_SUCCESS ;
	PRINTF_FL("Initiating :::::::::\n");

	/*UDP stuff*/
	int 				rx_sock_fd ; /* rx socket */
	ssize_t 			rx_tdmapkt_len ;
	struct sockaddr_in	si_me ; /* sockaddr for the received packet */
	
	//socklen_t 			si_other_len ;


	/* prepare UDP socket */ 
	if ( (rx_sock_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1 )
	{
		PRINTF_FL_ERR( "Error in udp socket (%s)\n", 
			strerror( errno ) );
		status = E_SOCKET ;
		goto error_socket1 ;
	}
	
	//CLEAR( si_me ) ;

#ifdef SIMULATOR
	/* uav waits packets at a specic address 127.0.0.myid - different per instance when simulating */
	prepare_sockaddr(getMyIP(),PORT_UAV_RX80211,&si_me);
#else
	si_me.sin_family = AF_INET ; //address format (ip4)
	si_me.sin_port = htons(PORT_UAV_RX80211); /* rcv always in same port */
	si_me.sin_addr.s_addr = htonl(INADDR_ANY); /* accept packets from any address */
#endif
	if ( bind(rx_sock_fd, (struct sockaddr*)&si_me, sizeof(si_me) ) == -1 ) //link port+address to the socket
	{
		PRINTF_FL_ERR( 
			"Error binding %s:%d (%s)\n", 
			inet_ntoa(si_me.sin_addr), ntohs(si_me.sin_port),
			strerror(errno));
		status = E_BIND;
		goto error_bind2;
	}
	PRINTF_FL_WARN(
		"ready to Receive TDMA pkts at %s:%d\n",
		inet_ntoa(si_me.sin_addr) , ntohs(si_me.sin_port) );

	/* tinmeout! */
	//{
		//struct timeval temp2_time ; /* for recvfrom() timeout, sec and usec */
		//temp2_time.tv_sec = 10 ; 
		//temp2_time.tv_usec = 000 ;
		//if ( setsockopt( rx_sock_fd , SOL_SOCKET, SO_RCVTIMEO , &temp2_time , sizeof(temp2_time) )  < 0)
		//{
			//PRINTF_FL_ERR( "Sck opt error (%s)\n", 
				//strerror( errno ) ) ;
			//status = E_INVALID_INPUT ;
			//goto error_sockopt3 ;
		//}
	//}

	/* get a memory piece to save incoming packets:	
	 * allocate 1500 bytes.  where received data will be. */
	void *rx_tdmapkt_ptr = calloc( MAX_PACKET_SIZE, sizeof(char) ) ;
	if(NULL == rx_tdmapkt_ptr)
	{
		PRINTF_FL_ERR("calloc() - Error (%s)\n",strerror(errno));
		status = E_NO_MEM ;
		goto error_mem4;
	}

	#ifdef LOGDATA
	PRINTF_FL("LOGGIN\n");

	FILE *tdma_log_rx = NULL ;
	{
		
		char prefix[10];
		snprintf( prefix , sizeof(prefix)-1 ,
			"TDMA#%02d" , Param_tdma.node_ip ) ;
		if ( open_log_file( prefix , "RX" , &tdma_log_rx ) < 0 )
		{
			PRINTF_FL_ERR( "Failed to open log file\n") ;
			status = E_FILE_CREATION ;
			goto error_file5 ;
		}
	}
	#endif
	
	/* init reading cycle */
	#ifdef SHARE_PDR
	double ref_time_shareslot = getEpoch() ;
	#endif
	
	status = E_TERMINATING ; /* default reason to get out of the while */
	struct sockaddr_in 	si_other ; /*sockaddr for myself*/
	socklen_t si_other_len = sizeof(si_other); /*size*/
	while ( !TExit_threads )
	{
		#ifdef LOGDATA
		error_t d_error = -1;
		#endif
		
		//memset( rx_tdmapkt_ptr , 0 , MAX_PACKET_SIZE ) ;
		//memset( (char *) &si_other , 0 , sizeof(si_other) ) ;
		
		/* ****** RECEIVE FROM SOCKET **** */
		
		//CLEAR( si_other ) ;
		rx_tdmapkt_len = recvfrom( 
			rx_sock_fd , 
			rx_tdmapkt_ptr , 
			MAX_PACKET_SIZE , 
			0, /* flags */
			(struct sockaddr*)&si_other, &si_other_len ) ;
		
			
		/* Error handling */
		if (0 > rx_tdmapkt_len)
		{
			/* timedout ?  */
			PRINTF_FL_ERR("rcvfrom() error: %s\n",strerror( errno ) ) ;
			continue ; /* time to leave */
		}
		
		//PRINTF_FL_WARN("Pkt Received\n");
		/* get pointer to rcvd TDMA header */
		tdma_header_t *tdma_header_ptr = (tdma_header_t*)rx_tdmapkt_ptr;
		Header = *tdma_header_ptr;
		//PRINTF_FL_ERR( "%s",
				//tdma_printHeader( tdma_header_ptr ) );
		
	
		

		//dumpData( (uint8_t*)rx_tdmapkt_ptr , num_bytes_read ) ; 
		uint8_t other_ip = getOtherIP( si_other ) ;
		//PRINTF_FL_WARN("pkt from %"PRIu8 "\n", other_ip);
		if (
			(MAX_IP < other_ip) || 
			(0 == other_ip) || 
			(other_ip == (uint8_t)getMyIP() )
		)
		{
			/*
			PRINTF_FL_ERR(
				"Source of packet (#%" PRIu8 ") is out of range.\n", 
				other_ip ) ;
			PRINTF_FL_ERR( "%s",
				tdma_printHeader( tdma_header_ptr ) );
			PRINTF_FL_ERR("size: %zd B\n", rx_tdmapkt_len );
			dumpData( rx_tdmapkt_ptr, rx_tdmapkt_len );
			*/
			#ifndef X86
			 drk_exit();
			#else
			 continue ;
			#endif
		}
		//PRINTF_FL_ERR( "Source si_other %"PRIu8"\n",
				//other_ip );
	
		
		 
		/* update local knowledge: 1) pair[Slot_id,IP] and  2) slot width */
		tdma_setIp2SlotId( other_ip, tdma_header_ptr->slot_id );
		
		/* VSP - Variable Slot Protocol - analyse incoming/outgoing requests */
		#if VSP
		if (Param_tdma.vsp)
			tdma_runRequests(*tdma_header_ptr) ;
		#endif
		
		
		/* check if:
		 * 1)it is a HIGH-Priority. 
		 * do not record delays. bc this pkt is out of a tdma slot.
		 * */
		uint16_t hi_prio = tdma_getPrio(tdma_header_ptr); ;/*get hi-byte */
		//PRINTF_FL_WARN(
			//"Got a %s pkt - %"PRIu16" \n",
			//(hi_prio) ? "hiprio" : " normal",
			//tdma_header_ptr->type );
		/* normal pkts: come sent within a slot. so record delays 
		 * hi-prio do not, so ignore */
		if (!hi_prio)
		{
			
			/* Record TDMA header to sync our own clock later on */
			if (Param_tdma.sync_flag)
			#ifdef LOGDATA
				if((spanning_tree[(Param_tdma.slot_id)-1][(tdma_header_ptr->slot_id)-1]) != 0){
					d_error = tdma_recordPktDelay( *tdma_header_ptr ) ; /* compute and record delays */
				}
			#else
			if((spanning_tree[(Param_tdma.slot_id)-1][(tdma_header_ptr->slot_id)-1]) != 0){
				tdma_recordPktDelay( *tdma_header_ptr ) ; /* compute and record delays */
			}
			#endif
		}
		
		//PRINTF_FL_WARN("got a pkt from #%"PRIu8"\n",other_ip);
		/* Analyze inPDR: PDR *FROM* transmitter to *ME* */
		PDR_estimate( 
			tdma_header_ptr->seq_num , /* this seq num is guaranteed 
				to be unique between transmiter and us */
			other_ip ) ;
		
		
		
		/* parse packet by type - lowbyte */
		uint16_t type = tdma_getType(tdma_header_ptr);
		
		switch (type)
		{
		case PDR:
			PDR_parsePkt( rx_tdmapkt_ptr, rx_tdmapkt_len , 
				other_ip  ) ;
			break;
		case PM:
			//PRINTF_FL_WARN("got a PM pkt\n");
			parserPmPkt(rx_tdmapkt_ptr, rx_tdmapkt_len, other_ip ) ;
			break;
		case PURE:
			parserPurePkt(rx_tdmapkt_ptr, rx_tdmapkt_len, other_ip ) ;
			break;
		case REQUEST:
			/* packets sent to creat traffic in the upstream direction
			 * as all others, they carry slot requests */
			/* currently requests go in the header of every packet.*/
			// nothing to parse 
			PRINTF_FL_WARN("Got a REQUEST pkt\n");
			break;	
		default:
			PRINTF_FL_WARN("Received a TDMA packet of (unknown) type %d\n", type );
			dumpData( rx_tdmapkt_ptr, (uint16_t)sizeof(tdma_header_t) ) ;
			dumpData( rx_tdmapkt_ptr, (uint16_t)rx_tdmapkt_len ) ;
		}
		
		/* log rcdv data */
		#ifdef LOGDATA
		if ((PDR == type) || (PM == type) || (REQUEST == type) || (PURE == type))
		{
			tdma_logPacket( tdma_log_rx , /* file descriptor */
				(tdma_header_t*)rx_tdmapkt_ptr , /* rcvd header */
				0 , /* added-to-queue timestamp isn't useful here, since it's the current timestamp */
				other_ip , /* where this just came from */
				getMyIP() , /* it was destinated to myself - since i just got it */
				rx_tdmapkt_len ,
				tdma_getRxBufferUse() ,
				tdma_getBandwidthIn( other_ip ) ,
				d_error ) ;
		}
		#endif
			
		#ifdef SHARE_PDR
		/* send a feedback PDR packet to everyone 
		 * about the pdr from them to us (inPDR) */
		if ( getEpoch() > ref_time_shareslot + PERIOD_SHARE_SLOT ) 
		{
			PDR_shareWithNeighbors() ;
			ref_time_shareslot = getEpoch() ;
		}
		#endif
		
		
		
		//PRINTF_FL(
				//"Posted a pkt. "
				//"from #%" PRIu8 ", %zd B\n" , 
				//other_ip ,
				//num_bytes_read ) ;
	} /* end of while ( 1 )*/
	
	#ifdef LOGDATA
	fclose( tdma_log_rx ) ;  
	error_file5: 
	#endif
	free( rx_tdmapkt_ptr ) ;
	error_mem4: 
	shutdown( rx_sock_fd, 2 ) ; /* shutdown Rx & Tx*/
	//error_sockopt3:
	error_bind2:
	error_socket1:

	TExit_threads = 1 ;
	Tdma_initiated = 0 ;
	if ( E_TERMINATING > status )
	{
		PRINTF_FL_ERR( 
			"[TDMA RX] Thread terminated with Errors \t[%08x]\n", 
			(unsigned int)pthread_self() );  
	}
	else
	{
		PRINTF_FL( 
			"[TDMA RX] Thread terminated: normally\t[%08x]\n", 
			(unsigned int)pthread_self() );  
	}
	
	return NULL ;

}




/* just beginning my txslot*/
error_t begin_of_slot_operations(void)
{
	
	static int sw_slot_cnter = 0 ;
	
	/* let's sync our 'clock', i.e, move the slot  */
	if (Param_tdma.sync_flag) 
	{
		int32_t delta=0;
		if (
			(tdma_syncronizeSlot( &delta ) > 0) && 
			(0 != delta)
		) /* changes slot bounderies according to measured delay from neighbors,  */
		{
			//PRINTF_FL_WARN("Shifting %+"PRId32"ms\n", delta );	
		}
		
		if (!tdma_insideSlot())
		{
			#if TDMA_DEBUG
			slot_limits_t tmp_s = TDMA_getOwnSlot();
			PRINTF_FL_WARN("Shifted left us out\n");
			PRINTF_FL_WARN( 
			"t_r =%"PRIu16"ms, "
			"we are outside our slot [%"PRIu16",%"PRIu16"]ms\n" , 
			(uint16_t)tdma_getCurrentRoundTime() ,
			tmp_s.begin_ms,
			tmp_s.end_ms ) ;
			#endif
			/* could not start slot-> we are outside our slot again due to shift*/
			tdma_sleepTillSlotBegin();
			 
			if (1 == TExit_threads) /* if it is time to terminate, leave*/
				return E_TERMINATING;
			//return -11 ; /* could not start slot-> we are outside our slot again due to shift*/
		}
	}
	
	
	
	
	
	sw_slot_cnter++;
	//PRINTF_FL_WARN( 
		//"SW#%d\n",
		//sw_slot_cnter );
	
	/* save current time in timi_beginning */
	double tmp_epoch = getEpoch();
	if (0 > tmp_epoch)
	{
		PRINTF_FL_ERR("Get epoch failed\n") ;
		return (int)tmp_epoch;
	}	
	else
	{
		Tdma_statistics.epoch_init_slot = tmp_epoch;
	}
	
	//PRINTF_FL_WARN( 
		//"Begin (%.3fms)\n" ,
		//tdma_getCurrentRoundTimeD() ); /* type: double */
	
	
	//#ifdef VERBOSE
	//tdma_printSlot( TDMA_getOwnSlot());
	//#endif
	
	//#ifdef VERBOSE
	//tdma_printBin(0);
	//#endif
	
	
	/* ERASE rx stats */
	tdma_updateRxStats();
	
	#if VSP
	if ( Param_tdma.vsp ) 
	{
		//error_t ret = 
		tdma_makeRequest() ;
		//PRINTF_FL_WARN("tdma_ makerequ %d\n", (int)ret );
	}
	/* if VSP is active, 
	 * add 2 small pkts to dw and up slot neighbors */
	if ( Param_tdma.vsp )
	{
		sendBeacons();
	}	
	#endif
		
	

	
	
	return E_SUCCESS ;
}
			


/* just finished my tx slot */
error_t end_of_slot_operations(void)
{
	
	//PRINTF_FL_WARN(
		//"End of Slot (%.3fms)\n" ,
		//tdma_getCurrentRoundTimeD() );
	#ifndef X86
	#ifdef PRINTSTATS
	tdma_printStats();
	#endif
	#endif 
	
	//#ifdef VERBOSE
	//tdma_printBout(0);
	//#endif
	
	/* update statistics */
	#define ts Tdma_statistics
	
	//if (Param_tdma.vsp)
	//{
		//printf("BsentAny: %f\n",ts.slot_bytes_sent[0]);
		//printf("Bsent13: %f\n",ts.slot_bytes_sent[13]);
		//tdma_printBout(0);
	//}
	
	//PRINTF_FL_WARN("Finished slot #%"PRIu64"\n", ts.slot_round_counter );
	#define ALL_IPs 0
	Bandwidth.out[ ALL_IPs ] = 0 ; /* sum of all outs */
	for ( uint8_t i = 0 ; i < sizeof(DRONE_LIST) ; i++ )
	{
		uint8_t ip = DRONE_LIST[i] ;
		
		/* Bout (KBps) = the total of sent (Bytes) divided by used slotwidth (ms) */
		slot_limits_t used_slot = TDMA_getOwnSlot();
		used_slot.end_ms = ts.last_tx_pkt_ms ;
		if ( ts.slot_bytes_sent[ip] > 100 ) /* update estimate */
		{	
			float time_span_ms = tdma_getSlotWidth( used_slot ) ; 
			if (1 > time_span_ms )
				time_span_ms = 1;
			float new_estimate_b_out = MIN( 4000 , 
				ts.slot_bytes_sent[ ip ]  /
				time_span_ms ) ;
			Bandwidth.out[ip] = 
				0.9 * Bandwidth.out[ ip ] + 
				0.1 * new_estimate_b_out ;
				
			//if (ip==12)
				//PRINTF_FL_WARN("new estimate %.2fkb/s\n",new_estimate_b_out);
			//if ( 
				////( Bandwidth.out[ ip ] >  10 ) &&
				//( ts.slot_round_counter % 1 == 0 )
				//)
				//tdma_printBout( ip ) ;
		}
		 /* sum up ALL bandwidth out */
		Bandwidth.out[ALL_IPs] += Bandwidth.out[ip] ;
		#undef ALL_IPs
		//tdma_printBout( ip );
		/* erase stats */
		ts.slot_bytes_sent[ip]  = 0 ;
		//for (uint8_t i=0;i<sizeof(DRONE_LIST);i++)
			//ts.round_bytes_rcvd[DRONE_LIST[i]] = 0 ;
		
	}
	
	
	ts.slot_round_counter++ ; /* first round is #0 . set new round at the END of one.*/
	ts.last_tx_pkt_ms = 0 ;
	ts.slot_bytes_sent[0] = 0 ;
	#undef ts


	
	
	
	return E_SUCCESS;
}




/** control thread: run/sleep
 * --> uses sem to sleep
**/
error_t control_tx_thread(void)
{
	
	//PRINTF_FL("waiting for buf\n");
	/* check socket buffer!*/
	 /* hOLD HERE if SEND BUFFER is full, 
	  * or till timeout */	
	int timeout =  1 ; /* true . return if already outside the slot */
	error_t ret1 = tdma_waitTillBufFree( g_tx_sock_fd , timeout );
	if ( 0 > ret1 )
	{	
		PRINTF_FL_ERR( "ERROR on tdma_waitTillBufFree\n" );
		PRINTF_FL_ERR( "NIC Buffer Error. Terminating\n" );
		return ret1 ;
	}	//PRINTF_FL("buf ok\n");
	
	/* buffer is ok to send stuff.
	 * check if we are inside slot
	 * otherwise, 
	 * sleep */
	if (tdma_insideSlot_guarded() == 0)
	{
		/* means slot has terminated */
		//PRINTF_FL("slot end\n");
		end_of_slot_operations(); /* do some operations at the end of our slot */
		
		#if TDMA_DEBUG
		tdma_debugInsideSlot();
		#endif
		tdma_sleepTillSlotBegin();
		
		/* we are now inside our slot! */	
		/* do some operations right at the begin of slot */
		/* this includes, synchronizing ie shifting the slot*/
		if (0 > (ret1 = begin_of_slot_operations()))
		{
			return ret1;
		}
		
		
		//PRINTF_FL("ready to send a bunch of pkts\n");
		
	}
	
	
	
	
	return E_SUCCESS;

}
	

	
int buildTDMAPacket(
	tdma_tx_queue_item_t * in_item_packet, 
	tdma_gen_packet_t * const tdma_pkt_output)
{
	
	/* get time in ms precision, to put in payload
	 * (range 0-Tms, where T is the round period */
	double cur_time_ms = tdma_getCurrentRoundTimeD();
	if (0 > cur_time_ms)
	{
		PRINTF_FL_ERR("Could not get round time. Terminating\n");
		return cur_time_ms ;
	}


	/* build the TDMA header. 1) header first */
	#if VSP
	tdma_pkt_output->header.request_slot_width_ms = 
		tdma_getRequestedWidth( in_item_packet->dest_ip ) ; /* request #dest_ip a new slotwidth */
	#endif
	tdma_pkt_output->header.timestamp_ms 	= cur_time_ms ;
	tdma_pkt_output->header.timestamp_partms= (cur_time_ms - floor(cur_time_ms))*256 ; /* 0.5 ? then set this to 128 */
	tdma_pkt_output->header.slot 			= TDMA_getOwnSlot() ;
	tdma_pkt_output->header.slot_id 		= Param_tdma.slot_id ;
	tdma_pkt_output->header.type 			= in_item_packet->type ;
	tdma_pkt_output->header.seq_num			= getTDMASeqNum( in_item_packet->dest_ip );
	tdma_pkt_output->header.seq_num_gen			= getTDMASeqNumG();
				
	/* 2)then payload . copy the payload */
	memcpy( 
		(void*)tdma_pkt_output->payload, 
		in_item_packet->payloadpkt_ptr, 
		(size_t)in_item_packet->payloadpkt_len ) ; 
	free(in_item_packet->payloadpkt_ptr) ;
	
	int pkt_len = (int)in_item_packet->payloadpkt_len + (int)sizeof(tdma_header_t) ;
	return pkt_len ;
}

error_t processPacketErrors( error_t ret,
	tdma_tx_queue_item_t *item_packet )
{


		
	if ( ret == E_TIMEDOUT )
	{
		/* nothing to be sent */
		//PRINTF_FL_WARN("nothing to send (timedout)\n");
		return E_EMPTYQUEUE;
	}	
	
	/* handling contents of item_packet */
	if (
		( item_packet->dest_ip == 0 ) || 
		( item_packet->dest_ip > MAX_IP ) )
	{
		PRINTF_FL_WARN( 
			"Pkt to send: weird dest_ip %"PRIu8". Ignoring.\n", 
			item_packet->dest_ip) ;
		return E_UNK_DST ;
	}
	/* handling contents of item_packet */
	if( item_packet->dest_ip == getMyIP() )
	{
		PRINTF_FL_WARN( 
			"Pkt to send:  dest_ip %"PRIu8" is myself.\n", 
			item_packet->dest_ip ) ;
		return E_UNK_DST ;
	}
	
	
	if ( item_packet->payloadpkt_len > MAX_PACKET_SIZE )
	{
		PRINTF_FL_WARN( 
			"Pkt to send: too big %dB. Ignoring.\n", 
			(int)item_packet->payloadpkt_len );
		return E_TOOBIG ;
	}	
	
	return E_SUCCESS ;
}

/** this goes into while (1) till this returns negative 
 * keep sending packets .
 * control_tx_thread is a rendezvous point. holds the thread if outside slot **/
error_t tx_thread_main_handler( void ) 
{

	
	/* control thread run/pause 
	 * this should pause when outside slot.
	 * or resume when it is ok to send a packet */
	error_t rt = control_tx_thread();
	if (0 > rt)
		return rt;
	
	
	

	/*************************
	 *  ### Time to Transmit! ### *
	 * ****************************/
	
	/** GRAB PACKET ---------------------
	 * if there is one, fill item_packet and return sucess 
	 * if no packet, wait till the end of slot. 
	 * still none? return E_TIMEOUT 
	 * other returns: errors **/
	tdma_tx_queue_item_t item_packet ;
	//PRINTF_FL_WARN("grabing tdmapkt\n");
	error_t ret = getOldestTDMAPkt( &item_packet ) ; /* get packet, FIFO . Free packet.payloadpkt_ptr at the end */
	//PRINTF_FL_WARN("grabing done (ret%d). txqueue at:%d\n",ret,tdma_getTxBufferUse());
	
	
	/* error handling ------------*/
	if (0 > ret)
	{
		if (E_TERMINATING != ret) 
		{
			/* unexpected error */
			PRINTF_FL_ERR("Terminating due to error. Dumping\n");
			printError( ret );
			TDMA_dumpTxQueue();
		}
		return ret ;/* return now, and inform caller it is time to close */
	}
	if (E_TIMEDOUT == ret)
	{
		//PRINTF_FL_WARN("getOld timdout Q=%"PRIu16" items\n", tdma_getTxBufferUse());
		return E_TIMEDOUT;/* nothing in Q . inform caller everything is ok */
	}
		
	
	
	
	/* inspect packet inside to detect if it is valid */
	//PRINTF_FL("processing\n");
	ret = processPacketErrors( ret, &item_packet );
	if (0 > ret)
		return E_SUCCESS;/* invalid packet, abort sending, inform caller everything is ok, gways */
		
	
	
	/** ----------------All set to send the packet --------------------------
	 * do not block this with pauses/semwaits until packet is sent and logged. 
	 * ISR can run no problem 
	 */
	
	/* error debug */
	//tdma_debugInsideSlot();
	//if (tdma_debugInsideSlot()< 0)
		//inside_slot=0;
	
	/* build, send and log */
	int tdma_pck_len = build_send_log( &item_packet );
	if (0>tdma_pck_len)
		return tdma_pck_len;
		
	
	#ifdef VERBOSE
	#if VSP
	if ( Param_tdma.vsp ) 
		tdma_printBout(0);
	#endif
	#endif
	/* update stats - tdma_utls */
	tdma_updateTxStats( 
		(uint16_t)tdma_pck_len , 
		item_packet.dest_ip, 
		(uint16_t)tdma_getCurrentRoundTime() );
		
	/* update clock stats - tdma_utils */
	if ( 0 > tdma_updateClockStat2() ) 
		PRINTF_FL_ERR( "ERROR UDPATE STATS\n\n");
	
		
	#ifdef THROTTLE 
	PRINTF_FL( "NOOOOOOOOOOOO\n" );
	/* Find how much we should sleep to reach Trottle throughput  */
	double sleep_seconds ; /* tmp save here time to sleep */ 
	sleep_seconds = (double)Tdma_statistics.slot_bytes_sent[0] /
		( 1e6*Param_tdma.throughput_target_MBps ) - 
		Tdma_statistics.slot_clock_seconds  ; 
	
	if ( Tdma_statistics.slot_bytes_sent[0] > 2 ) /* start throttling after 2bytes are sent */
	{
		if ( sleep_seconds > 0 )
		{
			microsleep( MAX((unsigned int)(sleep_seconds*1e6), 100 ) );
		}
	}
	#endif
	
	return E_SUCCESS ;
}

static int build_send_log(tdma_tx_queue_item_t * const in_item_packet)
{
	
	/* build a tdma packet,ie.: 
	 * header: tdma stuff. payload is whatever is saved in txqueue 
	 * input: item_packet
	 * output: tdma_packet */
	tdma_gen_packet_t tdma_packet ; /* get mem for packet */
	CLEAR( tdma_packet ) ; /* not needed (?) */
	int tdma_packet_len = buildTDMAPacket(in_item_packet, &tdma_packet);
	if (0 > tdma_packet_len)
		return tdma_packet_len ;
	
	
	/* ---SEND to the driver ------*/
	error_t err_ret = actuallySend( 
		&tdma_packet , 
		(size_t)tdma_packet_len , 
		in_item_packet->dest_ip  , /*255,	*/
		g_tx_sock_fd 
		);
	if (0 > err_ret)
	{
		PRINTF_FL_ERR("actuallySend as failed. Exiting thread..\n");
		return err_ret ;
	}
	//PRINTF_FL("sent pkt to #%"PRIu8". seqnum %"PRIu32". size %u\n",
		//in_item_packet->dest_ip,
		//tdma_packet.header.seq_num,
		 //(size_t)tdma_packet_len );			
	
	
	/* packet sent successful */
	/* ---- Log a bunch of parameters --- */
	#ifdef LOGDATA 
	tdma_logPacket( 
		tdma_log_tx, /* log file */
		(tdma_header_t*)&tdma_packet, /* tdma header - first bytes of the packet are the header */
		in_item_packet->timestamp, /* epoch when packet was added to TDMA_tx_queue via TDMA_send (s) */
		getMyIP(), in_item_packet->dest_ip,  /* outgoing, so: src=myip ; dest=nexthop */
		(uint16_t)tdma_packet_len, /* num of bytes given to sendto() */
		tdma_getTxBufferUse(), /* current #pkts in the tx queue */
		tdma_getBandwidthOut(in_item_packet->dest_ip) ,
		0/* tx packets have no measured delay */
		) ; 
	#endif
	
	/* return bytes sent */
	return tdma_packet_len ;
}

#if 0
/* add a packt to the txqueue at current read pointer */
int inject( uint8_t dest_ip, uint16_t pkt_len , void* pkt_ptr )
{

	if ( 0 == dest_ip )
		dest_ip = TDMA_upStream() ;
	
	if (
	(Param_tdma.node_ip == dest_ip) || 
	(0 == dest_ip)
	)
	{
		PRINTF_FL_ERR("Can't send pkts to myself or #0\n ") ;
		return E_INVALID_INPUT ;
	}

	pthread_mutex_lock( &TDMA_tx_queue_mutex ) ; /* lock */

	tdma_tx_queue_item_t new_item ;  
	new_item.timestamp = getEpoch() ;
	new_item.dest_ip = dest_ip ;
	new_item.payloadpkt_len = pkt_len ;
	new_item.payloadpkt_ptr = malloc( (size_t)pkt_len ) ;
	new_item.type = REQUEST & 0x00FF ; /* low prio */

	
	
	if ( NULL == new_item.payloadpkt_ptr )
	{
		/* failed to add pkt to queue. no memory available */
		pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ; /* lock */
		PRINTF_FL_ERR("Cannot get mem\n");
		return E_NO_MEM ; 
	}
	memcpy( new_item.payloadpkt_ptr, pkt_ptr, (size_t)pkt_len ) ;
	/* inject packet at the current READ pointer */
	TDMA_tx_queue[TDMA_tx_queue_r_ptr] = new_item ; 

	pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ; /* Unlock */
	
	//PRINTF_FL_WARN("injectng %"PRIu16"B\n", pkt_len );
	return E_SUCCESS ; /* ok */
}
#endif

 /********************************
 * Retrieve the oldest TDMA-packet from the queue (FIFO) 
 * if nothing found for two seconds, it returns
 * Delete it from the source ie queue, afterwards
 * also: len & dest_id of payload 
 * return:	error (<0) , descriptive error code 
 * 			success : E_SUCCESS
 *****************************************************/
error_t getOldestTDMAPkt(tdma_tx_queue_item_t *packet) 
{
	#if 1
	//PRINTF_FL_WARN("Waiting for pkt [TDMA_Transmit_sem]\n");

	error_t ret ;
	useconds_t t_us = tdma_getTimeTillSlotEnd();
	//PRINTF_FL_WARN(
		//"t_r %.3fms -- t_ms %ld\n", 
		//tdma_getCurrentRoundTimeD(),
		//t_us/1000 );
		
	/* wait here until txthread tells us there's a packet to transmit
	 * maximum wait <till the END of the slot>.
	 * then it returns 
	 * returns before if TExit_thrs goes up */
	ret = sem_timedwait_safe(
		&TDMA_Transmit_sem, "TDMA_Transmit_sem",
		&TExit_threads, t_us ); 
	//if ( E_TIMEDOUT == ret )
		//PRINTF_FL_WARN(
		//"TIMEOUT tdma found no pkts (%.3fms)\n" ,
		//tdma_getCurrentRoundTimeD() );
		
		
	if ( E_SUCCESS != ret)
		return ret;
	
	
	//int val2 =-1;
	//sem_getvalue(&TDMA_Transmit_sem, &val2);
	//PRINTF_FL_WARN("Found a pkt to send [now:TDMA_Transmit_sem %d]\n",val2);
	
	#else
	PRINTF_FL_WARN("EWaiting for pkt [TDMA_Transmit_sem = 0]\n");
	
	int ret ;
	semtrywait:
	ret = sem_wait_safe(
		&TDMA_Transmit_sem, 
		"TDMA_Transmit_sem", 
		&TExit_threads); 
	if (0 > ret) 
	{
		PRINTF_FL_ERR("erro\n");
		return ret;
	}
	
	int val2 =-1;
	sem_getvalue(&TDMA_Transmit_sem, &val2);
	PRINTF_FL_WARN("EFound a pkt to send [now:TDMA_Transmit_sem %d]\n",val2);
	#endif
	
	//int ret ;
	//semtrywait:
	//ret = sem_trywait(&TDMA_Transmit_sem); 
	//if (0>ret && EAGAIN==errno)
	//{
		//PRINTF_FL_WARN("sem is locked...\n");
		//sem_getvalue(&TDMA_Transmit_sem,&val2);
		//PRINTF_FL_WARN("sem has val %d!\n",val2);
		//usleep(10000);
		//goto semtrywait;
	//}
	//PRINTF_FL_WARN("sem returned %d!\n",ret);
	if (!tdma_insideSlot())
	{
		//PRINTF_FL(
			//"Found a pkt, on time, but still ... too late (%.3fms) "
			//"\n" ,
			//tdma_getCurrentRoundTimeD() );
		//return E_TIMEDOUT;
	}

	
	

	
	/* Grab packet ! lock queue and go : */
	if (0 != pthread_mutex_lock( &TDMA_tx_queue_mutex )) /* lock */
	{
		PRINTF_FL_ERR("mutex failed!\n ");
		return E_TERMINATING;
	}
		
	if ( TDMA_tx_queue_w_ptr == TDMA_tx_queue_r_ptr )
	{
		PRINTF_FL_ERR("not possible!\n");
		pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ; /* unlock */
		return E_OTHER ;  /* nothing to get */
	}
	// printf("TDMA:getOldest@ TDMA_tx_queue_r_ptr=%d , TDMA_tx_queue_w_ptr=%d\n" , TDMA_tx_queue_r_ptr, TDMA_tx_queue_w_ptr); fflush( stdout ) ;
	memcpy( 
		packet , 
		(void*)&TDMA_tx_queue[TDMA_tx_queue_r_ptr] , 
		sizeof( tdma_tx_queue_item_t ) );
	packet->payloadpkt_ptr = malloc( (size_t)packet->payloadpkt_len ) ;
	memcpy( 
		packet->payloadpkt_ptr , 
		TDMA_tx_queue[TDMA_tx_queue_r_ptr].payloadpkt_ptr , 
		(size_t)packet->payloadpkt_len ) ; /* copy payload from txqueue */
	
	/* clean the original element from the queue */
	free( TDMA_tx_queue[TDMA_tx_queue_r_ptr].payloadpkt_ptr ) ; /* delete item from txqueue */
	CLEAR( TDMA_tx_queue[TDMA_tx_queue_r_ptr] ) ;
	
	/* update reader pointer */
	TDMA_tx_queue_r_ptr = (TDMA_tx_queue_r_ptr+1) % TDMA_TX_QUEUE_SIZE ; 
	
	pthread_mutex_unlock( &TDMA_tx_queue_mutex ) ; /* unlock tx queue */
	

	/* inform "TDMA_send" there's a new empty spot in the queue */
	sem_post( &tx_queue_freespots_sem ) ;

	
	// //if by now, we are already outside the slot...let's wait ..
	//while ( !tdma_insideSlot( (uint16_t)tdma_getCurrentRoundTime() , TDMA_getOwnSlot() ) ) /*  if we are outside txslot ..then... just inform the user */
	//{
		//slot_limits_t tmp_s = TDMA_getOwnSlot();
		//PRINTF_FL_ERR( 
			//"t_r =%"PRIu16"ms, we are outside .our slot [%"PRIu16",%"PRIu16"]ms\n" , 
			//(uint16_t)tdma_getCurrentRoundTime() ,
			//tmp_s.begin_ms,
			//tmp_s.end_ms 
			//);
			//usleep( 1000 ) ;
	//}
	
	//PRINTF_FL("Packet copied\n");
	return E_SUCCESS ; /* success */
}

/* generate and return the new seq number for the flow [MYSELF]->[SINK NODE]*/
static uint32_t getTDMASeqNum( uint8_t dest_IP )
{
	if ( MAX_IP < dest_IP || 0 == dest_IP )
	{
		PRINTF_FL_ERR(
			"Invalid sink_ip %"PRIu8"\n",
			dest_IP ) ;
		return 0 ;
	}
	
	/* save this - increment every time */
	static uint32_t seq_num[MAX_IP+2] = {0} ; 
	return (++seq_num[dest_IP+1]) ;
}


/* generate and return the new seq number for the flow [MYSELF]->[wherever]
 * this is a counter of  sent packets */
static uint32_t getTDMASeqNumG(void)
{
	/* save this - increment every time */
	static uint32_t seq_num_gen = 0; 
	return (++seq_num_gen) ;
}

error_t rx_queue_savePacket(
	void 		*rx_tdmapkt_ptr, 
	uint16_t 	num_bytes_read, 
	int8_t 		other_IP,
	int 		overwrite_q )
{
	pthread_mutex_lock( &(TDMA_rx_queue_ptr.mutex) ) ;
	TDMA_rx_queue[TDMA_rx_queue_ptr.w].payloadpkt_len = 
		(uint16_t)(num_bytes_read-sizeof(tdma_header_t)) ; /* payload size */
	TDMA_rx_queue[TDMA_rx_queue_ptr.w].payloadpkt_ptr = 
		malloc( (size_t)(num_bytes_read - sizeof(tdma_header_t)) ) ; /* payload ptr */
	if (NULL == TDMA_rx_queue[TDMA_rx_queue_ptr.w].payloadpkt_ptr )
		return E_NO_MEM;
	memcpy( TDMA_rx_queue[TDMA_rx_queue_ptr.w].payloadpkt_ptr ,
		(char*)rx_tdmapkt_ptr + sizeof(tdma_header_t), 
		(size_t)(num_bytes_read - sizeof(tdma_header_t))   ) ; /* copy the packet, WITHOUT the header*/
	TDMA_rx_queue[TDMA_rx_queue_ptr.w].src_ip = other_IP ;
	TDMA_rx_queue_ptr.w =( TDMA_rx_queue_ptr.w+1) % TDMA_RX_QUEUE_SIZE ;
	#ifdef ALWAYS_ACCEPT 
	if ( 1 == overwrite_q )
		TDMA_rx_queue_ptr.r = (TDMA_rx_queue_ptr.r+1) % TDMA_RX_QUEUE_SIZE ; /* ignore current R packet */
	#endif 
	pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex) ) ;
	
	return E_SUCCESS;
}

/** process incoming tdma pure packet, 
 * check if it can saved and save it into the Q */
error_t parserPurePkt( 
	void 		*rx_tdmapkt_ptr, 
	uint16_t 	num_bytes_read, 
	uint8_t 	other_IP )
{
		
	
	int overwrite_q = 0 ;
	#if ALWAYS_ACCEPT == 0
	/* sleep here until there's queue space */
	{
	error_t ret = rx_queue_waitForFreeSpots();
	if (0>ret)
		return ret;
	}
	
	/* can we save this packet ? check if buffer is full*/
	while ( 1 == TDMA_isRxBufferFull() )
	{
		microsleep(1000);
		PRINTF_FL_WARN("SPINNING!THIS should never happen\n");
	}
	#else
	if ( 1 == TDMA_isRxBufferFull() )
	{
		overwrite_q = 1 ;
		//PRINTF_FL_WARN("overwrittin\n");
	}
	#endif
	
	
	/* save PEELED packet into RX queue .
	 * if full, overwrite queue */
	{
	error_t ret = rx_queue_savePacket(
		rx_tdmapkt_ptr, num_bytes_read , other_IP, overwrite_q );
	if (0 > ret)
		return ret;
	}
	
	
	/* update stats */
	#define ts Tdma_statistics
		
	const tdma_header_t *h_p = (tdma_header_t*)rx_tdmapkt_ptr ;
	
	/* compute position of packet within the senders-slot */
	const int64_t mpos = tdma_getMsgPosition(*h_p);
	/* save when we got our last packet 
	 * do not further update if we got a pkt from a new slot.
     * wait for reset */
	if ( mpos > ts.last_rx_pkt_pos_ms[other_IP])
	{
		/* keep track of received data */
		ts.last_rx_pkt_pos_ms[other_IP] = (uint16_t)mpos ;
		ts.round_bytes_rcvd[other_IP] += num_bytes_read ;
		//tdma_printBin(0);
	}

	
		
		
	//PRINTF_FL("use %" PRIu16 "pkts\n" , tdma_getRxBufferUse( ) ) ;
	

	
	/* let know TDMA_receive we got a new pkt.
	 * or not, if new pkt overwrote another one. */
	#if ALWAYS_ACCEPT == 1
	 if ( 0 == overwrite_q )
	#endif
	 if ( 0 != sem_post( &Receive_sem ) )
	 {
		PRINTF_FL_ERR("Sem post failed\n");
		return E_SEMAPHORE ;
	 }
	//{int val ;
	//sem_getvalue( &Receive_sem, &val );
	//PRINTF_FL_WARN("saved (received pkts: %d) - rxqueue %d\n", 
		//val,
		//tdma_getRxBufferUse() );}
	return E_SUCCESS ;
}

error_t parserPmPkt( 
	void 		*rx_tdmapkt_ptr, 
	uint16_t 	num_bytes_read, 
	uint8_t 	other_IP )
{
	
	/* check if:
	 * 1)it is a HIGH-Priority and to continue along the network => FWD RIGHT NOW!
	 * otherwise, save to Q to be fwd later on.
	 * */
	
	/* get pointer to rcvd TDMA header */
	tdma_header_t *tdma_hdr_ptr = (tdma_header_t*)rx_tdmapkt_ptr;
	
	uint16_t hi_prio = tdma_getPrio(tdma_hdr_ptr);

	if (hi_prio) /* 1) - we are hi-prio */
	{
		
			
		pm_header_t *pm_hdr = (pm_header_t*)
			( (char*)rx_tdmapkt_ptr + sizeof(tdma_header_t) );
		uint8_t sink_ip = pm_hdr->sink;
		
		/* 2) we are not the final dest .*/
		if ((int16_t)sink_ip != getMyIP())
		{
			PRINTF_FL_WARN(
				"parsing a foreign %s PMpkt - %"PRIu16" \n",
				(hi_prio) ? "hiprio" : "normal",
				tdma_hdr_ptr->type );
			PRINTF_FL_WARN("*FWd ASAP* to #%"PRIu8" (sink #%"PRIu8")!\n",
				PM_getNextHop( sink_ip ),
				sink_ip );
							
			tdma_tx_queue_item_t item_packet;
			item_packet.timestamp = getEpoch();
			item_packet.dest_ip = PM_getNextHop( sink_ip ) ;
			item_packet.type = tdma_hdr_ptr->type ;
			
			item_packet.payloadpkt_len = num_bytes_read - (uint16_t)sizeof(tdma_header_t);
			item_packet.payloadpkt_ptr = malloc((size_t)item_packet.payloadpkt_len) ; 
			memcpy( item_packet.payloadpkt_ptr, pm_hdr, item_packet.payloadpkt_len );
		
			int tdma_pck_len = build_send_log( &item_packet );
			//if (0 > tdma_pck_len)
			return tdma_pck_len; /* always return. all done */
		}
	}
	
	
	
	
	int overwrite_q = 0 ;
	#ifndef ALWAYS_ACCEPT
	/* sleep here until there's queue space */
	{
	error_t ret = rx_queue_waitForFreeSpots();
	if (0>ret)
		return ret;
	}
	
	/* can we save this packet ? check if buffer is full*/
	while ( 1 == TDMA_isRxBufferFull() )
	{
		microsleep(1000);
		PRINTF_FL_WARN("SPINNING!THIS should never happen\n");
	}
	#else
	if ( 1 == TDMA_isRxBufferFull() )
	{
		overwrite_q = 1 ;
		//PRINTF_FL_WARN("overwrittin\n");
	}
	#endif
	
	
	/* save PEELED packet into RX queue .
	 * if full, overwrite queue */
	{
	error_t ret = rx_queue_savePacket(
		rx_tdmapkt_ptr, num_bytes_read , other_IP, overwrite_q );
	if (0 > ret)
		return ret;
	}
	
	
	/* update stats */
	#define ts Tdma_statistics
		
	const tdma_header_t *h_p = (tdma_header_t*)rx_tdmapkt_ptr ;
	
	/* compute position of packet within the senders-slot */
	const int64_t mpos = tdma_getMsgPosition(*h_p);
	/* save when we got our last packet 
	 * do not further update if we got a pkt from a new slot.
     * wait for reset */
	if ( mpos > ts.last_rx_pkt_pos_ms[other_IP])
	{
		/* keep track of received data */
		ts.last_rx_pkt_pos_ms[other_IP] = (uint16_t)mpos ;
		ts.round_bytes_rcvd[other_IP] += num_bytes_read ;
		//tdma_printBin(0);
	}

	
		
		
	//PRINTF_FL("use %" PRIu16 "pkts\n" , tdma_getRxBufferUse( ) ) ;
	

	
	/* let know TDMA_receive we got a new pkt.
	 * or not, if new pkt overwrote another one. */
	#ifdef ALWAYS_ACCEPT
	 if ( 0 == overwrite_q )
	#endif
	 if ( 0 != sem_post( &Receive_sem ) )
	 {
		PRINTF_FL_ERR("Sem post failed\n");
		return E_SEMAPHORE ;
	 }
	//{int val ;
	//sem_getvalue( &Receive_sem, &val );
	//PRINTF_FL_WARN("saved (received pkts: %d) - rxqueue %d\n", 
		//val,
		//tdma_getRxBufferUse() );}
	return E_SUCCESS ;
}


/*******************
 * QUEUE RELATED
 * *****************/
/* protected - number of packets to be read */
uint16_t tdma_getRxBufferUse( void )
{
	pthread_mutex_lock( &(TDMA_rx_queue_ptr.mutex) ); /* lock */
	int32_t buff_use = (int32_t)TDMA_rx_queue_ptr.w - (int32_t)TDMA_rx_queue_ptr.r ;
	if ( 0 > buff_use )
		buff_use = 
			( TDMA_RX_QUEUE_SIZE - (int32_t)TDMA_rx_queue_ptr.r ) +
			(int32_t)TDMA_rx_queue_ptr.w ;
	pthread_mutex_unlock( &(TDMA_rx_queue_ptr.mutex) ); /* unlock */

	return (uint16_t)buff_use;
}


/*  protected - number of packets to be sent */
uint16_t tdma_getTxBufferUse( void )
{
	pthread_mutex_lock( &TDMA_tx_queue_mutex ); /* lock */
	int32_t buff_use = (int32_t)TDMA_tx_queue_w_ptr - (int32_t)TDMA_tx_queue_r_ptr ;
	if ( buff_use < 0 )
		buff_use = ( TDMA_TX_QUEUE_SIZE - (int32_t)TDMA_tx_queue_r_ptr ) + 
			(int32_t)TDMA_tx_queue_w_ptr ;
	pthread_mutex_unlock( &TDMA_tx_queue_mutex ); /* unlock */
	
	return (uint16_t)buff_use;
}


error_t rx_queue_waitForFreeSpots(void)
{
	
	int ret ;
	/* sleep thread until free spot in rx queue */
	ret = sem_wait_safe(
		&rx_queue_freespots_sem, 
		"rx_queue_freespots_sem", 
		&TExit_threads); 
	if (0 > ret) 
		return ret ;

	
	
	return E_SUCCESS ;
}
	
#undef TDMA_IMPORT
