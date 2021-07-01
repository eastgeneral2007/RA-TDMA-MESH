/*******************************************
 ***************PACKAGE MANAGER MODULE **********
*******************************************/
#define PM_IMPORT

#include "drk/packet_manager.h" /* self */
#include "drk/tdma.h" /* tdma_receive , tdma_send */
#include "drk/tdma_utils.h" /* tdma_getbandwidth */
#include "drk/utils.h"
#include "drk/drk.h"
#include "drk/error.h"


#include <inttypes.h> /* int8_t etc */
#include <stdio.h> // printf
#include <stdlib.h> /* atoi*/
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <linux/sockios.h> // SIOCOUTQ
#include <sys/ioctl.h> // SIOCOUTQ
#include <pthread.h> // we have threads
#include <unistd.h> // sleep
#include <string.h> // memset, memcpy
#include <errno.h> // strerror
#include <sys/wait.h> // waitpid
#include <math.h> /* some math at some point */
#include <inttypes.h>	/* int8_t etc */
#include <netinet/in.h> // udp packets







/*******************
 * Evil extern globals 
 * *******************/
//extern uint8_t 	global_exit_all_threads ; /* volatile - variable *accessed* by multiple threads*/ 



/************************
 *  Evil static GLOBALS *
 * ***********************/
static volatile int		 		PExit_threads = 0 ; /* flag if is time to close this module */
static volatile Param_pm_t		Param_pm ;
static pthread_t 				Thread_tids[2] ; /* thread ids : tx and rx */
static int 						PM_initiated = 0 ; /* module initiated ? block all ops while not initiated */
static sem_t 					Receive_sem ;
static int 						N_route_entries = 0 ;

/** tx queue stuff **/
static pthread_mutex_t 				PM_tx_queue_mutex = PTHREAD_MUTEX_INITIALIZER ; /* mutex to read/write to the queue , multiple threads need access to it */
static volatile pm_tx_queue_item_t 	tx_queue[PM_TX_QUEUE_SIZE] ; /* the queue */ 
static volatile uint16_t 			PM_tx_queue_w_ptr , PM_tx_queue_r_ptr ; /* pointers to r+w */
static sem_t 						Transmit_sem ;
static sem_t 						PM_tx_queue_sem ;

/** rx queue stuff **/
static pthread_mutex_t 				rx_queue_mutex = PTHREAD_MUTEX_INITIALIZER ; /* mutex to read/write to the queue , multiple threads need access to it */
static volatile pm_rx_queue_item_t 	rx_queue[PM_RX_QUEUE_SIZE] ; /* the queue */ 
static volatile uint16_t			PM_rx_queue_w_ptr ;
static volatile uint16_t 			PM_rx_queue_r_ptr ; /* pointers to r+w */

/* Flow stuff */
static uint8_t 	 					ip_dw = 0 , ip_up = 0 ;







/****************************************************************** 
 *********************** CODE ** STARTS ** HERE 
 * ***************************************************************/

/* 
 * initiate this module 
 * */
error_t drk_PM_init( uint8_t my_ip )
{
	//block_all_signals() ;
	unblock_one_signal(SIGUSR1);
	//sig_set_action( NULL, SIGUSR1);
	
	if ( loadSettings( my_ip ) < 0 ) 
	{
		PRINTF_FL_ERR( "Failed to load settings\n" ) ;
		return -1 ;
	}


	if ( initQueue() < 0 ) /* init tx queue */
	{
		PRINTF_FL_ERR( "Failed to init tx queue \n" ) ;
		return -2;
	}
	

	if ( initThreads() < 0 ) 
	{
		PRINTF_FL_ERR( "Failed to init threads\n" ) ;
		return -3 ;
	}

	PM_initiated = 1 ;
	return E_SUCCESS ; /* all good */
}

/** close it properly **/
error_t drk_PM_close(void)
{
	PM_LOADED_CONDITION E_NO_INIT ; 
		
	PExit_threads = 1 ;
	PM_initiated = 0 ;
	//PRINTF_FL("[PM] sending signals \n");
	while (0==pthread_kill( Thread_tids[0] , SIGUSR1 ) ) {microsleep(1000);}
	while (0==pthread_kill( Thread_tids[1] , SIGUSR1 ) ) {microsleep(1000);} 
	//PRINTF_FL("[PM] joining \n");
	pthread_join( Thread_tids[0] , NULL ) ; 
	pthread_join( Thread_tids[1] , NULL ) ; 
	
	PRINTF_FL("[PM] Closed\n");
	
	return E_SUCCESS ; 
}

uint8_t PM_getNextHop( uint8_t sink_ip ) 
{
	
	for (uint8_t i=0 ; i<N_route_entries ; i++ )
		if ( Param_pm.sink_IP[i]  == sink_ip )
			return Param_pm.dest_IP[i] ;
	
	return 0 ;
	
}

/** set the new routing table **/
error_t PM_newTopology(pair_ips_t const topology[], int entries ) 
{
	PM_LOADED_CONDITION		E_NO_INIT ;
	/* Topology- x y ; h i ; etc.. */
	/* pairs of (final_src ; nxt_hop) */
	int i ;
	for ( i = 0 ; (i < entries) && (i<=MAX_ROUTING_LINES-1) ; i++ )
	{
		Param_pm.sink_IP[i] = topology[i].sink_IP;
		Param_pm.dest_IP[i] = topology[i].dst_IP;
	}
	N_route_entries = entries ;
	return PM_printRoutingSettings() ;
}

/** 
 * APPlication layer uses this to send new packets.
 * - sink_IP can be several hops away. 
 * - PM eventually gives the packet to TDMA module,
 * - pkt_ptr is copied internally. caller can reuse it.
 * - pkt_len must be leq than MAX_APP_PACKET_SIZE
 **/
error_t PM_send( 
	uint8_t sink_IP, 
	uint16_t pkt_len, 
	const void * const pkt_ptr,
	int hi_prio )
{
	//dumpData((uint8_t*)pkt_ptr , pkt_len );
	PM_LOADED_CONDITION		E_NO_INIT ;
		
	
	/* check destination before anything else */
	if ( 0 == PM_getNextHop( sink_IP ) )
		return E_ROUTE_UNKNOWN ;
	//PRINTF_FL_WARN("Sink #%"PRIu8". Send to %"PRIu8".\n",
		//sink_IP,
		//PM_getNextHop( sink_IP ));
	if (0 == sink_IP )
	{
		PRINTF_FL_ERR("Trying to send a packet to invalid node\n");
		return E_INVALID_INPUT ;
	}
	
	/* check packet size */
	if (MAX_APP_PACKET_SIZE < pkt_len)
	{
		PRINTF_FL_ERR("Packet is bigger than %dB\n",
			MAX_APP_PACKET_SIZE);
		return E_INVALID_INPUT ;
	}
	/** 
	 * do we have free spots to add this packet? 
	 * if not, wait FOREVER...! except:
	 * PExit_thread goes up or EINTR.
	 * txThread will tell us smth eventually 
	 * **/
	int ret ;
	ret = sem_wait_safe(
		&PM_tx_queue_sem, 
		"PM_tx_queue_sem", 
		&PExit_threads); /* sleep thread until free spot - dec value */
	if (0 > ret)
		return ret ;
	
	//PRINTF_FL_WARN("PM has free space\n");
	/* Ready to read and write into txqueue . LOCK */
	my_mutex_lock( &PM_tx_queue_mutex ); 
	
	/* try to move the write pointer */
	if ( 0 > moveWPtr( &PM_tx_queue_w_ptr, &PM_tx_queue_r_ptr, PM_TX_QUEUE_SIZE , 0 ) )
	{
		//PRINTF_FL("Can't add new packet => full queue\n");
		//PRINTF_FL("unlocked\n");
		pthread_mutex_unlock( &PM_tx_queue_mutex ) ;
		return E_FULL_QUEUE ; /* full queue - return  */
	}
	
	//PRINTF_FL_WARN("PM pointer moved \n");
	/* build PM item */
	pm_tx_queue_item_t new_item ; 
	new_item.next_ip 	= PM_getNextHop( sink_IP ) ;
	new_item.prev_ip 	= 0 ; /* there was no prev_hop */
	new_item.timestamp 	= getEpoch()  ;
	new_item.pkt_len 	= pkt_len + (uint16_t)sizeof( pm_header_t ) ;
	new_item.pkt_ptr 	= malloc( (size_t)pkt_len + (size_t)sizeof( pm_header_t ) ) ;
	
	if ( NULL == new_item.pkt_ptr )
	{
		PRINTF_FL_ERR( "Failed to add pkt to queue. No memory available\n" );
		my_mutex_unlock( &PM_tx_queue_mutex ); /* lock */
		return E_NO_MEM ; /* failed to add pkt to queue. No memory available */
	}
	
	//PRINTF_FL_WARN("PM is adding pkt\n");
	/* all ok to add new packet...  */
	
	/* fill PM header */
	((pm_header_t*)new_item.pkt_ptr)->or_src 	= getMyIP() ;
	((pm_header_t*)new_item.pkt_ptr)->sink 		= sink_IP ;
	((pm_header_t*)new_item.pkt_ptr)->seq_num 	= getSeqNum( sink_IP ) ; /* every packet generated to a given sink, has a unique seq num */
	/* (or_src & sink & seq_num) is unique triple */
	
	((pm_header_t*)new_item.pkt_ptr)->need_ack 	= 0 ; /* no ack for now. type <PYLD> pkts should have it on. <mgt>? 0 */
	((pm_header_t*)new_item.pkt_ptr)->type 		= (hi_prio<<8) | APP ; // make it HI-prio, besides APP
	
	/* paste the payload */
	memcpy( (char*)new_item.pkt_ptr + sizeof(pm_header_t) , 
		pkt_ptr, pkt_len ) ;
	tx_queue[PM_tx_queue_w_ptr] = new_item ;
	pthread_mutex_unlock( &PM_tx_queue_mutex ) ;
	
	
	//PRINTF_FL_WARN("a\n");semaphoreDebugger(0); /* always inconsitent! MORE IN QUEUE ,. NOT POSTED YET */
	
	/* inform getOldestPMPkt() that there is a new packet to transmit!! */
	sem_post(&Transmit_sem); 
	//int val =-1;
	//sem_getvalue(&Transmit_sem,&val);
	//PRINTF_FL_WARN("posted - val %d!\n",val);
	semaphoreDebugger(0);

	
	return E_SUCCESS ; /* done -> ok */

}

/* print current routing table */
error_t PM_printRoutingSettings(void)
{
		
	for ( uint8_t i = 0 ; i < N_route_entries ; i++ )
		if ( 0 != Param_pm.sink_IP[i] ) 
		{
			PRINTF_FL(
				"Destination ["BASEIP"%"PRIu8"]? --> "
				"Send to ["BASEIP"%"PRIu8"]\n", 
				Param_pm.sink_IP[i] , 
				Param_pm.dest_IP[i] ) ;
		}

	return E_SUCCESS ;
}

/** ratio of usage (0-1) **/
float PM_getRxBufferUse( void )
{
	if ( PM_initiated == 0 )
		return E_NO_INIT ;
	return 1.0*getRxBufferUse() ;
}


/** NUMBER of packets inqueue **/
/** to be used by TDMA or others **/
float PM_getTxBufferUse( void )
{
	PM_LOADED_CONDITION E_NO_INIT ;
	
	return 1.0*getTxBufferUse() ;
}

/** **/
error_t PM_registerFlow( uint8_t or_src, uint8_t sink )
{
	if ( PM_initiated == 0 )
		return E_NO_INIT ;

	ip_dw = PM_getNextHop( sink ) ;
	ip_up = PM_getNextHop( or_src ) ;
	if ( ip_up == 0 && ip_dw == 0 ) 
		return E_ROUTE_UNKNOWN ;
	
	PRINTF_FL( "UP %" PRIu8 ", DW %" PRIu8 "\n", ip_up, ip_dw );
	return E_SUCCESS ;

}


/** for upper layers to read a packet in our Rx queue **/
error_t PM_receive( pm_pkt_t *pkt , uint8_t blocking ) 
{
		
	if ( 0 == PM_initiated )
	{
		PRINTF_FL_ERR( "Call PM_init() first\n" );
		return E_NO_INIT ;
	}

	if ( pkt == NULL || pkt->payload_ptr == NULL )
	{
		PRINTF_FL_ERR( "Provided null pointer(s) \n" );
		return E_INVALID_INPUT ;
	}
	
	while ( !PExit_threads )
	{
		my_mutex_lock( &rx_queue_mutex ); /* to access the queue */
		if ( PM_rx_queue_w_ptr == PM_rx_queue_r_ptr ) /* queue is empty */
		{	
			my_mutex_unlock( &rx_queue_mutex ) ; /* free the queue */
			
			if ( blocking == 0 ) 
			{
				PRINTF_FL_WARN("nothing found, leave now\n");
				return 0 ;  /* nothing found, leave */
			}
			
			
			
			/* empty queue .  */
			/* wait here until rxThread receives a packet */
			int ret ;
			ret = sem_wait_safe(
				&Receive_sem, 
				"PM Receive_sem", 
				&PExit_threads ); 
			if (0 > ret) 
				return ret ;
			
			continue; /* goback up */
			
			//PRINTF_FL("Got a pkt at pm rxthread\n" );

		}
		
		
		PM_rx_queue_r_ptr = (PM_rx_queue_r_ptr + 1 ) % PM_RX_QUEUE_SIZE ; /* update reader pointer */
		
		/* some error checking */
		if ( rx_queue[PM_rx_queue_r_ptr].pkt_ptr == NULL )
		{
			PRINTF_FL_ERR( "Reading a null pointed packet!\n");
			PRINTF_FL_ERR( "R %"PRIu16", W %"PRIu16 "\n" , 
				PM_rx_queue_r_ptr ,
				PM_rx_queue_w_ptr ) ;
			dumpRxQueue();
			exit(1);
		}
			
			
	
		
		size_t app_pkt_len = (size_t)(rx_queue[PM_rx_queue_r_ptr].pkt_len) - (size_t)sizeof(pm_header_t) ;
		void *app_pkt_ptr = (void*)(((char*)rx_queue[PM_rx_queue_r_ptr].pkt_ptr) + sizeof(pm_header_t)) ;
		memcpy( 
			pkt->payload_ptr ,
			app_pkt_ptr , 
			app_pkt_len ) ; /* copy the payload only, to upperlayers */
		pkt->or_src = ((pm_header_t*)rx_queue[PM_rx_queue_r_ptr].pkt_ptr)->or_src ; /* return the original source */ 
		pkt->dst = getMyIP() ;
		pkt->size = app_pkt_len ;

			
			
		#ifdef DUMP
		//PRINTF_FL("Got a pkt. Reading at %"PRIu16" (W %"PRIu16 ").\n" , 
				//PM_rx_queue_r_ptr ,
				//PM_rx_queue_w_ptr ) ;
		//dumpRxQueue();
		#endif
		
		int ret = deleteRxElement( ) ;
		my_mutex_unlock( &rx_queue_mutex ) ;
		if ( ret < 0 ) 
		{
			PRINTF_FL_ERR("Error at PM_receive\n");
			return ret ; /* report that pm_Receive failed */
		}
		
		return E_SUCCESS ; /* leave normally */
		

		/* if we are here, it means caller wants a blocking call - we only leave when a new packet is present  */
		//PRINTF_FL("---\n" ) ;
		//microsleep(1000);
		
	} /* end of while ( !exit_threads ) */
	return E_TERMINATING ;
}

/* send a string to another node within a PM packet */
void sendString(char *instring, uint8_t dst)
{
	string_pkt_t spkt;
	spkt.type = STRING ;
	strcpy(spkt.string, instring);
	/* packetsize is type field plus string size + nullchar. no need to send MTU */
	PM_send(
		dst, /* destination */
		sizeof(spkt.type) + strlen( spkt.string )+1 , /*pkt len */
		(void*)&spkt , 1) ;
}

#if defined(VERBOSE) 
error_t parserString( pm_pkt_t pkt ) 
{
	
	PRINTF_FL( 
		"#%"PRIu8": "KRED"<%.150s>"KDFL"\n" ,
		pkt.or_src,
		((string_pkt_t *)(pkt.payload_ptr))->string 
		) ;
	
	return E_SUCCESS ;
}
#endif

/*******************************************************
 * PRIVATE - internal functions
 * ***********************************************************/
/** init rx and tx threads - PM layer **/
error_t initThreads( void )
{
	/* create thread attributes : stack size */
	size_t desired_stack_size = 400000 ; // thread stack size 400KB
	pthread_attr_t thread_attr;
	pthread_attr_init( &thread_attr );
	pthread_attr_setstacksize( &thread_attr, desired_stack_size ) ;

	/* Create thread that will continuously send messages from other nodes */
	if ( pthread_create( &Thread_tids[0] , 
		&thread_attr, &txThread , NULL) != 0 )
	{
		PRINTF_FL_ERR("[Tx Thread] - Error spawning (%s)\n", 
			strerror(errno) );
		return -1 ;
	}
	//PRINTF_FL("[Tx Thread] created with success\n");
    
    /* Create thread that will 
     * continuously receive messages from other nodes */
	if ( pthread_create( &Thread_tids[1] , 
		&thread_attr, &rxThread , NULL) != 0 )
	{
		PRINTF_FL_ERR("[RX Thread] - Error spawning (%s)\n",
			strerror(errno));
		return -2 ;

	}
	//PRINTF_FL("[RX Thread] created with success\n");
	
	if ( 0 != pthread_attr_destroy( &thread_attr ) )
		return -3 ;

	/* wait for the threads to enter their cycles */
	microsleep(10000) ;

	
	/* if threads are already dead, lets join and return -1 */
	if ( PExit_threads ) 
	{
		/* wait here until all app threads terminate! */
		for ( int j = 0 ; j <= 1 ; j++ )
			pthread_join( Thread_tids[j] , NULL ) ;
		return -1 ;
	}
	return E_SUCCESS ;
}

/** init the queue with zeros **/
error_t initQueue()
{
	CLEAR ( tx_queue ) ;
	PM_tx_queue_r_ptr = 0 ; 
	PM_tx_queue_w_ptr = 0 ;
	/* initiate semaphores */
	if ( sem_init(&Transmit_sem,0,0) < 0 ) /* used to communicate when there pkts to send in tx_queue */
	{
		PRINTF_FL_ERR("TraSem init: %s ", strerror( errno ) ) ;
		return E_SEMAPHORE;
	}
	if ( sem_init(&PM_tx_queue_sem,0,PM_TX_QUEUE_SIZE-1) < 0 ) /* used to communicate when tx_queue has space */
	{
		PRINTF_FL_ERR("tx_qSem init: %s ", strerror( errno ) ) ;
		return E_SEMAPHORE;
	}
	
	
	//PRINTF_FL_WARN("INIT\n");
	semaphoreDebugger(0);
	
	
	
	CLEAR ( rx_queue ) ;
	PM_rx_queue_r_ptr = 0 ; 
	PM_rx_queue_w_ptr = 0 ;
	if (sem_init(&Receive_sem, 0, 0) < 0)
	{
		PRINTF_FL_ERR("Sem init %s ", strerror( errno ) ) ;
		return E_SEMAPHORE;
	}

	
	//TODO: malloc queue instead ? unless we need dynamic space ..
	return E_SUCCESS ; /* ok */
}


/** load all stuff network related : throughput, and slots **/
error_t loadSettings( const uint8_t my_ip ) 
{
	Param_pm.node_ip = my_ip ;
	return loadRouteSettings() ;
}

/** set route configs **/
error_t loadRouteSettings( void )
{

	//PRINTF_FL_WARN("entering route settngs\n");
	
	FILE *another_file = fopen(FILENAME_ROUTE,"r") ;
	if (another_file == NULL)
	{
		PRINTF_FL_ERR( "Where is %s ?\n", FILENAME_ROUTE ) ;
		return E_FILE_NOT_FOUND ;
	}
	
	char buf[20] ; /* load a whole line from file */
	CLEAR(buf);
	N_route_entries = 0 ;
	while ( NULL != fgets( buf , sizeof(buf) , another_file) ) /* read one line */
	{
		if (
		( 2 != sscanf( buf , "%" SCNu8 " %" SCNu8 "", 
			&(Param_pm.sink_IP[N_route_entries]) , 
			&(Param_pm.dest_IP[N_route_entries]) ) ) | /* parse line's data */
		(0 == Param_pm.sink_IP[N_route_entries]) | 
		(0 == Param_pm.dest_IP[N_route_entries]) 
		)
		{
			PRINTF_FL_ERR(
				"Confirm you have 2 integers per line in %s\n" , 
				FILENAME_ROUTE ) ;
			PRINTF_FL_ERR(
				"Current read: "
				"[sink #%" PRIu8 "--> dest #%" PRIu8 "]\n", 
				Param_pm.sink_IP[N_route_entries]  , 
				Param_pm.dest_IP[N_route_entries] ) ;
			return E_INVALID_INPUT ;
		}
	
		N_route_entries++ ;
	}
	fclose(another_file) ;
	PM_printRoutingSettings();
	PRINTF_FL("leaving route settngs w/ entries %d\n", N_route_entries );

	return E_SUCCESS ;
}







error_t parserAppPkt( void* recv_pmpkt_ptr , uint16_t num_bytes_read , uint8_t other )
{

	/* save this packet to the local queue 
	 * for APP to read later on */
	(void)other; /* where this packet just came from */
	 
	/* can we save this packet ? wait till buffer is ok*/
	my_mutex_lock( &rx_queue_mutex ) ;
	while ( moveWPtr( &PM_rx_queue_w_ptr , &PM_rx_queue_r_ptr , PM_RX_QUEUE_SIZE , 1 ) < 0 )
	{	
		PRINTF_FL_ERR( "rx PM is full\n");
		PRINTF_FL_ERR( "( w_ptr=%" PRIu16 " /\\ r_ptr=%" PRIu16 " : size %u ) \n" ,
			PM_rx_queue_w_ptr, PM_rx_queue_r_ptr , PM_RX_QUEUE_SIZE ) ;
		

		my_mutex_unlock( &rx_queue_mutex ) ;
		PRINTF_FL_WARN("SPINNING\n");
		microsleep(2000);
		my_mutex_lock( &rx_queue_mutex ) ;

	}

	/* we have space:  save peeled packet into RX queue */
	rx_queue[PM_rx_queue_w_ptr].pkt_len = (uint16_t) num_bytes_read ;
	rx_queue[PM_rx_queue_w_ptr].pkt_ptr = malloc( (size_t)num_bytes_read ) ;
	if (rx_queue[PM_rx_queue_w_ptr].pkt_ptr == NULL )
	{
		my_mutex_unlock( &rx_queue_mutex ) ; /* unlock */
		return E_NO_MEM ; 
	}
	memcpy( 
		rx_queue[PM_rx_queue_w_ptr].pkt_ptr , 
		recv_pmpkt_ptr , 
		(size_t)num_bytes_read ) ; /* copy the packet */

	/* debug */
	//PRINTF_FL(
		//"Posted a pkt at w_ptr %"PRIu16", "
		//"from #%" PRIu8 ", Sink #%" PRIu8 ", %zd B\n" , 
		//PM_rx_queue_w_ptr ,
		//other ,
		//((pm_header_t*)recv_pmpkt_ptr)->sink , 
		//num_bytes_read ) ;
	my_mutex_unlock( &rx_queue_mutex ) ;
	
	
	sem_post( &Receive_sem ) ; /* let PM_receive() know that we have a pckt! */

	return E_SUCCESS ;
}

/* check if header makes sense */
//error_t checkPMerrors2( const pm_header_t* const recv_pmpkt_ptr ) 
//{
	
	//uint32_t snum = recv_pmpkt_ptr->seq_num % 
		//( (uint32_t)recv_pmpkt_ptr->or_src*1000000 ) ;
	
	//if ( snum > MAX_SEQ_NUM )
	//{
		//PRINTF_FL_ERR(
			//"Invalid PM header <Src %"PRIu8" , Snum = %"PRIu64 "> (snum %"PRIu64")\n",
			//recv_pmpkt_ptr->or_src, 
			//recv_pmpkt_ptr->seq_num ,
			//snum) ;
		//return E_INVALID_FORMAT ;
	//}
	
	//return E_SUCCESS ;
//}



error_t checkPMerrors( ssize_t num_bytes_read , uint8_t other_ip )
{
	/* do not show as error if terminating , everytng else, do */
	if ( E_TERMINATING == num_bytes_read ) 
		return E_TERMINATING;
		
	if ( E_TERMINATING > num_bytes_read ) 
	{
		PRINTF_FL_WARN( "TDMA_receive() failed\n") ; 
		return num_bytes_read ;
	}

	if ( num_bytes_read == 0 )
	{
		/* if we are doing a BLOCKING call and got a 0B payload , 
		 * this should not happen */
		PRINTF_FL_ERR( "There are no messages\n") ; 
		return 0 ;
	}

	if ( (uint16_t)num_bytes_read < sizeof(pm_header_t) )
	{
		PRINTF_FL_ERR( 
			"Received an invalid PM packet - total %zdB."
			" PM header has %zdB\n",
			num_bytes_read,
			sizeof(pm_header_t) );
		return 0 ;
	}

	if ( (uint16_t)num_bytes_read == sizeof(pm_header_t) )
	{
		PRINTF_FL( "Empty PM packet\n" ) ;
		return 0 ;
	}

	if ( 
		(other_ip > MAX_IP) || 
		(other_ip == 0 ) || 
		0//(other_ip == (uint8_t)getMyIP() )
		)
	{
		PRINTF_FL_ERR(
			"Got a packet from an unexpected source (#%"PRIu8")\n", 
			other_ip) ;
		return 0 ;
	}
	
	return E_SUCCESS ;
}

error_t addPMpacketTxQueue( 
	void *recv_pmpkt_ptr , 
	ssize_t num_bytes_read , 
	uint8_t other )
{
	/* error checking */
	uint8_t next_hop = 
		PM_getNextHop( ((pm_header_t*)recv_pmpkt_ptr)->sink  ) ;
	if ( next_hop == 0 )
	{
		PRINTF_FL_ERR( 
			"I dont know where to send this packet."
			"Sink: %" PRIu8 ". "
			"Rcvd from: %"PRIu8". "
			"%zdB "
			"No route.\n", 
			((pm_header_t*)recv_pmpkt_ptr)->sink,
			other,
			num_bytes_read );
		return E_ROUTE_UNKNOWN;
	}
	
	/* check if there's space in the tx_queue */
	/* sleep thread until free spot - decrease value */
	int ret ;
	ret = sem_wait_safe(&PM_tx_queue_sem, "PM_tx_queue_sem", &PExit_threads); 
	if (0 > ret) 
		return ret ;
	
	
	
	
	my_mutex_lock( &PM_tx_queue_mutex ); /* lock */
	#define OVRWRITE 1
	while 	( moveWPtr( 
				&PM_tx_queue_w_ptr , &PM_tx_queue_r_ptr , 
				PM_TX_QUEUE_SIZE , OVRWRITE ) < 0 
			)
	{	
		my_mutex_unlock( &PM_tx_queue_mutex ) ;
		PRINTF_FL_ERR( "Can't add rcvd packet to be relayed. Full txQueue. SPINNING \n" );
		PRINTF_FL_ERR( "SHOULD NOT HAPPEN since we have a sem_wait\n" );
		semaphoreDebugger(1);
		
		microsleep(10000);
		my_mutex_lock( &PM_tx_queue_mutex ); /* lock */
	}
	#undef OVRWRITE
	
	/* Save rcv pkt into the outgoing queue, to be relayed */
	struct timespec time_temporary ;
	getCurrentTime( &time_temporary) ; 
	tx_queue[PM_tx_queue_w_ptr].timestamp  =
		(double)( time_temporary.tv_sec )  + (double)( time_temporary.tv_nsec )/BILLION  ;
	tx_queue[PM_tx_queue_w_ptr].prev_ip = other ;
	tx_queue[PM_tx_queue_w_ptr].next_ip = next_hop ;
	tx_queue[PM_tx_queue_w_ptr].pkt_len = (uint16_t) num_bytes_read  ;
	tx_queue[PM_tx_queue_w_ptr].pkt_ptr = malloc( (size_t)num_bytes_read ) ;
	if ( tx_queue[PM_tx_queue_w_ptr].pkt_ptr == NULL )
	{
		my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
		return E_NO_MEM ;
	}
	memcpy( tx_queue[PM_tx_queue_w_ptr].pkt_ptr , 
		recv_pmpkt_ptr , 
		(size_t)num_bytes_read ) ; /* copy the whole thing , with PM-header */
	my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
	
	
	
	
	/* update semaphores  */
	//PRINTF_FL("unlocked\n");
	sem_post(&Transmit_sem); /* inform getOldestPacket() there's a new pkt to read */

	semaphoreDebugger(0);
		

	
	return E_SUCCESS ;
}

/* sdebug current values of sem.
 * selfLOCKs under TX_QUEUE_MYTEX */
void semaphoreDebugger( int err ) 
{
	
	/* Debug begin*/
	my_mutex_lock( &PM_tx_queue_mutex ) ; /* lock */
	int val =0; 
	if ( sem_getvalue( &PM_tx_queue_sem , &val ) < 0 )
	{
		PRINTF_FL_ERR("error on getvalue\n");
		goto error_code; 
	}	
	int val2 =0; 
	if ( sem_getvalue( &Transmit_sem , &val2 ) < 0 )
	{
		PRINTF_FL_ERR("error on getvalue\n");
		goto error_code; 
	}
	
	

	
	if ( val2 > getTxBufferUse()+2 || val2 < getTxBufferUse()-2 )
	{

		PRINTF_FL_ERR("INCONSISTENCY\n");
		goto error_code;
	}
	
	if ((val2 > PM_TX_QUEUE_SIZE+1) || (val > PM_TX_QUEUE_SIZE+1 ))
	{

		PRINTF_FL_ERR("FAIL uniq SEMAPHORES\n");
		goto error_code;
	}
	
	if (val2 + val > PM_TX_QUEUE_SIZE+2 )
	{

		PRINTF_FL_ERR("FAIL SUM SEMAPHORES\n");
		goto error_code;
	}

	if ( err > 0 )
	{
		PRINTF_FL_ERR("semaphore let me go .but no space!\n");
		goto error_code;
	}	
		
		
	my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
	return ;
	
	error_code:
		PRINTF_FL_WARN(
			"Queue is %"PRIu16". ( w_ptr=%d | r_ptr=%d ) . "
			"PM_tx_queue_sem %d (free spots) " 
			"Transmit_sem %d \n" ,
			getTxBufferUse(),
			PM_tx_queue_w_ptr, PM_tx_queue_r_ptr ,
			val ,
			val2 ) ;
		dumpTxQueue();
		my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
		exit(1);
			
	
	/* debug end */
	
}
/** RECEIVER : 
 * this function calls TDMA_Receive to check for new packet. 
 * if there is a packet for self, save it. 
 * otherwise put it in the outqueue **/
void *rxThread()
{

	//PRINTF_FL("Initiating :::::::::\n");
	error_t status = E_SUCCESS ;

	/* get a memory piece to save incoming packets:
	 * allocate 1500 bytes, where received data will be. */
	void *recv_pmpkt_ptr = calloc( MAX_PM_PACKET_SIZE , sizeof(char) ) ; 
	
	if( recv_pmpkt_ptr == NULL )
	{
		status = E_NO_MEM ; 
		goto error_mem1 ;
	}

	/* Open a log file for rcvd PM-packets */
	#ifdef LOGDATA
	FILE *pm_rx_logfile = NULL ; 
	{
	char prefix[10];
	snprintf( prefix , sizeof(prefix)-1, "PM#%02d" , Param_pm.node_ip ) ;
	error_t ret ;
	if ( ( ret = open_log_file( prefix , "RX" , &pm_rx_logfile ) ) < 0 )
	{
		status = ret ; 
		goto error_file2 ;
	}
	}
	#endif
	
	//uint16_t num_bytes_read ; /* save here number of bytes read */

	/* init cycle */
	while ( !PExit_threads )
	{
		error_t ret;
		// memset((char *) recv_pmpkt_ptr, 0, 1024 ) ; /* erase space */
		uint8_t other ; /* imediate source where this came from */
		ssize_t num_bytes_read = TDMA_receive( recv_pmpkt_ptr , &other ) ;/* note: tdma header is no longer here. blocking = 1 .  */
		//PRINTF_FL_WARN("num_bytes_read %ld\n", num_bytes_read);
		
		
			
		/* packet size error checking, and others */
		{
		ret = checkPMerrors( num_bytes_read , other ) ;
		if ( ret < 0 )
		{
			status = ret ;
			break;
		}
		if ( ret == 0 )
		{
			PRINTF_FL_ERR("From: %" PRIu8 "\n", other);
			dumpData( recv_pmpkt_ptr , num_bytes_read );
			continue ;
		}
		}
		

		
		/* log newly rx'ed PM Packet into file */
		#ifdef LOGDATA
		logPacket( pm_rx_logfile , *(pm_header_t*)recv_pmpkt_ptr , 0 ,
			other, 
			0,  
			(uint16_t)num_bytes_read , 
			getRxBufferUse() , 
			TDMA_getBandwidth( ) ) ;
		#endif
		

		/* header error checking */
		//{
		//error_t ret = checkPMerrors2( (pm_header_t*)recv_pmpkt_ptr ) ;
		//if (ret<0)
		//{
			//continue ;
		//}
		//}
	

		
		/* printout pdr */
		//#ifdef VERBOSE
		//printPDR( other ) ;
		//#endif
		
		/* check what to do with it */
		uint8_t sink = ((pm_header_t*)recv_pmpkt_ptr )->sink ;
		if ( isThisMe( sink ) == 1 ) /* -> packet to be parsed/saved locally */
		{
			pm_type_t type = ((pm_header_t*)recv_pmpkt_ptr)->type & 0x00FF ;
			if ( type == ROUTING )
			{
				/* parse PM_PDR type */
				//parserPdrPkt( recv_pmpkt_ptr , num_bytes_read, other);
			}
			
			if ( type == APP )
			{
				/* parse app file */
				parserAppPkt( recv_pmpkt_ptr , num_bytes_read , other );
			}
			
			
		}
		else /* -> packet to be relayed: */
		{
			addPMpacketTxQueue( recv_pmpkt_ptr , num_bytes_read , other );
		} /* end of packet to be relayed */



	} /* end of while 1 */

	#ifdef LOGDATA
	fclose( pm_rx_logfile ) ;
	error_file2:
	#endif
	free( recv_pmpkt_ptr ) ;
	error_mem1 :
		

	PM_initiated = 0 ; /* block users to use this module */
		
		

	

	



	if (status < E_TERMINATING )
	{
		printError( status ) ; /* print the reason we are here */
		PRINTF_FL_ERR( "[PM RX] Thread terminated with Errors \t[%08x] *******\n", 
			(unsigned int)pthread_self() );  
	}
	else
	{
		PRINTF_FL( "[PM RX] Thread terminated: normally\t[%08x] *******\n", 
			(unsigned int)pthread_self() );  
	}

	// TODO: we should free all elements in the rx_queue 
	return NULL ;
}


#ifdef DUMP
void dumpTxQueue( void )
{
	for ( uint16_t i = 0 ; i< PM_TX_QUEUE_SIZE ; i++ )
		PRINTF_FL("Tx_queue[%" PRIu16 "] = { Prv%u, Nxt%u , len %uB }\n" ,
			i,
			tx_queue[i].prev_ip ,
			tx_queue[i].next_ip , 
			tx_queue[i].pkt_len ) ;
}


void dumpRxQueue( void )
{
	for ( uint16_t i = 0 ; i < PM_RX_QUEUE_SIZE ; i++ )
	{
		PRINTF_FL("Rx_queue[%" PRIu16 "] - len %uB - %p \n" ,
			i,
			rx_queue[i].pkt_len,
			rx_queue[i].pkt_ptr ) ;
	}
}
#endif

#ifdef LOGDATA
void logPacket( 
	FILE *logfile , 
	pm_header_t header, 
	double timestamp ,
	uint8_t prev_hop, uint8_t next_hop, 
	uint16_t pm_pkt_len , 
	uint16_t buf_use ,
	bandwidth_t bandwidth )
{
	/* if loging an incoming pkt, next_hop = 0 */
	/* and timestamp is 0, since it has no meaning. packt was added to the queue just now */
	double epoch_time = getEpoch() ;
	if ( timestamp == 0 )
		timestamp = epoch_time ;
	
	char file_buf[200];
	snprintf( file_buf, 199 ,
		"%.6fs | " /* epoch time */
		"%02.6fms | " /* time in the queue */
		"%02" PRIu16 "pkts | "
		"[%02" PRIu8 "-...-%02" PRIu8 "-{%02" PRIu8 "}-%02" PRIu8 "-...-%02" PRIu8 "] | " 
		"ack %" PRIu8 " | " 
		"T %" PRIu8 " | "
		"%04" PRIu16 "B | "
		"s# %02" PRIu8 "%02" PRIu8 "%06" PRIu32 " | " /* orsrc-sink-seqnum */
		"Bin %.1f Bout %.1f",
		epoch_time ,
		1e3*(epoch_time - timestamp) , /* time spent in the queue */
		buf_use , /* pm queued packets */
		header.or_src , prev_hop, Param_pm.node_ip, next_hop , header.sink , /* route */
		header.need_ack , /* expected to send an ack when received */
		header.type , /* App packet , mgt , etc */
		pm_pkt_len , 
		header.or_src , header.sink , header.seq_num , /* this forms a unique triple */
		bandwidth.in[ip_up] ,
		bandwidth.out[ip_dw] ) ;
	fprintf( logfile , "%s\n", file_buf );
	fflush( logfile ) ;
}
#endif


/** move the writer pointer to the next place
 * if queue is full, then : deny moving OR overwrite **/
static error_t moveWPtr( 
	volatile uint16_t* const w_ptr, 
	volatile uint16_t* const r_ptr , 
	uint16_t queue_size , 
	uint8_t overwrite )
{
	// we are inside a mutex lock  
	uint16_t tmp = ( *w_ptr+1 ) % queue_size ;
	if ( tmp == *r_ptr ) /* overwriting ...*/
	{
		if  ( !overwrite )
			return -1; /* do nothing. can't move pointer */
		
		/* let's ovrwrite oldest data in the queue */
		if ( deleteTxElement( *r_ptr ) > 1 )
		{
			PRINTF_FL_ERR("Pkt Dropped\n") ;
		//dumpTxQueueItem( tx_queue[ *r_ptr] ) ;
			PRINTF_FL_ERR("OVRWR (r%" PRIu16 "-w%" PRIu16 ")\n", *r_ptr , *w_ptr );
		}
		*r_ptr = ( *r_ptr+1 ) % queue_size ; 
		//PRINTF_FL_ERR("overwritin..\n");
		
		
	}
	*w_ptr = tmp ; /* update pointer */
	
	return E_SUCCESS ; 
}


#ifdef VERBOSE
void dumpTxQueueItem( pm_tx_queue_item_t item )
{

	PRINTF_FL( "<Item: time%fs,%"PRIu8",%"PRIu8", %"PRIu16"B, ptr %p>\n" ,
		item.timestamp,
		item.prev_ip,
		item.next_ip,
		item.pkt_len,
		item.pkt_ptr );
		
}
#endif


 /********************************
 * Retrieve the oldest PM-packet from the tx queue (FIFO) 
 * also: len & dest_id of payload 
 * waits 2secs fr packets
 * return:	if no files in queue => -1
 * 			otherwise: 1
 * Delete packet before returning
 *****************************************************/
static error_t getOldestPMPkt( 
	void 	* const payload_p, 
	uint16_t * const len, 
	uint8_t * const prev_hop, 
	uint8_t	* const next_hop, 
	double	* const timestamp ) 
{
	/* wait for transmit_sem, timeout 2 secs or PExit_thr goes up */
	int ret = sem_timedwait_safe( &Transmit_sem, "PM Transmit_sem", 
		&PExit_threads, 
		2000000 ); /* 2 sec */
		
	/* proceed only if no success */
	if ( E_SUCCESS != ret )
		return ret ; 
		
	#if 0
	int val=-1;
	sem_getvalue(&Transmit_sem,&val);
	PRINTF_FL("Reading packets (val %d)\n",val);
	#endif
	
	
	/* entering critical section - editing tx queue */
	my_mutex_lock( &PM_tx_queue_mutex ); /* LOCK (unlock before return) */
	
	

	
	
	
	while ( PM_tx_queue_w_ptr == PM_tx_queue_r_ptr ) /* nothing in the queue? so sleep a little. this sh'd not happen */
	{
		my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock , to sleep */

		PRINTF_FL_ERR("ERR\n"); semaphoreDebugger(1);
		
		microsleep( 10000 ) ; /* sleep for a while waiting for ..*/

		PRINTF_FL_WARN( "Spining\n" );
		my_mutex_lock( &PM_tx_queue_mutex ); /* lock */


		if ( PExit_threads )
		{
			my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
			return E_TERMINATING ;  /* time to stop */
		}
	}

	PM_tx_queue_r_ptr = ( PM_tx_queue_r_ptr + 1 ) % PM_TX_QUEUE_SIZE ; /* update reader pointer */
	//PRINTF_FL("PM_tx_queue_r_ptr=%d , PM_tx_queue_w_ptr=%d\n" , PM_tx_queue_r_ptr, PM_tx_queue_w_ptr); 
	*timestamp 	= tx_queue[PM_tx_queue_r_ptr].timestamp ; /* grab epoch time when this was added to the queue */
	*len 		= tx_queue[PM_tx_queue_r_ptr].pkt_len ;
	*next_hop 	= tx_queue[PM_tx_queue_r_ptr].next_ip ;
	*prev_hop 	= tx_queue[PM_tx_queue_r_ptr].prev_ip ;
	memcpy( payload_p , tx_queue[PM_tx_queue_r_ptr].pkt_ptr , (size_t)(*len) ) ; /* copy payload from txqueue */

	
	if ( tx_queue[PM_tx_queue_r_ptr].pkt_ptr == NULL )
	{
		PRINTF_FL_ERR(
			"TERMINATING: ERROR Reading queue element with a null payload\n") ;
		#ifdef VERBOSE
		dumpTxQueueItem( tx_queue[PM_tx_queue_r_ptr] ) ;
		#endif
		PRINTF_FL_ERR( "W%" PRIu16 " , R%" PRIu16 "\n" ,
			PM_tx_queue_w_ptr , PM_tx_queue_r_ptr) ;
		my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
		return E_INVALID_ITEM ; 
	}
	
	/* delete item */
	//PRINTF_FL_ERR("rptr %d\n", PM_tx_queue_r_ptr ) ;
	
	if ( deleteTxElement( PM_tx_queue_r_ptr ) < 0 )
		PRINTF_FL_ERR( "Nothing was deleted\n") ;
	
	my_mutex_unlock( &PM_tx_queue_mutex ) ; /* unlock */
	
	//PRINTF_FL_WARN("x\n");semaphoreDebugger(0);
	
	sem_post(&PM_tx_queue_sem); /* a free spot is in place */	
	
	semaphoreDebugger(0);
	
	return E_SUCCESS ; /* success */
}


error_t deleteTxElement( uint16_t r_ptr ) 
{
	if (NULL == tx_queue[r_ptr].pkt_ptr)
	{
		//PRINTF_FL_ERR(
			//"Tried to delete a null element [%" PRIu16 "]!\n" ,
			//r_ptr ) ;
		return E_NOTHING_DELETABLE ; /* nothing to delete */
	}
	
	free( tx_queue[r_ptr].pkt_ptr ) ; /* delete item from txqueue */
	tx_queue[r_ptr].pkt_ptr = NULL ; /* erase item from tx queue */
	tx_queue[r_ptr].timestamp = 0 ; /* erase item from tx queue */
	tx_queue[r_ptr].pkt_len = 0 ; /* erase item from tx queue */
	tx_queue[r_ptr].next_ip = 0 ; /* erase item from tx queue */
	tx_queue[r_ptr].prev_ip = 0 ; /* erase item from tx queue */
	return E_SUCCESS ;
}






int isThisMe( uint8_t ip )
{
	if ( ip == Param_pm.node_ip ) 
		return 1 ;
	
	//~ if ( ip == 10 + Param_pm.node_ip )
		//~ return 1 ;
	
	return 0 ;
}


void persistentSend( uint8_t dest_ip , uint16_t pkt_len , 
	const void* const pkt_ptr, int hi_prio )
{
	//dumpData( (uint8_t*)pkt_ptr , pkt_len ) ;
	
	int ret ;
	//while ( (ret = TDMA_send( dest_ip , pkt_len , pkt_ptr, hi_prio )) < 0 ) 
	while ((ret = TDMA_sendAnyPacket(dest_ip, (hi_prio<<8) | PM, pkt_len, pkt_ptr))<0)
	{
		if ( ret == E_TERMINATING )
		{
			break;
		}
		
		if ( ret == E_INVALID_INPUT )
		{
			break;
		}
		if ( ret == E_NO_INIT )
		{
			break;
		}
		PRINTF_FL_ERR("TDMA_send failed. \n" );
		printError( ret ) ;
		microsleep( 100000 );
	}

	
}


void* txThread()
{
	int8_t status = E_SUCCESS ;

	uint8_t next_hop = 0 ; /* use this to save nexthop of oldest packet in txqueue */
	uint8_t prev_hop = 0 ; /* idem */ 
	uint16_t pkt_len = 0 ;  /* use this  to save the pktlen of oldest packet in txqueue */
	double timestamp = -1;   /* use this  to save the added-to-queue timestamp of oldest packet in txqueue */

	void *tmppkt = malloc( (size_t)MAX_PM_PACKET_SIZE ) ;
	if ( tmppkt == NULL ) /* get this space to save the oldest packet in txqueue */
	{
		PRINTF_FL_ERR( "Malloc failed !\n") ;
		status = E_NO_MEM ;
		goto error_mem1 ;
	}


	/* Open a log file for tx'ed PM-packets */
	#ifdef LOGDATA
	FILE *pm_tx_logfile = NULL ; 
	{/*scope B*/
	char prefix[10];
	snprintf( prefix , sizeof(prefix)-1 , "PM#%02d" , Param_pm.node_ip ) ;
	error_t ret ; 
	if ( 0 > (ret = open_log_file( prefix, "TX", &pm_tx_logfile )) )
	{
		PRINTF_FL_ERR(  "Failed to open log file") ;
		status = ret ;
		goto error_file2 ; 
	}
	}/*scope E*/
	#endif
	
	//PRINTF_FL_WARN("txThread on PM initing..\n");

	while ( !PExit_threads )
	{

	   { /* scope - begin */
		
		/* grab oldest packet in the queue 
		 * timeout = 2secs */
		//PRINTF_FL_WARN("getting\n");
		int ret = getOldestPMPkt( 
			tmppkt, 
			&pkt_len, 
			&prev_hop, 
			&next_hop, 
			&timestamp ) ;
		//PRINTF_FL_WARN("getOldestPacket() - ret %d\n", ret);	
		if ( ret == E_TIMEDOUT )
		{
			//PRINTF_FL_WARN("nothing to send (timedout 2s)\n");
			continue ;
		}	
		
		if ( ret == E_TERMINATING ) 
		{
			break ;
		}
		
		/* unexpected error handling */
		if (0 > ret)
		{
			/* nothing available */
			/* should never happen, since this is a blocking call.
			 * unless we are terminating */
			PRINTF_FL_WARN("terminating...\n");
			dumpTxQueue();
			status = ret ;
			break ;
		}
	   } /* scope - end*/


		if ( 0 == next_hop )
		{
			PRINTF_FL_ERR("Weird case - don't know where to send this\n") ;
			continue ;
		}
		
	
		/* update the only modifiable part of the pm header */
		(((pm_header_t*)tmppkt)->try) 	= 0 ;
		(((pm_header_t*)tmppkt)->total) = 1 ; /* TODO: FEC - depending on linkPDR we will set this number !*/
		for ( 
			uint8_t i = 0 ; 
			i < (((pm_header_t*)tmppkt)->total) ;
			i++ ) /* send multiple copies */
		{
			(((pm_header_t*)tmppkt)->try)++;
			
			int hi_prio = (((pm_header_t*)tmppkt)->type)>>8;/*lo-byte*/
			persistentSend( next_hop , pkt_len, tmppkt , hi_prio ) ; /* keep trying, until TDMA accepts the pkt */
			//dumpData( tmppkt,(uint16_t)sizeof(pm_header_t));
			//PRINTF_FL("PM sent! - current queued %" PRIu16 "\n" , getTxBufferUse() );
			
			
			#ifdef LOGDATA
			logPacket( pm_tx_logfile , 
				*((pm_header_t*)tmppkt) , 
				timestamp , /* when this packet was added to tx_queue via PM_send or from rx_thread */
				prev_hop , next_hop , 
				pkt_len ,
				getTxBufferUse() , 
				TDMA_getBandwidth() ) ;
			#endif
			
			
			
		}
	
	}
	
	
	free( tmppkt ) ;
	// TODO: here we should also free/delete all tx_queue elements 
	
	#ifdef LOGDATA
		fclose( pm_tx_logfile ) ;
	error_file2 :
	#endif
	error_mem1 : 
	


	
	/* clean up */
	// TODO: here we should also free all tx_queue elements 
	PM_initiated = 0 ; /* block external calls to public functions */
	
	if ( status < E_TERMINATING )
	{
		printError( status ) ;
		PRINTF_FL_ERR(
			"[PM TX] Thread terminated with Errors\t[%08x]n", 
			(unsigned int)pthread_self() );  
	}
	else
	{
		PRINTF_FL( 
			"[PM TX] Thread terminated: normally\t[%08x]\n", 
			(unsigned int)pthread_self() );  
	}

	PExit_threads = 1 ;
	
	return NULL ;

}

/* delete current Reading element */
error_t deleteRxElement( void )
{
	//PRINTF_FL( "Deleting item %" PRIu16 
		//" (w_ptr=%"PRIu16")\n" , 
		//PM_rx_queue_r_ptr , 
		//PM_rx_queue_w_ptr );
	if (rx_queue[PM_rx_queue_r_ptr].pkt_ptr  == NULL)
	{
		PRINTF_FL_ERR(
			"FAIL to delete Rx item! %p\n", 
			rx_queue[PM_rx_queue_r_ptr].pkt_ptr );
		PRINTF_FL_ERR( "R%" PRIu16 "\n" , PM_rx_queue_r_ptr ) ;
		return E_NOTHING_DELETABLE ;
	}
	

	free( rx_queue[PM_rx_queue_r_ptr].pkt_ptr ) ; /* delete item from txqueue */
	rx_queue[PM_rx_queue_r_ptr].pkt_ptr = NULL ; /* erase item from tx queue */
	rx_queue[PM_rx_queue_r_ptr].pkt_len = 0 ; 

	return E_SUCCESS ;
}


/* number of packets to be read */
uint16_t getRxBufferUse( void )
{
	pthread_mutex_lock( &rx_queue_mutex ); /* lock */
	int32_t buff_use = (int32_t)PM_rx_queue_w_ptr - 
		(int32_t)PM_rx_queue_r_ptr ;
	if ( buff_use < 0 )
		buff_use = ( PM_RX_QUEUE_SIZE - (int32_t)PM_rx_queue_r_ptr ) + 
			(int32_t)PM_rx_queue_w_ptr ;
	pthread_mutex_unlock( &rx_queue_mutex ); /* unlock */
	
	return (uint16_t)buff_use;
}

/* _number of packets_ in the queue to go to TDMA */
/* call this inside PM_tx_queue_mutex */
uint16_t getTxBufferUse( void )
{
	//PRINTF_FL("locking...");
	//my_mutex_lock( &PM_tx_queue_mutex ); /* lock */
	//printf("locked.\n");
	int32_t buff_use = 
		(int32_t)PM_tx_queue_w_ptr - 
		(int32_t)PM_tx_queue_r_ptr ;
	if ( 0 > buff_use )
	{
		buff_use = 
			( PM_TX_QUEUE_SIZE - (int32_t)PM_tx_queue_r_ptr ) + 
			(int32_t)PM_tx_queue_w_ptr ;
	}
	//PRINTF_FL("unlocked\n");
	//my_mutex_unlock( &PM_tx_queue_mutex ); /* unlock */
	
	return (uint16_t)buff_use;
}

/* return current seq number for the flow [MYSELF]->[SINK NODE]*/
uint32_t getSeqNum( uint8_t sink_IP )
{
	if ( sink_IP > MAX_IP || sink_IP == 0 )
	{
		PRINTF_FL_ERR(
			"Invalid sink_ip %"PRIu8"\n",
			sink_IP ) ;
		return 0 ;
	}
	
	
	/* save this - increment every time */
	static uint32_t seq_num[MAX_IP] = {0} ; 
	return (++seq_num[sink_IP+1]) ;
}

#undef PM_IMPORT
