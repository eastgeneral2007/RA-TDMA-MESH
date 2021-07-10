
/**************************************************
 * ***************TDMA Slot operations  ********************
 * description: functionalities related with Slot operations 
*********************************************************/
#define TDMA_SLOT_IMPORT


/** tdma self and sibing modules **/
#include "drk/tdma_slot.h" /*  self */
#include "drk/tdma_types.h" /* */
#include "drk/tdma.h" /* */
#include "drk/tdma_utils.h" /*  */
#include "drk/packet_manager.h" /*  */

#include "drk/drk.h" /* drk_exit */
#include "drk/utils.h" /* generic methods and data types used across the files */


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
#include <time.h>  //rand

/** defines **/
#define PREFIX		"TDMA_S"
#define CSI			0.2500 //constant used to limit Slot shifting


/*****************************************************
 * GLOBALS STATIC (only to be used inside this file)
 *****************************************/
/* table to store IP,SLOTID,SLOTWIDTH data*/
static volatile uint16_t 			Table_slotid_2_slotwidth[MAX_IP+1] = {0}; /* knowledge about other slots (width ms ) . 0 is not used */
static volatile uint8_t 			Table_ip_2_slotid[MAX_IP+1] = {0}; /* IP range : 1 to 19 */

// TODO: use struct
static pthread_mutex_t 				Mutex_delay = PTHREAD_MUTEX_INITIALIZER ; /* mutex to read/write to the queue , multiple threads need access to it */
static volatile int64_t 			Pktdelay_array_counter = 0 ; /* keep track of total number of received pkts to use to sync */
static volatile float 				Pktdelay_array[PKTDELAY_ARRAY_SIZE] ; /* save here the computed delay for rx pkts.  */
static volatile float				Pktdelay_array_sender[PKTDELAY_ARRAY_SIZE] ; /* save here the computed delay for rx pkts */

static volatile double 				last_outreqst = -1 ; 
static volatile uint16_t 			Requests_slot_width_ms[MAX_IP+1] = {0}; /* save here outgoing requests -  0 is not used */
static volatile slot_limits_t 		Myslot ; /* init and end of MY TDMA slot */
static volatile int 				Under_incoming[MAX_N_SLOTS] ; /* flag incoming requsts from every slot id possible  0 is not used */
static pthread_mutex_t 				Mutex_slot = PTHREAD_MUTEX_INITIALIZER ; // mutex to define and read slots. they change depending on special rcvd packets



/***********************************************
 * public functions go here - prefix <TDMA_>
 * ******************************************/
 
 /** public **/

slot_limits_t TDMA_getOwnSlot(void) 
{
	pthread_mutex_lock( &Mutex_slot );
	slot_limits_t copy = Myslot ;
	pthread_mutex_unlock( &Mutex_slot );
	return copy ; 
}


#if VSP
/** provide a list of IPs of nodes using TDMA **/
error_t TDMA_newSlotList( const uint8_t * const slot_list, int len )
{
	TDMA_LOADED_CONDITION E_NO_INIT; 
	
	for (int i = 0; i < len; i++) 
	{
		if (slot_list[i] == Param_tdma.node_ip)
		{ /* we are part of the list, so activate TDMA */
			
			Param_tdma.slot_total = len-1 ;
			Param_tdma.standard_width_ms = 
				Param_tdma.round_period_ms / 
				Param_tdma.slot_total ;
			Param_tdma.slot_id = i+1;
			TDMA_rigid(); // default to DVSP?	
			
			
			char strin[100];
			snprintf( strin , sizeof(strin), 
				"Our slot id %"PRIu8 "/%"PRIu8"; "
				"slot-width %"PRIu16"ms (of %"PRIu8"ms)",
				Param_tdma.slot_id ,
				Param_tdma.slot_total,
				getSlotWidth_givenIp(Param_tdma.node_ip),
				Param_tdma.round_period_ms
				); 
			
			PRINTF_FL_WARN( "%s\n", strin ) ;
			sendString( strin, 22 );
			
			return E_SUCCESS;
		}
	}
	/* we are not part of the active list, so disable tdma */
	TDMA_off();
	PRINTF_FL_WARN("TDMA Disabled\n");
	
	return E_SUCCESS ;
}
#endif

#if VSP
/** 
 * higher layers can request a slot width to a neighbor. 
 * when dst_ip is 0, assumed upStream neighbor (myslot_id-1)
 * **/
void TDMA_reqSlotWidth( uint16_t req_ms, uint8_t dst_ip )
{
	if (0 == dst_ip)
	{
		dst_ip = TDMA_upStream() ;
	}
	if (0 == dst_ip)
	{
		PRINTF_FL("unkn dst ip\n");
		return ;
	}	
	
	if (dst_ip == Param_tdma.node_ip)
	{
		PRINTF_FL_ERR("dst ip is self\n");
		return ;
	}	
	
	/* if still running a request , ignore this */
	if ( 0 != underOutgoingRequest())
	{
		//PRINTF_FL_ERR("Under out\n");
		return ;
	}
	
	/* if someone requested us to change, lets wait until this change is done */
	if ( 0 != underIncomingRequest() )
	{
		//PRINTF_FL_ERR("under in (%d)\n", underIncomingRequest() );
		return ;
	}
	
	
	if ( 0 == getSlotWidth_givenIp(dst_ip) )
	{
		PRINTF_FL_ERR("unknown slot width from #%"PRIu8"\n",dst_ip);
		return ;
	}
	
	/* sanitize req_ms */
	#define MAXRWIDTH \
		(getSlotWidth_givenIp( dst_ip ) + \
		TDMA_getOwnSlotWidth() - 5)
	req_ms = MIN(
		req_ms ,
		MAXRWIDTH 
		) ;
	#undef MAXRWIDTH
	
	/* ignore request if dst_ip has <req_ms> slot width already */
	if ( getSlotWidth_givenIp( dst_ip ) == req_ms )
		return ;
	
	/* make the actual request: */	
	Requests_slot_width_ms[ getId_givenIp(dst_ip) ] = req_ms ;
	PRINTF_FL_WARN(
		"Launching  REQ to #%" PRIu8 "-> "
		"new: [%" PRIu16 "]ms (changes by %+"PRId32 "ms)\n" , 
		dst_ip ,
		req_ms ,
		(int32_t)req_ms - (int32_t)getSlotWidth_givenIp(dst_ip)) ;
	
	//PRINTF_FL_WARN("current knowledge:");
	//for (uint8_t id=1;id<3;id++)
		//printf("[%d]",getSlotWidth_givenId(id));
	//printf("\n");
	
	/* some debug */
	/* printout debug */
	//PRINTF_FL_WARN(
		//"Bin[%"PRIu8"]=%.0f ; "
		//"Bout[ALL]=%.0f\n" , 
		//TDMA_upStream(), 
		//Bandwidth.in[TDMA_upStream()],
		//Bandwidth.out[0] ) ;
	
	#ifdef VERBOSE
	pthread_mutex_lock( &Mutex_slot );
	tdma_printSlot( Myslot );
	pthread_mutex_unlock( &Mutex_slot );
	#endif
	
	return ;
}
#endif




/** ****************************************
 * ************** end of public functions 
 * ********************************************/


/**********************************************
 * SEMIprivate functions go here - prefix <tdma_>
 * only available to sibling modules tdma_*
 * */


 /** get slot (B_E) **/
slot_limits_t tdma_getOwnSlot_unsafe(void) 
{
	return Myslot ; 
}

/* Get slot (B E), with some guard perid*/ 
slot_limits_t tdma_getOwnSlot_guarded(void)
{
	slot_limits_t local_slot = TDMA_getOwnSlot(); /* get  current slot  */
	local_slot.end_ms += Param_tdma.round_period_ms - GUARD_INTERVAL_MS ; /* stop my slot slightly before (guard) */
	local_slot.end_ms %= Param_tdma.round_period_ms ; /* bound it to [0-PERIOD] */
	return local_slot ;
}







/** Get slot width (in ms) of a given IP,
 * when 0 is given, assumed from upStream neighbor **/
uint16_t getSlotWidth_givenIp( uint8_t ip )
{
	if ( ip == 0 )
		return 0 ;
	return getSlotWidth_givenId(getId_givenIp(ip)) ; /* maximum slot id . 0 is not used */
}

/** Get slot id associated with a given IP **/
uint8_t getId_givenIp( uint8_t ip )
{
	return Table_ip_2_slotid[ ip ]; /* maximum slot id . 0 is not used */
}


/** Get slot Width, given slot id **/
uint16_t getSlotWidth_givenId(uint8_t slot_id) 
{
	return Table_slotid_2_slotwidth[slot_id] ;
}







/** 
 * set new TDMA slot limits based on saved delays
 * delta is the target shift for slot
 * delta is pointer since final shift can end up different
 * **/
/* THIS FUNCTION is the only that *WRITES* on >myslot< */
int tdma_syncronizeSlot(int32_t *delta)
{
	//return E_SUCCESS;
	//PRINTF_FL_ERR("My table slot width %" PRIu16 "ms\n" , getSlotWidth_givenId(Param_tdma.slot_id) );
	
	//if ( getSlotWidth_givenId(Param_tdma.slot_id) != TDMA_getOwnSlotWidth() )
	//{
		//PRINTF_FL("Updating slot width\n");
	//}
	
	#if 1 // safe-read pktdelay array
	/* 
	 * Grab a SAFE copy of recorded delays 
	 * Erase the delay records at the end of sync
	 * */
	int64_t tmp_pktdelay_array_len = 0 ;
	float tmp_pktdelay_array[PKTDELAY_ARRAY_SIZE] ;
	float tmp_pktdelay_array_sender[PKTDELAY_ARRAY_SIZE];
	pthread_mutex_lock( &Mutex_delay ) ; /* protect from editing - getDelay could be writing at the same time */
	tmp_pktdelay_array_len = Pktdelay_array_counter ; /* number of elemens in the array */
	memcpy(
		(void*)tmp_pktdelay_array ,
		(void*)(1 + Pktdelay_array) ,  /* pos 0 of pktdelay array not used */
		sizeof(Pktdelay_array[0]) * Pktdelay_array_counter ) ;
	memcpy(
			(void*)tmp_pktdelay_array_sender ,
			(void*)(1 + Pktdelay_array_sender) ,  /* pos 0 of pktdelay array not used */
			sizeof(Pktdelay_array_sender[0]) * Pktdelay_array_counter ) ;
	pthread_mutex_unlock( &Mutex_delay ) ; /* protect from editing */
	
	/* Compute average */
	//int32_t average = computeAverage( delay_array , delay_array_len ) ;
	//int32_t delta = 0 ;
	
	/* Compute some metrics using delay of every prev packet */
	
	PRINTF_FL("ARRAY SIZE: %ld\n", tmp_pktdelay_array_len);
	if (tmp_pktdelay_array_len >= 1) //10
	{
		//mean = computeMean( tmp_pktdelay_array , tmp_pktdelay_array_len );
		//int32_t median = computeMedian( tmp_pktdelay_array, tmp_pktdelay_array_len );
		//std = computeStd( tmp_pktdelay_array , tmp_pktdelay_array_len , mean ) ;
		//int32_t min = computeMin( tmp_pktdelay_array, tmp_pktdelay_array_len ) ;
		//int32_t max = computeMax( tmp_pktdelay_array, tmp_pktdelay_array_len ) ;
		
		//if ( 10<mean )
			//PRINTF_FL_WARN( 
				//"Pkt delay: Mean %" PRId32
				//"ms, std %"PRId32 ", "
				////", Min %"PRId32
				////", Max %"PRId32
				//"n=%"PRId64 ", "
				//"\n",
				//mean, 
				//std, 
				//tmp_pktdelay_array_len
			////min , max 
			//);
		
		/* use *1st-pkt* to syncronize */
		//*delta = tmp_pktdelay_array[0];

		#define NUM_DRONES 4

		float aggByNode[NUM_DRONES] = {0};
		float delayByNode[NUM_DRONES][PKTDELAY_ARRAY_SIZE];
		int delayByNodeCounter[NUM_DRONES] = {0};
		int isSender[NUM_DRONES] = {0};
		float curr_delay;


		for(int i = 0; i < tmp_pktdelay_array_len; i++){
			uint8_t sender = tmp_pktdelay_array_sender[i];
			PRINTF_FL("DELAY %f: %f\n", tmp_pktdelay_array_sender[i], tmp_pktdelay_array[i]);
			delayByNode[sender-1][delayByNodeCounter[sender-1]] = tmp_pktdelay_array[i];
			delayByNodeCounter[sender-1]++;
			isSender[sender-1] = 1;
		}

		for (int nj = 0; nj < NUM_DRONES; nj++){
			if (!isSender[nj]) continue;
			PRINTF_FL("COMPUTING DELAYS FOR NODE %d\n", nj+1);
			//aggByNode[nj] = computeMedian(delayByNode[nj],delayByNodeCounter[nj]);
			//aggByNode[nj] = computeMax(delayByNode[nj],delayByNodeCounter[nj]);
			//YaggByNode[nj] = computeMin(delayByNode[nj],delayByNodeCounter[nj]);
			aggByNode[nj] = computeMean(delayByNode[nj],delayByNodeCounter[nj]);
			PRINTF_FL_WARN("Agg result: %f \n", aggByNode[nj]);
		}

		// MAX or MIN
		for (int nj = 0; nj < NUM_DRONES; nj++){
			if (!isSender[nj]) continue;
			if (aggByNode[nj] > *delta) *delta = aggByNode[nj];
			//if (aggByNode[nj] < *delta) *delta = aggByNode[nj];
			PRINTF_FL("ComputeDelta = %d\n", *delta);
		}


		//// AVG
		//uint8_t counter = 0;
		//float temp = 0;
		//for (int nj = 0; nj < NUM_DRONES; nj++){
		//	if (!isSender[nj]) continue;
		//	temp += aggByNode[nj];
		//	counter ++;
		//}
		//*delta = temp/counter;

		PRINTF_FL("ComputeDelta = %d\n", *delta);

		// Max of Max delay
		//PRINTF_FL_WARN("Computing delta max\n");
		//for (int i = 0; i <  tmp_pktdelay_array_len; i++){
		//	PRINTF_FL("DELAY %f: %f\n", tmp_pktdelay_array_sender[i], tmp_pktdelay_array[i]);
		//	curr_delay = tmp_pktdelay_array[i];
		//	*delta = fmax(curr_delay, *delta);
		//};

		/* use *median** to syncronize */
		//*delta =  median ;
		
		/* use *min** to syncronize */
		//*delta = min ; // tmp_pktdelay_array[0]
		
		/* use *max** to syncronize */
		//*delta = max ; // tmp_pktdelay_array[0]
		
		/* ignore deltas if std is too high */
		
		//if ( std > 17 ) 
		//{
			//*delta = 0 ;
		//}
		
		/* ignore deltas of |5ms| or less */
		//if ( (*delta>-5) && (*delta < 5) ) 
			//*delta = 0 ;
			
		if (*delta > CSI*getSlotWidth_givenIp( Param_tdma.node_ip ) ) 
			*delta = CSI*getSlotWidth_givenIp( Param_tdma.node_ip ) ;
			
		if (*delta < 0)
			*delta = 0 ;
		
	}
	#endif
	
	uint8_t lim = Param_tdma.standard_width_ms*0.10;
	*delta = BOUND( *delta, lim) ; /* at most move the slot <lim>ms per round */
	float rnd = (rand() % 101) * 0.01;
	*delta = *delta * (0.8 +0.2*rnd);
	

	PRINTF_FL("Random part: %f", rnd);
	PRINTF_FL("Delta = %d\n", *delta);
	//delta = BOUND( *delta, lim ) ; /* at most move the slot <lim>ms per round */
	
		
	/* update slot position and width  */
	slot_limits_t tmpslot = TDMA_getOwnSlot() ; //current B e E


	PRINTF_FL_WARN(
	"cur width %"PRIu16", "
	"NEW slot [%"PRIu16",%"PRIu16"]ms\n" ,
	getSlotWidth_givenIp( Param_tdma.node_ip ),
	tmpslot.begin_ms,
	tmpslot.end_ms ) ;


	shiftSlot(&tmpslot, *delta);
	
	PRINTF_FL_WARN(
		"cur width %"PRIu16", "
		"NEW slot [%"PRIu16",%"PRIu16"]ms\n" ,
		getSlotWidth_givenIp( Param_tdma.node_ip ),
		tmpslot.begin_ms,
		tmpslot.end_ms ) ;

	if ( tmpslot.begin_ms > Param_tdma.round_period_ms )
		return -1; /* impossible !*/
	if ( tmpslot.end_ms > Param_tdma.round_period_ms )
		return -2;  /* impossible ! */

	/* if we change the slot, change BOTH bounds at the same time*/
	setOwnSlot_safe(tmpslot);

	#if 1 //"erase" pkt delay array
	pthread_mutex_lock( &Mutex_delay ) ; /* protect from editing - getDelay could be writing at the same time */
	Pktdelay_array_counter = 0 ; /* erase all delays. we moved the slot. so all current delays are wrong  */
	pthread_mutex_unlock( &Mutex_delay ) ; /* protect from editing */
	#endif
	
	/* some debug */
	#ifdef VERBOSE
	if ( *delta != 0 ) tdma_printSlot( Myslot );
	#endif

	return 3 ; //success
}

void shiftSlot( slot_limits_t *tmpslot, int32_t delta)
{
	/* init time is current + average delay  */
	tmpslot->begin_ms += delta ; 
	tmpslot->begin_ms %= Param_tdma.round_period_ms ; /* bound this to period */
	tmpslot->end_ms = tmpslot->begin_ms + 
		getSlotWidth_givenIp( Param_tdma.node_ip )  ; /* use the new slot width */
	tmpslot->end_ms %= Param_tdma.round_period_ms ; /* bound this to period */
}

/** read TDMA header of a pckt , 
 * analyze its netwkr delay, 
 * save delay**/
error_t tdma_recordPktDelay( tdma_header_t tdma_header ) 
{

	float delay_ms = tdma_getDelay( tdma_header ) ;
	if ( delay_ms < -1000){
		PRINTF_FL("No delay Computed\n");
		return E_OTHER ; /* no delay computed */
	}

	delay_ms -= 0; /* compensate transmisson delay*/
	
	pthread_mutex_lock( &Mutex_delay ) ; /* protect from editing - tdma_syncronizeSlot edits stuff too */
	Pktdelay_array_counter++ ; /* keep track of how many samples we are using to sync our 'clock', i.e., slot */
	PRINTF_FL("Pktdelay array counter: %ld", Pktdelay_array_counter);
	Pktdelay_array[Pktdelay_array_counter] = delay_ms ; /* pos 0 is not used */
	Pktdelay_array_sender[Pktdelay_array_counter] = tdma_header.slot_id;
	pthread_mutex_unlock( &Mutex_delay ) ;
	PRINTF_FL("Delay %f\n", delay_ms );

	return E_SUCCESS ; /* delay saved */

}



/** first TDMA slot limits definition  **/
error_t tdma_initSlotLimits(void)
{
	PRINTF_FL_WARN(
		"I'm slot %u, width %ums\n",
		Param_tdma.slot_id,
		Param_tdma.standard_width_ms);
	/* my default is to have 1/N*period of slot */
	
	if (Param_tdma.sync_flag) 
	{
		tdma_slot_setstd();
	}
	else
	{
		///* if no sync is requested, 
		 /* then use the whole Period to transmit - 
		  * guardperiod is still in use anyways whiles ending 
		 * CSMA takes care of MAC contention*/
		slot_limits_t tmpslot;
		tmpslot.begin_ms = 0;
		tmpslot.end_ms = Param_tdma.round_period_ms; 
		setOwnSlot_safe(tmpslot);
	}
	
	
	
	#ifdef VERBOSE
	tdma_printSlot(TDMA_getOwnSlot()) ;
	#endif
	
	return E_SUCCESS ;
}

/* set standard slot length */
void tdma_slot_setstd(void)
{
	slot_limits_t tmpslot;
	tmpslot.begin_ms = 
			Param_tdma.standard_width_ms*(Param_tdma.slot_id-1)+0; 
		
	tmpslot.end_ms=tmpslot.begin_ms+Param_tdma.standard_width_ms+0 ;
	setOwnSlot_safe(tmpslot);
}

/** zero all tables and variables **/
void tdma_clearStuff(void)
{
	
	tdma_initSlotLimits(); /* begin end  set */
	
	/* clear requests */
	#if VSP
	clearRequests();
	#endif
	
	/* update table-knowledge of slot widths */
	for (
		uint8_t slot_id = 1; 
		slot_id <= MAX_N_SLOTS; 
		slot_id++
		)
	{
		setSlotWidth(slot_id, Param_tdma.standard_width_ms );
	}
	
	
	/* current (local) knowledge of the pair (Slot-id ; IP ) */
	/* we typically do not change (IP,slotID) but it might change */
	CLEAR(Table_ip_2_slotid);
	Table_ip_2_slotid[ Param_tdma.node_ip ] = Param_tdma.slot_id ;
	
	
	
}


/* Return the IP of the node down stream (nxt Slot Id) */
uint8_t TDMA_dwStream( void )
{

	uint8_t dw_slot_id = Param_tdma.slot_id + 1 ;
	//if ( dw_slot_id > Param_tdma.slot_total ) 
		//dw_slot_id = 1 ;
	return tdma_getIp_givenId( dw_slot_id ) ;
}


/* Return the IP of the node upstream (prev Slot Id) */
uint8_t TDMA_upStream( void )
{
	//if ( Param_tdma.slot_total == 1 )
	//	return 0 ; /* ignore */
	uint8_t up_slot_id = Param_tdma.slot_id - 1 ;
	//PRINTF_FL("Slot: total %d, mine %d, TDMA_upStream #%d\n",
		//Param_tdma.slot_total , 
		//Param_tdma.slot_id ,
		//tdma_getIp_givenId( up_slot_id ) );
	//if ( up_slot_id == 0 ) 
		//up_slot_id = Param_tdma.slot_total ;
	return tdma_getIp_givenId( up_slot_id ) ;
}





void setOwnSlot_safe( slot_limits_t tmpslot )
{
	pthread_mutex_lock( &Mutex_slot ); 
	Myslot.begin_ms = (uint16_t)tmpslot.begin_ms  ;
	Myslot.end_ms 	= (uint16_t)tmpslot.end_ms  ;
	pthread_mutex_unlock( &Mutex_slot );
	return;
}




/** get ip of a given Slot ID **/
uint8_t tdma_getIp_givenId( uint8_t slot_id )
{
	if ( slot_id == 0 )
		return 0; /* not valid */
		
	for ( uint8_t ip = 1 ; ip <= MAX_IP ; ip++ )
		if ( getId_givenIp( ip ) == slot_id )
			return ip ;
	return 0 ;/* not valid */
}



/** **/
int32_t	tdma_getLastRecordedPktDelay(void)
{
	int32_t delay ;
	pthread_mutex_lock( &Mutex_delay ) ; /* protect from editing - tdma_syncronizeSlot edits stuff too */
	delay = Pktdelay_array[Pktdelay_array_counter] ; /* pos 0 is not used */
	pthread_mutex_unlock( &Mutex_delay ) ;
	return delay ;
}



///** Set slot id associated with a given IP **/
void tdma_setIp2SlotId( uint8_t ip , uint8_t id )
{
	Table_ip_2_slotid[ ip ] = id ; /* maximum slot id . 0 is not used */
}






/*******************************
 * private functions go here 
 * *************************
 * *****************************/


/** define a new width for the slot **/
void setSlotWidth( uint8_t slot_id, uint16_t width_ms)
{
	if ( ARRAYSIZE(Table_slotid_2_slotwidth) <= slot_id)
	{
		PRINTF_FL_ERR("invalid input id%"PRIu8 "--- %"PRIu16"ms\n",
			slot_id ,
			width_ms);
		return; /* error */
	}
	Table_slotid_2_slotwidth[slot_id] = width_ms;
	PRINTF_FL("iSlotd id%"PRIu8 " has %"PRIu16"ms\n",
			slot_id ,
			width_ms);
}


/* VSP functions - TODO: move to a new file */

#if VSP
/** flag if we have a outging requests or not
 * return:
 * 	slot_id >> we have an outgoing request with that ID
 * 	0 , no outgoing requests.
 *  100, still going  **/
int underOutgoingRequest(void)
{
	for (int slot_id = 1 ; 
		slot_id <= Param_tdma.slot_total ; 
		slot_id++
		)
	{
		if (slot_id == Param_tdma.slot_id)
			continue ; /* not possible.*/

		if ( 0 != Requests_slot_width_ms[slot_id] )
		{
			//PRINTF_FL("There's a outgoing req (slot-id%"PRIu8") -> (%"PRIu16"ms)\n", 
				//slot_id, Requests_slot_width_ms[ slot_id ] ) ;
			return slot_id ;
		}
	}
	
	/* 
	 * outgoing is only over when we efectively 
	 * sync & change our slot width.
	 * so flag we are UnderOut = 1 . 
	 * My_slot width should be the same as the one in
	 * Table_slotid_2_slotwidth[] array */
	if ( TDMA_getOwnSlotWidth() != getSlotWidth_givenId( Param_tdma.slot_id ) )
	{
		return 100 ;		
	}
	
	
	return 0 ; /* No- i'm free */
}
#endif


#if VSP
/** given Ip, retrun requested width from there **/
int16_t tdma_getRequestedWidth( uint8_t ip )
{
	return Requests_slot_width_ms[ Table_ip_2_slotid[ ip ] ];
}
#endif


#if VSP
/** **/
int underIncomingRequest(void)
{
	//TODO: USE BITWISE operations instead of ARRAY for binary_arrays
	/* 
	 * if there's one Under_incoming flag equal to true
	 * this means we are still processing a previous incoming request 
	 * */
	for (	int slot_id = 1 ; 
				slot_id <= Param_tdma.slot_total ; 
				slot_id++
		)
	{
		if ( Under_incoming[ slot_id ] == 1 )
		{
			return slot_id ;
		}
	}
	
	
	///* not under any in-request:*/
	////...
	
	///* THIS SHOULD NOT HAPPEN */
	//if ( TDMA_getOwnSlotWidth() != getSlotWidth_givenId( Param_tdma.slot_id ) )
	//{
		//PRINTF_FL_ERR(
			//"ERROR My_slot is %"PRIu16 "ms. " 
			//"however, i received req to change to %"PRIu16"ms." 
			//"check whole protocol \n\n\n\n",
			//TDMA_getOwnSlotWidth(),
			//getSlotWidth_givenId( Param_tdma.slot_id ) ) ;
		//return 100 ;
	//}
	
	return 0 ; /* Nope */
}
#endif


#if VSP
/* dvsp related */
void clearRequests(void)
{
	CLEAR( Requests_slot_width_ms ) ;
	CLEAR( Under_incoming ) ;
}
#endif

#if VSP
void processIncomingRequest( tdma_header_t tdma_header )
{
	/* find delta (increase/shrink) 
	 * <request_slot_width_ms>: slottime sender wants I have. 
	 * <delta_ms>: slottime INCREASE sender wants I do */
	int32_t delta_ms = 
		(int32_t)tdma_header.request_slot_width_ms - 
		(int32_t)getSlotWidth_givenId( Param_tdma.slot_id ) ;

	int32_t current_sender_slot_width_ms = 
		(int32_t)tdma_getSlotWidth( tdma_header.slot ) ; 
	
	/* check error */
	if ( current_sender_slot_width_ms < delta_ms ) 
	{
		PRINTF_FL_ERR("ERROR - ID%u has %dms\n" , 
			tdma_header.slot_id,
			current_sender_slot_width_ms ) ;
		PRINTF_FL_ERR(
			"ERROR - ID%u is asking me to change from %dms to %dms\n" ,
			tdma_header.slot_id,
			(int32_t)getSlotWidth_givenId( Param_tdma.slot_id ),
			(int32_t)tdma_header.request_slot_width_ms  ) ;
		return ;
	}
	
	/* 
	 * Write down my NEW slot width (in the table). 
	 * During syncrhonization stage, 
	 * this number will be used to *update* our slot 
	 * */ 
	setSlotWidth( Param_tdma.slot_id, tdma_header.request_slot_width_ms);
	
	/* sender will be sending me this slot width next time, 
	 * so lets update our knowledge .
	 * later we will chck if this is actualyl updated. */
	setSlotWidth( 
		tdma_header.slot_id, 
		tdma_getSlotWidth( tdma_header.slot ) - 
		(uint16_t)delta_ms);
	//PRINTF_FL_WARN("current knowledge:");
	//for (uint8_t id=1;id<3;id++)
		//printf("[%d]",getSlotWidth_givenId(id));
	//printf("\n");
	
	//PRINTF_FL_ERR(	
		//"Incoming Request: "
		//"self [%" PRIu16 "]->[%" PRIu16 "]ms (from ID%"PRIu8")\n",
		//TDMA_getOwnSlotWidth(),/* current */
		//getSlotWidth_givenId(Param_tdma.slot_id), /* new value */
		//tdma_header.slot_id ) /* sender's ID */;
	
	Under_incoming[ tdma_header.slot_id ] = 1 ; /* Lock ourselves */
}
#endif


#if VSP
void processOutgoingRequest( tdma_header_t tdma_header )
{
	/* sender is already using the new slot width? */
	int req_sender_accepted = 
			Requests_slot_width_ms[ tdma_header.slot_id ] == 
			tdma_getSlotWidth( tdma_header.slot ) ;
	
	if (!req_sender_accepted) 
		return; /* still waiting for a change */
		
	/* finish handshake! */	

	/* sender-node has now a slotwidth equal to what we requested 
	 * so: 
	 * update my slot, and knowledge of its slot
	 * */
	 
	/* sender-slot shrinked by <delta_ms> */
	int32_t delta_ms = 
		(int32_t)getSlotWidth_givenId(tdma_header.slot_id) -
		(int32_t)tdma_getSlotWidth( tdma_header.slot ) ;
	
	/* so we now INCREASE our slot by <delta_ms> */
	setSlotWidth(
		Param_tdma.slot_id,
		(uint16_t)
		(
		(int32_t)getSlotWidth_givenId(Param_tdma.slot_id) + /*mine*/
		(int32_t)delta_ms
		)) ;
	
	/* *update* my knowldge of the sender */
	setSlotWidth(
		tdma_header.slot_id,
		tdma_getSlotWidth( tdma_header.slot ) 
	);
				
	
		
	

	/* check inconsistencies - print all slots */
	//uint16_t sum = 0 ;
	//PRINTF_FL_ERR("[");
	//for ( int i = 1 ; i <= Param_tdma.slot_total ; i++ )  
	//{
		//if ( i == Param_tdma.slot_id )
			//fprintf( stderr , "*" ) ;
		//fprintf(stderr,"%" PRIu16 ,  getSlotWidth_givenId(i) ) ;
		//if ( i == Param_tdma.slot_id )
			//fprintf( stderr , "*" ) ;
		//if ( i < Param_tdma.slot_total )
			//fprintf( stderr , "|" ) ; 
		//sum += getSlotWidth_givenId(i ) ;
	//}
	//fprintf(stderr,"]ms -");
	//fprintf(stderr, " T=%" PRIu16 "ms\n", sum );

	/* erase my original outgoing request*/
	//PRINTF_FL_WARN(
		//"SlotID%"PRIu8" accepted my req!. UNLOCK\n" ,
		//tdma_header.slot_id ) ;
	
	/* stop sending requests. 
	 * we will consider under Outrequest while
	 * our actual slot width is diff than the knowledge table */
	Requests_slot_width_ms[tdma_header.slot_id] = 0 ;
	
	

	
	//return;
}
#endif

#if VSP
void processIncomingRequest2(tdma_header_t tdma_header)
{
	if ( 
			getSlotWidth_givenId(tdma_header.slot_id) == 
			tdma_getSlotWidth(tdma_header.slot) 
		)  
	{
		Under_incoming[tdma_header.slot_id] = 0 ;
		//PRINTF_FL_WARN(
			//"Requester id%"PRIu8" has now %dms. UNLOCK\n", 
			//tdma_header.slot_id,
			//tdma_getSlotWidth(tdma_header.slot)
		//);
	}
}
#endif

#if VSP
/** **/
void tdma_runRequests( tdma_header_t tdma_header ) 
{
	/* error checks */
	if ( tdma_header.slot_id > Param_tdma.slot_total )
		return ; /* not valid, ignore */

			
	
	int ret= underOutgoingRequest();
	if (ret)
	{	
		if ( ret == (int)tdma_header.slot_id )
		{
			/* try to finish an outgoing request. 
			 * we got a packet from the node we are 
			 * currently handshaking with.
			 * check if it is completed. */
			/* process a pending outgoing request. */
			processOutgoingRequest(tdma_header) ;
			return;
		}
		/* othrwise, we just deny processing more requests */
		//PRINTF_FL_WARN(
			//"we are still under out. req with id%d\n",
			//ret);
		return ;
	}
	
	
	
	/* we are not under outgoing requests, 
	 * check incoming requests, now */
	 
	
	int ret2 = underIncomingRequest() ; 
	if (ret2)
	{
		/* check if sender is the one previously requesting a slot change
		 * and check if it has the new slot size as expected. 
		 * this means handshake as reeached an END */
		if ( ret2 == (int)tdma_header.slot_id )
		{
			processIncomingRequest2(tdma_header);
			return;
		}
		/* othrwise, we just deny processing more requests */
		//PRINTF_FL_ERR(
			//"Denying in req from id %"PRIu8","
			//" cause we are stil in-working with %d\n",
			//tdma_header.slot_id ,
			//ret2 );
		return ;	
		
	}
	
	
	/* no in or out requests are currently operatin. 
	 * check if we got a new one*/
	
	/* do we have an incoming request ? */
	if (!tdma_header.request_slot_width_ms ) 
		return;/*no*/
	
	/* we have one new in-request: */
	
	/* is it NEW? 
	 * someone requested we set slot to:
	 * <tdma_header.request_slot_width_ms>
	 * Our current width is:
	 * <getSlotWidth_givenId( Param_tdma.slot_id )> */
	int req_new = 
		tdma_header.request_slot_width_ms != 
		getSlotWidth_givenId( Param_tdma.slot_id ) ;
	if ( !req_new ) 
		return; /* not new. return */
		
	//PRINTF_FL_WARN(
		//"incom. req from id%" PRIu8 "-> %"PRIu16"ms\n", 
		//tdma_header.slot_id ,
		//tdma_header.request_slot_width_ms );
	
	
	
	/* we have a new incoming request, size:
	 * <tdma_header.request_slot_width_ms>*/	
	
	/* Let's process this Incoming request */
	//PRINTF_FL_WARN("processing incoming reqst!\n");
	processIncomingRequest( tdma_header ) ;
	
	
	
}
#endif



#if VSP
/** analyze current IN and OUT bandwidth,
 * decide on the new slot size for upstream neigh and me
 * **/
error_t tdma_makeRequest(void)
{
	/* do we need to run this? */
	if ( !(Param_tdma.vsp) || !( getEpoch() - last_outreqst > 2 ) )
		return -10; /* nothing to be done */

	const uint8_t ip_up = TDMA_upStream();
	/* Bandwidth.out[0] = bandwidth out to EVERYONE */
	const uint8_t ip_dw = 0 ; 
	if ( (Bandwidth.out[ip_dw] < -1) || (Bandwidth.in[ip_up] < 10) )
	{
		//tdma_printBin( ip_up ) ;
		return -20; /* nothing to be done */
	}
	
	/* let's run requests to our UPSTREAM neighbor */
	last_outreqst = getEpoch();
					
	/* BW ratio */
	float ratio = 
		Bandwidth.in[ ip_up ] / 
		(Bandwidth.in[ ip_up ] + Bandwidth.out[ ip_dw ] ) ;
	
	/* debug */	
	#ifdef PRINTSTATS
	tdma_printBin(ip_up);
	tdma_printBout(ip_dw);
	#endif
	
	/* sum of slots (ms) */
	const float slot_sum = 
		(float)getSlotWidth_givenIp( ip_up ) + 
		(float)getSlotWidth_givenIp( Param_tdma.node_ip ) ;
	PRINTF_FL_WARN("C = [%.0f]+[%.0f*] = %.0f\n" ,
		(float)getSlotWidth_givenIp( ip_up ),
		(float)getSlotWidth_givenIp( Param_tdma.node_ip ) ,
		(float)getSlotWidth_givenIp( ip_up ) +
		(float)getSlotWidth_givenIp( Param_tdma.node_ip ) ) ;
	
	/**
	 * new slot width is computed based on:
	 * the BW ratios OR
	 * in case we have too many buffered pkts, 
	 * on the time needed to send them
	 * **/
	#define BUF_USE		1.0*(tdma_getTxBufferUse()+PM_getTxBufferUse())
	#define BUF_BYTES 	(0.95 * 110 * BUF_USE) /* B */
	#define BOUT		(Bandwidth.out[ip_dw])/* KB/s */
	const float time_to_send_buffered = 
		BUF_BYTES / BOUT ; /* ms */
	float new_MY_slot = MAX( 
		ratio*slot_sum , /* theoretical slot (steadystate) - ms */
		time_to_send_buffered /* time needed to send all buffered - ms */
		) ;
	new_MY_slot = round( new_MY_slot );
	new_MY_slot = MIN( new_MY_slot, slot_sum-10 ) ; /* top limit */
	
	/* some debug */
	//PRINTF_FL_WARN(
		//"ratio*slotsum=%.2f*%.0f=%.1fms || "
		//"Buf/Bout =%.0fKB/%.0fKBps=%.1fms\n" ,
		//ratio,
		//slot_sum,
		//ratio*slot_sum,
		//BUF_BYTES/1e3,
		//BOUT,
		//time_to_send_buffered ) ;
	
	
	
	/* some debug */
	//PRINTF_FL("| %.0f <-|-> *%0.0f* | ms \t(old=| %.0f | *%0.0f* |)\n" , 
		//slot_sum - new_MY_slot ,
		//new_MY_slot ,
		//(float)getSlotWidth_givenIp( ip_up ),
		//(float)getSlotWidth_givenIp( Param_tdma.node_ip ) 
		//) ;
	//PRINTF_FL(
		//"#pkts current queued: %.0fpkts\n" , 
		//BUF_USE );
	#undef BOUT
	#undef BUF_USE
	#undef BUF_BYTES
	
	
	/* call actual request */	
	#define LIMITT 6
	float delta = 
		new_MY_slot - 
		(float)getSlotWidth_givenIp( Param_tdma.node_ip ) ;
	if (LIMITT < delta)
		new_MY_slot = 
			(float)getSlotWidth_givenIp( Param_tdma.node_ip ) + 
			LIMITT ;
	else if (-LIMITT > delta)
		new_MY_slot = 
			(float)getSlotWidth_givenIp( Param_tdma.node_ip ) - 
			LIMITT ;
	#undef LIMITT
	
	/* make request */
	TDMA_reqSlotWidth( 
		(uint16_t)(slot_sum - new_MY_slot), ip_up ) ;
	return E_SUCCESS;

}
#endif





#undef TDMA_SLOT_IMPORT
