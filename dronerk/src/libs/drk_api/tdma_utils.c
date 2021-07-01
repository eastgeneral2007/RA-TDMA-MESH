/**********************************************
 * ************   TDMA UTILS MODULE ***********
 *
 * a good set of tools used by main TDMA module
**********************************************/
#define TDMA_UTILS_IMPORT
#include "drk/tdma_utils.h" /*  my own */

/** sibling headers **/
#include "drk/tdma.h" /*   */
#include "drk/tdma_slot.h" /*   */
#include "drk/tdma_types.h" /* */

/** other modules **/
#include "drk/utils.h" /* generic methods and data types used across the files */
#ifdef SIMULATOR
#include "drk/sim_clock.h"
#endif
#include "drk/pdr.h"


/** system headers **/
#include <arpa/inet.h> /* udp packets */
#include <errno.h> /* strerror */
#include <inttypes.h> /* int8_t etc */
#include <ifaddrs.h> /* udp packets */
#include <linux/sockios.h> // SIOCOUTQ */
#include <math.h> /* some math at some point */
#include <netinet/in.h> /* udp packets */
#include <pthread.h> /* we have threads */
#include <stdio.h> /* printf */
#include <stdlib.h> /* atoi*/
#include <string.h> /* memset, memcpy */
#include <sys/time.h> /* */
#include <sys/ioctl.h> /* ioctls */
#include <sys/wait.h> /* waitpid */
#include <unistd.h> /* sleep */





#define PREFIX "TDMA_U"



/** externable for tdma* modules **/
volatile bandwidth_t 	Bandwidth ; /* estimation of bandiwdth to/from other neighbor nodes (Kbps) */
volatile tdma_stats_t 	Tdma_statistics ; /* save stats here  */
volatile param_tdma_t 	Param_tdma ; /* all local TDMA *constants* are saved here, loaded from a file */
volatile tdma_header_t Header; /*header from a packet */

/*
 *  Adicionado Dino
 */
uint32_t TDMA_getSeqNum( void ){
	return Header.seq_num_gen;
}

int TDMA_getRoundTime( void ){
	// uint16_t
	return 1000*Param_tdma.round_period_ms;
}

void sleepToStart ( void ){
	tdma_sleepTillSlotBegin();
}

/*----------------------------------*/


/** **************************
 * * Public functions - any module/binary can use
 * **************************/

/** Get slot width based on its limits (and PERIOD) **/
uint16_t tdma_getSlotWidth( slot_limits_t slot )
{
	if ( slot.begin_ms <= slot.end_ms )
		return slot.end_ms - slot.begin_ms  ;

	return ( Param_tdma.round_period_ms-slot.begin_ms ) + slot.end_ms ;
}

uint16_t TDMA_getOwnSlotWidth( void )
{
	return tdma_getSlotWidth( TDMA_getOwnSlot() ) ; 
}


/**  estime KB/s entering and leaving the node **/
bandwidth_t TDMA_getBandwidth( void ) 
{
	return Bandwidth ;
}







/***** semi private 
 * 
 * *************************/


/** 
 * returns 0, if inside slot
 * otherwise, -1 and prints warn message 
 * **/
int tdma_debugInsideSlot(void)
{
	float t_rnd = tdma_getCurrentRoundTimeD();
	slot_limits_t slot = TDMA_getOwnSlot();
	if ( !tdma_insideSlot() )
	{
		PRINTF_FL_WARN( 
			"Sending @ t_r =%.2fms - outside our slot "
			"[%"PRIu16",%"PRIu16"]ms\n" , 
			t_rnd ,
			slot.begin_ms,
			slot.end_ms 
			);
			return -1;
	}
	return 0;
}


/**
 * @brief retrives the current round time
 * 
 * @return negative means error code, 
 * positive is the current time in the round
 */
int32_t tdma_getCurrentRoundTime( void )
{
	struct timespec time_temporary ; /* to timestamp_ms pkts */
	int ret ;
	if ( (ret = getCurrentTime( &time_temporary ) ) < 0 ) /* get current epoch time */
	{	
		PRINTF_FL_ERR("Time ERROR!\n");
		return ret ;
	}
	
	/* get cur round time -> within period */
	const uint16_t cur_time_ms = ( uint16_t )
		(
			( (int64_t)( (double)time_temporary.tv_sec * 1000.0 +
			(double)time_temporary.tv_nsec / MILLION ) ) %
			( (int64_t)Param_tdma.round_period_ms ) 
		) ; 
	
	return (int32_t)cur_time_ms ;
}

double tdma_getCurrentRoundTimeD( void )
{
	struct timespec time_temporary ; /* to timestamp_ms pkts */
	int ret ;
	if ( (ret = getCurrentTime( &time_temporary ) ) < 0 ) /* get current epoch time */
	{	
		PRINTF_FL_ERR("Time ERROR!\n");
		return ret ;
	}
	
	#define GC_SECS		((int64_t)(time_temporary.tv_sec))
	#define GC_NANOSECS	((int64_t)(time_temporary.tv_nsec)) 
	#define GC_PERIOD	((int64_t)Param_tdma.round_period_ms)
	/* get cur round time -> within period */
	double cur_time_ms = ( double )
		(
			(GC_SECS*1000 + GC_NANOSECS / 1000000 ) 
			%
			GC_PERIOD 
		) ;
	cur_time_ms += -(GC_NANOSECS/1000000) + (GC_NANOSECS/1000000.0)  ; /* ms - fractional part of epoch (in ms) */
	
	//PRINTF_FL("nano %"PRId64", msecs = int%"PRId64", fractional part %f\n", 
		//GC_NANOSECS,
		//GC_NANOSECS / 1000000, 
		//-(GC_NANOSECS/1000000) + (GC_NANOSECS/1000000.0) );
	#undef GC_SECS
	#undef GC_NANOSECS
	#undef GC_PERIOD
	
	return cur_time_ms ;
}



/** Print <slot> limits **/
#ifdef VERBOSE
void tdma_printSlot( const slot_limits_t slot ) 
{

	PRINTF_FL( "Current slot: "
		"[%" PRIu16 ", "
		"%" PRIu16 "]ms\n" , 
		slot.begin_ms , 
		slot.end_ms ) ;
}
#endif





/** 
 * load all stuff network related : 
 * throughput, and slots 
 * **/
error_t tdma_loadSettings( uint8_t my_id ) 
{
	Param_tdma.slot_id = my_id ;
	Param_tdma.node_ip = (uint8_t)getMyIP() ;
	error_t ret ;
	if ( ( ret = loadThroughputSettings() ) < 0 ) 
		return ret ;
	if ( ( ret = loadSlotSettings() ) < 0 ) 
		return ret ;

	
	return E_SUCCESS ; /* ok */
}


/**
 * return pkt delay (computed for each rx packet) 
 * if pkt arrives before we expect, has negative delay
 * only computes delay if this comes from a neighbor slot
 * return INT32_MIN otherwise **/
int32_t tdma_getDelay( tdma_header_t tdma_header )
{
	if ( tdma_header.slot_id > Param_tdma.slot_total )
		return -10000 ; /* ignore */
		
	/* compute position of this packt inside sender's slot */
	float msg_position = tdma_getMsgPosition(tdma_header) ; /* value in [0 - senderslotwidth]ms */ // TODO: uint16_t please
	
	PRINTF_FL_ERR(
		"Sender: slot.init [%d]ms, "
		"Msg.timestmp [%"PRIu16"ms] => "
		"msg_position [%fms]\n",
		tdma_header.slot.begin_ms,
		tdma_header.timestamp_ms ,
		msg_position ) ;

	/* Find difference between slot order of myself and sender */
	float slot_difference ; /* next slot: returns 1, previous slot: -1 */
	slot_difference = (float)tdma_header.slot_id - (float)Param_tdma.slot_id ;
	/* first and last slots are neighbor slots */
	if ( +slot_difference == Param_tdma.slot_total - 1 ) 
		slot_difference = -1 ;
	else if ( -slot_difference == Param_tdma.slot_total - 1 ) 
		slot_difference = 1 ;
			
	//if ( (slot_difference > 1) || (slot_difference < -1 ))
		//return -10000 ; /* ignore sync'ing pkts that are not direct slot-neighbors */
	
	//slot_difference = (int64_t)Param_tdma.slot_total + (int64_t)slot_difference ;
	/* *************/

	/* get time where sender's slot should have started, acording to my current slot */
	//TODO change this to a switch
	float ideal_slot_init_of_sender = 0 ;
	slot_limits_t tmp_slot = TDMA_getOwnSlot();
	if ( slot_difference == 1 )
		ideal_slot_init_of_sender = (int32_t)tmp_slot.end_ms ; 
	else if ( slot_difference == -1 )
		ideal_slot_init_of_sender = 
			(int32_t)tmp_slot.begin_ms - (int32_t)tdma_getSlotWidth( tdma_header.slot ) ;
	else{
		ideal_slot_init_of_sender = tmp_slot.begin_ms + slot_difference * Param_tdma.standard_width_ms + Param_tdma.round_period_ms;
	}
	
/*
 * 43ms | mys [39 49]ms |
 *  hdr [03 03 57]ms 2/3 | 
 * D 00ms
 * */
/*
 * slot diff = 2-1 = 1
 * ideal_slot_init_of_sender = endofslot = 49
 * curtime = 43
 * msg_position = 0 
 * pkt_delay_ms = 43 - 0 - 49  = -6 = 90 = -6
 * 
 * */
	PRINTF_FL( "slot-diference %f, Sender slot should start at %f ms\n",
		slot_difference, ideal_slot_init_of_sender) ;
	float cur_time_ms = (float)tdma_getCurrentRoundTimeD();/* current time in ms, modulo TDMA_ROUND_PERIOD_MS */
	PRINTF_FL( "mmytime %fms, slot-diference %f, Sender slot should start at %fms\n",
		cur_time_ms, slot_difference, ideal_slot_init_of_sender) ;
	
	/* compute pkt delay : */
	float pkt_delay_ms = 
		cur_time_ms -
		msg_position - 
		ideal_slot_init_of_sender ;
	
	/* convert to a positve number */
	while (pkt_delay_ms < 0)
		pkt_delay_ms += Param_tdma.round_period_ms ;
	
	/* now we can make modulo safely */
	pkt_delay_ms = fmodf(pkt_delay_ms, Param_tdma.round_period_ms); /* notneeded*/
	
	/* being delayed more than half period, is the same as being EARLY */
	if (pkt_delay_ms > Param_tdma.round_period_ms/2)
		pkt_delay_ms -= Param_tdma.round_period_ms ;


	//if ( msg_position <1 )
	//PRINTF_FL_ERR( "DELAY %" PRId64 "ms - "
		//"curtime[%" PRId64 "], "
		//"myslot[%" PRIu16 "], "
		//"ideal snder slt init at[%" PRId64 "] "
		//"slt_diff %" PRId64 "\n", 
		//pkt_delay_ms ,
		//cur_time_ms,
		//myslot.begin_ms,
		//ideal_slot_init_of_sender ,
		//slot_difference ) ;

	return (float)pkt_delay_ms ; 
}



/** return true (1) or false (0) 
 * whether current round time is 
 * within slot boundaries or not **/
int tdma_insideSlot_guarded( void ) 
{
	slot_limits_t slot_limits = tdma_getOwnSlot_guarded();
	int32_t t_rnd_ms = tdma_getCurrentRoundTime();
	if ( t_rnd_ms<0 )
		return 0;

	int condition = -1 ; /* init as false */
	
	if ( slot_limits.begin_ms < slot_limits.end_ms ) 
		condition = ( (uint16_t)t_rnd_ms >= slot_limits.begin_ms ) &&
			( (uint16_t)t_rnd_ms < slot_limits.end_ms  ) ;
	else
		condition = ( (uint16_t)t_rnd_ms >= slot_limits.begin_ms ) || 
			( (uint16_t)t_rnd_ms < slot_limits.end_ms ) ;
	
	return condition ; 
}


/** return true (1) or false (0) 
 * whether current round time is 
 * within slot boundaries or not **/
int tdma_insideSlot( void ) 
{
	slot_limits_t slot_limits = TDMA_getOwnSlot();
	int32_t t_rnd_ms = tdma_getCurrentRoundTime();
	if ( t_rnd_ms<0 )
		return 0;

	int condition = -1 ; /* init as false */
	
	if ( slot_limits.begin_ms < slot_limits.end_ms ) 
		condition = ( (uint16_t)t_rnd_ms >= slot_limits.begin_ms ) &&
			( (uint16_t)t_rnd_ms < slot_limits.end_ms  ) ;
	else
		condition = ( (uint16_t)t_rnd_ms >= slot_limits.begin_ms ) || 
			( (uint16_t)t_rnd_ms < slot_limits.end_ms ) ;
	
	return condition ; 
}






uint16_t tdma_getType(const tdma_header_t * const header)
{		
	return (header->type) & 0x00FF ; /* get lower byte */
}

uint16_t tdma_getPrio(const tdma_header_t * const header)
{
	return (header->type) >> 8; /* get higher byte */
}



/* log tx/rx packet into a file */
#ifdef LOGDATA
void tdma_logPacket( 
	FILE *logfile, 
	tdma_header_t *header, 
	double timestamp, 
	uint8_t src_ip, 
	uint8_t dest_ip, 
	uint16_t tdma_pkt_len, 
	uint16_t buf_use,
	float bandwidth, 
	error_t delay_error /* */
	)
{
	/* grab time stamps */
	//struct timespec curtimee ;
	//getCurrentTime( &curtimee) ; 
	//double epoch_time = (double)( curtimee.tv_sec )  +
		//(double)( curtimee.tv_nsec )/BILLION ;
	//uint64_t round_time = ((uint64_t)((double)curtimee.tv_sec*1000.0 +
		//(double)curtimee.tv_nsec/MILLION)) % 
		//( (uint64_t)Param_tdma.round_period_ms ) ;
	
	char other_string[30];
	if (PM == tdma_getType(header))
	{
		buildPMSeqnumString(header, other_string);
		
	}
	else
		snprintf(other_string,sizeof(other_string), "pms# 0"); /* it is not a pm pkt */


	int32_t delay ;
	if ( 0 < delay_error )
	{
		delay = tdma_getLastRecordedPktDelay();/* no error */
	}
	else
	{
		delay = 0 ;
	}

	//int32_t cur_time_ms = tdma_getCurrentRoundTime() ;
	double cur_time_ms = tdma_getCurrentRoundTimeD();
	double cur_epoch = getEpoch();
	slot_limits_t tmpslot = TDMA_getOwnSlot();
	/* write a line */
	char file_buf[250]; 
	snprintf( file_buf, sizeof(file_buf),
		"%.6fs "
		"%.2fms | "
		//"%02" PRId32 "ms | "
		"mys [%02" PRIu16 " %02" PRIu16 "]ms | " /* my local slot */
		"c %"PRIu64" | " /* slot counter */
		"%07.3fms | " /* delay time spent in the queue */
		"%02" PRIu16 "pkts | " /* #pkts in queue */
		"hdr [%02" PRIu16 " %.2f %02" PRIu16 "]ms "
		"%" PRIu8 "/%" PRIu8 " | "  /* id/total */
		"Rq %02" PRIu16 "ms | " /* requesting */
		"s# %02" PRIu8 "%02" PRIu8 "%06" PRIu32 " | " /* link tx-rx-seqnum*/
		"[#%02" PRIu8 "]->[#%02" PRIu8 "] | " /* src - dst */
		"T %"PRIu16" | " /* type of packet - pm , pdr , other tdma-module specific */
		"%04" PRIu16 "B | " /* tdma pkt size */
		"D %02" PRId32 "ms | "
		"Bw %03.1fBps | "
		"%s",
		cur_epoch , /* epoch - seconds */
		cur_time_ms,  /* current time in ms, modulo round_period_ms*/
		tmpslot.begin_ms , tmpslot.end_ms , /* my current slot */
		Tdma_statistics.slot_round_counter , /* current slot counter */
		(timestamp == 0 ? 0 : 1e3*(cur_epoch-timestamp)) , /* time spent in the queue - ms*/
		buf_use , /*  */
		header->slot.begin_ms , header->timestamp_ms + header->timestamp_partms/256.0 , header->slot.end_ms ,
		header->slot_id , Param_tdma.slot_total , /* id/total */
		#if VSP
		header->request_slot_width_ms ,
		#else
		0,
		#endif
		src_ip , dest_ip , header->seq_num , /* unique key -> [tx-rx-seqnum]*/
		src_ip , dest_ip , /* src , dst */
		header->type ,
		tdma_pkt_len , /* tdma pkt size */
		delay,
		bandwidth ,
		other_string ) ;
	fprintf( logfile , "%s\n", file_buf );
	fflush( logfile ) ;
}
#endif 




/************************************************************************************
 ***************************** PRINT functions **************************************
 * ********************************************************************************/
 

#ifdef PRINTSTATS
void tdma_printBin( uint8_t ip )
{
	/* print some stuff */
	if (0 == ip)
		ip = TDMA_upStream() ;
	
	//uint8_t ip_dw = 0 ;
	
	//if ( bandwidth.in[ip_up] != 0 )
	PRINTF_FL(
		"bandwidth.in[%" PRIu8 "] = %.1f KBps "
		"| new estimate %.1fB , %d ms | round #%" PRIu64 "\n" ,
		ip ,
		Bandwidth.in[ip],
		Tdma_statistics.round_bytes_rcvd[ip] ,
		//slotId2SlotWidth( ip2SlotId( ip ) ) ,
		Tdma_statistics.last_rx_pkt_pos_ms[ip], /* rel. position of the last pkt received  */
		Tdma_statistics.slot_round_counter
		) ;
}
#endif

#ifdef PRINTSTATS
void tdma_printBout( uint8_t ip )
{
	/* print some stuff */
	if (0 == ip)
		ip = TDMA_dwStream() ;
	slot_limits_t s  = TDMA_getOwnSlot();
	uint64_t delta ;
	if ( Tdma_statistics.last_tx_pkt_ms > s.begin_ms )
		delta = Tdma_statistics.last_tx_pkt_ms - s.begin_ms ;
	else
		delta = (Param_tdma.round_period_ms - s.begin_ms ) + 
			Tdma_statistics.last_tx_pkt_ms;
	//if ( bandwidth.out[ip_dw] != 0 )
	char str_ip[4];
	sprintf(str_ip,"%d",ip);
	PRINTF_FL(
		"Bandwidth.out[%s] = " 
		"%.1fKBps | new estimate >> %.0fB , %"PRIu64" ms | round #%" PRIu64 "\n" ,
		ip==0 ? "All" : str_ip ,
		Bandwidth.out[ip] ,
		Tdma_statistics.slot_bytes_sent[ip],
		delta , 
		Tdma_statistics.slot_round_counter );

}
#endif

#ifdef PRINTSTATS
void tdma_printStats( void )
{
	static unsigned counter = 0 ;
	
	if ( counter != 500 )//change this to control print freq.
	{
		counter++;
		return ;
	}
	counter = 0 ;

	#define ts Tdma_statistics /* alias */
	double total_throughput_KBs	= 
		(double)ts.total_bytes_sent / 
		(ts.total_clock_seconds+1e-6) / 1e3 ;
	double slot_throughput_KBs 	= 
		(double)ts.slot_bytes_sent[0]  / 
		(ts.slot_clock_seconds+1e-9) / 1e3 ;
		
	/* 1st col, 9chars*/	
	PRINTF_FL(
		"-|- TOTAL --------------------------- "
		"| ---- %4" PRIu64 "th SLOT ---|-\n",
		ts.slot_round_counter	);
	PRINTF_FL( 
		" | Th.put   | " /* 1st col, 13chars */
		"%-8.1fKBps." /* 2nd col.. 12chars here*/
		"           | " /* 3rd col, 11chars */
		"%-8.1fKBps." /* 4th col, 14chars */
		"       |\n", /* 5th col, 7ch*/
		total_throughput_KBs , 
		slot_throughput_KBs ) ;
	
	float data_sent = ts.total_bytes_sent/1e3;
	char units[]={'K','M','G'};
	int unit = 0 ;
	while (data_sent>1e3)
	{
		data_sent/=1e3;
		unit++;
	}
	PRINTF_FL(
		" | Tx'ed    | " /* 1st col, 13chars */
		"%-8.1f%cB  ." /* 2nd col.. 12chars here*/
		"%6" PRIu64 "msgs | " /* 3rd col, 12chars */
		"%-8.1fKB  ." /* 4th col, 13chars*/
		"       |\n", /* 5th col, 7ch*/
		data_sent ,
		units[unit],
		ts.total_pktsent ,
		ts.slot_bytes_sent[0]/1e3 ) ;
	PRINTF_FL( 
		" | Clock    | " /* 1st col, 9chars*/
		"%-8.3fs   ." /* 2nd col.. 11chars here*/
		"           | " /* 3rd col, 12chars */
		"%-8.2fms  ."/* 4th col, 13chars*/
		"       |\n", /* 5th col, 7ch*/
		ts.total_clock_seconds ,
		ts.slot_clock_seconds*1e3 ) ;
	PRINTF_FL(
		" | Buf use  | "/* 1st col, 9chars*/
		"%-7" PRIu16 " pkts." /* 2nd col.. 11chars here*/
		"           . " /* 3rd col, 12chars */
		"            ." /* 4th col, 11chars*/
		"       |\n", /* 5th col, 7ch*/
		tdma_getTxBufferUse() ) ;
	PRINTF_FL(
		"-|------------------------------------"
		"----------------------|-\n");
	PRINTF_FL(
		" | PDR %02"PRIu8"-%02"PRIu8"| "
		"%-6.2f  %%   ." /* 2nd col.. 12chars here*/
		"           . " /* 3rd col, 11chars */
		"            ." /* 4th col, 11chars*/
		"       |\n", /* 5th col, 7ch*/
		TDMA_upStream() , 
		getMyIP() , 
		PDR_get_in( TDMA_upStream() ) ) ;
	PRINTF_FL(
		" | PDR %02"PRIu8"-%02"PRIu8"| "
		"%-6.2f  %%   ." /* 2nd col.. 12chars here*/
		"           . " /* 3rd col, 11chars */
		"            ." /* 4th col, 11chars*/
		"       |\n", /* 5th col, 7ch*/
		getMyIP() , 
		TDMA_dwStream() ,
		PDR_get_out( TDMA_dwStream() ) ) ;
	PRINTF_FL(
		"-|------------------------------------"
		"----------------------|-\n");
}
#endif

#if 0
int isRxBufferEmpty( void  )
{
	pthread_mutex_lock( &(rx_queue_ptr.mutex) ) ;
	if ( rx_queue_ptr.w == rx_queue_ptr.r) /* if next writting point is the cur reading point */
	{
		pthread_mutex_unlock( &(rx_queue_ptr.mutex) ) ;
		return 1;
	}
	pthread_mutex_unlock( &(rx_queue_ptr.mutex)  ) ;
	return 0 ;
}
#endif




















/** 
 * Sleep while out NIC buffer is above OUT_BUFF_THRESHOLD 
 * Returns: 
 * a) buf is free
 * b) if outside slot 
 * c) if Texit_threads = 1 
 **/
int tdma_waitTillBufFree( int socket, int timeout )
{
	static int test_id = 0;
	int buff_in_use = 100000000 ;
	if ( ioctl( socket, SIOCOUTQ, (int*)&buff_in_use) < 0 )
	{
		PRINTF_FL_ERR("IOCTL ERROR!\n"); 
		return -2 ;
	}	
	
	while ( buff_in_use  > OUT_BUFF_THRESHOLD  )
	{
		//PRINTF_FL_WARN( "%d:%d B\n", test_id, buff_in_use);  
		microsleep( 2000 ) ;
		if ( ioctl( socket, SIOCOUTQ, (int*)&buff_in_use) < 0 )
		{
			PRINTF_FL_ERR("IOCTL ERROR!\n"); 
			return -2 ;
		}
		
		if ( TExit_threads )
			return 2 ;
			
		if ( timeout && !tdma_insideSlot() )
			return 3;

	}
	test_id++;
	//PRINTF_FL("buf %d\n", buff_in_use ) ;
	return E_SUCCESS ;
}

/* sleep caller until the begin of slot */
void tdma_sleepTillSlotBegin(void)
{
	/* lets go idle now till slot begins */
	microsleep( tdma_getTimeTillSlotBegin() );
}

/* get ms time until the begin of slot */
useconds_t tdma_getTimeTillSlotBegin(void)
{
	//PRINTF_FL("waiting to begin..\n");
	useconds_t t_us ;
	double t_rnd = tdma_getCurrentRoundTimeD() ;
	if (0 > t_rnd) // not possible
		return 0;
		
	slot_limits_t slt = TDMA_getOwnSlot();
	if ( t_rnd > slt.begin_ms )
		t_us = (1000.0*Param_tdma.round_period_ms - 1000.0*t_rnd) + 
			1000.0*slt.begin_ms ;
	else
		t_us = 1000.0*slt.begin_ms - 1000.0*t_rnd ;
	/* lets go idle now till slot begins */
	return t_us ;

}

/* get ms time until the begin of slot */
useconds_t tdma_getTimeTillSlotEnd(void)
{
	//PRINTF_FL("waiting to begin..\n");
	useconds_t t_us ;
	double t_rnd = tdma_getCurrentRoundTimeD() ;
	if (t_rnd < 0 )
		return 0;
		
	slot_limits_t slt = TDMA_getOwnSlot();
	if ( t_rnd > slt.end_ms )
		t_us = (1000.0*Param_tdma.round_period_ms - 1000.0*t_rnd) +
			1000*slt.end_ms ;
	else
		t_us = 1000.0*slt.end_ms - 1000.0*t_rnd ;
	/* lets go idle now till slot begins */
	return t_us-500.0 ;

}

/******************************************
 ***************** GET STUFF
***************************************/

/** get a copy of Param_tdma **/
param_tdma_t TDMA_getParams( void ) 
{
	return Param_tdma ;
}

/** print current settings **/
#ifdef VERBOSE
void TDMA_printSettings(void)
{
	PRINTF_FL("Throughput PHY: %.1fKBps\n",
		1e3*Param_tdma.phy_datarate_MBps ) ;
	PRINTF_FL("Target: %.1fKBps\n",
		1e3*Param_tdma.throughput_target_MBps ) ;
	PRINTF_FL("Slot: id=%" PRIu8 "/%" PRIu8 " ||"
		" T=%" PRIu16 "ms\n", 
		Param_tdma.slot_id , Param_tdma.slot_total,
		Param_tdma.round_period_ms ) ;
}
#endif


/**************************
 * statistics function set starts here, 
 * bandwidth included
 *********************************/



int tdma_updateClockStat2(void)
{
	double cur_time = getEpoch();
	if ( cur_time < 0 )
		return E_OTHER ;
	
	/* update elapsed time since ref time */
	Tdma_statistics.total_clock_seconds = cur_time - 
		Tdma_statistics.time_beginning ; 
	Tdma_statistics.slot_clock_seconds = cur_time - 
		Tdma_statistics.epoch_init_slot  ; 
		
	if ( Tdma_statistics.total_clock_seconds < 0 )
	{
		PRINTF_FL_ERR("CUR %fs ; initREF %fs\n\n\n\n" , 
			(double)cur_time ,
			(double)Tdma_statistics.time_beginning ) ;
		return E_INVALID_INPUT ;
	}
	if ( Tdma_statistics.slot_clock_seconds < 0 )
	{
		PRINTF_FL_ERR("CUR %fs ; slotREF %fs\n\n\n\n" , 
			(double)cur_time ,
			(double)Tdma_statistics.time_beginning ) ;
		return E_INVALID_INPUT ;
	}

	return E_SUCCESS ;

}

/** **/
tdma_stats_t tdma_getStats(void)
{
	return Tdma_statistics ;
}

/** **/
void tdma_clearStats(void)
{
	CLEAR(Tdma_statistics) ; 
}
	

/** update stats when every time node sends a packet **/
error_t tdma_updateTxStats( uint16_t bytes_sent, 
	uint8_t dest_IP, uint16_t timestamp ) 
{
	if ( dest_IP > MAX_IP )
		return E_INVALID_INPUT ;
		
	#define ts Tdma_statistics
	ts.slot_bytes_sent[0] 		 += (double)bytes_sent ;
	ts.slot_bytes_sent[dest_IP]	 +=	(double)bytes_sent ;
	ts.total_bytes_sent 		 += (double)bytes_sent ;
	ts.total_pktsent			 += 1 ;
	ts.last_tx_pkt_ms 			 = timestamp ;
	#undef ts
	return E_SUCCESS ;
}

/** **/
void tdma_updateRxStats( void )
{
	#define ts Tdma_statistics

	for ( uint8_t i = 0; i < sizeof(DRONE_LIST); i++ )
	{
		uint8_t ip = DRONE_LIST[i] ;
		/* Bin (KBps) =  the total of recv (Bytes) divided by the sender slot width(ms) */
		if (1 > ts.last_rx_pkt_pos_ms[ip] )
				ts.last_rx_pkt_pos_ms[ip] = 1;
		float new_estimate_bin = 
			ts.round_bytes_rcvd[ip] /
			ts.last_rx_pkt_pos_ms[ip];
			
		//if (ip==12)
		//{
			//PRINTF_FL_WARN(
				//"tot rcvd from %" PRIu8 " = %.0fB | "
				//"pos[%"PRIu16"] ~=  %.1fKB/s \n" , 
				//ip , 
				//ts.round_bytes_rcvd[ ip ] ,
				//ts.last_rx_pkt_pos_ms[ ip ],
				//1.0*(ts.round_bytes_rcvd[ ip ])/
				//ts.last_rx_pkt_pos_ms[ ip ]
				//);
		//}
		if ( new_estimate_bin > 10 )
			Bandwidth.in[ ip ] = 
				0.9*Bandwidth.in[ ip ] + 
				0.1*new_estimate_bin ;
			
		//if (
			//( Tdma_statistics.slot_round_counter % 1 == 0 ) 
			//&& 
			//( Bandwidth.in[ ip ] > 10 ) 
			//)
		//{
			//tdma_printBin( ip );
		//}
		
		/* erase*/
		ts.round_bytes_rcvd[ip]  	= 0 ;
		ts.last_rx_pkt_pos_ms[ip] 	= 0 ;
		
	
	}
	#undef ts 
}



/**  **/
float tdma_getBandwidthOut( uint8_t ip ) 
{
	return Bandwidth.out[ ip ];
}

/** **/
float tdma_getBandwidthIn( uint8_t ip ) 
{
	return Bandwidth.in[ ip ];
}

float tdma_getMsgPosition(tdma_header_t tdma_header)
{
	uint16_t msg_position = tdma_header.timestamp_ms - tdma_header.slot.begin_ms ;
	msg_position += (uint16_t)Param_tdma.round_period_ms; /*makeit positive */
	msg_position %= (uint16_t)Param_tdma.round_period_ms;  
	return msg_position + tdma_header.timestamp_partms/256.0 ;
}


/********************
 * private to 
 *************************/
/** **/



#ifdef VERBOSE
/** Print header of a TDMA packet **/
char* tdma_printHeader( tdma_header_t *tmp_h_pkt )
{
	static char msg[100];
	uint8_t slot_id = tmp_h_pkt->slot_id ;
	snprintf(msg, sizeof(msg) ,
		"HEADER { %.2fs in [%" PRIu16 "-%" PRIu16 "]ms} [slot ID: %" PRIu8 "]\n",
		tmp_h_pkt->timestamp_ms + tmp_h_pkt->timestamp_partms/256.0, 
		tmp_h_pkt->slot.begin_ms, 
		tmp_h_pkt->slot.end_ms ,
		slot_id ) ;
	return msg;
}
#endif

/** **/
void buildPMSeqnumString( void *tdmapkt_ptr, char *other_string ) 
{

	uint8_t *pm_payload = (uint8_t*)tdmapkt_ptr + sizeof(tdma_header_t);
	
	uint32_t pm_seqnum ;
	uint8_t pm_src, pm_sink ;
	memcpy( &pm_seqnum 	,pm_payload,
		sizeof(pm_seqnum) );
	memcpy( &pm_src , pm_payload + sizeof(pm_seqnum) ,
		1 );
	memcpy( &pm_sink, pm_payload + sizeof(pm_seqnum) + sizeof(pm_src) ,
		1 );
	sprintf( other_string ,
		"pms# %"PRIu8"%"PRIu8"%06"PRIu32 , pm_src, pm_sink , pm_seqnum );
}


/** TDMA sets PHY and trottles data to target throuhgput  **/
error_t loadThroughputSettings(void)
{

	FILE* another_file = fopen( FILENAME_NET , "r" ) ;
	if (another_file == NULL)
	{
		PRINTF_FL_ERR( "Can't find file: " FILENAME_NET "\n"  ) ;
		fflush( stdout ) ;
		return E_FILE_NOT_FOUND ;
	}

	/* read one line */
	char buf[16] ;
	CLEAR( buf ) ; /* erase buf */
	if ( NULL == fgets( buf , sizeof(buf) , another_file)) 
	{
		PRINTF_FL_ERR( "Failed to read line!\n"  );
		PRINTF_FL_ERR(
			"Confirm you have "FILENAME_NET" like this:\n"); 
		PRINTF_FL_ERR(	
			"<PHY datarate (MBps) > <target_throughput (MBps)>\n"); 
		return E_INVALID_INPUT ;
	}
	fclose( another_file ) ;
	
	float phy_datarate_MBps=0 , throughput_target_MBps=0 ;
	if (
		( 2!= sscanf( buf , "%f %f" , &phy_datarate_MBps , &throughput_target_MBps ) ) |
		( phy_datarate_MBps < 0 ) | ( throughput_target_MBps < 0 ) /* parse data*/
	)
	{	
		PRINTF_FL_ERR(
			"Confirm you have " FILENAME_NET " like this:\n");
		PRINTF_FL_ERR( 
			"<PHY datarate MBps> <target_throughput MBps>\n");
		PRINTF_FL_ERR( 
			"(current read : %f %f)\n",
			phy_datarate_MBps , 
			throughput_target_MBps  ); 
		return E_INVALID_INPUT ;
	}
	// copy params to global Param_tdma struct
	Param_tdma.phy_datarate_MBps		= phy_datarate_MBps;
	Param_tdma.throughput_target_MBps	= throughput_target_MBps ; 

	#if 0
	pid_t pid = fork();
	if (pid == 0) 
	{
		// im the new child process 
		char tmp_datarate_string[5];
		snprintf( tmp_datarate_string , 4,"%.0fM", Param_tdma.phy_datarate_MBps ); 
		
		printf("[ARM] Setting PHY rate to %sbps\n", 
			tmp_datarate_string ) ;
		fflush ( stdout ) ;
		execlp("iwconfig","iwconfig", "ath0" , "rate", 
			tmp_datarate_string, NULL); // this replaces the current process.
		printf("Execlp failed\n") ; exit(1);

		
	}
	#endif
	return E_SUCCESS ; /* success */
}


/** GET TDMA SLOT configs **/
error_t loadSlotSettings(void)
{

	FILE *another_file = fopen(FILENAME_TDMA,"r") ;
	if (another_file == NULL)
	{
		PRINTF_FL_ERR( "Where is "FILENAME_TDMA"?\n" ) ;
		return E_FILE_NOT_FOUND ;
	}
	
	char buf[20] ; /* load a whole line from file */
	CLEAR(buf) ; 
	if ( NULL == fgets( buf , sizeof(buf) , another_file)) /* read one line */
	{
		PRINTF_FL_ERR("failed to read line!\n");
		PRINTF_FL_ERR(
			"Confirm you have 4 integers [<0/1>  <N>  <XXX> <0/1>] in "
			FILENAME_TDMA "\n" ) ;
		PRINTF_FL_ERR(
			"[VSP?=<0/1>; no of slots=<N>; round period=<XXX>; autosync?=<0/1>] \n" ) ;
		return E_INVALID_INPUT ;
	}
	fclose(another_file) ;

	uint8_t vsp = UINT8_MAX , slot_total = UINT8_MAX ; /* my own slot ID, and the total number slots in existence */
	uint8_t sync_flag = 111 ; /*  0 or 1 , sincronization : shift my own slots based on info from others uavs */
	uint16_t round_period_ms = UINT16_MAX ;
	if (
		(4 != sscanf( buf , "%" SCNu8 "%" SCNu8 " %" SCNu16 " %" SCNu8 "", 
		&vsp , &slot_total , &round_period_ms , &sync_flag ) ) | /* parse line's data */
		( round_period_ms == 0 ) | 
		( vsp > 2 ) |
		( sync_flag > 1 )
	)
	{
		PRINTF_FL_ERR(
			"Confirm you have 4 integers [<0/1>  <N>  <XXX> <0/1>] in "
			FILENAME_TDMA "\n" ) ;
		PRINTF_FL_ERR(
			"[VSP?=<1/0>; no of slots=<N>; round period=<XXX>; autosync?=<0/1>] \n" ) ;
		PRINTF_FL_ERR( 
			"Current read: VSP [%" PRIu8 "] , "
			"%" PRIu8 " slots , Period %" PRIu16 "ms, Sync: %" PRIu8 "\n", 
			vsp , slot_total , round_period_ms , sync_flag ) ;
		return E_INVALID_INPUT ;
	}
	
	if (vsp == 1)
	{PRINTF_FL("VSP: ON\n");}
	else
	{PRINTF_FL("VSP: OFF\n");}
	
	if (sync_flag == 1)
	{PRINTF_FL("Self-Synch: ON\n");}
	else
	{PRINTF_FL("Self-Synch: OFF\n");}
	
	#if VSP
	Param_tdma.vsp				= vsp ;
	#endif
	Param_tdma.round_period_ms	= round_period_ms ;
	Param_tdma.slot_total		= slot_total ;
	Param_tdma.standard_width_ms= Param_tdma.round_period_ms / (uint16_t)Param_tdma.slot_total ;
	Param_tdma.sync_flag		= sync_flag ; 
	
	#ifdef X86 // ifndef ARM !
	//TDMA_off();
	#endif
		
	
	/** clear requests,
	 * set slot to standard size **/
	tdma_clearStuff();
	

	
	/* current (local) knowledge of the pair (Slot-id ; IP ) */
	//PRINTF_FL("my ID %d , my IP  %d\n" , 
		//Param_tdma.slot_id ,
		//Param_tdma.node_ip ) ;

	return E_SUCCESS ;
}




#undef TDMA_UTILS_IMPORT
