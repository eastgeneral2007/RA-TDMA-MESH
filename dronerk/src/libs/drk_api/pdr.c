/** 
 * PDR - packet delivery ratio 
 * Parse PDR - pm packets
 **/
#include "drk/pdr.h"

#include "drk/tdma.h"
#include "drk/tdma_utils.h"
#include "drk/tdma_slot.h"
#include "drk/internal_actuator_api.h"


#include <string.h> 
#include <math.h>
 




#define PREFIX "PDR "
#define N_SEQ_TABLE 300
/*************
 * GLOBALS
 * ***********/
static uint32_t seq_num_table[MAX_IP+1][N_SEQ_TABLE]={{0}} ;
static uint16_t seq_num_w_ptr[MAX_IP+1]={0} ;
static float PDR_estimation_out[MAX_IP+1]={-1}; /* save here PDR reports sent by other nodes (LINK me->others)*/
static float PDR_estimation_in[MAX_IP+1]={-1}; /* save here my estimations of pdrs (LINK others->me)*/


/** *************
 * private prototypes 
 * ************/
void dumpSeqTable( uint8_t ip, uint16_t idx );
#ifndef X86
void moveDroneToBalancePDRs(void);
#endif

/*******
 * ******** CODE STARTS HERE *********
 * ******/
/** analyze PDR of a given LINK [OTHER]-->[MYSELF] **/
error_t PDR_estimate( uint32_t s_num, uint8_t other_IP ) 
{

	/* error handling */
	if ( other_IP > MAX_IP )
	{
		PRINTF_FL_ERR("IP higher than max array size. \n");
		return E_INVALID_INPUT ;
	} 


	/* error check */

	
	/* update seq num table : */
	seq_num_table[other_IP][seq_num_w_ptr[other_IP]] = s_num ;
	seq_num_w_ptr[other_IP] = 
		(seq_num_w_ptr[other_IP] + 1 ) % N_SEQ_TABLE;

	/* update  PDR  estimate*/
	uint32_t missed = 0 ;
	uint32_t min = s_num , max = s_num;
	for ( int i = 0 ; i < N_SEQ_TABLE - 1 ; i++ )
	{
		int cur  = ( i ) 	% N_SEQ_TABLE ;
		if ( seq_num_table[other_IP][cur] == 0 )
			continue;
		int next = ( i+1 )	% N_SEQ_TABLE ;
		if ( seq_num_table[other_IP][next] == 0 )
			continue;
		int64_t diff = (int64_t)seq_num_table[other_IP][next] - 
			(int64_t)seq_num_table[other_IP][cur] - 1 ;
		if ( diff > 0 )
		{
			missed += (uint32_t)diff ;
			//PRINTF_FL("cur %d next %d\n", cur, next );
			//dumpSeqTable( seq_num_table[ip] , seq_num_w_ptr ) ;
		}
		min = MIN( min , seq_num_table[other_IP][cur] ) ;
		if ( seq_num_table[other_IP][cur] > s_num )
		{
			PRINTF_FL_WARN(
				"Got a seq num lower than current max seqnum\n"
				"Dumping&erasing table of #%"PRIu8"..\n", 
				other_IP);
			
			dumpSeqTable( other_IP , seq_num_w_ptr[other_IP] ) ;
			
			CLEAR ( seq_num_table ) ;
			return E_OTHER;
		}
	}
	
	float range = 1.0 * (max-min)  ;
	if ( range == 0 )
		return E_SUCCESS ;/* nothing interesting */
		
	if (range < 0 ) 
	{
		PRINTF_FL_ERR("something vry wrong\n" );
		PRINTF_FL_ERR(
			"min value is %"PRIu32", max is %"PRIu32 " , range =%.1f\n", 
			min , max, range );
		return E_OTHER;
	}
	
	PDR_estimation_in[other_IP] = 
		100.0 * 
		(range - missed) / 
		range ;
		
	if ( PDR_estimation_in[other_IP] < 40 )
	{
		//PRINTF_FL_WARN(
			//"Super low PDR (%.1f%%). "
			//"Missed from #%"PRIu8 "=> %"PRIu32"pkts\n", 
			//PDR_estimation_in[other_IP],
			//other_IP , 
			//missed );
		//PRINTF_FL_WARN(
			//"min value is %"PRIu32", "
			//"max is %"PRIu32", "
			//"range is %.0f. "
			//"\n", 
			//min,
			//max,
			//range 
			//);
		
		//dumpSeqTable( other_IP , 
			//seq_num_w_ptr[other_IP] ) ;
		//CLEAR ( seq_num_table ) ;
		//#ifndef X86
		//drk_exit();
		//#endif
	}
	return E_SUCCESS ;
}





/** printout pdr **/
#if VERBOSE
void PDR_print( uint8_t ip )
{
	PRINTF_FL(
		"PDR[#%"PRIu8"] = %.2f%%\n" , 
		ip , 
		PDR_estimation_in[ip] ) ;
}
#endif




float PDR_get_in( uint8_t ip ) 
{
	return PDR_estimation_in[ ip ] ;
}

float PDR_get_out( uint8_t ip ) 
{
	return PDR_estimation_out[ ip ] ;
}


error_t PDR_init(void)
{
	
	CLEAR( seq_num_table );
	CLEAR( seq_num_w_ptr );
	
	return E_SUCCESS ;
}

/* parsing a rcvd pdr packet */
error_t PDR_parsePkt( const void* const rx_tdmapkt_ptr , 
	uint16_t num_bytes_read , uint8_t other_IP )
{
	#ifndef X86
	static double last_run2 ;
	#endif 
	/* error parser */
	if ( num_bytes_read != sizeof(tdma_pdr_packet_t) )
	{
		PRINTF_FL_ERR("Invalid PDR packet\n");
		return E_INVALID_FORMAT ;
	}
	float pdr = ((tdma_pdr_packet_t*)rx_tdmapkt_ptr)->pdr ;
	
	//PRINTF_FL_WARN("Got a PDR packet from %"PRIu8"\n",other_IP);
	//float pdr = *(float*)((char*)recv_pmpkt_ptr + sizeof(pm_header_t));
	//PRINTF_FL_WARN(
		//"PDR %"PRIu8"-%"PRIu8 " = %.3f%% (rcv pkt)\n" ,
		//(uint8_t)getMyIP(),
		//(uint8_t)other_IP ,
		//pdr );
	PDR_estimation_out[other_IP] = pdr ;
	
	
	/* actuate on the lights */
	#ifndef X86
	//PRINTF_FL("last run %.2lfs\n", last_run2 );
	if (getEpoch()-last_run2 > 1)
	{
		last_run2 = getEpoch();
		moveDroneToBalancePDRs();
	}
	#endif


	return E_SUCCESS;
}

/* actually call tdma to send a packet */
//error_t sendPdr( uint8_t dest_ip, uint16_t pkt_len , 
	//const void* const pkt_ptr )
//{
	//return TDMA_sendAnyPacket( dest_ip , PDR , pkt_len , pkt_ptr ) ;
//}

/**  **/
error_t PDR_shareWithNeighbors( void )
{
	
//for ( 	uint8_t i = 0 ; 
		//i < sizeof(DRONE_LIST) ; 
		//i++ ) 
//{
	//uint8_t ip = DRONE_LIST[i];
	uint8_t ip = TDMA_upStream();
	/* error handling */
	//if ( ip == (uint8_t)getMyIP() )
		//continue ;
	
	//PRINTF_FL_WARN(":x:\n");

	float pdr = PDR_estimation_in[ip] ;
	#ifdef X86
	PRINTF_FL_WARN(
		"Link %" PRIu8 "-%"PRIu8" > "
		"PDR=%.3f%% \n"
		"\n", 
		ip ,
		(uint8_t)getMyIP() ,
		pdr 
	);
	#endif
	
	if ( pdr < 0 ) /* no PDR information, then */
		return 0; /* no error , nothing shared */
		
	error_t ret = TDMA_sendAnyPacket( ip , PDR, sizeof( pdr ) , &pdr ) ;
	if ( ret < 0)
	{
		PRINTF_FL_ERR("TDMA PDR send failed\n");
		return ret ;
	}
		
		//PRINTF_FL_WARN(":x:\n");

//}
	//PRINTF_FL_WARN(":PDR sent:\n");
	return E_SUCCESS; 
}



/**  **/
#ifndef X86
void moveDroneToBalancePDRs(void)
{
	//PRINTF_FL_WARN(
		//"%"PRIu8 "-%" PRIu8 "= %.1f%% | "
		//"%"PRIu8 "-%" PRIu8 "= %.1f%% \n",
		//TDMA_upStream(),
		//(uint8_t)getMyIP(),
		//PDR_estimation_in[TDMA_upStream()] , 
		//(uint8_t)getMyIP(),
		//TDMA_dwStream(),
		//PDR_estimation_out[TDMA_dwStream()]
	//) ;
		

	if ( fabs( PDR_estimation_in[TDMA_upStream()] - PDR_estimation_out[TDMA_dwStream()] ) < 2 )
	{
		drk_play_LED_animation( GREEN , 5.0 , 2 );
		return;
	}
	
	if ( PDR_estimation_in[TDMA_upStream()] > PDR_estimation_out[TDMA_dwStream()] )
	{
		drk_play_LED_animation( LEFT_GREEN_RIGHT_RED , 5.0 , 2 );
		//PRINTF_FL_WARN("Left good\n");
	}
	else
	{
		drk_play_LED_animation( LEFT_RED_RIGHT_GREEN , 5.0 , 2 );
		//PRINTF_FL_WARN("Right good\n");
	}
	
}
#endif


/**  **/
void dumpSeqTable( uint8_t ip, uint16_t idx  )
{
	PRINTF_FL_WARN("Table : ") ;
	for (int i = 0 ; i < N_SEQ_TABLE ; i++ )
	{
		if ( idx == i )
			printf("*");
			
		fprintf(stderr,"[%" PRIu32 "] " , seq_num_table[ip][i] ) ;
		fflush(stdout);
	}
	fprintf(stderr,"\n") ;
}
