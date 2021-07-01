/*! \file pdr.h
    \brief A Documented file.
    
    Details. Layer PM - packet manager 
*/

#ifndef _PDR_H
#define _PDR_H

#include <inttypes.h>

#include "tdma_types.h"
#include "utils.h"

typedef struct 
{
	tdma_header_t header ;  /* */
	float 		pdr ; /* packet delivery ratio (percentage %) 4 bytes */
}  __attribute__ ((__packed__)) tdma_pdr_packet_t ;



/********************
 * public PROTOTYPES
 * ***************/
error_t PDR_shareWithNeighbors( void );
error_t PDR_estimate( uint32_t s_num, uint8_t other_IP ) ;
error_t PDR_init(void);
/* parsing a rcvd pdr packet */
error_t PDR_parsePkt( const void* const rx_tdmapkt_ptr , 
	uint16_t num_bytes_read , uint8_t other_IP );
//error_t sendPdr( uint8_t dest_ip, uint16_t pkt_len , 
	//const void * const pkt_ptr );
/* print PDR in */
void PDR_print( uint8_t ip );

float PDR_get_in( uint8_t ip ) ;
float PDR_get_out( uint8_t ip ) ;



#endif
