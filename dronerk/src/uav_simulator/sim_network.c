/*************
 * Functionalities:
 * 1) get times to transmit packets, 
 * 2) simulate packet drop depending on topology
 * 3) reads incoming UPD packets, and adds them to the event list.
 **************/

#include "sim_network.h"

/* get PDR level as function of distance, based on WFCS16 paper -aerial link model */
static float network_getPDR( float distance ) ;

/* get time based on bits on wire and datarate */
float network_getWireTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate )
{
	if ( bit_data_rate == 0 )
		bit_data_rate = DATARATE ;
		
	return	T_PREAMBLE + 8.0 * payload_size_in_bytes / bit_data_rate ;
}


/* time to receive an ACK from 802.11 unicast */
float network_getAckTime()
{
	return 	T_SIFS + 
		network_getWireTime( IEEE80211_ACK_SIZE , ACK_DATARATE ) ;
}

/* time to send a dataframe */
float network_getFrameTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate ) 
{
	if ( bit_data_rate == 0 )
		bit_data_rate = DATARATE ;

	return	network_getWireTime( IEEE80211_UNICAST + payload_size_in_bytes , bit_data_rate ) ;
}

/* time to send frame and get ack */
float network_getUnicastTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate ) 
{
	if ( bit_data_rate == 0 )
		bit_data_rate = DATARATE ;
		
	return	network_getFrameTime( payload_size_in_bytes , bit_data_rate ) +
			network_getAckTime() ;

}

/* time to send a broadcast*/
float network_getBroadcastTime( uint16_t payload_size_in_bytes )
{
	return	T_DIFS +
			T_PREAMBLE +
			network_getWireTime( payload_size_in_bytes , BCAST_DATARATE ) ;
}


/* bernoulli experiment . 
 * check if a packet is transmitted. 
 * it is based on the current distance between nodes */
int network_isPacketDelivered( uint8_t src, uint8_t dst )
{
	printf("%"PRIu8 "and %"PRIu8" are at %.1fm away. PDR %.2f\n",
		src,dst,
		layout_getDistance(src-1,dst-1),
		network_getPDR(layout_getDistance(src-1,dst-1)));
	float random_num = 1.0*rand() / RAND_MAX ;
	if ( random_num < network_getPDR(layout_getDistance(src-1,dst-1)))
		return 1;
	return	0;
}

/* get PDR level as function of distance, based on WFCS16 paper -aerial link model */
static float network_getPDR( float distance ) 
{
	return exp( -log(2) * pow( ( distance / RADIUS ), ALPHA ) ) ;
}
