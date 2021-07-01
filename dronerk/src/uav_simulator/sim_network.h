/*************
 * Functionalities:
 * 1) get times to transmit packets, 
 * 2) simulate packet drop depending on topology
 * 3) reads incoming UPD packets, and adds them to the event list.
 **************/
 
#ifndef SIM_NETWORK_H_
#define SIM_NETWORK_H_

#include <stdint.h> // atoi
#include <inttypes.h>// int16_t etc
#include <stdint.h> // int16_t etc
#include <stdlib.h> // calloc
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <stdio.h> // printf
#include <math.h> /* pow */


/** characteristics of the PDR curve (network_getPDR()
 * (cf. "Channel Characterisation of COTS UAVs" , Luis Pinto et al, in WFCS16 **/
#define ALPHA		10.1
#define RADIUS		68



/** times, rates and sizes of 802.11g **/
/* typical DS PHY (802.11g) */
#define T_SIFS							(0.000010) // 10us
#define T_SLOTTIME						(0.000009) // short 9us, long 20us
#define T_DIFS							(T_SIFS + 2*T_SLOTTIME) // 28 us
//#define T_BACKOFF						(0.00135)
#define T_PREAMBLE						(0.000020) //20 us

#define CWMIN							16 //min and max window
#define CWMAX							1023

#define MBPS(x) x ## 000000
#define BCAST_DATARATE 					MBPS(1)
#define ACK_DATARATE 					MBPS(6)
#define DATARATE						MBPS(24)	/* default datarate  Mbits/s */


//#define RADIOTAP_HEADER_SIZE			(18) //all 802.11 have this . radio related.
#define FCS_SIZE						4 /* aka CRC */
#define IEEE80211_ACK_SIZE				(10+FCS_SIZE) // type 0x01 (subtype 1101) wlan.fc.type_subtype == 0x1d 
// IBSS dataframe => frame control + duration/id + add1 + add2 + add33 + sqctl + add4 + ... + FCS
#define IEEE80211_IBSS_DATAFRAME		(30+FCS_SIZE) // type 0x02 (subtype 0000) 
/* Frame body: < LLC + IP4 + UDP + useful_data >*/
#define LLC_HEADER_SIZE					(8) /* LLC encapsulation is used to carry an IP packet */
#define IP4_HEADER_SIZE					(20)
#define UDP_HEADER_SIZE					(8)
#define IEEE80211_UNICAST				(IEEE80211_IBSS_DATAFRAME + LLC_HEADER_SIZE + IP4_HEADER_SIZE + UDP_HEADER_SIZE)
//#define IEEE80211_BEACON_HEADER_SIZE	(10) // this is for mgt and ctl packets
//#define IEEE80211_OPEN_QOSDATA_HEADER_SIZE	(26) // some strange data is this one (subtype 0x28)



/****** 
 * Function prototypes 
 * *******/
/* times in seconds */

/* get time based on bits on wire and datarate */
float network_getWireTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate );

/* time to receive an ACK from 802.11 unicast */
float network_getAckTime();

/* time to send a frame */
float network_getFrameTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate ) ;

/* time to send frame and get ack */
float network_getUnicastTime( uint16_t payload_size_in_bytes , uint32_t bit_data_rate ) ;

/* time to send a broadcast*/
float network_getBroadcastTime( uint16_t payload_size_in_bytes );

/* bernoulli experiment . check if a packet is rcved. it is based on the current distance between nodes */
int network_isPacketDelivered( uint8_t src, uint8_t dst );

///* get PDR level as function of distance, based on WFCS16 drone model */
//static float network_getPDR( float distance ) ;


/****************
 * externs!
 * *******************/
//#include "sim_network.h" // MAX NUM DRONES
//#include "event_scheduler.h" // event_schedule()
//#include "sim_clock.h" // getTime
#include "sim_layout.h" // layout_getDistance

#endif
