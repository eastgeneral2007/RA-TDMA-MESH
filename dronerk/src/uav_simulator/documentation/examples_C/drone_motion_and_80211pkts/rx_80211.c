/* 80211 rx */
#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "generic.h"  
#include "p80211.h"
#include "utils.h"//preparesock


#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <pthread.h> // we have threads in the app
#include <stdio.h> // printf
#include <stdlib.h> // atoi

#define PREFIX	"RX80211"

uint16_t array_of_rcv_pkts[200];
uint8_t w_ptr = 0 ;
uint8_t pdr_list[5][5] ; /* estimation of pdr, pdr_list[src][dst] is the pdr at the link src->dst */

/* this thread receieve packets from other drones */
void * thread_rx_80211(void *arg)
{
	int *ti = (int*)arg ; /* copty to tinfo struct input params */
	uint8_t my_id = (uint8_t)ti[0] ; 
	
	PRINTF_FL("TID %"PRIu64" Mission Thread to RX 80211 packets\n" , 
		pthread_self()  );
	

	
	/*****/
	
	/**  wifi_rx = dsp.UDPReceiver( ...
            'LocalIPPort' , UDP_PORT_BASE_TX_80211 + my_id , ...
            'MaximumMessageLength' , 1500 ) ;
    **/
    
	/* UDP stuff */
	int sock_fd; // socket 
	ssize_t num_bytes_read ; //return for recvfrom()
	struct sockaddr_in 	si_other ; // sockaddr for the received packet
	socklen_t slen = (socklen_t)sizeof( si_other ) ;



	/* prepare UDP socket to receive */
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		PRINTF_FL_ERR( "error geting udp socket\n");
	
	/* bind lsitening ip and port */
	struct sockaddr_in		si_me ; // for bind socket
	prepare_sockaddr(my_id, PORT_UAV_RX80211 , &si_me);
	if ( bind( sock_fd , (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR("error binding sokt\n" );
		return NULL ;
	}
	PRINTF_FL(
		"listening at %s:%hu for 80211pkts.\n", 
		inet_ntoa(si_me.sin_addr), 
		si_me.sin_port
		);

	void *recv_data = malloc( MAX_80211_SIZE ) ; //allocate some bytes. ; // where received data will be. 
	if (NULL == recv_data)
	{
		PRINTF_FL_ERR("Error calloc()\n");
		return NULL ;
	}
	 PRINTF_FL_WARN("ready\n");	sleep(1);


	p80211_simple_t pkt;
	uint8_t type, r_ptr ;
	while (1)
	{
		num_bytes_read = recvfrom( sock_fd, recv_data, MAX_80211_SIZE, 0,
			(struct sockaddr*)&si_other , &slen) ;
		memcpy( &pkt, recv_data, sizeof(p80211_simple_t)) ;
		
		PRINTF_FL(
			"Got %.5dB, sqnum %.5d ---> payload:%.50s \n" ,
			(int)num_bytes_read, 
			(int)pkt.seq_num,
			pkt.payload ) ;
		
	}
	return NULL ;
	
}
