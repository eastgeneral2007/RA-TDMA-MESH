/* 80211 tx */


#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "generic.h"  
#include "p80211.h"

#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <pthread.h> // we have threads in the app
#include <stdio.h> // printf
#include <stdlib.h> // printf

#include "utils.h" // microsleep
#define PREFIX	"TX80211"



extern uint8_t pdr_list[5][5] ; /* defined in rx_80211,  estimation of pdr, matrix [src][dst] */


/* this thread sends 802.11 packets to other drones
 * every 100 packets sent, 
 * shares its PDR knowledge  */
void * thread_tx_80211(void *arg)
{
	int *ti = (int*)arg ; /* copty to tinfo struct input params */

	int my_id  =ti[0]  ; 
	int sender =ti[1]  ;
	int dst_id =ti[2]  ; 

	
	PRINTF_FL_WARN("TX80211@ TID %"PRId64", "
		"Mission Thread to TX 802.11 packets ."
		"sender ? %d, dst #%d. i'm #%d\n" ,
		pthread_self() ,
		sender,
		dst_id,
		my_id 
		);
	
	//UDP stuff
	int txsock_fd ; // socket - this is used to SEND data. not to receive

	/* prepare UDP socket */
	if ( ( txsock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		PRINTF_FL_ERR("ERROR geting udp socket\n");
		
	/*bind source port */
	struct sockaddr_in	si_me ; // for bind socket
	prepare_sockaddr( 0, UDP_PORT_BASE_SRC_80211 + my_id, &si_me);
	if ( bind( txsock_fd , (SA)&si_me, SLEN ) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR("Error binding sokt\n" );
		return NULL ;
	}
	PRINTF_FL(
		"sending 80211pkts from %s:%hu\n", 
		inet_ntoa(si_me.sin_addr), 
		si_me.sin_port
		);
	
	
	p80211_simple_t pkt; /* the simplest packet we can send is an array of 200bytes*/
	/* payload we will send is always the same */	
	uint16_t seq_num = 0 ;
	struct sockaddr_in si_send ; // sockaddr for destination 
	prepare_sockaddr( dst_id+1, SIM_RX_ALL, &si_send);
	/* to send pkts to #0 use 127.0.0.1 */
	
	while (sender ? 1 : !printf("i am not a sender\n")) 
	{			
		/* sending a packet to some userinput destination*/
		pkt.seq_num = ++seq_num;
		sprintf( pkt.payload, "a minha mensagem secreta #%u",pkt.seq_num) ; /* paste some payload  */

		int ret = (int)sendto( txsock_fd, &pkt, sizeof(p80211_simple_t), 0,
					(const struct sockaddr *)&si_send, (socklen_t)sizeof(si_send) ) ;
		
		if (ret < 0)
		{
			printf(
				"[CLOCK %07luus] Error delivering UDPpkts to %d:%s\n\n", 
				SimClock_get(),
				(int)ntohs( si_send.sin_port),
				inet_ntoa(si_send.sin_addr) );
			break;
		}
	
		
        microsleep(1000000); /* every 10ms , send another packet */ 
	}
	
	PRINTF_FL_WARN("[tx80211] thread is terminating\n");
	return NULL ;
}

