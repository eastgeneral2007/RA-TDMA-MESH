/* actuation tx */


#include "tx_actuation.h"
#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "utils.h" // microsleep

#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <pthread.h> // we have threads in the app
#include <stdio.h> // printf
#include <stdlib.h>

#define PREFIX 	"TXACT"

extern uint8_t pdr_list[5][5] ; /* defined in rx_80211,  estimation of pdr, matrix [src][dst] */

actuator_t actuation_vector;

void setActuation( actuator_t act_v )
{
	actuation_vector = act_v;
	/*printf(" Setting actuation => u=(%.1f,%.1f,%.1f) :: w=%.1fº\n" , 
			actuation_vector.thrust.X , actuation_vector.thrust.Y, actuation_vector.thrust.Z , actuation_vector.torque ) ;
	fflush( stdout ) ;*/
}


/* this thread sends actuation packets to other drones */
void * thread_tx_actuation(void *arg)
{
	int *ti = (int*)arg ; /* copty to tinfo struct input params */
	uint8_t my_id = (uint8_t)ti[0] ; 
	PRINTF_FL("TID %"PRIu64" Mission Thread to TX actuation packets (im#%d)\n" ,
		pthread_self(),my_id);
	

	//UDP stuff
	int sock_fd ; /* socket */

	
	/* prepare UDP socket */
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		PRINTF_FL_ERR("error geting udp socket\n");
	struct sockaddr_in si_me;
	prepare_sockaddr( 0,  UDP_PORT_BASE_SRC_ACT + my_id, &si_me );
	if ( bind( sock_fd , (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR("error binding sokt\n" );
		return NULL ;
	}
	PRINTF_FL(
		"send actuation from %s:%hu\n", 
		inet_ntoa(si_me.sin_addr),
		ntohs(si_me.sin_port) );
	
	//actuation_vector.thrust.X = 1 ;
	//actuation_vector.thrust.Y = 2.5 ;
	//actuation_vector.thrust.Z = 0 ;
	//actuation_vector.torque = 0.1 ;
	
	struct sockaddr_in 	si_send ; 
	prepare_sockaddr( my_id+1, SIM_RX_ALL, &si_send );
	/* to send pkts to #0 use 127.0.0.1 */

	PRINTF_FL(
		"ready to send to %s:%hu\n", 
		inet_ntoa(si_send.sin_addr),
		ntohs(si_send.sin_port) );
		
	//int ret ; /* sendto return */
	//TODO: use a connect. since we are sending to same dst all the time

	while (1) 
	{
		char cmd_parameter[200];
		snprintf(cmd_parameter, sizeof(cmd_parameter)-1,
			"AT*PCMD=%ld,%d,%d,%d,%d,%d\r",
			1234L,
			1, /* 1 - move, 0-hover*/
			*(int*)&(actuation_vector.thrust.Y),
			*(int*)&(actuation_vector.thrust.X),
			*(int*)&(actuation_vector.thrust.Z),
			*(int*)&(actuation_vector.torque) 
			) ;
		int ret = (int)sendto( sock_fd , cmd_parameter , strlen( cmd_parameter ) , 0 ,
						(const struct sockaddr *)&si_send , (socklen_t)sizeof(si_send) ) ;
		if ( ret < 0 )
		{
			PRINTF_FL_ERR("[CLOCK %07luus] #%d - Error delivering actuation to simulator! * TERMINATING :S\n\n" , 
				SimClock_get() , (int)my_id ) ;
				return NULL ; 
		}
		else
		{
			//printf("[CLOCK %07lu us] ACtuation Delivered to {127.0.0.1:%d}\n", 
			//	getSimClock() , UDP_PORT_BASE_TX_80211 + My_id ) ;
			//printf("\t"); 
			//printPositions();
		}
		 //act_tx = dsp.UDPSender(...
            //'LocalIPPortSource','Property' , ...
            //'LocalIPPort', UDP_PORT_BASE_SRC_ACT + my_id ,...
            //'RemoteIPPort' , UDP_PORT_BASE_RX_ALL + my_id , ...
            //'RemoteIPAddress' , IP ) ;
            
        sleep(1) ;    /* send this once a second +- */
    }
	return NULL ;
}

