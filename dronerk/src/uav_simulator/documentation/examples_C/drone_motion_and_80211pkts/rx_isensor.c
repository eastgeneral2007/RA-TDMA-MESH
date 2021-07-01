/* sensor rx */
#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "generic.h"  
#include "rx_isensor.h"
#include "utils.h" // microsleep

#include <stdlib.h>
#include <stdio.h> // printf
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <pthread.h> // we have threads in the app
#include <stdlib.h> /* calloc */

#define PREFIX	"ISENSOR"


/** **********
 * evil static globals
 *****************/
static volatile isensor_t  ireading ; /* save here a copy of the last rcvd reading */



/***
 *  this thread receives sensor readings (from drone_simulator) 
 * */
void * thread_rx_isensor(void *arg)
{
	
	ireading.heading_degrees = 0 ;
	
	int *ti = (int*)arg ; /* copty to tinfo struct input params */
	uint8_t my_id = (uint8_t)ti[0] ; 
	PRINTF_FL("TID %"PRIu64", Thread to get sensor readings (i'm #%" PRIu8 ").\n" , 
		pthread_self(), my_id);

	//UDP stuff
	int16_t port_sensor_rx = PORT_ISENSOR  ; /* i'll receive sensor data on this port */
	int sock_fd ; // socket 
	ssize_t num_bytes_read ; //return for recvfrom()
	struct sockaddr_in 	si_other ; // sockaddr for the received packet
	socklen_t slen = (socklen_t)sizeof(si_other) ;

	/* prepare UDP socket */
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		PRINTF_FL_ERR("error geting udp socket\n");
	struct sockaddr_in si_me ; // for bind socket
	prepare_sockaddr(my_id+1, PORT_ISENSOR, &si_me);
	if ( bind( sock_fd , (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR(
		"error listening at %s:%u for Isensor data - %u\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port),
		PORT_ISENSOR );
		return NULL ;
	}
	PRINTF_FL(
		"listening at %s:%u for Isensor data - %u\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port),
		PORT_ISENSOR );
		
	void *recv_data = calloc( MAX_SENSOR_SIZE, sizeof(char) ) ; //allocate some bytes. ; // where received data will be. 
	if (NULL == recv_data)
	{
		PRINTF_FL_ERR("error calloc\n");
		return NULL ;
	}	
	
	//position.X = 0 ; 
	while (1)
	{
		num_bytes_read = recvfrom( sock_fd , recv_data , MAX_80211_SIZE , 0, (struct sockaddr*)&si_other , &slen) ; /* blocking */
		//memcpy( &isensor_reading, recv_data, sizeof(isensor_t) ) ;
		int ret = sscanf( recv_data , 
			"state=%"SCNx32",battery=%d,compass=%f,ultra=%f" ,
			&ireading.state,
			&ireading.battery,
			&ireading.heading_degrees,
			&ireading.ultra_altitude 
			);
			
		if (ret == 4)
			PRINTF_FL("#%d - compass [%.1fº] \n" , 
				(int)my_id, 
				ireading.heading_degrees ) ; /* pointing NORTH compass= 0º, pointing EAST compass=90º */
			
		// TODO: run this cycle with period T (read sim clock for that ,and sleep)
	}
	return NULL ;
}

isensor_t isensor_getReading(void)
{
	return ireading ; 
}

