/* sensor rx */
#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "generic.h"  
#include "rx_esensor.h"
#include "utils.h"//preparesock


#include <stdio.h> // printf
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <pthread.h> // we have threads in the app
#include <stdlib.h> /* calloc */
#include <inttypes.h>


#define PREFIX 	"ESENSOR"

/** **********
 * evil static globals
 *****************/
static volatile esensor_t ereading ; /* save here a copy of the last rcvd reading */



/***
 *  this thread receives sensor readings (from drone_simulator) 
 * */
void *thread_rx_esensor(void *arg)
{
	int *ti = (int*)arg ; /* copty to tinfo struct input params */
	uint8_t my_id = (uint8_t)ti[0] ; 
	PRINTF_FL("TID %"PRIu64", "
		"Thread to get esensor readings (i'm #%"PRIu8")\n" , 
		pthread_self(),my_id);
	
	//UDP stuff
	int16_t port_sensor_rx = PORT_ISENSOR  ; /* i'll receive sensor data on this port */
	int sock_fd ; // socket 
	ssize_t num_bytes_read ; //return for recvfrom()
	struct sockaddr_in 	si_other ; // sockaddr for the received packet
	socklen_t slen = (socklen_t)sizeof( si_other ) ;

	/* prepare UDP socket */
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		fprintf(stderr , "ESENSOR@ error geting udp socket\n");
	
	struct sockaddr_in si_me ; // for bind socket
	prepare_sockaddr(my_id+1, PORT_ESENSOR, &si_me);
	if ( bind( sock_fd , (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR(
		"error listening at %s:%u for Esensor data - %u\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port),
		PORT_ESENSOR );
		return NULL ;
	}
	PRINTF_FL(
		"listening at %s:%u for Esensor data - %u\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port),
		PORT_ESENSOR );
		
	void *recv_data = calloc(MAX_SENSOR_SIZE, sizeof(char)) ; //allocate some bytes. ; // where received data will be. 
	if (NULL == recv_data)
	{
		PRINTF_FL_ERR("error calloc %s:%d\n",__FUNCTION__,__LINE__);
		return NULL ;
	}
	
	CLEAR(ereading);
	//position.X = 0 ; 
	while (1)
	{
		num_bytes_read = recvfrom(sock_fd, recv_data, MAX_80211_SIZE,
			0, (struct sockaddr*)&si_other , &slen) ; /* blocking */
		//printf("ESENSOR: %s\n", (char*)recv_data);
		
		/*
		 *typedef struct {
	uint32_t	rtk_sample_id ;
	int8_t		rtk_num_sats;
	int32_t 	north; 
	int32_t		east;
	int32_t		down;
	int32_t		vel_north;
	int32_t		vel_east; 
	int32_t		vel_down;
	uint32_t	spp_sample_id; 
	int8_t 		spp_num_sats ;
	int32_t		latitude ; 
	int32_t		longitude  ;
	} esensor_t ; 
	*/
	
		//int ret = sscanf( recv_data ,
			///******id,sats, N, E, D,vN,vE,vD,ID,NSAT,la,lo**************************************/
			//"$GPGGP"
			//",%"SCNu32",%"SCNd8
			//",%"SCNd32",%"SCNd32",%"SCNd32,
			//&(ereading.rtk_sample_id) , &(ereading.rtk_num_sats), 
			//&(ereading.north), &(ereading.east), &(ereading.down)
		//) ;
		
		int ret = sscanf( recv_data ,
			/******id,sats, N, E, D,vN,vE,vD,ID,NSAT,la,lo**************************************/
			"$GPGGP"
			",%"SCNu32",%"SCNd8
			",%"SCNd32",%"SCNd32",%"SCNd32
			",%"SCNd32",%"SCNd32",%"SCNu32
			",%"SCNu32",%"SCNd8
			",%"SCNx32",%"SCNx32, 
			&(ereading.rtk_sample_id) , &(ereading.rtk_num_sats), 
			&(ereading.north), &(ereading.east), &(ereading.down), 
			&(ereading.vel_north), &(ereading.vel_east), &(ereading.vel_down),
			&(ereading.spp_sample_id), &(ereading.spp_num_sats) ,
			&(ereading.latitude), &(ereading.longitude)
		) ;
			
		
		if ( ret == 12 )
		{
			PRINTF_FL("#%"PRIu8", "
				"NED %.1fm,%.1fm ---"
				"GPS %.6fº,%.6fº\n",
				my_id, 
				ereading.north/1000.0, 
				ereading.east/1000.0 ,
				ereading.latitude/1e6, 
				ereading.longitude/1e6) ;
		}
		else
		{
			printf("got %ldB . only mathed %d items\n",num_bytes_read,ret);
			dumpData((uint8_t*)recv_data, num_bytes_read);
		}
		// TODO: run this cycle with period T (read sim clock for that ,and sleep)
	}
	return NULL ;
}

esensor_t esensor_getReading(void)
{
	return ereading ; 
}

