/* sensor rx */
/* this code pretends we have a gps connected to the usb port
 * drone-simulated-code receives the gps from the simulator itself (via UDP) */

//#include "libsbp/read_simgps.h"
#include "libsbp/read_piksi.h"

#include "drk/world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "drk/utils.h"
#include "drk/gps.h"


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
#include <signal.h> //pthr_kill
#define PREFIX 	"SIMGPS"


#ifndef PIKSI_LOADED_CONDITION
#define PIKSI_LOADED_CONDITION(x) \
	if (Piksi_initiated == 0){PRINTF_FL_WARN("Piksi not initiated\n"); return x;}
#endif



typedef struct {
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
} esensor_t ; /* simulator rtk sensor t */


static void copytogps(void);
int process_gpsmessage(char *revdata,  ssize_t num_bytes_read);
static void *thread_rx_esensor(void *arg);


/** **********
 * evil static globals
 *****************/
static gps_t*		 			gps_data ; // struct defined in read_piksi.h
static const llh_t 				llh_zero = {0,0,0,0,0}; 
static const ned_t 				ned_zero = {0,0,0,0,0}; 
static pthread_mutex_t			gps_mutex ; /* to access the gps_data struct  */
static volatile uint8_t 		Exit_threads = 0 ; /* flag if is time to close this module */
static volatile int 			Piksi_initiated = 0 ; /* module initiated ? block all ops while not initiated */

static esensor_t ereading; /* save here a copy of the last rcvd reading */
static tinfo_t tinfo;

error_t drk_piksi_init( int usb_port )
{
	

	/*  create a gps_data buf */
	if (NULL==(gps_data = calloc(1, sizeof(gps_t)) ))
	{
		PRINTF_FL_ERR( "Error allocating <gps_data>\n");
		return E_NO_MEM ;
	}
	
	/* init mutex */
	if ( pthread_mutex_init( &(gps_mutex), NULL) != 0 )
	{
		PRINTF_FL_ERR( "Gps_mutex - init failed");
		return E_SEMAPHORE ;
	}
	
	
	tinfo.n=0;
	tinfo.inputs[tinfo.n][0] = usb_port ;/* use the usb port to tell our ID */
	if (pthread_create( &(tinfo.tid[tinfo.n]), NULL, &thread_rx_esensor, tinfo.inputs[tinfo.n] ) != 0 )
		return E_OTHER;		
	
	Piksi_initiated = 1;
	return E_SUCCESS;		
}


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
	int sock_fd ; // socket 
	struct sockaddr_in si_other ; // sockaddr for the received packet
	socklen_t slen = (socklen_t)sizeof( si_other ) ;

	/* prepare UDP socket */
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) ==-1)
		fprintf(stderr , "ESENSOR@ error geting udp socket\n");
	
	struct sockaddr_in si_me ; // for bind socket
	prepare_sockaddr(my_id, PORT_ESENSOR, &si_me);
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
		PRINTF_FL_ERR("error calloc\n");
		return NULL ;
	}
	
	CLEAR(ereading);
	//position.X = 0 ; 
	while (1)
	{
		ssize_t num_bytes_read = recvfrom(sock_fd, recv_data, 
			MAX_80211_SIZE,
			0, (struct sockaddr*)&si_other , &slen) ; /* blocking */
		// recvfrom only returns when a message is received. simGPS sends a msg every 100ms
		//printf("ESENSOR: %s\n", (char*)recv_data); // show GPS received
	
		process_gpsmessage(recv_data,num_bytes_read);
	}
	return NULL ;
}

int process_gpsmessage(char *recv_data, ssize_t num_bytes_read)
{
	
	uint32_t a,b;
	int ret = sscanf( recv_data ,
			/******id,sats, N, E, D,vN,vE,vD,ID,NSAT,la,lo =12 fields **************************************/
			"$GPGGP"
			",%"SCNu32",%"SCNd8
			",%"SCNd32",%"SCNd32",%"SCNd32
			",%"SCNd32",%"SCNd32",%"SCNd32
			",%"SCNu32",%"SCNd8
			",%"SCNx32",%"SCNx32, 
			&(ereading.rtk_sample_id) , &(ereading.rtk_num_sats), 
			&(ereading.north), &(ereading.east), &(ereading.down), 
			&(ereading.vel_north), &(ereading.vel_east), &(ereading.vel_down),
			&(ereading.spp_sample_id), &(ereading.spp_num_sats) ,
			&a,&b ) ;
			
		
		if (12 != ret)
		{
			printf("got %ldB . only mathed %d items\n",num_bytes_read,ret);
			dumpData((uint8_t*)recv_data, num_bytes_read);
			return E_OTHER ;
		}
		
		
		ereading.latitude = *(int32_t*)(&a);
		ereading.longitude = *(int32_t*)(&b);
		copytogps(); /* save to gps_data */
		
		/* some debug */
		//PRINTF_FL(
			//"sats #%d "
			//"NED %.1fm,%.1fm ---"
			//"GPS %.6fº,%.6fº\n",
			//ereading.rtk_claramente num_sats,
			//ereading.north/1000.0, 
			//ereading.east/1000.0 ,
			//ereading.latitude/1e6, 
			//ereading.longitude/1e6) ;	
		return E_SUCCESS ;
}

void copytogps(void)
{
	gps_data->ned = (ned_t){
		ereading.rtk_num_sats,
		ereading.north/1e3,
		ereading.east/1e3,
		ereading.down/1e3,
		getEpoch()};
		
	gps_data->spp = (llh_t){
		ereading.latitude/1.0e6,
		ereading.longitude/1.0e6,
		-1,
		ereading.spp_num_sats,
		getEpoch() };
	gps_data->best = gps_data->spp;
	//PRINTF_FL_WARN("lat: %.3fº\n ",gps_data->best.latitude);
	
	// convert arclen to radians
	double north_radians = gps_data->ned.north / LAT_RADIUS;
	double east_radians = gps_data->ned.east / LON_RADIUS;
	// convert radians to degrees
	double north_degrees = radians_to_degrees(north_radians);
	double east_degrees = radians_to_degrees(east_radians);
	//gps_data->basestation = (llh_t){
		//gps_data->spp.latitude - north_degrees ,
		//gps_data->spp.longitude - east_degrees ,
		//-1,//gps_data->spp.altitude - gps_data->ned.down ;
		//MIN(ereading.spp_num_sats,ereading.rtk_num_sats),
		//getEpoch()
	//};
	
	
	
}



/******** prototypes ***/
/* get BS LLH  */
llh_t drk_piksi_get_bs(void)
{
	PIKSI_LOADED_CONDITION(llh_zero); 
	pthread_mutex_lock( &(gps_mutex) ) ;
	llh_t mycpy = gps_data->basestation ; /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	return mycpy ;
}

/* set BS LLH  */
error_t drk_piksi_set_bs( llh_t bs_llh )
{
	PIKSI_LOADED_CONDITION(E_NO_INIT); 
	pthread_mutex_lock( &(gps_mutex) ) ;
	gps_data->basestation = bs_llh ;
	pthread_mutex_unlock( &(gps_mutex) ) ;
	PRINTF_FL(
		"Basestation set to (%3.6lfº,%3.6lfº,%3.2fm)\n", 
		bs_llh.latitude, 
		bs_llh.longitude,
		bs_llh.altitude );
	return E_SUCCESS ;
}


llh_t drk_piksi_get_spp(void)
{
	PIKSI_LOADED_CONDITION(llh_zero); 	
	//PRINTF_FL_WARN("grabing gps_data->spp\n");
	pthread_mutex_lock( &(gps_mutex) ) ;
	llh_t mycpy = gps_data->spp  ; //best /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	//PRINTF_FL_WARN("returning\n");
	//sleep(1);
	return mycpy ; 
}

ned_t drk_piksi_get_ned(void)
{
	PIKSI_LOADED_CONDITION(ned_zero); 	
	//PRINTF_FL_WARN("grabing gps_data\n");
	pthread_mutex_lock( &(gps_mutex) ) ;
	ned_t mycpy = gps_data->ned  ; //best /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	//PRINTF_FL_WARN("returning\n");
	return mycpy ; 
}

llh_t drk_piksi_get_best(void)
{
	PIKSI_LOADED_CONDITION(llh_zero); 	
	//PRINTF_FL_WARN("grabing gps_data\n");
	pthread_mutex_lock(&gps_mutex) ;
	llh_t mycpy = gps_data->best  ; //best /* get the best out of the piksi */
	//llh_t mycpy = gps_data->spp  ; /* get spp out of the piksi */
	pthread_mutex_unlock(&gps_mutex) ;
	//PRINTF_FL_WARN("returning\n");
	return mycpy ; 
}

error_t drk_piksi_close( void ) 
{

	PIKSI_LOADED_CONDITION(E_NO_INIT); 
	
	//PRINTF_FL_WARN("closing piksi \n");
	Exit_threads = 1 ;
	Piksi_initiated = 0 ;
	
	//PRINTF_FL("["PREFIX"] sending signals \n");
	for (int i = 0; i<tinfo.n;i++)
		while ( 0 != pthread_kill(tinfo.tid[i], SIGTSTP ) ) 
		{
			usleep(1000);
		}

	//PRINTF_FL("["PREFIX"] joining \n");
	for (int i = 0; i<tinfo.n;i++)
		pthread_join( tinfo.tid[i] , NULL ) ; 
	
	//PRINTF_FL("[" PREFIX "] Closed\n");
	
	return E_SUCCESS ; 
	
	

}



esensor_t esensor_getReading(void)
{
	return ereading ; 
}

