/* Piksi USB reader */
/* Module does:
 1) init: opens serial port, registers callbacks, 
 2) callbacks: feed a struct with latest GPS data (NED and SPP) 
 3) reading: user reads global struct assync, to access latest data
 */

#include <sys/wait.h> // waitpid
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/select.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h> // we have threads

#include "libsbp/sbp.h"
#include "libsbp/navigation.h"
#include "libsbp/settings.h"

#include "drk/utils.h"
#include "libsbp/read_piksi.h"

#define PREFIX	"PIKSI"
#ifndef PIKSI_LOADED_CONDITION
#define PIKSI_LOADED_CONDITION \
	if (Piksi_initiated == 0) return 
#endif
#define N_THREADS		1
#define EARTH_RADIUS 6368913 // lets use the radius at 41.17º lat
#define LAT_RADIUS   6368913 // = R_lon,  lat = 41.17º . NORTH SOUTH deltas
#define LON_RADIUS   4801205 // = a.cos( lat ) ,  lat = 41.17º. WEST EAST deltas

#define FILENAME_USB "/dev/ttyUSB"
//#define FILENAME_USB "/dev/usb"

/***********
 * Globals
 * **********/
static gps_t*		 			gps_data ; // struct defined in read_piksi.h
static const llh_t 				llh_zero = { 0,0,0,0,0}; 
static const ned_t 				ned_zero = { 0,0,0,0,0}; 
static pthread_mutex_t			gps_mutex ; /* to access the gps_data struct  */
static int 						usb_fd ; /* usb fd */
static volatile uint8_t 		Exit_threads = 0 ; /* flag if is time to close this module */
static pthread_t 				Thread_tids[N_THREADS] ; /* thread ids : tx and rx */
static volatile int 			Piksi_initiated = 0 ; /* module initiated ? block all ops while not initiated */
static volatile sbp_state_t 	State;

/******** prototypes ***/
void *piksiThread();
error_t initThreads(void);
error_t openUSBPort( int usb_port );
error_t closeUSBPort( void );
int usbHasData(void);
uint32_t my_read(uint8_t *buff, uint32_t n, void *context);
uint32_t my_write(uint8_t *buff, uint32_t n, void *context);
void processNED(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
void processLLH(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);

s8 piksi_save_to_flash( sbp_state_t * s );
s8 piksi_frontend_antenna_selection( sbp_state_t * s , char* value  ) ;
s8 piksi_simulator_enabled( sbp_state_t * s, char* status );

/************* code starts here ************/

/* get BS LLH  */
llh_t drk_piksi_get_bs(void)
{
	PIKSI_LOADED_CONDITION llh_zero ; 
	pthread_mutex_lock( &(gps_mutex) ) ;
	llh_t mycpy = gps_data->basestation ; /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	return mycpy ;
}

/* set BS LLH  */
error_t drk_piksi_set_bs( llh_t bs_llh )
{
	PIKSI_LOADED_CONDITION E_NO_INIT ; 
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
	PIKSI_LOADED_CONDITION llh_zero ; 	
	//PRINTF_FL_WARN("grabing gps_data\n");
	pthread_mutex_lock( &(gps_mutex) ) ;
	llh_t mycpy = gps_data->spp  ; //best /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	//PRINTF_FL_WARN("returning\n");
	return mycpy ; 
}

ned_t drk_piksi_get_ned(void)
{
	PIKSI_LOADED_CONDITION ned_zero ; 	
	//PRINTF_FL_WARN("grabing gps_data\n");
	pthread_mutex_lock( &(gps_mutex) ) ;
	ned_t mycpy = gps_data->ned  ; //best /* get the best out of the piksi */
	pthread_mutex_unlock( &(gps_mutex) ) ;
	//PRINTF_FL_WARN("returning\n");
	return mycpy ; 
}

llh_t drk_piksi_get_best(void)
{
	PIKSI_LOADED_CONDITION llh_zero ; 	
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

	
	PIKSI_LOADED_CONDITION E_NO_INIT ; 
	
	//PRINTF_FL_WARN("closing piksi \n");
	Exit_threads = 1 ;
	Piksi_initiated = 0 ;
	
	//PRINTF_FL("["PREFIX"] sending signals \n");
	for ( int i=0 ; i < N_THREADS ; i++ )
	{
		while ( 0 == pthread_kill( Thread_tids[i] , SIGTSTP ) ) 
		{
			usleep(1000);
		}
	}
	
	//PRINTF_FL("["PREFIX"] joining \n");
	for ( int i = 0 ; i < N_THREADS ; i++)
	{
		pthread_join( Thread_tids[i] , NULL ) ; 
	}
	
	//PRINTF_FL("[" PREFIX "] Closed\n");
	
	return E_SUCCESS ; 
	
	

}

sbp_msg_callbacks_node_t node1 ;  /* no need to init */
sbp_msg_callbacks_node_t node2 ;  /* no need to init */

/* initiate PIKSI reading thread,
 * and respective structs */
error_t drk_piksi_init( int usb_port )
{

	PRINTF_FL( "Initiating...\n");
	

	/* init State_t struct */
	sbp_state_init(&State);
	
	/* define 2 callbacks: */

	/* 1) define call back for LLH */
	{
		sbp_msg_callback_t cb = &processLLH ;
		void* context = NULL ; /* not really needed */
		s8 ret = sbp_register_callback( 
			&State , 
			SBP_MSG_POS_LLH_DEP_A , 
			cb , 
			context,
			&node1 ) ;
		if ( ret < 0 )
		{
			PRINTF_FL_ERR("LLH callback - failed (%"PRId8")\n", ret );
			return E_OTHER;
		}
		else
		{
			PRINTF_FL("LLH pkt callback - Defined!\n");
		}
	}


	/* 2) define call back for NED */
	{
		sbp_msg_callback_t cb = &processNED ;
		void* context = NULL ; /* not really needed */
		
		s8 ret = sbp_register_callback( 
			&State , 
			SBP_MSG_BASELINE_NED_DEP_A , 
			cb , 
			context,
			&node2 ) ;
		if ( ret < 0 )
		{
			PRINTF_FL_ERR("NED pkt callback - failed (%"PRId8")\n", ret );
			return E_OTHER;
		}
		else
		{
			PRINTF_FL("NED pkt callback - Defined\n");
		}
		
	}
	
	
	
	/* open usb */
	{
	 error_t ret = openUSBPort( usb_port );
	 if ( ret < 0 )
		return ret;
	}
	PRINTF_FL( 
		"USB port is open\n");
	
	
	
	#if 0
	//piksi_simulator_enabled( &State , "False" ) ;
	piksi_simulator_enabled( &State , "True" ) ;
	piksi_frontend_antenna_selection( &State , "Patch" ) ;
	//piksi_frontend_antenna_selection( &State , "External" ) ;
	
	/* save to flash :*/
	piksi_save_to_flash( &State );
	#endif

	
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
	
	
	
	
	{
	 PRINTF_FL( "Spawning thread to start reading USB%d\n", usb_port );
	 error_t ret = initThreads(); // init piksiThread thread
	 if ( ret < 0 )
		return ret;
	}
	
	Piksi_initiated = 1 ;
	return E_SUCCESS ;

}


error_t initThreads( void )
{
	/* create thread attributes : stack size */
	size_t desired_stack_size = 200000 ; // thread stack size 400KB
	pthread_attr_t thread_attr;
	pthread_attr_init( &thread_attr );
	pthread_attr_setstacksize( &thread_attr, desired_stack_size ) ;

	/* Create thread that will continuously send messages from other nodes */
	if ( pthread_create( &Thread_tids[0] , 
		&thread_attr, &piksiThread , NULL) != 0 )
	{
		PRINTF_FL_ERR("[Piksi Thread] - Error spawning (%s)\n", 
			strerror(errno) );
		return E_PTHREAD ;
	}
	
	/* add here other threads */
	//...
	
	
	if ( 0 != pthread_attr_destroy( &thread_attr ) )
		return E_PTHREAD ;

	/* wait for thread(s) to enter their cycles */
	usleep(10000) ;

	
	/* if threads are already dead, lets join and return -1 */
	if (Exit_threads)  //someone already called Exit
	{
		/* wait here until all app threads terminate! */
		for ( unsigned int j = 0 ; j < sizeof(Thread_tids) ; j++ )
			pthread_join( Thread_tids[j] , NULL ) ;
		return E_TERMINATING ;
	}
	return E_SUCCESS ;
}

/* this is a swpaned thread */
void *piksiThread()
{
	/* start reading serial port , process packet and call callback*/
	while ( usbHasData() >= 0 ) /* error or time to close : returns < 0 */
	{
		//printf("serial data available!\n");
		s8 ret = sbp_process( &State, &my_read);
		switch ( ret )
		{
			case SBP_OK_CALLBACK_EXECUTED :
				//printf("MSG COMPLETE. and callback called!\n");
				//printf("msglen %"PRIu8"B..\n", s.msg_len );
				
				break;
			case SBP_OK: 
				//printf("processed - msglen %"PRIu8"B..\n", s.msg_len );
				break;
			case SBP_CRC_ERROR:
				PRINTF_FL_WARN("crc error\n");
				break;
			case SBP_OK_CALLBACK_UNDEFINED:
				//printf("MSG COMPLETE. nothing done\n");
				//printf("msglen %"PRId8"B..\n", s.msg_len );
				//printf(
					//"msgtype %04"PRIx16". "
					//"(we're looking for %04"PRIx16")\n", 
					//s.msg_type , 
					//SBP_MSG_BASELINE_NED );
				break;
			default:
				PRINTF_FL_ERR("closing. weird return\n");
				goto out_while;
		}
	}

	out_while: ;
	/* time to end thread ... */
	//PRINTF_FL_WARN("out of while\n");
	
	Piksi_initiated = 0 ;
	
	/* close usb port */
	closeUSBPort() ;
	
	/* delete Serial mutex */
	int ret = pthread_mutex_destroy( &(gps_mutex) );
	if ( ret < 0 )
	{
		PRINTF_FL_ERR( 
			"GPS mutex - failed destroy - %s\n",
			strerror(errno));
	}
	//else
	//{
		//PRINTF_FL( "["PREFIX"] GPS mutex destroyed\n");
	//}
	

	free(gps_data);
	
	PRINTF_FL("Thread terminated normally\t[%08x]\n", 
		(unsigned int)pthread_self() );  

	return NULL; /* thread is finished */
}

error_t closeUSBPort(void)
{
		int ret;
		#ifdef SIMULATOR
		ret = close( usb_fd ); 
		#else
		if (usb_fd == 0)
			return E_FILE_NOT_FOUND ;
		ret = close(usb_fd); 
		#endif
		if ( ret < 0 )
		{
			PRINTF_FL_ERR("USB port - Close() FAILED \n");
			return E_OTHER ;
		}
		
		
		//PRINTF_FL( "USB port - Closed\n");
		return E_SUCCESS ;
		
}

/* returns >0,  when data is available. 
 * returns <0, when error or time2close */
int usbHasData(void)
{
	/* Watch <fd> to see when it has input. */
	fd_set readfds ;
	FD_ZERO(&readfds);
	FD_SET( usb_fd , &readfds );
	//printf("waiting for data\n");
	int ret = select( usb_fd+1 , &readfds, NULL , NULL, NULL );
	
	if ( ret < 0 )
	{
		if (errno == EINTR)
		{
			return ret ;
		}
		else
		{
			PRINTF_FL_ERR("select failed %s\n", strerror( errno ) );
		}
	}
	// if ret = 0 , means timeout 
	
	//if ( Exit_threads == 1 )
		//ret = -2 ;
		
	return ret ;
}

/* my definition to read from USB */
uint32_t my_read(uint8_t *buff, uint32_t n, void *context)
{
	//PRINTF_FL_WARN("reading %" PRIu32 "B..\n",n);
	//printf("Read called with n=%"PRIu32"B\n", n );
	/* context not needed*/
	context=NULL;
	uint32_t i ;
	for (i=0; i<n; i++)
	{
		if (usbHasData())
		{
			read( usb_fd, (void*)&(buff[i]) , 1 );
			//printf("[%02d]", buff[i] ) ;
		}
		else
		{
			break;
		}
	}
	//printf("..\n" ) ;
	return i ;
}

/* my function that write to usb */
uint32_t my_write(uint8_t *buff, uint32_t n, void *context)
{
	//printf("Read called with n=%"PRIu32"B\n", n );
	context=NULL;/* context not needed*/
	return (uint32_t)write( usb_fd, (void*)buff , n );


}



/* open serial port */
error_t openUSBPort( int usb_port )
{
	char usb_port_str[16];
	//snprintf( 
		//usb_port_str ,
		//sizeof(usb_port_str)-1, 
		//FILENAME_USB "%hu", 
		//usb_port ) ;
	snprintf( usb_port_str ,15, "/dev/ttyUSB%hu", usb_port ) ;
	
	long baud, databits, stopbits, parityon, parity;
	//place for old and new port settings for serial port
	struct termios newtio ;

	//int got_connection = 0;
	//int reply_mode = LAST_CONNECTION;

	//PARITYON = PARENB;
	//PARITY = PARODD;
	/* open the device(com port) to be non-blocking (read will return immediately) */
	usb_fd = open( usb_port_str , O_RDWR | O_NOCTTY | O_NONBLOCK ) ;
	if (usb_fd < 0) 
	{
		PRINTF_FL_ERR( "Failed to open %s (%s)\n" , usb_port_str , strerror(errno) ) ;
		return E_FILE_NOT_FOUND ;
	}
	
	/* set new port settings for canonical input processing */
	baud = B1000000 ;
	databits = CS8 ;
	stopbits = 0 ;
	//STOPBITS = CSTOPB;
	parityon = 0 ;
	parity = 0 ;
	newtio.c_cflag = baud | databits | stopbits | parityon | parity | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;       //ICANON;
	newtio.c_cc[VMIN] = 1; /* return one byte at a time */
	newtio.c_cc[VTIME] = 0; /* no timeout */
	tcsetattr( usb_fd, TCSANOW, &newtio);

	//tcflush (port_fd, TCIOFLUSH); //io flushes data not read & not written
	//usleep(100000);
	//tcflush( usb_fd , TCIFLUSH); //input .flushes data not read & not written
	//usleep(100000);
	//tcflush (port_fd, TCOFLUSH); //out .flushes data not read & not written	
	return E_SUCCESS ; /* success */
}

/* what to do when a NED msg is recvd */
void processNED(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
	//PRINTF_FL_WARN("proc NED \n");
	// Process msg.
	context = NULL ; /* d.c.*/
	sender_id = 0 ; /* d.c. */
	len = 0 ; /* d.c */
	
	/* grab sample */
	msg_baseline_ned_t sample = *(msg_baseline_ned_t*)msg;
	//memcpy( (void*)&ned_sample , (void*)msg , len ) ;
	
	/* printout */
	//printf( 
		//"epoch %f "
		//"nstats %" PRIu8 " "
		//"North %"PRId32" "
		//"East %"PRId32" "
		//"Down %"PRId32 " "
		//"\n" ,
		//getEpoch(),
		//sample.n_sats ,
		//sample.n , 
		//sample.e ,
		//sample.d		
		//);
		
	pthread_mutex_lock( &(gps_mutex) ) ;
	gps_data->ned.num_sats = sample.n_sats ; 
	gps_data->ned.north = sample.n/1e3 ; 
	gps_data->ned.east = sample.e/1e3 ; 
	gps_data->ned.down = sample.d/1e3 ; 
	gps_data->ned.timestamp = getEpoch();
	
	/* compute best gps location based on NED and fixed BS data */

	// convert arclen to radians
	double north_radians = gps_data->ned.north / LAT_RADIUS;
	double east_radians = gps_data->ned.east / LON_RADIUS;

	// convert radians to degrees
	double north_degrees = radians_to_degrees(north_radians);
	double east_degrees = radians_to_degrees(east_radians);

	// lat = base + n, lon = base + e
	//best_lat = base_lat + north_degrees;
	//best_lon = base_lon + east_degrees;
	
	/* Set BS based on NED knowledge. More GPS available? update BS data */
	if ( gps_data->basestation.num_sats < gps_data->ned.num_sats )
	{
		/* IF my ned is [10,20]
		 * means BS is 10south of me
		 * and 20m west of me */
		llh_t BS ;
		BS.latitude = 
			gps_data->spp.latitude - north_degrees ;
			
		BS.longitude = 
			gps_data->spp.longitude - east_degrees ;
			
		BS.altitude = 
			gps_data->spp.altitude - gps_data->ned.down ;

		BS.timestamp = getEpoch() ;
		
		/* sample.n_sats should be the answer all the times:*/
		BS.num_sats = MIN(gps_data->spp.num_sats , sample.n_sats);
		
		gps_data->basestation = BS ;
			
		PRINTF_FL_WARN("BS updated [%.6fº ; %.6fº ; %.1fm]\n",
			BS.latitude, 
			BS.longitude,
			BS.altitude );
			
		PRINTF_FL_WARN( 
		"nsats %" PRIu8 " "
		"North %"PRId32" "
		"East %"PRId32" "
		"Down %"PRId32 " "
		"\n" ,
		sample.n_sats ,
		sample.n , 
		sample.e ,
		sample.d		
		);
		
		PRINTF_FL_WARN( 
		"nsats %" PRIu8 " "
		"lat %lf "
		"lon %lf "
		"height %lf "
		"\n" ,
		gps_data->spp.num_sats ,
		gps_data->spp.latitude , 
		gps_data->spp.longitude ,
		gps_data->spp.altitude
		 );
	}
	
	
	/* We just got NED coordinates.
	 * Set BEST gps coordinates: BS+NED ? */
	gps_data->best.num_sats		= sample.n_sats ; 
	gps_data->best.latitude 	= gps_data->basestation.latitude 	
		+ north_degrees; 
	gps_data->best.longitude 	= gps_data->basestation.longitude 	
		+ east_degrees ;
	gps_data->best.altitude 	= gps_data->basestation.altitude 
		+ sample.d ; 
	gps_data->best.timestamp 	= getEpoch();
	
	pthread_mutex_unlock( &(gps_mutex) ) ;

}

/* what to do when a SPP LLH msg is obtained */
void processLLH( uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
	//PRINTF_FL_WARN("proc LLH\n");
	
	/* Process msg. */
	context = NULL ; /* d.c.*/
	sender_id = 0 ; /* d.c. */
	len = 0 ; /* d.c */
		
	/* grab sample */
	msg_pos_llh_dep_a_t sample = *(msg_pos_llh_dep_a_t*)msg;
	//memcpy( (void*)&ned_sample , (void*)msg , len ) ;
	
	/* printout */
	//PRINTF_FL( 
		//"epoch %lf "
		//"nstats %" PRIu8 " "
		//"lat %lf "
		//"lon %lf "
		//"height %lf "
		//"\n" ,
		//getEpoch(),
		//sample.n_sats ,
		//sample.lat , 
		//sample.lon ,
		//sample.height
		 //);
	
	pthread_mutex_lock( &(gps_mutex) ) ;
	
	
	
	gps_data->spp.num_sats = sample.n_sats ; 
	gps_data->spp.latitude = sample.lat ; 
	gps_data->spp.longitude = sample.lon ; 
	gps_data->spp.altitude = sample.height ; 
	gps_data->spp.timestamp = getEpoch();
	
	
	
	/* if there's no NED data for 1second or more, replace Best with this SPP sample */
	if ( getEpoch() - gps_data->best.timestamp > 1.0 )
	{
		gps_data->best.num_sats = sample.n_sats ; 
		gps_data->best.latitude = sample.lat ; 
		gps_data->best.longitude = sample.lon ; 
		gps_data->best.altitude = sample.height ; 
		gps_data->best.timestamp = getEpoch();
	}
	pthread_mutex_unlock( &(gps_mutex) ) ;
	
	
}


s8 piksi_simulator_enabled( sbp_state_t * state, char* value )
{
	const char section[] = "simulator";
	const char setting[] = "enabled"; 
	
	char msg[100] ;
	uint8_t it = 0 ; 
	memcpy( (uint8_t*)msg, section , sizeof(section) ); /* includes \0 at the end */
	it += sizeof(section);
	memcpy( (uint8_t*)msg+it, setting , sizeof(setting) ); /* includes \0 at the end */
	it += sizeof(setting);	
	
		memcpy( (uint8_t*)msg+it , value , strlen(value)+1 ); /* includes \0 at the end */
	it += strlen(value)+1;	

	
	uint8_t len = it ;
	
	s8 ret = sbp_send_message(
		state,
		SBP_MSG_SETTINGS_WRITE,
		0x42,
		len, 
		(uint8_t*)msg ,
		&my_write ) ;
	printf(
		"ret %"PRId8 ". wrote %"PRIu8" Bytes of settings\n",
		ret,
		len );
	return ret ;
}
	
s8 piksi_save_to_flash( sbp_state_t * state )
{
	s8 ret = sbp_send_message(
		state,
		SBP_MSG_SETTINGS_SAVE,
		0x42,
		0,
		NULL ,
		&my_write );
	printf(
		"Send_msg--> ret %" PRId8 ". Saving settings to flash\n",
		ret );	
	return ret ;
}

s8 piksi_frontend_antenna_selection( sbp_state_t * state , char* value  ) 
{


	const char section[] = "frontend";
	const char setting[] = "antenna_selection"; 
	
	char msg[100] ;
	uint8_t it = 0 ; 
	memcpy( (uint8_t*)msg, section , sizeof(section) ); /* includes \0 at the end */
	it += sizeof(section);
	memcpy( (uint8_t*)msg+it, setting , sizeof(setting) ); /* includes \0 at the end */
	it += sizeof(setting);	
	memcpy( (uint8_t*)msg+it , value , strlen(value)+1 ); /* includes \0 at the end */
	it += strlen(value)+1;	
	
	
	uint8_t len = it ;
	
	s8 ret = sbp_send_message(
		state,
		SBP_MSG_SETTINGS_WRITE,
		0x42,
		len, 
		(uint8_t*)msg ,
		&my_write ) ;
	printf(
		"ret %"PRId8 ". wrote %"PRIu8" Bytes of settings\n",
		ret,
		len );
	return ret ;
}
