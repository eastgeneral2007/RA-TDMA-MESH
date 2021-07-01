/**
 * Luis Pinto 2012
 * internal_sensor_api.h
 * aka IAA
 * use these functions to: send AT commands 
 */


#include <sys/timerfd.h> /*makeperiodic, wait_period */
#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <sys/socket.h>
#include <math.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <arpa/inet.h>
#include <stdarg.h> /* variadic functions */
#include <signal.h>

#include "drk/sim_clock.h"
#include "drk/internal_actuator_api.h"
#include "drk/drk.h"  /* drk_getMyIP , drk_error , drk_sem_create */
#ifdef SIMULATOR
#include "drk/sim_clock.h"  /* getClock , etc ... */
#endif
#include "drk/navdata.h"
#include "drk/utils.h"
#include "drk/ar_configuration.h"
#include "drk/utils.h"

#define ACT_PERIOD_US	50000 /* us - period to run the actuation thread (50ms) */
#define SA				struct sockaddr
#define MAXLINE			(2048)
#define PREFIX			"IAA"

#ifdef SIMULATOR
	/* simulator stuff */
	#define ACT_LOCAL_PORT_BASE			(41000)
	#define ACT_DATA_PORT				(50000)
#else 
	#define ACT_DATA_PORT				(5556)
#endif


#ifndef ACTUATOR_LOADED_CONDITION
#define ACTUATOR_LOADED_CONDITION \
	if (Actuator_initiated == 0) return 
#endif


/* typedefs */
typedef struct {
	float X ; /* 4 bytes */
	float Y ; /* 4 bytes */
	float Z ; /* 4 bytes */
} truple_t ; /* 12 bytes */

typedef struct {
	truple_t 	thrust ;
	float 		torque ;
} actuator_t ; 



/**********************
 * static PROTOTYPES 
 **********************/
static void *actuator_thread(void *args);



/************
 * Externs
 ***********/
 /* @todo replace this by a local exit flag, add <close> and <init> functions */
//extern uint8_t 		global_exit_all_threads ;



/*******************************************************************************
 * Evil Globals
 ******************************************************************************/
static pthread_t 	act_tid ;
struct global_actuator_		*actuator_buf ; 
volatile int				Actuator_initiated = 0 ;/* flag module initiated or not */
int 						Actuator_exit = 0 ;
int							drone_timestamp;
sem_t 						time_sem;
int 						drone_sockfd_at; /* socket to send at cmds */
struct sockaddr_in 			servaddr_at; /* destination (autopilot or uavsimulator) . int_sensor_api uses this too */
volatile sig_atomic_t 		watchdog_triggered = 0;




void handle_alarm() {
    watchdog_triggered = 1 ;
    //printf("*********************************************************************************\n");
}





/* init actuator module */
error_t drk_actuator_init( void )
{
	int ret ; /* generic returns */
	/* get a piece of mem for the actuator struct */
	if( ( actuator_buf = malloc( sizeof(struct global_actuator_)) ) == NULL) 
	{
		PRINTF_FL_ERR( "[ACTUATOR] error allocating memory actuator_buf\n");
		return E_NO_MEM;
	}

	/* create sems */
	ret = 0 ;
	/* drk_sem_Create in drk.c 	*/
	ret += drk_sem_create( "actuator_sem" , &(actuator_buf->semaphore) ) ; /* controls access to actuator vector */ 
	ret += drk_sem_create( "timestamp_sem" , &(actuator_buf->time_sem) ) ; /* controls access to timestamp increments */
	if (2 != ret)
	{
		PRINTF_FL_ERR( "[ACTUATOR] Only %d/2 Semaphores created \n", ret );
		return E_SEMAPHORE ;
	}
	/* set time to 0 */
	sem_wait(actuator_buf->time_sem); 
	actuator_buf->global_timestamp = 0;
	sem_post(actuator_buf->time_sem); 
	

	/* build connection for sending at commands */    
	drone_sockfd_at = socket(PF_INET, SOCK_DGRAM, 0); 
	if ( drone_sockfd_at == -1)
	{
		PRINTF_FL_ERR( "[ACTUATOR] socket - error");
		return E_SOCKET ;
	}
	
	
	/* create sockaddr for the destination 
	 * send to 127.0.0.X , where X is the UAV # 
	 * in case this is not a simulation, loopback works fine */
	CLEAR(servaddr_at);
	prepare_sockaddr(getMyIP(), ACT_DATA_PORT, &servaddr_at);

	//#ifdef SIMULATOR
	//PRINTF_FL( "[SIM STUFF]\n");
	//servaddr_at.sin_port = htons( ACT_DATA_PORT + drk_getMyIP() );
	//#else
	//servaddr_at.sin_port = htons( ACT_DATA_PORT );
	//#endif 
	//servaddr_at.sin_family = AF_INET; 
	//servaddr_at.sin_addr.s_addr = htonl(INADDR_LOOPBACK) ;
	//char string[16];
	//sprintf(string, "127.0.0.%d",getMyIP());
	//PRINTF_FL("Sending ACTUATIOn to %s:%d\n",string,ACT_DATA_PORT);
	//ret =  inet_pton( AF_INET, string, &(servaddr_at.sin_addr.s_addr));
	//if (ret != 1)
	//{
		//PRINTF_FL_ERR("error on inet_pton!\n");
		//exit(1);
	//}
	//inet_pton( AF_INET , drone_ip_address, &servaddr_at.sin_addr); 
	

	/* Simulator sends us NAV data to one specific localport = <ACT_LOCAL_PORT_BASE + drk_getMyIP() > 
	 * we need to bind this socket . UAVsim distinguishes act packets from 80211, based on the sourceport 
	 * UAVsim distinguishes source of packets based on the sourceport */
	#ifdef SIMULATOR
	int yes = 1 ; /* when app closes unexpectedly, this is useful */
	if (setsockopt(drone_sockfd_at, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
		PRINTF_FL_ERR( "fail to reuse\n");
	
	struct sockaddr_in		si_me ; // for bind socket
	prepare_sockaddr(getMyIP(), ACT_LOCAL_PORT_BASE + getMyIP(), &si_me);
	if ( bind( drone_sockfd_at , (struct sockaddr*)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR( "S bind - error.  port %hu ",
			ntohs(si_me.sin_port));
		PRINTF_FL_ERR( "%s\n", strerror(errno) );
		return E_SOCKET ;
	}
	else
	{
		PRINTF_FL( "Si_me ACT bind - ok . Port %hu\n" ,
			ntohs(si_me.sin_port) );
	}
	#endif //#ifdef SIMULATOR

	/* destination of this socket will always be the same, so connect */
	//if ( connect( drone_sockfd_at, (const SA *)&servaddr_at, sizeof(servaddr_at)) < 0 ) 
	//	PRINTF_FL( "drone_sockfd_at connect - failed ");
	//else
	//	PRINTF_FL( "drone_sockfd_at connect - ok . port %d\n" , ntohs(servaddr_at.sin_port) );
	actuator_buf->global_timestamp = 1 ;
	
	drk_send_at_command( 
		"AT*PMODE=%ld,2\r" , 
		actuator_buf->global_timestamp++ ) ;
	//PRINTF_FL("PMODE time %ld\n", actuator_buf->global_timestamp ) ;

	drk_send_at_command( 
		"AT*MISC=%ld,2,20,2000,3000\r" , 
		actuator_buf->global_timestamp++ ) ;
	//PRINTF_FL("MISC time %ld\n", actuator_buf->global_timestamp ) ;
	

	set_session_profile_app( 
		ARDRONE_SESSION_ID, 
		ARDRONE_PROFILE_ID, 
		ARDRONE_APPLICATION_ID );
	
	//drk_send_at_command( "AT*CONFIG_IDS=%ld,\"48877aaa\",\"00000000\",\"96eb3af0\"\r" ,
		//actuator_buf->global_timestamp++ ) ;
	//PRINTF_FL("CONFIGS time %ld\n" , actuator_buf->global_timestamp ) ;
	////drk_send_at_command( "AT*CONFIG=%ld,\"custom:session_id\",\"-all\"\r" ,
		//actuator_buf->global_timestamp++ ) ;
	//PRINTF_FL("SESSION ID time %ld\n" , actuator_buf->global_timestamp ) ;

	/* all available data will be sent: */
	//drk_send_at_command( "AT*CONFIG_IDS=%ld,\"00000000\",\"00000000\",\"00000000\"\r" ,
		//actuator_buf->global_timestamp++ ) ;
		
	microsleep(10000);
	call_config_ids();
	drk_send_at_command( "AT*CONFIG=%ld,\"general:navdata_demo\",\"FALSE\"\r" ,
		actuator_buf->global_timestamp++ ) ;

	microsleep(10000);
	//drk_send_at_command( "AT*CONFIG_IDS=%ld,\"00000000\",\"00000000\",\"00000000\"\r" ,
		//actuator_buf->global_timestamp++ ) ;
	call_config_ids();
	drk_send_at_command( "AT*CONFIG=%ld,\"general:video_enable\",\"FALSE\"\r" ,
		actuator_buf->global_timestamp++ ) ; /* */
	
	microsleep(10000);
	call_config_ids();
	drk_send_at_command( "AT*CONFIG=%ld,\"video:bitrate\",\"%d\"\r", 
		actuator_buf->global_timestamp++ ,
		0 ) ;
	
	/* Initialize all parameters in the actuate struct */
	sem_wait(actuator_buf->semaphore); 
	(actuator_buf->actuator).takeoff_flag = LANDING ;
	(actuator_buf->actuator).pitch.f 		= 0.0;
	(actuator_buf->actuator).roll.f 		= 0.0;
	(actuator_buf->actuator).gaz.f 			= 0.0;
	(actuator_buf->actuator).spin.f			= 0.0;
	(actuator_buf->actuator).pcmd_flag 		= 0 ;
	(actuator_buf->actuator).magneto_calib	= 0;
	(actuator_buf->actuator).emergency_flag = 0;
	sem_post(actuator_buf->semaphore); 
	
	
	/* create ACT thread */
	pthread_attr_t act_attr;
	pthread_attr_init(&act_attr);
	size_t act_desired_stack_size = 4000000;
	pthread_attr_setstacksize(&act_attr, act_desired_stack_size);
	if (pthread_create(&act_tid, &act_attr, &actuator_thread ,NULL) != 0)
	{
		PRINTF_FL_ERR( "[ACTUATOR] spawning thread: - Error - %s\n", 
			strerror(errno));
		return -1 ;
	}
	//PRINTF_FL( TABS "[ACTUATOR] spawning thread:  - success\n");
	
	
	

	Actuator_initiated = 1 ;
	return E_SUCCESS ;
	
}



/*******************************************************************************
                STRUCT UPDATERS
*******************************************************************************/


/* Sends a command to the drone every ~20ms */
static void *actuator_thread(void* args)
{
	unblock_one_signal( SIGALRM );
	struct sigaction sa ;
	CLEAR(sa);
	sa.sa_handler = handle_alarm ;
	sigaction( SIGALRM, &sa , NULL ) ;
	PRINTF_FL("SIGALRM set\n");

	(void)(args); // suppress unused parameter warning

	while (!Actuator_exit)
	{
		sem_wait(actuator_buf->semaphore);
		drk_actuator_update( &(actuator_buf->actuator) ) ; 
		sem_post(actuator_buf->semaphore);
		
		/* periodically update the actuator */
		microsleep( ACT_PERIOD_US ) ;
		//PRINTF_FL("watchdog_triggered %d\n",watchdog_triggered);
		if ( watchdog_triggered )
		{
			PRINTF_FL_WARN("hover-watchdog triggred!\n");
			/* Set the actuator values */
			struct drone_actuator_ temp_actuator;
			temp_actuator.takeoff_flag = 0 ;
			temp_actuator.pitch.f = 0.0;
			temp_actuator.roll.f = 0.0;
			temp_actuator.gaz.f = 0.0;
			temp_actuator.spin.f = 0.0;
			temp_actuator.pcmd_flag = 0 ; /* hover */
			temp_actuator.emergency_flag = 0;

			/* Pass them into the struct */
			sem_wait(actuator_buf->semaphore);
			memcpy(&(actuator_buf->actuator), &temp_actuator, sizeof(temp_actuator));
			sem_post(actuator_buf->semaphore);
			watchdog_triggered=0;/* only sigalarm triggers this */
		}
		
	}


	PRINTF_FL( "["PREFIX"] Thread terminated normally\t\t[%08x] *******\n",
		(unsigned int)pthread_self() );  

	return NULL;
}


int call_emergency( struct drone_actuator_* const p )
{
	PRINTF_FL( "Calling emergency\n"); 
	sem_wait(actuator_buf->time_sem);
	long time1 = (actuator_buf->global_timestamp)++;
	long time2 = (actuator_buf->global_timestamp)++;
	long time3 = (actuator_buf->global_timestamp)++;
	sem_post(actuator_buf->time_sem);

	/* keep alive, reset , keep alive: */
	char cmd_parameter[150]  ;
	snprintf(cmd_parameter,sizeof(cmd_parameter)-1,
			"AT*REF=%ld,290717696\r" 
			"AT*REF=%ld,290717952\r"
			"AT*REF=%ld,290717696\r",
			time1, time2, time3);

	p->emergency_flag = -1 ; /* reset flag */
	PRINTF_FL( "Emergency flag goes back to -1\n"); 
	
	int ret = drk_send_at_command(cmd_parameter); 
	return ret ;

}


int call_calib( struct drone_actuator_* const p )
{

	PRINTF_FL( "Magneto Calib\n");
	sem_wait(actuator_buf->time_sem);
	char cmd_parameter[150] ;
	snprintf(cmd_parameter,sizeof(cmd_parameter)-1,
		"AT*CALIB=%ld,0\r",
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	p->magneto_calib=0 ; /* reset flag */
	
	return drk_send_at_command(cmd_parameter); 

}



int call_landing(  void )
{
	//PRINTF_FL("Call landing\n");
	sem_wait(actuator_buf->time_sem);
	char cmd_parameter[150] ;
	snprintf(cmd_parameter,sizeof(cmd_parameter)-1,
		"AT*REF=%ld,290717696\r", /* landing */
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	return drk_send_at_command(cmd_parameter); 

}

int call_takingoff( void  )
{

	PRINTF_FL("calling takinfoff..\n");
	sem_wait(actuator_buf->time_sem);
	int ret = drk_send_at_command(
		"AT*REF=%ld,290718208\r",
		actuator_buf->global_timestamp++ ) ;
	sem_post(actuator_buf->time_sem);

	return ret ; 

}

int call_move( struct drone_actuator_* const p )
{
	//PRINTF_FL( "P,R,G,S (%d)= %f %f %f %f\n", 
		//p->pcmd_flag,
		//p->roll.f,p->pitch.f,p->gaz.f,p->spin.f) ;
	
	sem_wait(actuator_buf->time_sem);
	char cmd_parameter[150] ;
	/*PCMD_MAG=
	 * seq,
	 * flag,roll,pitch,gaz,
	 * yaw,magneto_psi, magneto_psi_accuracy */
	/* magneto_psi & accuracy are used only while using absolute control */
	snprintf(cmd_parameter, sizeof(cmd_parameter)-1,
		"AT*PCMD=%ld,%d,%d,%d,%d,%d\r",
		actuator_buf->global_timestamp++,
		p->pcmd_flag, /* 1 - move, 0-hover*/
		p->roll.i,
		p->pitch.i,
		p->gaz.i,
		p->spin.i ) ;
	sem_post(actuator_buf->time_sem);
	return drk_send_at_command(cmd_parameter); 

}

/** Send actuator commands to the drone 
*       Handles take-off, landing, emergency cut-off
*       Flat trimming & magnetometer calibration        
**/
int drk_actuator_update( struct drone_actuator_ *const p )
{

	/* p is a COPY , snapshot taken from the actuator_buf->actuator at some point. 
	* This function is called periodically.
	* actuator_buf->actuator is changed aperiodically.
	* sem_wait(actuator_buf->time_sem) is the semaphore that controls the AT-timestamp we send
	*/
	/* 
	 * TODO: sprintf takes ages. it should be outside semaphores.
	 * nevertheless, this function is the only one that increments global timestamp. 
	 * so semaphore is not really needed. to be confirmed.
	*/
	//#ifdef SIMULATOR // hack the cmd_parameter and send an actuation vector 
	//sem_wait(actuator_buf->time_sem);
	//char cmd_parameter[150] ;
	//snprintf(cmd_parameter,149,
		//"AT*PCMD=%ld,%d,%f,%f,%f,%f\r",
		//actuator_buf->global_timestamp++,
		//p->pcmd_flag,
		//p->roll.f,
		//p->pitch.f,
		//p->gaz.f,
		//-p->spin.f);
	//sem_post(actuator_buf->time_sem);
	////PRINTF_FL( "sending u=(%.1f,%.1f,%.1f) :: w=(%.1fº)\n" , 
	////	p->roll.f, p->pitch.f, p->gaz.f , p->spin.f ) ;
	//return drk_send_at_command(cmd_parameter); 
	//#endif
	

	
	//#ifndef SIMULATOR
	if ( p->emergency_flag == 1 ) 
	{
		return call_emergency( p ) ;
	} 

	if ( p->magneto_calib == 1 )
	{
		return call_calib( p ) ;
	}


	if ( p->takeoff_flag == LANDING )
	{
		return call_landing() ;
	}

	if ( p->takeoff_flag == TAKINGOFF )
	{ // Take off 
		return call_takingoff( ) ;
	}
	
	/* Heart beat to the drone */
	return call_move( p );
	//#endif // ifndef SIMULATOR
	


	/* send the UDP AT command to the drone */
	//printf("<emerg%d, takeoff%d, pcmd%d, magnet%d, trim%d, %f,%f,%f,%f>\n",
	//		p->emergency_flag, p->takeoff_flag,p->pcmd_flag,p->magneto_calib,p->flat_trim,
	//		p->roll.f,p->pitch.f,p->gaz.f,p->spin.f);
	//PRINTF_FL( "sending\n"); fflush( stdout ) ;

	
}


/* Play an animated sequence on the LEDs 
 * frequency Hz
 * duration seconds
*/

int drk_play_LED_animation(enum LED_ANIMATION animation, float frequency, int duration)
{
	/* float memory is parsed as 32-bit int, then sent */

	if (sizeof(float) != 4) return -1; // floats must be 4 bytes
	int32_t frequency32d ; 
	memcpy( &frequency32d , &frequency , 4 ) ;

	char command_buffer[1024]=""; // max is 1024 characters, hardcoded 

	sem_wait(actuator_buf->time_sem);
	
	snprintf(command_buffer, sizeof(command_buffer)-1,
		"AT*LED=%ld,%d,%d,%d\r", 
		actuator_buf->global_timestamp++, 
		animation,
		frequency32d,
		duration ) ;
	sem_post(actuator_buf->time_sem);

	drk_send_at_command(command_buffer);

	return 1 ;
}

//void drk_update_navdata_packets()
//{
	//uint32_t option_mask = 0;
	//option_mask |= (1 << NAV_DEMO_TAG);
	//option_mask |= (1 << NAV_VIS_DETECT_TAG);
	//option_mask |= (1 << NAV_MAGNETO_TAG);
	//option_mask |= (1 << NAV_PRESSURE_RAW_TAG);
	//option_mask |= (1 << NAV_KALMAN_PRESSURE_TAG);
	////option_mask |= (1 << NAV_CHECKSUM_TAG); // TODO: this line gives a warning.
	//option_mask |= (1 << NAV_PHYS_MEAS_TAG);
	//option_mask |= (1 << NAV_RAW_MEAS_TAG);

	//char buf[80] = "";
	//sem_wait(actuator_buf->time_sem);
	//sprintf(buf, "AT*CONFIG=%ld,\"general:navdata_options\",\"%d\"\r", 
		//actuator_buf->global_timestamp++, option_mask);
	//sem_post(actuator_buf->time_sem);
	//drk_send_at_command(buf);
    
//}


/*******************************************************************************
                        AT COMMANDS
*******************************************************************************/

/**
 * drk_send_at_command, sends the parsed AT command to the drone
 *      
 *      intput:  the pointer of the drone_senor struct;
 *      output:  0 for failure, 1 for success.
 **/
int drk_send_at_command( const char *format, ... )
{       
	//if(debug2) 
	//printf("cmd: \n<%s>\n", send_command );
	//#ifdef SIMULATOR // hack the cmd_parameter and send an actuation vector 
	//PRINTF_FL( "[SIM STUFF]\n");
	//size_t len = sizeof( actuator_t ) ;
	//#else 
	/* read variadic template */
	va_list arg ;
	va_start( arg , format );
	char command_buffer[1024] ;
	vsnprintf( command_buffer , sizeof(command_buffer)-1 , format , arg ) ;
	va_end( arg ) ; /* mustdo this after a va_start */

	//char tmp[150];
	//strcpy( tmp , send_command ); 
	//for (uint8_t i=0; i<(uint8_t)len; i++)
	//	if ( tmp[i] == '\r' ) tmp[i]='#'; /* print at most 12 chars */
	//PRINTF_FL( "sending command <%s>\n", tmp );

	//#endif
	if( sendto(drone_sockfd_at, 
		command_buffer , 
		strlen(command_buffer) , 
		0, (SA *)&servaddr_at, sizeof(servaddr_at)) <  0 )
	{
		PRINTF_FL( "sendto() - ERROR \n");
		return -1;
	}

	return 1;
}



/**
 * Helper Function 
 * Used to send comwdg command, periodically. 
 * We send motion commands instead
 **/ 
int drk_send_at_command_comwdg( void )
{

	PRINTF_FL( "Reset watchdog\n");
	sem_wait(actuator_buf->time_sem);
	drk_send_at_command("AT*COMWDG=%ld\r", actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);

	return 0;
}


error_t drk_actuator_close( void )
{
	ACTUATOR_LOADED_CONDITION E_NO_INIT ;
	
	Actuator_initiated = 0 ;
	Actuator_exit = 1 ;
	
	pthread_kill( act_tid , SIGTSTP ) ;
	pthread_join( act_tid , NULL) ; 

	//PRINTF_FL_WARN("joined actuator\n");
	if ( drone_sockfd_at != -1 )
	{
		close(drone_sockfd_at) ;
		PRINTF_FL( "["PREFIX"] Socket closed\n");
	}

	if ( actuator_buf != NULL )
	{
		if ( drk_sem_cleanup( "timestamp_sem" , actuator_buf->time_sem ) < 0 )
			return E_OTHER ;
		if ( drk_sem_cleanup( "actuator_sem" , actuator_buf->semaphore ) < 0 )
			return E_OTHER ;
	}
	
	
	return E_SUCCESS ;

}


