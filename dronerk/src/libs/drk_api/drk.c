/*! \file drk.c
    \brief A Documented file.
    
    Details.
*/

// this should go to drkP header
#include <sys/time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h> /* pidwait */
#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <netinet/in.h>
#include <semaphore.h> /* drk_sem_cleanup */
#include <math.h> /* fmod */
#include <sched.h> 
#include <ifaddrs.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "drk/drk.h"
#include "drk/ar_configuration.h" /* set max,min speeds , etc */
#include "drk/autonomous.h" /* autonomous_init() */
#include "drk/flight_control_api.h" /* drk_emergency() */
#include "drk/internal_actuator_api.h" /* global act data */
#include "drk/internal_sensor_api.h" /* global nav data */
#include "drk/internal_sensor_raw.h" /* drk_battery() */
#include "drk/navdata.h" /* print_navd_data(); */
#include "drk/keyboard.h" /* keyboard_init() */
#include "drk/utils.h" /* x */

#include "drk/tdma.h"
#include "drk/packet_manager.h"

#include "libsbp/read_piksi.h"

#ifdef SIMULATOR
#include "drk/sim_clock.h"
#endif

/***************
 * Defines
 * ******************/
#define NAV_STATE_FILE	"/nav.shared"
#define ACT_STATE_FILE	"/act.shared"
#define PREFIX_DRK		"DRK\t\t\t"
#define PREFIX_EDRK		"EMERGENCY\t\t"
#define PREFIX "DRK"

#define LOADTDMAMODULE 1
#define LOADPMMODULE 0
#define LOADEXTERNAL 1
#define LOADKEYBOARD 1

/*** *************
 * structs and enums 
 * ****************/
enum Exit_codes{
	CLOSURE_STOP = 1 ,
	CLOSURE_INT = 2 
} ;


/** extern globals **/
extern char*					program_invocation_short_name;
extern struct global_navdata_* 	navdata_buf;
/*******************************************************************************
 * Evil Globals
 ******************************************************************************/
volatile uint8_t 		global_exit_all_threads=0;//struct _actuator_buf *actuator_share;
static volatile uint8_t global_my_id = 0 ;
uint8_t 				Exit_code = 0 ;
static sem_t 			Exit_semaphore ; /* sig handlers post this , exit thread waits on this */
//static struct timespec last_rec;
//static uint8_t SHOW_MAP = 0 ;

/* thread tids */
// TODO: each module we initiate should clçse itself, we avoid using these
//pthread_t 	act_tid = 0 ;
//pthread_t 	nav_tid = 0 ;
//pthread_t 	key_tid = 0 ;
//pthread_t 	ser_tid = 0 ;
pthread_t 	exit_tid = 0 ;
//pthread_t 	auto_tid = 0 ;
pid_t 		emergency_process = -1 ;

/* function pointers */
void(*Callback)() = NULL ;





/******************************************** 
 * Private prototypes go here: 
 * **********************************************/
/* using drone 2.0 API to set some params */
//static void config_drone();

/* Initiate the navigation and control modules (threads) */
static int 		act_sensor_init( void );

/* terminate app graciously */
static void 	*closureThread() ;

/* initiate a cleanup exit thread */
static error_t 	closureThread_init(void) ;

//int8_t _drk_sem_cleanup( char *sem_name , sem_t *sem_ptr );

/** SIGNAL HANDLERS **/
/* When you get a SIGINT, cut the motors. *//* CTRL+C */
static void		sigint_handler(int num);

/* When you get a SIGSTP, land the drone. *//* CTRL+Z */
static void 	sigtstp_handler(int signum);

/* When you get a segfault, execute a land command */
//static void sigsegv_handler(int signum);

/* */
//static void ignore_signal(int signum);

/*******************************************************************************
              CODE!
*******************************************************************************/



/** Initialize all of the DroneRK components/modules
 * inputs: 
 * 1) usb port to open to read gps 
 * 2) my unique ID in the network.
 * 3) function to call when drk exits.
 * **/
int drk_init( uint8_t usb_port_num, uint8_t my_id, void(*callback)() )
{
	#ifdef SIMULATOR //if we are simulating, lets set our fake ip using the id
	setMyIP(my_id); // utils.h
	#endif
	
	(void)usb_port_num ;
	printf( 
		"------------------------"
		" [Drklib] Loading ... "
		"-------------------------\n" );

	Callback = callback ; /* function to call at drk_close() */
		
	#ifdef SIMULATOR
	global_my_id = usb_port_num ;
	#endif

	/* before creating thereads. lets block all signals */
	block_all_signals(); 
	
	//thread that cleans up at the end
	if (0 > closureThread_init())
		return E_OTHER ;
	PRINTF_FL( "[Closure] Thread - Initiated\n");

	/* define what to do with STOP signal: */
	if (0 > sig_set_action(sigtstp_handler, SIGTSTP))
		return E_OTHER ;
	
	/* define what to do with INT signal: */
	if (0 > sig_set_action(sigint_handler, SIGINT))
		return E_OTHER ;
	PRINTF_FL( "[2 signal handlers] - OK\n") ;
		
	#ifdef SEG_FAULT
	if (0 > sig_set_action(sigsegv_handler, SIGSEGV))
		return E_OTHER ;
	#endif

	

	/** with simulation, we need to read its clock, so lets init **/
	#ifdef SIMULATOR
	if (0 > SimClock_init())
	{
		PRINTF_FL_ERR( "failed sim clock init\n");
		return E_OTHER ;
	}
	PRINTF_FL( "[Simclock] init - OK\n");
	#endif

	
/** create a child process just to initiate emergency "land" binary.**/
	#ifdef FAILSAFE //not active for now
	emergency_process = fork() ;
	
	if( 0 > emergency_process )
	{
		PRINTF_FL_ERR(  "Failed to fork()...\n");
		PRINTF_FL_ERR(  "%d: %s\n", errno, strerror(errno));
		return -1 ;
	}

	if ( emergency_process == 0 )
	{ 
		/*im the child*/
		printf("DRK-child@:\t----BEGIN ---\n"); 
		char* argv[3] ;
		argv[0] = "/data/video/land" ;
		argv[1] = "-s" ;
		argv[2] = NULL ;
		if ( execv(argv[0], argv) == -1 ) // i hold here until ../land exits.
		{
			printf("DRK-child@"); 
			printf("\tERROR: Unable to execute emergency process [land] (Error code: %d)\n",errno);
			printf("\t\tIn case [%s] malfunctions for some reason\n"\
					"\t\t[land] will try to land the drone\n", 
				program_invocation_short_name );
			printf("\t\tIf this doesn't work, flip the drone. Engines will shut down..\n");
		}
		printf("DRK-child@\t----END----\n"); 
		exit( CLOSURE_SUCCESS ) ; // exit , cause i'm the child 
	}
	#endif
	
	
	/* set init time */
	drk_elapsedTimeSecs();
	sleep(1) ;
	
	
	/* init a thread that reads NAvdata and sends AT commands to the drone. */
	// under simulation, we will receive data from the simulator via localhostUDP
	if (0 > act_sensor_init())
	{
		PRINTF_FL_ERR( "[ACTUATOR]+[NAVDATA] - Failed to init \n") ;
		return -1 ;
	}
	PRINTF_FL( "[ACTUATOR]+[NAVDATA] Running **\n") ;


	drk_remove_emergency();


	/** setup drone stuff **/
	drk_ar_change_max_vertical_speed(2000) ;
	drk_ar_set_indoor_flight() ;
	drk_ar_set_indoor_hull() ;
	drk_ar_flat_trim() ;
	
	
	
	#if LOADTDMAMODULE
	/** load TDMA module **/
	if ( 0 > drk_TDMA_init( my_id ) ) /* tdma.c */
	{
		PRINTF_FL_ERR(  "[TDMA] module - ERROR loading\n") ;
		return -3 ;
	}
	PRINTF_FL( "[TDMA] module - ok \n");
	#endif
	
	
	#if LOADPMMODULE
	/** load PM module **/
	if ( 0 > drk_PM_init( getMyIP() ) )
	{
		PRINTF_FL_ERR( "[PM] module - ERROR loading\n" ) ;
		return -4 ;
	}
	PRINTF_FL( "[PM] module - ok\n");
	#endif
	

	
	
	/* init external sensor module : parses serial messages from the piksi USB  */
	//(void)usb_port_num ;
	#if LOADEXTERNAL
	if (0 > drk_piksi_init(usb_port_num))
	{
		PRINTF_FL_ERR("Module [PIKSI] - INIT ERROR!\n") ;
		return -5 ;
	}
	PRINTF_FL("[PIKSI] module - OK\n")  ;
	#endif

	
	

	#if LOADKEYBOARD
	/* init a thread to read Keyboard commands */
	if ( 0 > drk_keyboard_init() )/* keyboard.c */
	{
		PRINTF_FL_ERR("Module [KBD] - INIT ERROR!\n")  ;
		return -6 ;
	}
	PRINTF_FL("Module [KBD] - OK\n")  ;
	#endif




	/* init a thread for the flight to GPS-waypoints: */
	#if 1
	/* controller_init only returns when ready so drk commands should start working fine */
	if ( drk_autonomous_init() < 0 ) 
	{
		
		PRINTF_FL_ERR("Module [AC  ] - failed to init\n");
		return -7 ;
	}
	PRINTF_FL("[AC  ] Module - OK\n");
	#endif

	//unblock_all_signals() ;
	
	
	
	printf( 
		"-------------------------\n"
		" [Drklib] Fully Loaded   \n"
		"-------------------------\n" );
	fflush(stdout);
	return E_SUCCESS ;





	//if ( getMaskAltitudeControlAlgo() != 0 )
	//{
		//PRINTF_FL( "Altitude control? YES\n") ;
	//}
	//else
	//{
		//PRINTF_FL( "Altitude control? NOPE\n") ;
	//}
	

}



//if( actuator_buf->semaphore != SEM_FAILED )
	//{ 
		///* Get semaphore */
		//sem_post( actuator_buf->semaphore );
		 ///* Close the Semaphore */
		//if (-1 ==  sem_close( actuator_buf->semaphore ) )
			//drk_error(PREFIX_DRK "actuator_sem close:");
		//else
		//{
			//PRINTF_FL( "actuator_sem Closed\n");			
			///* Delete the shared memory object */
			//if ( sem_unlink( "actuator_sem" ) == -1 )
				//drk_error(PREFIX_DRK "actuator sem_unlink:");
			////else
			////	PRINTF_FL( "actuator Sem Unlinked\n");
		//}
		//PRINTF_FL( "navdata semaphore cleaned\n");
	//}
	
	//if (actuator_buf == NULL )
	//{
		//PRINTF_FL( "actuator_buf was not defined\n");
	//}
	//else
	//{
		//if( actuator_buf->time_sem != SEM_FAILED )
		//{ 
			///* Get semaphore */
			//sem_post( actuator_buf->time_sem );
			 ///* Close the Semaphore */
			//if (-1 ==  sem_close( actuator_buf->time_sem ) )
				//drk_error(PREFIX_DRK "time_sem , sem_close:");
			//else
			//{
				//PRINTF_FL( "time_sem Sem Closed\n");			
				///* Delete the shared memory object */
				//if (-1 ==  sem_unlink( "time_sem" ) )
					//drk_error(PREFIX_DRK "time sem unlink:");
				//else
					//PRINTF_FL( "time Sem Unlinked\n");
			//}
			
		//}
	//}
	//PRINTF_FL( "time_sem semaphore cleaned\n");	

/**
 * init thread that will deal with signals to exit properly.
 * it will sleep most time 
 * returns:	-1 - error 
 * 			+1 - ok 
 * **/
error_t closureThread_init(void)
{
	
	/* signals will make use of this thread to close program safely */
	pthread_attr_t exit_attr;
	if ( 0 != pthread_attr_init(&exit_attr))
	{
		PRINTF_FL_ERR( "[exit] thread - Error initing attr: %s\n", 
			strerror(errno));
		return -1 ;
	}
	
	size_t exit_desired_stack_size = 40000;
	if ( 0 != pthread_attr_setstacksize(&exit_attr, exit_desired_stack_size) )
	{
		PRINTF_FL_ERR( "[exit] thread - Error settin stack: %s\n", 
			strerror(errno));
		return -1 ;
	}
		
	if ( 0 != pthread_create(&exit_tid, &exit_attr, &closureThread, NULL) ) 
	{
		PRINTF_FL_ERR( "[exit] thread - Error spawning: %s\n", 
			strerror(errno));
		return -1 ;
	}

	if ( 0 != pthread_attr_destroy(&exit_attr) )
	{
		PRINTF_FL_ERR( "[exit] thread - Error destroying attr: %s\n", 
			strerror(errno));
		return -1 ;
	}

	
	return E_SUCCESS ;
}


/** 
 * this threads stays sleeping until sigint or sigstp is recvd 
 * when this happens, it terminates the app as graciously as possible
 * sigstp ? Exit_code = CLOSURE_STOP   ... sigint ? Exit_code = CLOSURE_INT 
 ***********/
void *closureThread()
{
	unblock_one_signal( SIGTSTP );
	unblock_one_signal( SIGINT );

	//PRINTF_FL_ERR( "waiting on exit_sem\n" ) ;
	sem_init( &Exit_semaphore, 0, 0 ); 
	sem_wait( &Exit_semaphore ) ;
	printf("\n");
	PRINTF_FL("Closing DRK library...\n");
	//PRINTF_FL( "exit code# %d\n" , Exit_code) ;
	if ( CLOSURE_STOP == Exit_code )
	{
			// ctrl+z:
			PRINTF_FL_WARN( 
				"WARNING: SIGTSTP [CTRL+Z] detected. "
				"Emergency landing initiating....\n");
			drk_land();
			drk_remove_emergency();
			PRINTF_FL( 
				"Landing is complete. "
				"Exiting the program now.\n");
			//drk_emergency(); // set emergency mode on
			//PRINTF_FL( "Emergency mode ON.\n");
			//kill( nav_tid , SIGINT ) ; /* this makes sure all threads terminate, even with blocking functions */
			//pthread_kill( nav_tid , SIGINT ) ; /* this makes sure all threads terminate, even with blocking functions */
			
	}
	
	if ( CLOSURE_INT == Exit_code )
	{
		PRINTF_FL_WARN( "WARNING: SIGINT [CTRL+C] detected. Emergency cutout in progress.\n");
		drk_emergency(); /* run emergency land mode = STOP engines asap (in FCA.c)*/
		PRINTF_FL( "Cutout is complete. Exiting the program now.\n");
		drk_remove_emergency();
	}


	

	/* start closing all N=7 modules : 
	 * TDMA, PM ,NAVDATA, ACTUATOR, KBD,  PIKSI ,AC, */
	int ret  ;
	global_exit_all_threads = 1 ;
	int i = 0 ;
	#define N "7"
	ret = drk_TDMA_close() ;
	i++ ; 
	PRINTF_FL( "[TDMA] terminated - %hu/" N " -ret: %s\n" , i , getError(ret));

	
		
	ret =drk_PM_close() ;
	i++ ; 
	PRINTF_FL( "[PM] terminated - %hu/" N "-ret: %s\n" , i , getError(ret));
	

	ret =drk_actuator_close();
	i++;
	PRINTF_FL( "[ACTUATOR] terminated - %hu/" N "-ret: %s\n" , i , getError(ret));


	

	drk_piksi_close();
	i++ ; 
	PRINTF_FL( "[PIKSI] terminated - %hu/" N "-ret%d\n" , i,ret );
	
	drk_autonomous_close();
	i++ ; 
	PRINTF_FL( "[AC  ] terminated - %hu/" N "-ret%d\n" , i ,ret);
	

	
	
	drk_keyboard_close() ;
	i++ ; 
	PRINTF_FL( "[KEYBOARD] terminated - %hu/" N "-ret%d\n" , i,ret );

	
	

	ret = drk_sensor_data_close();
	i++ ; 
	PRINTF_FL( "[NAVDATA] terminated - %hu/" N "-ret: %s\n" , i , getError(ret));
	
	
	
	if ( NULL != Callback )
	{
		PRINTF_FL( "Calling user callback\n" );
		Callback();
	}
	else
	{
		PRINTF_FL( "No user callback to run\n" );
	}
	
	printf(
		"--------------------------\n"
		"[Drklib] Closed Correctly \n"
		"--------------------------\n\n");
	return NULL ;
}




#if 0
/** using drone 2.0 API to set some params **/
void config_drone(void)
{

  drk_ar_change_max_vertical_speed(1999); // check what is the max
  usleep(50000);
  drk_ar_change_max_yaw_speed(6.11);
  usleep(50000);
  drk_ar_change_max_angle(0.4);
  usleep(50000);
  drk_ar_change_min_altitude(3000);//in mm
  usleep(50000);
  drk_ar_change_max_altitude(10000);//in mm
  usleep(50000);
  drk_ar_set_outdoor_flight();
  usleep(50000);
  drk_ar_set_outdoor_hull();
  usleep(50000);
}
#endif





/** Create the NAVigation and ACTuator threads **/
int act_sensor_init(void)
{
	block_all_signals() ;
	int ret = drk_actuator_init(); /* IAA */
	if ( ret < 0 )
		return ret;
	PRINTF_FL( "[ACTUATOR] Module - OK\n");

	ret = drk_sensor_data_init(); /* ISA */
	if ( ret < 0 )
		return ret ;
	PRINTF_FL( "[NAVADATA] Module - OK\n");

	
	//sleep while drone state is not filled.
	PRINTF_FL( "Waiting for battery status ...\n");
	#define NUM_OF_TRIES 10
	for (int i = 0; i < NUM_OF_TRIES; i++ ) 
	{
		if ( 0 <= drk_get_battery() )
		{
			drk_print_battery();
			print_navdata_state();
			return E_SUCCESS ;
		}
		print_navdata_state();
		microsleep(1e6); /* waiting for sensor data informing us that everything is ok */
		
	}
	#undef NUM_OF_TRIES
	return -1 ;
}





/**********************************************************************
              SIGNAL HANDLERS
***********************************************************************/


/** 
 * When you get a SIGINT, cut the motors => emergency mode on
 * CTRL+C 
 **/
void sigint_handler(int signum)
{
	//signal(SIGINT, SIG_DFL); /* next time run SIG_DFL */
	(void)signum;
	//printf("\n\n");
	//PRINTF_FL( "WARNING: SIGINT [CTRL+C] detected.\n");


	Exit_code = CLOSURE_INT ;
	sem_post( &Exit_semaphore ) ;
 

}

/**
 * When you get a SIGTSTP - CTRL + Z, land the drone, call emergency 
 * closureThread() will do this
 ***/ 
void sigtstp_handler(int signum)
{
	(void)signum;
	drk_exit();
}
	
/** 
 * this tells closureThread() to:
 * land, call emergency, 
 * then terminate app graciously
 * **/
void drk_exit(void)
{
	Exit_code = CLOSURE_STOP ; 
	sem_post( &Exit_semaphore ) ;
}


void drk_error(char *msg)
{
	PRINTF_FL_ERR( "%s: %s\n", msg, strerror(errno) ) ;
}


/*
void drk_exit(int status)
{
	Exit_code = CLOSURE_STOP ;
	while( 1 ) sleep( 1 ) ;
  //drk_send_at_command_comwdg(); // reset watchdog
 
}
*/

/*
void ignore_signal(int sig)
{
  //(void)(sig); // suppress warning
  //#ifdef sched_something
  //if (sched != 0) drk_sched_reserve_delete(act_rid);
  //if (sched != 0) drk_sched_reserve_delete(udp_rid);
  //return;
  //#endif
}
*/

//signal(SIGTSTP, SIG_DFL);

	
	
//send SIGUSR2 to land app, to make it terminate
//if ( 	kill(emergency_process, SIGUSR2) < 0 )
//{
	//PRINTF_FL( "Error %d : %s\n" , errno , strerror(errno) ) ;
//}
//else
//{
	//int status;
	//int ret_wait = waitpid( -1, &status, 0 ) ;  
	//if (ret_wait == -1) 
	//{
		//perror("waitpid");
		//exit(CLOSURE_FAILURE);
	//}
	//if (WIFCLOSUREED(status))
		//printf("exited, status=%d\n", WCLOSURESTATUS(status) );
		
	//PRINTF_FL( " process Land has exited \n");
//}

	
#ifdef SEG_FAULT
/** When you get a segfault, inform process Land to land the UAV **/
void sigsegv_handler(int sig)
{

	(void)(sig); // suppress warning
	signal(SIGSEGV, SIG_DFL);
	//drk_isa_close();

	PRINTF_FL( 
		"WARNING: Your code segfaulted. "
		"Attempting an emergency landing.\n");
	//drk_land ();
	drk_restore_termios() ;

	
	/* send emergency_process a signal */
	if ( kill( emergency_process , SIGUSR1 ) < 0 ) 
	{
		PRINTF_FL( "USR1 signal failed %d : %s\n" , 
			errno , strerror(errno) ) ;
	}
	else
	{
		//int status;
		//waitpid( emergency_process, &status, NULL ) ;  
	}

	exit(CLOSURE_FAILURE); /* finish  */
}
#endif


#ifdef FAILSAFE
/** like drk_init() but just with one functionality: send commands to the drone to land **/
int _drk_init_emergency(void)
{
	//drk_setup_termios();
	//#ifdef __arm__
	//init a thread that send ATD commands to the drone. BLOCKING.
	int ret = act_sensor_init() ;
	if ( -1 == ret )
	{
		printf(PREFIX_EDRK "Controller - Failed to init \n") ;
		return -1 ;
	}
	printf(PREFIX_EDRK "Controller - Initiated!\n") ;
	//#endif 
	return 0;
}
#endif



/* Get the Drone's IP address  */
//uint8_t drk_getMyIP()
//{
	//#ifdef SIMULATOR
	//	return global_my_id ;

	//#else
		//uint8_t last_ip_chunk = 0;

		//struct ifaddrs * ifAddrStruct = NULL;
		//struct ifaddrs * ifa = NULL;
		//void * tmpAddrPtr = NULL;
		//getifaddrs(&ifAddrStruct);
		////ifa = ifAddrStruct->ifa_next->ifa_next->ifa_next;
		//ifa = ifAddrStruct->ifa_next->ifa_next->ifa_next; /* get the third? NIC in the uav */
		//char * nic_name = ifa->ifa_name ;
		
		//tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
		//static char ip_address_str[INET_ADDRSTRLEN];
		//inet_ntop(AF_INET, tmpAddrPtr, ip_address_str, INET_ADDRSTRLEN); /* paste our ip-address into the ip_address_str,*/
		//freeifaddrs(ifAddrStruct);  
		//sscanf( ip_address_str , "%*d.%*d.%*d.%hhu", &last_ip_chunk );
		//PRINTF_FL( "My IP: %s (NIC %s)\n" , drone_ip_address , nic_name ) ;
		//return last_ip_chunk ;
	//#endif
// }
/* returns number of seconds between now and the time when */

