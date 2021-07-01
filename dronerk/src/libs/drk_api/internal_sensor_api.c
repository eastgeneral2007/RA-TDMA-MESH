/*******************************************************************************
                Access NAV DATA
 ******************************************************************************//*  
 *	internal_sensor_api.c: 
 *	aka ISA	
 */


#include <netinet/in.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
//#include <sys/socket.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h> // pthread.self


//#include "drk/ar_configuration.h"
#include "drk/internal_sensor_api.h"
#include "drk/drk.h" // drk_getMyIP and drk_error, drk_sem_create
#include "drk/internal_actuator_api.h"
#include "drk/internal_sensor_raw.h" /* tc_struct_heading */
#include "drk/utils.h" /* sem cleanup  */
#include "drk/navdata.h"


#ifdef SIMULATOR 
	#include "drk/gps.h"
#endif

#define SA		struct sockaddr

#ifdef SIMULATOR
	/* simulator stuff */
	#define SENSOR_LOCAL_PORT_BASE	(61000) /* */
#endif

#define NAV_DATA_PORT			(5554) //autopilot elf

#define MAXLINE					(3048) //2048

#ifndef SENSOR_LOADED_CONDITION
#define SENSOR_LOADED_CONDITION \
	if (Navdata_initiated == 0) return 
#endif

#define PREFIX "ISA"
#define INTERNAL_SENSOR /* show some internal data */





/************
 * Externs
 ***********/

//TODO: call a actuator function instead !
extern struct global_actuator_*	actuator_buf ; /* we need to send some config packets - drk_update_navdata_packets() */
extern uint8_t 					global_exit_all_threads ; /* drk.c */
extern struct global_navdata_*	navdata_buf ; /* struct with all data  (defined in ISA.h ) */



/****************
 * Globals
 ***************/
//FILE *navfile;
pthread_t 				nav_tid ; /* drk.c */
int 					debug2 = 0 ;
volatile int 			Navdata_initiated = 0 ; /* flag module initated or not */
int 					drone_timestamp ;
//sem_t 					time_sem ;
int 					drone_sockfd_nav = -1 ;
struct sockaddr_in		servaddr_nav ;

double 					Zero_altitude = 0; /* (sea)altitude at takeoff */


/**************************
 * static prototypes
 * ***********************/
static void *nav_thread(void* args) ;



/************************************************
 * 				Definitions start here 
 * ********************************************/
/*******************************************************************************
                                UTILITIES
*******************************************************************************/
/* Prints and shows a graph of the battery */
#ifdef VERBOSE
void drk_print_battery(void)
{

	/* Gets the battery level */
	const int batt_max = 100 ;
	int battery = drk_get_battery() ;
	if ( battery > batt_max || battery < 0 )
	{
		PRINTF_FL_ERR( "\tBattery level: error\n") ;
		return ;
	}
	
	int index;
	char level_graph[60];
	const int batt_graph=50; /* Battery level shown as 50 blocks */
	for(index = 0; index < battery * batt_graph / batt_max ; index++)
		level_graph[index]='=';
	for(; index < batt_graph; index++)
		level_graph[index]='_';

	level_graph[index] = '\0';
	PRINTF_FL( "Battery level: [%d%%] [%s]\n", battery, level_graph ) ;
}
#endif

/*******************************************************************************
                            DRONE SENSOR ACCESSORS
*******************************************************************************/
/* return battery level 0-100 */
int drk_get_battery(void)
{

	SENSOR_LOADED_CONDITION -1;

	/* Grab a copy of the sensor data */
	if (sem_wait(navdata_buf->semaphore) < 0 )
	{
		PRINTF_FL_ERR("err sem\n") ;
		return 0 ;
	}
	int battery=(navdata_buf->sensor).battery;
	sem_post(navdata_buf->semaphore);
	return battery;
}








/***************************
 * ALTITUDE
 * *****************************/
 
/* Thread-safe altitude accessor */
double drk_ultrasound_raw(void)
{
	sem_wait(navdata_buf->semaphore);
	double altitude = (double) (navdata_buf->sensor).altitude;
	sem_post(navdata_buf->semaphore);
	return altitude;
}

/* Ultrasound altitude reading compensated for tilt */
double drk_ultrasound_altitude(void)
{
	double uncomp = drk_ultrasound_raw();
	return uncomp * cos(drk_drone_pitch_get()*M_PI/180) * cos(drk_drone_roll_get()*M_PI/180);
	//return drk_ultrasound_raw();
}

double drk_abs_altitude(void)
{
	sem_wait(navdata_buf->semaphore);
	double altitude = (double) (navdata_buf->sensor).absolute_altitude;
	sem_post(navdata_buf->semaphore);   
	return altitude;  
}

double drk_rel_altitude(void)
{
	return (drk_abs_altitude()-Zero_altitude) ; 
}



/*******************************
 * HEADING
 * *************************************************/
 /* Current BEST heading calculation (CW). If you find better, edit this */
/* 0º - NORTH */
/* +90º - EAST */
/* +180º - SOUTH */
/* +270º - WEST */
float drk_heading(void)
{
	return drk_compass_heading();
	
	//drk_tc_compass_heading();
	//return drk_fusion_heading();
}





/******************
 * Speed 
 * **********/


vector_t drk_drone_speed_get(void)
{
	
	vector_t speed;
	SENSOR_LOADED_CONDITION speed;

	sem_wait(navdata_buf->semaphore);
	speed.x = (double) (navdata_buf->sensor).vx;
	speed.y = (double) (navdata_buf->sensor).vy;
	speed.z = (double) (navdata_buf->sensor).vz;
	sem_post(navdata_buf->semaphore);
	return speed;
}


/* 
 * saves the altitude (from the sea-floor)
 * when the drone calls this function. 
 * */
void drk_zero_altitude( uint16_t dft )
{
	SENSOR_LOADED_CONDITION ;

	if ( dft > 0 )
		Zero_altitude = dft;
	else
	{
		double sum_altitude = 0;
		#define NSAMPLES 10
		for (int sample_id=0;sample_id<NSAMPLES;sample_id++)
		{ 
			sem_wait(navdata_buf->semaphore);
			sum_altitude += (double)( (navdata_buf->sensor).absolute_altitude ) ;
			sem_post(navdata_buf->semaphore);   
			usleep(400000); //400ms
		}
		Zero_altitude = sum_altitude/NSAMPLES ; 
		#undef NSAMPLES
	}
	PRINTF_FL("zero altitude set to: %6.2fm from the sea\n", 
		Zero_altitude ); 
}
/* printout all sensor data available */
void dump_sensors(void)
{
	SENSOR_LOADED_CONDITION ;

	#ifdef INTERNAL_SENSOR
	//Data from all the sensors
	int		state=0;    
	int		battery=0;
	float   ultrasound_altitude=0;
	float   absolute_altitude=0;  
	float   pitch=0.0;      
	float   roll=0.0;  
	float   yaw=0.0;
	float   vx=0.0;
	float   vy=0.0;
	float   vz=0.0;
	float   mag_x=0.0;
	float   mag_y=0.0;
	float   mag_z=0.0;
	float   mag_heading=0.0;
	float   mag_heading_gyro=0.0;
	float   mag_heading_fusion=0.0;
	//Raw sensor data
	int     raw_acc_x=0;
	int     raw_acc_y=0;
	int     raw_acc_z=0;
	float   phys_acc_x=0.0;
	float   phys_acc_y=0.0;
	float   phys_acc_z=0.0;
	#endif // ifdef INTERNAL_SENSOR
	
	#ifdef EXTERNAL_SENSOR
	//External Sensor Data
	double latitude=0.0;
	double longitude=0.0;
	double altitude_gps=0.0;
	double UTC=0.0;
	int quality=0;
	int num_sats=0;
	int date=0;
	float hPrecise=0.0;
	double trueTrack=0.0;
	double groundSpeed=0.0;
	char extSensors[512];
	gps_t temp_gps;
	temp_gps = drk_gps_data();
	#endif




	#ifdef DEBUG_ENABLE
	PRINTF_FL("Collecting sensordata\n");    
	#endif

	//Reads data from all External Sensor Data
	//Start - external sensors
	#ifdef EXTERNAL_SENSOR
	sem_wait(serial_buf->semaphore);
	latitude = (serial_buf->gps_buf).latitude;
	longitude = (serial_buf->gps_buf).longitude;
	altitude_gps = (serial_buf->gps_buf).altitude;
	UTC = (serial_buf->gps_buf).UTC;
	quality = (serial_buf->gps_buf).quality;
	num_sats = (serial_buf->gps_buf).num_sats;
	date = (serial_buf->gps_buf).date;
	hPrecise = (serial_buf->gps_buf).hPrecise;        
	trueTrack = (serial_buf->gps_buf).trueTrack;
	groundSpeed = (serial_buf->gps_buf).groundSpeed;
	sem_post(serial_buf->semaphore);

	#ifdef DEBUG_ENABLE
	PRINTF_FL("External sensors Consolitating...\n");
	#endif

	sprintf(extSensors,"%ld;%lf;%d,%d;%d;%lf;%lf;%lf;%f;%lf;%lf;",
	updateNo,UTC,date,quality,num_sats,
	latitude,longitude,altitude_gps,hPrecise,
	trueTrack,groundSpeed);

	PRINTF_FL("%s",extSensors);

	#endif
	//End - external sensors

	//Reads data from all Internal Sensor Data
	//Start - internal sensors
#ifdef INTERNAL_SENSOR
	char intSensors[1012];//512
	sem_wait(navdata_buf->semaphore);

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Battery\n");
	#endif
	state=(navdata_buf->sensor).state;
	battery=(navdata_buf->sensor).battery;

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Altitude\n");
	#endif
	ultrasound_altitude=(double) (navdata_buf->sensor).altitude;
	absolute_altitude=(navdata_buf->sensor).absolute_altitude;  

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Pitch roll yaw\n");
	#endif
	pitch=(navdata_buf->sensor).pitch;      
	roll=(navdata_buf->sensor).roll;  
	yaw=(navdata_buf->sensor).yaw;

	//Compass Data
	#ifdef DEBUG_ENABLE
	PRINTF_FL("Velocity\n");
	#endif
	vx=(navdata_buf->sensor).vx;
	vy=(navdata_buf->sensor).vy;
	vz=(navdata_buf->sensor).vz;

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Compass\n");
	#endif
	mag_x=(navdata_buf->sensor).mag_x;
	mag_y=(navdata_buf->sensor).mag_y;
	mag_z=(navdata_buf->sensor).mag_z;

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Gyro\n");
	#endif
	mag_heading=(navdata_buf->sensor).mag_heading;
	mag_heading_gyro=(navdata_buf->sensor).mag_heading_gyro;
	mag_heading_fusion=(navdata_buf->sensor).mag_heading_fusion;

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Accelerometer\n");
	#endif
	raw_acc_x=(navdata_buf->sensor).raw_vals.raw_acc_x;
	raw_acc_y=(navdata_buf->sensor).raw_vals.raw_acc_y;
	raw_acc_z=(navdata_buf->sensor).raw_vals.raw_acc_z;
	phys_acc_x=(navdata_buf->sensor).raw_vals.phys_acc_x;
	phys_acc_y=(navdata_buf->sensor).raw_vals.phys_acc_y;
	phys_acc_z=(navdata_buf->sensor).raw_vals.phys_acc_z;

	sem_post(navdata_buf->semaphore);

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Internal sensors Consolitating...\n");
	#endif

	puts("");
	char state_str[20];
	snprintf(state_str, sizeof(state_str), "0x%08X", state);
	sprintf(intSensors,
			"battery:\n\t%d\nstate:\n"
			"\t%s\nalt:\n\t%f\n"
			"\t%f\nPRY:\n\t%f\n"
			"\t%f\n\t%f\nvel:\n\t%f\n\t%f\n\t%f\nmag:\n"
			"\t%f\n\t%f\n\t%f\t%f\nmag2:\n\t%f\n\t%f\n"
			"\t%f\nraw_acc:\n\t%d\n\t%d\n"
			"\t%d\nphys_acc:\n\t%f\n\t%f\n\t%f",
			battery, state_str,
			ultrasound_altitude, absolute_altitude,
			pitch, roll, yaw,
			vx, vy, vz,
			mag_x, mag_y, mag_z, drk_gyro_heading(),
			mag_heading, mag_heading_gyro, mag_heading_fusion,
			raw_acc_x, raw_acc_y, raw_acc_z,
			phys_acc_x, phys_acc_y, phys_acc_z);
	PRINTF_FL("%s\n", intSensors ) ;

	#ifdef DEBUG_ENABLE
	PRINTF_FL("Completed...\n");
	#endif

	//End - internal sensors

	PRINTF_FL("\n");

#endif /* end of #ifdef INTERNAL_SENSORS */
}



/*******************************************************************************
                            FOR LATER USE
*******************************************************************************/

int drk_wait(void)
{
    return 1;
}


int drk_ctrl_config(void )
{
    return 1;
}


int drk_sensor_data_wait(void)
{
    return 1;
}



/******************************************************************************* 
                        NAVDATA MODULE - HANDLING FUNCTIONS
*******************************************************************************/
/*
 * drk_sensor_data_init function, building two connection, 
 * one for sending at commands, the other for nav_data
 *
 *      output:  0 for failure, 1 for success.
 */
int drk_sensor_data_init(void)
{
	if( ( navdata_buf = calloc(1 , sizeof(struct global_navdata_))) == NULL) //ISA.h
	{
		PRINTF_FL_ERR( "Error allocating memory navdata_buf  (%s)", 
			strerror( errno ) ) ;
		return -1;
	}
	//PRINTF_FL( "[NAVDATA] navdata_buf - OK\n")  ;

	// Initialize semaphore
	(navdata_buf->sensor).battery	= -1 ;

	//(navdata_buf->sensor).state = (unsigned int)ARDRONE_EMERGENCY_MASK ;
	/* Initialize 3 semaphores */
	int ret = 0 ;
	ret += drk_sem_create( "navdata_sem" , &(navdata_buf->semaphore) ) ;
	



	//navfile = fopen("./timestamps/nav.txt", "w");


	/*
	 * 
	if ( sem_init( &(navdata_buf->semaphore),0,1) != 0)
	{
		PRINTF_FL(  "navdata_buf sem - init failed!\n");
		PRINTF_FL(  "the error is : %d\n", errno);
	}
	*/


	/* build connection for sending and receiving NAV data (sensor) */   	
	drone_sockfd_nav = socket( PF_INET, SOCK_DGRAM, 0 ); 
	if (drone_sockfd_nav == -1)
	{
		PRINTF_FL_ERR( "[NAVDATA] socket - error (%s) " , strerror( errno) );
		return -1 ;
	}
	//PRINTF_FL(  "socket - ok.\n");
	/* populate struct sockaddr  */ 
	memset( &servaddr_nav , 0 , sizeof( servaddr_nav ) );
	servaddr_nav.sin_family = AF_INET; 
	servaddr_nav.sin_port = htons( NAV_DATA_PORT ); // send navdata to this port, to start receiving data later on
	servaddr_nav.sin_addr.s_addr = htonl( INADDR_LOOPBACK );
	/* socket for navdata */
		
	
	//PRINTF_FL( "[NAVDATA] init command will be sent to [%s:%d]\n",
			//inet_ntoa( servaddr_nav.sin_addr ) ,
			//ntohs( servaddr_nav.sin_port ) ) ;
	//#endif 
	/* Bind()  -> reuse port in case it's in use */
	//int yes= 1 ;
	//if (setsockopt(drone_sockfd_nav, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
	//	PRINTF_FL(  "fail to reuse\n");

	/* under simulation, we need to listen to ports  */
	#if SIMULATOR
	//PRINTF_FL(  "[SIM STUFF]\n");
	struct sockaddr_in	si_me ; // for bind socket
	prepare_sockaddr( getMyIP(), SENSOR_LOCAL_PORT_BASE, &si_me ); /* not needed */
	
	if ( bind(drone_sockfd_nav , (const SA *)&si_me, sizeof(si_me)) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR("[NAVDATA] Bind - Failed [local port %s:%d] (%s) " ,
			inet_ntoa( si_me.sin_addr ) ,
			ntohs( si_me.sin_port ) ,
			strerror( errno) );
		return -1 ;
	}
	PRINTF_FL( "[NAVDATA] Bind - Ok [Local port %s:%d]\n",
			inet_ntoa( si_me.sin_addr ) ,
			ntohs( si_me.sin_port ) ) ;
	
	#endif //#ifdef SIMULATOR

	/* destination of this socket will always be the same, so connect */
	//if ( connect( drone_sockfd_nav, (const SA *)&servaddr_nav, sizeof(servaddr_nav)) < 0 ) 
	//	PRINTF_FL(  "connect - failed ");
	//PRINTF_FL(  "connect - ok.\n");


	/* String to initiate the drone to send navdata */
	/* one command to initiate NAVdata packets */
	#ifndef SIMULATOR
	char initiateNavdata[]={0x01, 0x00, 0x00, 0x00}; /* message payload */
	/* send  */
	if( sendto( 
		drone_sockfd_nav, 
		initiateNavdata, 
		strlen(initiateNavdata),
		0,
		(SA *)&servaddr_nav, 
		sizeof(servaddr_nav) 
			) < 0 )
	{
		PRINTF_FL_ERR( "[NAVDATA] commands (%s:%d) - error %s\n", 
			inet_ntoa( servaddr_nav.sin_addr ) ,
			ntohs( servaddr_nav.sin_port ) ,
			strerror( errno ) ) ; 
		return -1;
	}
			
	//PRINTF_FL( "[NAVDATA] Commands sent - ok (%s:%d)!\n", 
		//inet_ntoa( servaddr_nav.sin_addr ) ,
		//ntohs( servaddr_nav.sin_port ) ) ;
	usleep(10000);
	#endif
	
	
	
	/* navdata thread to read incoming data  */
	#if 1
	pthread_attr_t nav_attr;
	pthread_attr_init(&nav_attr);
	size_t nav_desired_stack_size = 40000;
	pthread_attr_setstacksize(&nav_attr, nav_desired_stack_size);
	if (pthread_create(&nav_tid, &nav_attr, &nav_thread, NULL) != 0) // 
	{
		PRINTF_FL_ERR( "[NAVDATA] spawning thread - Error %s\n", strerror(errno));
		return -1 ;
	}
	#endif
	//PRINTF_FL( "[NAVDATA] spawning thread - OK\n");


	//drk_update_navdata_packets(); //TODO: should we run this or not?
	Navdata_initiated = 1 ;
	//PRINTF_FL_WARN("Navdata_initiated set\n");
	return 1;   
}

error_t drk_sensor_data_close(void)
{
	SENSOR_LOADED_CONDITION E_NO_INIT ;


	pthread_kill( nav_tid , SIGTSTP ) ;
	pthread_join( nav_tid , NULL) ;
	
	
	if ( drone_sockfd_nav != -1 )
	{
		close(drone_sockfd_nav) ;
		//PRINTF_FL( "socket closed\n");
	}
	
	//PRINTF_FL( "nav tid terminated\n");

	Navdata_initiated = 0 ;
	//PRINTF_FL_WARN("Navdata_initiated UNset\n");
	#if 0
	if ( navdata_buf != NULL )
	{
		//PRINTF_FL_WARN("cleaning up navdata_sem\n");
		if ( drk_sem_cleanup( "navdata_sem" , navdata_buf->semaphore ) < 0 )
		{
			PRINTF_FL_WARN("cleaning up navdata_sem - FAILED \n");
			return E_OTHER ;
		}
		//PRINTF_FL_WARN("cleaning up navdata_sem - SUCCESS \n");
	}
	#endif
	return E_SUCCESS ;
}

/* Read the incoming navdata from the drone */
void *nav_thread(void* args)
{

	//#ifdef __arm__
	//PRINTF_FL(  "NAV thread's head \n" );
	(void)(args); // suppress unused parameter warning

	drone_sensor_t 	temp_sensor;
	struct vision_tags_ 	temp_vision;

	//signal(SIGTSTP, ignore_signal);
	//signal(SIGINT, ignore_signal);

	//struct sched_param p;
	//p.sched_priority=6;
	//sched_setscheduler(getpid(),SCHED_FIFO,&p);

	temp_sensor.mag_filter_index = 0 ;

	while ( !global_exit_all_threads )
	{
		
		//receive and block  
		//PRINTF_FL(  "line %d\n" , __LINE__ );
		int n = drk_sensor_data_update( &temp_sensor, &temp_vision ) ; //ISA.c - rcvdfrom is blocking
		
		if (n == -1 )
			continue; /* Throw out bad input (one or more values NAN) */
		if (n == -2 )
			goto stop ; /* input pointer is NULL, or time to close */
			
		/* Keep a list of the last 
		 * <MAG_FILTER_SAMPLES> heading measurements
		 * this thread is the only one writing these two fields. 
		 * no sem needed */
		//temp_sensor.mag_heading_samples[temp_sensor.mag_filter_index] =
			//drk_tc_compass_heading();
		//temp_sensor.mag_filter_index = 
			//(temp_sensor.mag_filter_index + 1) % MAG_FILTER_SAMPLES; /* cyclic queue */
		//PRINTF_FL( "Testing\n" );
		if (global_exit_all_threads) goto stop;

		sem_wait( navdata_buf->semaphore );
		//temp_sensor.compass_offset 		= (navdata_buf->sensor).compass_offset ;
		//temp_sensor.fusion_offset 		= (navdata_buf->sensor).fusion_offset ;
		//temp_sensor.gyro_offset 		= (navdata_buf->sensor).gyro_offset ;
		memcpy( &(navdata_buf->sensor) , &temp_sensor, sizeof(temp_sensor) ) ;
		sem_post( navdata_buf->semaphore ) ; 


	//memcpy  (&(navdata_buf->vision), &temp_vision, sizeof(temp_vision)) ;
	}
	//#endif
	//printf("Exiting thread!\n") ;

	PRINTF_FL(  
		"[NAVDATA] Thread terminated normally\t[%08x] *******\n", 
		(unsigned int)pthread_self() );  
	return NULL ;
	
stop:
	PRINTF_FL_ERR( 
		"[NAVDATA] Thread terminated by error\t\t[%08x] *******\n", 
		(unsigned int)pthread_self() );  
	return NULL ;
}




/*******************************************************************************
                            OBJECT TRACKER SETUP
*******************************************************************************/
void drk_ar_object_tracker_setup()
{
    /*char command_buffer[200]="";

    int t_detect, h_detect, v_detect, colors;
    
    t_detect = CAD_TYPE_MULTIPLE_DETECTION_MODE;
    h_detect = TAG_TYPE_NONE; //TAG_TYPE_SHELL_TAG;
    v_detect = TAG_TYPE_ROUNDEL;
    colors = 2; // g = 1 , y = 2, b = 3;  
    
    int i;
    */

    /* Get vision data */
    /*for (i = 0; i < 10; i++)
    {   
        sem_wait(actuator_buf->time_sem);
        sprintf(command_buffer, "AT*CONFIG=%d,\"general:navdata_options\",\"65537\"\r",
                                actuator_buf->global_timestamp);
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);
        usleep(1000);
        */
        /* Set up multiple detection */ 
        /*sem_wait(actuator_buf->time_sem);
        sprintf(command_buffer, "AT*CONFIG=%d,\"detect:detect_type\",\"%d\"\r",
                                actuator_buf->global_timestamp, t_detect);
    
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);
        usleep(1000);
        */
        /* Set up horizontal detection */
        /*sem_wait(actuator_buf->time_sem);
        sprintf(command_buffer, "AT*CONFIG=%d,\"detect:detections_select_h\",\"%d\"\r",
                                actuator_buf->global_timestamp, h_detect);
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);
        usleep(1000);
        */
        /* Set up vertical detection */
        /*sem_wait(actuator_buf->time_sem);
        sprintf(command_buffer, "AT*CONFIG=%d,\"detect:detections_select_v\",\"%d\"\r",
                                actuator_buf->global_timestamp, v_detect);
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);
        usleep(1000);
        */
        /* No outdoor hull */
        /*sem_wait(actuator_buf->time_sem); 
        sprintf(command_buffer, "AT*CONFIG=%d,\"detect:enemy_without_shell\",\"0\"\r",
                            actuator_buf->global_timestamp);
    
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);       
        usleep(1000);
        */  
        /* Set up enemy colors */
        /*sem_wait(actuator_buf->time_sem);
        sprintf(command_buffer, "AT*CONFIG=%d,\"detect:enemy_colors\",\"%d\"\r",
                            actuator_buf->global_timestamp, colors);
        sendto(drone_sockfd_at,command_buffer,strlen(command_buffer),0,(SA *)&servaddr_at, sizeof(servaddr_at));
        actuator_buf->global_timestamp++;
        sem_post(actuator_buf->time_sem);
        usleep(1000);
    }*/
}

