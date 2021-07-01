/**
 * autonomous.c
 * aka AC (autonomous control)
 */

#include <pthread.h> // spawn threads, mutex 
#include <stdio.h>
#include <math.h> //cos and sin, etc 
#include <unistd.h>
#include <string.h> //memset
#include <sys/timerfd.h> /*makeperiodic, wait_period */
#include <errno.h> // strerror

/* drk stuff */
#include "drk/autonomous.h" /* public stuff */
#include "drk/flight_control_api.h" /* drk_translate() */
#include "drk/gps.h" /* we use gps */
#include "drk/internal_sensor_api.h" /* get ultra altitude */
#include "drk/navdata.h" // get heading 
#include "drk/keyboard.h" /* getLockDown() */
#include "drk/drk.h" /* drk_getMyIP() */
#ifdef SIMULATOR
#include "drk/sim_clock.h"  /* getClock , etc ... */
#endif

#include "drk/utils.h"

//#define MIN_NUM_SATS_TO_FLY 7
#define PREFIX "AC  "


#ifndef AUTO_LOADED_CONDITION
#define AUTO_LOADED_CONDITION \
	if (Auto_initiated == 0) return 
#endif


#define CONTROL_PERIOD			100000 /* us - microseconds */ /* 100ms  */
#define MIN_ALTITUDE_SAFETY		3 /* in meters */
#define DESIRED_STACK			400000

#define LOGFILE_FORMAT_STRING	\
	"%.2f x "\
	"%2.3f %2.3f %2.3f x "\
	"%"PRIu8" %.6f %.6f x " \
	"%"PRIu8" %.6f %.6f x "\
	"%"PRIu8" %.6f %.6f x "\
	"%.6f %.6f %.1f x "\
	"%.1f %.1f %.1f x "\
	"%.3f %.3f %.3f x "\
	"\n"





// default waypoint
#define DFL_LAT 41.176752 // feup canteen
#define DFL_LON -8.596227 // feup canteen


#ifndef TRUE
	#define TRUE 1
#endif 
#ifndef FALSE
	#define FALSE 0
#endif 








/*****************************
 * * protoypes 
 * ********************/
static void* autonomous_move_thread();
error_t doVerticalControl( gps_waypoint_t wypt , actuation_t* act ) ;
error_t doHorizontalControl( controlpid_t* const error_value , 
	gps_waypoint_t wypt , actuation_t* const act );

//**
// * Distance in degrees from the spin 
// * POSITIVE VALUE: You are to the left, turn right
// * NEGATIVE VALUE: You are to the right, turn left
// */
//static double _drk_get_degree_error(double spin, double target);

// TODO: use macro



/*******************************************************************************
 * Evil Globals
 ******************************************************************************/
static volatile int 				Auto_initiated = 0 ; /* is this module initiated ? */
static volatile int 				Auto_exit = 0 ; /* flag closure */

/* Variables to control autonomous state, and mutex: */
static pthread_t 					Auto_tid ; /* there's a thread on this module that keeps running motion cmnds */
static volatile enum flight_status	Autonomous_state ; // W MUTEX PROTECTED
static pthread_mutex_t 				mutex_autonomous_state = PTHREAD_MUTEX_INITIALIZER;

/* Variable to control if UAV is on the current waypoint. */
static volatile uint8_t 			global_target_reached 	= T_NOT_REACHED; /* used in drk_autonomous_target_reached() */
static pthread_mutex_t 				mutex_reached 			= PTHREAD_MUTEX_INITIALIZER; /* mutex to determine if we've reachd target */

/* PID constants */
static volatile double	Kp_h = 0.0250 , 
						Ki_h = 0.0000 , 
						Kd_h = 0.0 , 
						Kp_v = 0.6 ; 

/* Variables to control current waypoint, and mutex: */
static pthread_mutex_t 			mutex_waypoint = PTHREAD_MUTEX_INITIALIZER; //mutex to set a new waypoint
static volatile uint8_t 		global_new_waypoint = 0 ; // W MUTEX PROTECTED . used to flag the entry of a new waypoint
static volatile gps_waypoint_t 	global_waypoint ; // RW MUTEX PROTECTED . the current waypoint the controller uses
static const gps_waypoint_t 	zero_waypt = {{0,0,0,0,0},0,0} ;

static FILE* 					sensor_log = NULL ;




/*******************************************************************************
	AUTONOMOUS FLIGHT FUNCTIONS
******************************************************************************/
void drk_autonomous_set_PID( double _kp_h , double _ki_h , 
	double _kd_h , double _kp_v )
{
	AUTO_LOADED_CONDITION ;
	// WRITE ONLY
	
	// [h]orizontal control:
	Kp_h = _kp_h;
	Ki_h = _ki_h;  
	Kd_h = _kd_h;

	// [v]ertical control:
	Kp_v = _kp_v;  

	//print changes:
	PRINTF_FL_WARN( 
		"PID set to kP_h=%.5f kI_h=%.5f kD_h=%.5f kp_v=%.5f\n",
		_kp_h, _ki_h , _kd_h, _kp_v );

}

void drk_autonomous_pause(void)  
{
	AUTO_LOADED_CONDITION ;
	pthread_mutex_lock( &mutex_autonomous_state );
	Autonomous_state = PAUSED ;
	pthread_mutex_unlock( &mutex_autonomous_state );
	PRINTF_FL( "State: <Paused>\n");
	drk_hover( 0 ) ; // removes all previous translate commands
}


void drk_autonomous_resume(void) // user may calls this
{
	AUTO_LOADED_CONDITION ;
	
	pthread_mutex_lock( &mutex_autonomous_state );
	Autonomous_state = FLYING ;
	pthread_mutex_unlock(&mutex_autonomous_state);


	pthread_mutex_lock(&mutex_waypoint);
	PRINTF_FL( 
		"State: <Flying> to [Lat%3.6lfº,Lon%3.6lfº,Alt%.1lfm]\n", 
		global_waypoint.llh.latitude, 
		global_waypoint.llh.longitude, 
		global_waypoint.llh.altitude );
	pthread_mutex_unlock(&mutex_waypoint);
}

// Returns the state of autonomous flight:
// 1 if paused
// 0 if flying to cur target
enum flight_status drk_autonomous_get_state(void){
	
	AUTO_LOADED_CONDITION UNLOADED;
	
	pthread_mutex_lock(&mutex_autonomous_state) ;
	enum flight_status temp = Autonomous_state ;
	pthread_mutex_unlock(&mutex_autonomous_state) ;
	return temp; 
}



// initiates autonmous controller
// returns -1 if thread can't be spawned
error_t drk_autonomous_init( void )
{

	/* check if GPs module was loaded ..*/
	// TODO
	
	

	pthread_mutex_lock(&mutex_autonomous_state);
	Autonomous_state = PAUSED;
	pthread_mutex_unlock(&mutex_autonomous_state);

	pthread_mutex_lock(&mutex_waypoint);
	global_waypoint.llh.latitude 		= DFL_LAT ;
	global_waypoint.llh.longitude 		= DFL_LON ;
	global_waypoint.llh.altitude 		= 15 ;
	global_waypoint.max_output 			= 1; /*magnitude, or, vector-norm horizontal plane*/
	global_waypoint.distance_tolerance 	= 2 ;
	global_new_waypoint 				= TRUE ; /* let know the AC there's a new wpt */
	pthread_mutex_unlock(&mutex_waypoint);
	
	
	llh_t bsllh;
	
	bsllh.latitude = DFL_LAT;
	bsllh.longitude = DFL_LON ;
	bsllh.altitude = 0 ;
	drk_piksi_set_bs(bsllh);
	//drk_autonomous_set_PID(  0.0250 , 0.0000 ,  00 , 0) ;

	/* Log-file vars */
	//double vtg_heading , vtg_speed ; // not used for now
	//uint8_t my_id = drk_getMyIP() ; 




	#ifdef LOGDATA
	{
		char prefix[15];
		snprintf( prefix, sizeof(prefix)-1,
			"AC_#%02d", getMyIP() ) ;
		error_t status = open_log_file( prefix , "" , &sensor_log );
		if ( 0 > status )
		{
			PRINTF_FL_ERR( "failed to create the file\n");
			return E_FILE_CREATION ;
		}
	}


	/* log file header - first line: */
	fprintf( sensor_log , 
		"time GPSalt Ualt qlty nsats rlat rlon slat slon "
		"tlat tlon dist head absBear relBear pitch roll up\n" );
	#endif
	
	pthread_attr_t attr;
	pthread_attr_init( &attr ) ;
	size_t desiredstacksize = DESIRED_STACK ;
	if (pthread_attr_setstacksize( &attr,desiredstacksize ) != 0)
	{
		PRINTF_FL_ERR( 
			"Stacksize was not defined properly - %dB (%s)\n", 
			DESIRED_STACK ,
			strerror(errno) );
		return E_PTHREAD;
	}
	// create movement thread
	if (pthread_create(&Auto_tid, &attr, autonomous_move_thread, NULL) != 0) 
	{
		PRINTF_FL_ERR( "[AUTO] thread - Error spawning (%s)\n",
			strerror(errno) );
		return E_PTHREAD;
	}
	pthread_attr_destroy( &attr ) ;

	Auto_initiated = 1 ; /* Flag module is initiated */
	return E_SUCCESS ;
}

#if VERBOSE
void print_target_info( llh_t llh_wypt )
{
	PRINTF_FL( 
		"New wypnt: [Lat%.6lfº, Lon%.6lfº, h=%.1fm] :: "
		"Distance = %.1fm\n",
		llh_wypt.latitude, 
		llh_wypt.longitude, 
		llh_wypt.altitude ,
		drk_gps_mydistance_to(llh_wypt)
		);

	/* convert to X-0-Y ref. make 0ºNorth == 90ºXoY , and positive direction is CCW.  */
	double absolute_heading = wrapTo360( 90.0 - drk_heading() ); 


	/* (beta) Get direction of target also known as BEARING @ earth referential  */
	double absolute_bearing = drk_gps_true_bearing( llh_wypt ) ; 


	/* rel_bearing = beta-alpha ;  delta of the two angles ~> is the direction to move, regarding the uav-nose */
	double relative_bearing = wrapTo360( absolute_bearing - absolute_heading ) ; 
	
	/* gps location */
	llh_t gps_sample = drk_gps_data() ;
	
	PRINTF_FL( 
		"MyPos: [Lat%.6fº,Lon%.6fº,uAlt%.1fm]\n",
		gps_sample.latitude , 
		gps_sample.longitude , 
		drk_ultrasound_altitude() );
	PRINTF_FL(
		"Bearing %.1fº, Heading: %.1fº,  Rel.bearing: [%.1fº]\n" , 
		absolute_bearing ,
		absolute_heading , 
		relative_bearing ) ;		
	
	/* ex:
	 *  ^
	 *  |  ô    x   
	 *  |
	 *  |____> 
	 * 
	 *  Drone is pointing NORTH (90º) . (Abs Heading)
	 *  Target it to the East (0º) . (Abs Bearing)
	 *  Drone should move towards its <RIGHT> . ( Rel Bearing = 0-90 = -90º )
	 * */
}
#endif

error_t doVerticalControl( gps_waypoint_t wypt , actuation_t* act ) 
{
	
	/* Control is only Proportional - 
	 * no error_value status to maintain */
	
	/* get altitude samples - pressure, ultra and gps: */
	
	//pressure_altitude_sample = drk_lpf_abs_altitude(); /* this works even without taking off ;) has a LP Filter embedded */
	double ultra_altitude_sample = drk_ultrasound_altitude(); /* this works only after taking off. is relative to the groundplane. */
	//double gps_altitude_sample = gps_get_altitude();
	double best_altitude_sample ;
	
	
	
	
	/* Get the best Altitude value */
	//if ( gps_sample.quality == 2 ) /* RTK signal OK */
	//{
	//	/* gps_altitude might be ok, but ultra says it's touching the floor */
	//	best_altitude_sample = MIN( gps_altitude_sample , ultra_altitude_sample) ;
	//}
	//else
	//{
		best_altitude_sample = ultra_altitude_sample  ;
		//PRINTF_FL_WARN("current altitude %.1fm\n", best_altitude_sample);
	//}
	//printf("best altitude: %lfm- target %lf\n" , best_altitude_sample, local_waypoint.taltitude ) ;
	
	/* Altitude Control */
	double vertical_error = wypt.llh.altitude - best_altitude_sample ; 
	act->vertical = Kp_v * vertical_error ;
	//PRINTF_FL("error %f ,outvert %f\n" , vertical_error , output.vertical) ;
	act->vertical = BOUND( act->vertical , 0.98) ;


	return E_SUCCESS ;
}


error_t doHorizontalControl( controlpid_t* const error_value , 
	gps_waypoint_t wypt , actuation_t* const act )
{
		if ( ( !error_value ) || (!act) )
			return E_OTHER ;
			
		/* how fast the controller runs the cycle (in Hz) */
		const double refresh_rate = 4 ; 

		
		/* Error value - [P]roportional */
		error_value->p = drk_gps_mydistance_to( wypt.llh ); 
		
		
		/* Error value - [D]erivative 
		 * Hz*m = m/s . 
		 * calculates instantaneous speed 
		 * based on diff of distances over time
		 */
		error_value->d = 
			( error_value->p - error_value->last_p ) * 
			refresh_rate ; /* = divide by time */  
			
		/* Eror value - [I]ntegrative */	
		error_value->i += error_value->p /
			refresh_rate ; /* = multiply by time */  
		
		
		/* [PID] Get Magnitude */
		act->horizontal_magnitude = 
			Kp_h * error_value->p + 
			Ki_h * error_value->i + 
			Kd_h * error_value->d ;
			
		/* bound it to max allowed */					
		act->horizontal_magnitude = BOUND( 
			act->horizontal_magnitude , 
			wypt.max_output  )  ;
		
		/* 
		 * when the UAV moves too fast, it loses altitude =>
		 * if we need to go UP full throtle, decrease horizontal output
		*/
		#if 0
		if ( act->vertical >= 0.98  || act->vertical <= -0.98 )
		{
			//PRINTF_FL_WARN( "Compensating altitude h=%.1fm\n", drk_ultrasound_altitude() );
			act->horizontal_magnitude *= 0.7 ;
		}
		#endif
		
		
		/******* 
		 * [PID] Direction to move <relative_bearing>: 
		 * (direction now)
		 *****/
		 	 
		/* 
		 * Get heading (compass) :
		 * convert to X-0-Y ref. 
		 * make 0ºNorth == 90ºXoY , and positive direction is CCW.  
		 */
		//double absolute_heading = wrapTo360( 90.0 - drk_heading() ); 
		double absolute_heading = wrapTo360( 90-drk_heading() ); 

		/* 
		 * Direction drone must move, relative to mag north 
		 * Get direction of target also known as BEARING, @ earth referential 
		 */
		double absolute_bearing = drk_gps_true_bearing( wypt.llh ) ;
		
		
		 /* ie, direction of target, as seen by the nose of the drone 
		  * delta of the two angles ~> 
		  * is the direction to move, regarding the uav-nose 
		  */
		double relative_bearing = wrapTo360( absolute_bearing - absolute_heading ) ; 
		
		
		/********
		 * GET VECTOR COMPONENTS to ouput:
		 * *********
		 * Translate this Magnitude and Direciton into PITCH and ROLL:
		 * pitch: -1.0* because FRONT (nose direction) needs negative component.
		 * roll: -1.0* because RIGHT needs positive component.
		 * ex: to move FWD we need 0º relative_bearing 
		 *****/
		act->horizontal_tangential	= -1.0 	* act->horizontal_magnitude * 
			cos( degrees_to_radians( relative_bearing ) ) ; 
		act->horizontal_normal		= -1.0 	* act->horizontal_magnitude * 
			sin( degrees_to_radians( relative_bearing ) ) ; 
		
			
		#if 0
		PRINTF_FL_WARN(
			"Magnitude:%.1f = Tg:%.2f + Nrm:%.2f | "
			"Angle: %.1fº = %.1frad\n",
			act->horizontal_magnitude, 
			act->horizontal_tangential ,
			act->horizontal_normal ,
			relative_bearing ,
			degrees_to_radians( relative_bearing ) );
		#endif
		
		/* 
		 * [PID] - Update error_value for next iteration: 
		 */
		error_value->last_p = error_value->p ;
		
		
		return E_SUCCESS ;
		
}
//if ( getStateMagnetometer() )
	//	printf("\n\nyou nned to Calibrate Magnemoter !!\n\n");
	
	//absolute_heading = 90-drk_heading(); /* in ISR.c */
	//char str[3] ;
	//if ( heading < 10 && heading > 350 ) 
		//sprintf( str, "N") ;
	//else 
		//sprintf( str, "hum") ;
	/* convert to X-0-Y ref: North is 0º - on YY-axis :: Positive direction is CCW */

/**
 * CONTINUOUS THREAD that Goes to target: using GPS and heading from =GYRO= 
 * Reaches the point and stays there.
 * GYRO is used, compass used to calibrate. 
 *******/
static void* autonomous_move_thread( )
{

	

	// Used for spin control loop (in degrees) 
	// TODO: change the whole thing to radians, cause it is the SI unit

	double distance 		= -1 ;
	
	
	// used for altitude control loop (SI units)
	//double vertical_error = 0 ; 

	// used for linear movement control - PID loop (in SI units)
	controlpid_t error_value = {0,100000,0,0} ;

	//// outputs of the control loop.
	//actuation_t output = {0,0,0,0};
	

	// save here local copy of global_waypoint
	gps_waypoint_t local_waypoint = zero_waypt ;
	
	uint8_t local_target_reached = T_REACHED ;



	/* sensor readings for altitude and gps */
	//double ultra_altitude_sample = 0 , gps_altitude_sample= 0 , pressure_altitude_sample = 0 , best_altitude_sample = 0 ;




	uint8_t printflag = 0 ;

	//time_t seconds_past_epoch ;

	while (!Auto_exit) 
	{

		/* reset outputs of the control loop. */
		actuation_t output = {0,0,0,0};
		

	
		/* 
		 * if there is a new waypoint,
		 * load it to the controller 
		 * */
		if ( global_new_waypoint == TRUE ) /* reading has no lock */
		{
			/* Grab a copy from the global structure */
			pthread_mutex_lock( &mutex_waypoint ); 
			local_waypoint = global_waypoint ;
			/* new waypoint was read and loaded, flip flag */
			global_new_waypoint = FALSE ; /* only write inside lock */
			pthread_mutex_unlock( &mutex_waypoint );
			
			
			#if VERBOSE
			print_target_info( local_waypoint.llh );
			#endif
			
			/* new wpy : set Target not reached. */
			pthread_mutex_lock(&mutex_reached) ;
			global_target_reached = T_NOT_REACHED ;
			pthread_mutex_unlock(&mutex_reached) ;
			local_target_reached = T_NOT_REACHED ;
		}
		
		

		
		/* do vertical control (altitude ) */
		#if 1
		error_t ret = doVerticalControl( local_waypoint , &output );
		if ( ret < 0 )
			PRINTF_FL_ERR("Vert Control - failed \n" );
		#endif


		//printf("\n\nHeading (%.0lfºN) , ABS-bearing %.0lfº, REL-bearing %.0ºlf \n\n", wrapTo360(drk_heading()) , absolute_bearing, relative_bearing );
				
		if ( !drk_gps_myfix()  )
		{
			/* no gps signal!
			 * the drone should STOP if the GPS is gone ! */
			if ( Autonomous_state == FLYING )
			{
				drk_autonomous_pause() ;
				PRINTF_FL( "**** GPS OUTAGE ****\n"); /* under 3 sats */
			}
			//drk_gps_print();
			goto log_data ; /* jump to log data */
		}
		
		
		/* 
		 * there is gps , 
		 * so lets do some autonomous control 
		 * */
		/* Get GPS lastest sample */
		//llh_t gps_sample = drk_gps_data() ;
		
		distance = drk_gps_mydistance_to( local_waypoint.llh ); 
				
		if ( distance < 0 )
			goto log_data ;
			
		//PRINTF_FL( "My distance to target: %3.1lfm\n", distance ); 
		
		/* WE REACHED THE TARGET - set flags */
		if  ( distance < local_waypoint.distance_tolerance ) 
		{
			
			/* REACHED! Hold position horizontally. */
			if ( local_target_reached == T_NOT_REACHED )
			{
				PRINTF_FL( "Waypoint Reached! **\n") ;
				pthread_mutex_lock( &mutex_reached ); 
				global_target_reached = T_REACHED ;     
				pthread_mutex_unlock( &mutex_reached ) ;
				local_target_reached = T_REACHED ;
				error_value.i = 0 ; /* reset PID */
			}
		}
		
		/* PAUSE state ? there's nothing to Control */ 
		if ( Autonomous_state != FLYING ) 
			goto log_data ; 
			
			
		/* 
		 * we HAve gps && we need to Fly (Flying state) to hold position :
		 * PID computation here!
		 * */
		error_value.p = distance ;
		/* 				alias ^ */
		error_t ret1 = doHorizontalControl( 
			&error_value ,  
			local_waypoint , &output 
			) ;
		if ( ret1 < 0 )
		{
			PRINTF_FL_ERR("Horizontal control - Failed!\n");
		}
	
	


	 

		/* AC-LOG@ Write log line with Sensor readings, and Outputs give  */
		// with or without gps signal
		//vtg_speed = gps_sample.groundSpeed / 3.6 ; /* in SI (km/h -> m/s) */
		//vtg_heading = gps_sample.trueTrack ; /* in degrees //TODO: use SI rads */
		
		
		log_data:;
		/* log a line
		 * time gpsalt ualt qlty sat  rlat  rlon  slat  slon  lat   lon   tlat  tlon dist head ber ber_rel 3-o-u-t-p-u-t--s  */
		#ifdef LOGDATA 
		char buf[1204];
		llh_t gps_spp = drk_piksi_get_spp(); //libsbp piksi
		llh_t gps_best = drk_piksi_get_best(); //libsbp piksi
		ned_t gps_ned = drk_piksi_get_ned(); //libsbp piksi
		
		double absolute_heading = wrapTo360( 90.0 - drk_heading() ); 

		double absolute_bearing = drk_gps_true_bearing( local_waypoint.llh ) ;
		double relative_bearing = wrapTo360( absolute_bearing - absolute_heading ) ; 


		sprintf(buf, LOGFILE_FORMAT_STRING , 
			getEpoch(),
			//gps_ned.down, ultra_altitude_sample, pressure_altitude_sample ,
			gps_ned.down , drk_ultrasound_altitude() , 0.0,
			gps_ned.num_sats, gps_ned.north, gps_ned.east, 
			gps_spp.num_sats, gps_spp.latitude, gps_spp.longitude, 
			gps_best.num_sats, gps_best.latitude, gps_best.longitude, 
			local_waypoint.llh.latitude , local_waypoint.llh.longitude , distance,
			absolute_heading , absolute_bearing, relative_bearing , 
			output.horizontal_tangential,  output.horizontal_normal, output.vertical );
		fprintf( sensor_log, "%s", buf); /* write to log-file */
		fflush( sensor_log ) ; /* we need this before we go to sleep, to make sure it is written.*/
		#endif
		/* write log, ends here */





		/* **PID** ACTUATE the system (if autonomous_state == FLYING):*/
		pthread_mutex_lock( &mutex_autonomous_state );
		int tmp_state = Autonomous_state ;
		pthread_mutex_unlock( &mutex_autonomous_state );
		if ( tmp_state == FLYING )
		{
			/* translate. pitch, roll, spin, up, 0*/
			
			if ( printflag++ % 10==0)
			{
				PRINTF_FL( "Moving. Actuation: T %.2fº, N %.2fº, V%.1f :: Distance: %.1fm\n",
					output.horizontal_tangential, 
					output.horizontal_normal, 
					output.vertical	, 
					distance );
			}
			drk_translate( 
				output.horizontal_tangential, 
				output.horizontal_normal, 
				0.0, 
				output.vertical,
				0.0 ) ;

			//usleep(1000000/refresh_rate);
		}
		else /* autonomous_state == PAUSED */
		{
			error_value.i = 0 ; // erase sum distance to avoid huge 'I' error component.
			error_value.d = 0 ; 
			sleep( 1 ) ; /* while paused, let's relax this thread */
			
			//if ( !getLockDown() ) /* if */
			//{
			//	drk_hover( 0 ) ;
			//}
			//else
			//{
				//printf("AC\t\t@"); 
				//drk_gps_print();
				//printf("\tHeading [%lfºN]", heading ) ;
				//printf("\tDistance [%3.1lfm]\n", distance ); 
			//}			
		}


		microsleep( CONTROL_PERIOD ) ;


	} /*while( ) - ends here*/
	
	PRINTF_FL("["PREFIX"] Thread terminated normally \t[%08x] *******\n" ,
		(unsigned int)pthread_self() );   
	return NULL ;

} 

void drk_autonomous_close(void)
{
	AUTO_LOADED_CONDITION ;
	Auto_exit = 1 ;
	// pthread_kill( Auto_tid , SIGTSTP ) ; /* not realyl necessary. no blocking calls */
	pthread_join( Auto_tid, NULL) ; 
	if (sensor_log != NULL) fclose( sensor_log ) ;
}

void drk_autonomous_goBS( void )
{
	gps_waypoint_t tmpw ;
	tmpw.llh = drk_piksi_get_bs(); /* get LLH where NED={0,0,0} */
	tmpw.llh.altitude = 3 ;
	tmpw.max_output =  0.5 ;
	tmpw.distance_tolerance = 1 ; 
	drk_autonomous_set_waypoint	( tmpw ) ;	
				
}


/* go north, if dist_m > 0  
 * go south, if dist_m < 0*/ 
void drk_autonomous_goNorth( double dist_m )
{
	gps_waypoint_t tmpw = drk_autonomous_get_waypoint() ; 
	tmpw.llh.latitude += dist_m/ LAT_RADIUS/3.14159*180 ; 
	drk_autonomous_set_waypoint	( tmpw ) ;
}


/* go east, if dist_m > 0  
 * go west, if dist_m < 0*/ 
void drk_autonomous_goEast( double dist_m )
{
	gps_waypoint_t tmpw = drk_autonomous_get_waypoint() ; 
	tmpw.llh.longitude += dist_m/ LON_RADIUS/3.14159*180;
	drk_autonomous_set_waypoint	( tmpw ) ;
	
}		
			
				
				
void drk_print_map( void ) 
{
	AUTO_LOADED_CONDITION ;
	double lat_k = 111132.0 ;
	double lon_k = 78847.0 ;
	char x_line[90] ;

	llh_t gps_sample = drk_gps_data() ;
	llh_t bs = drk_piksi_get_bs() ;
	gps_waypoint_t waypoint = drk_autonomous_get_waypoint() ;
	
	double uav_y = (gps_sample.latitude - bs.latitude ) * lat_k;
	double uav_x = (gps_sample.longitude - bs.longitude ) * lon_k ;
	
	double wpt_y= (waypoint.llh.latitude - bs.latitude ) * lat_k;
	double wpt_x= (waypoint.llh.longitude - bs.longitude ) * lon_k ;
	
		PRINTF_FL_WARN("bs %.6f %.6f\n ",
		bs.latitude, bs.longitude);
		
	PRINTF_FL_WARN("wpt %.6f %.6f\n ",
		waypoint.llh.latitude, waypoint.llh.longitude);
	
	PRINTF_FL_WARN("Duav %f %f// Dwpt %f %f\n",
		(gps_sample.latitude - bs.latitude)* lat_k,
		(gps_sample.longitude - bs.longitude)*lon_k,
		 (waypoint.llh.latitude - bs.latitude )* lat_k,
		 (waypoint.llh.longitude - bs.longitude ) *lon_k
		);
		
	int16_t index ;
	
	uav_x=BOUND( uav_x , 100 ) ;
	uav_y=BOUND( uav_y , 100 ) ;
	wpt_x=BOUND( wpt_x , 100 ) ;
	wpt_y=BOUND( wpt_y , 100 ) ;
	

	
	
	uav_y = - uav_y ; // invert y-axis. North = up
	wpt_y = - wpt_y ; // invert y-axis. North = up
	
	//printf("delta (x,y) = (%.1lf,%.1lf)\n" , uav_x , uav_y ) ;
	//printf("delta (x,y) = (%.1lf,%.1lf)\n" , wpt_x , wpt_y ) ;
		
	int uav_yy =  (int)( uav_y/20.0 ) ;
	int wpt_yy =  (int)( wpt_y/20.0 ) ;
	
	int uav_xx =  (int)( uav_x/10.0 ) ;
	int wpt_xx =  (int)( wpt_x/10.0 ) ;
	
	//printf("uav (xx,yy) = (%d,%d)\n" , uav_xx , uav_yy ) ;
	//printf("wpt (xx,yy) = (%d,%d)\n" , wpt_xx , wpt_yy ) ;
	

	printf(" _____________________ (+100m;+100m) 'o' = drone, '.'= BS , 'x'= target \n");
	//uint8_t edited ;
	for( index = -5 ; index  <=  5 ; index++ ) // tens of meters, #row,
	{
		
		//edited = 0 ;
		memset( x_line , ' ' , 90 );  // set all chars to 'a'
		
		
		x_line[23] = '\0' ; 
		x_line[0]  = '|' ;
		x_line[22] = '|' ;
		
		if ( index == 0 ) // draw basestation	
			x_line[11] = '.' ;
			
				
		// if there is any element, build. 
		if (  
			( uav_yy == index ) && 
			( wpt_yy == index ) &&
			( uav_xx == wpt_xx ) 
			) // uav and wpt are at the same dot.
		{
			x_line[ uav_xx + 11 ] = 'v' ;
		
		}	

		else 
		{
			if ( uav_yy == index )
			{
				x_line[ uav_xx + 11 ] = 'o' ;
			}	
			if 	(wpt_yy == index ) 
			{
				x_line[ wpt_xx + 11 ] = 'x' ;
			}
		}
			
		
				
		printf("%s\n", x_line );
		
		
		
	}
	
	
	
	printf(" ---------------------\n") ;// (+100m;+100m) 'o' = drone, '.'= BS , 'x'= target \n");
	printf(
		". - BS "
		"[d=%.1fm , trubear=%.1fº]\n"
		"x - Target "
		"[d=%.1fm , trubear=%.1fº]\n",
		drk_gps_mydistance_to( bs ),
		drk_gps_true_bearing( bs ) ,
		drk_gps_mydistance_to( waypoint.llh ),
		drk_gps_true_bearing( waypoint.llh ) 		
		);
}
	










// Returns 1 if current position is target position (+- distance_tolerance)
// 0 otherwise
enum target_status drk_autonomous_target_reached(void)
{
	AUTO_LOADED_CONDITION 0;

	pthread_mutex_lock( &mutex_reached ); 
	uint8_t r = global_target_reached ;
	pthread_mutex_unlock( &mutex_reached );
	
	return r ;
}

// return current waypoint, thread safe.
gps_waypoint_t drk_autonomous_get_waypoint(void)
{
	AUTO_LOADED_CONDITION zero_waypt ;
	gps_waypoint_t tmp_waypt ;
	pthread_mutex_lock(&mutex_waypoint);
	tmp_waypt = global_waypoint ;
	pthread_mutex_unlock ( &mutex_waypoint ) ;
	return tmp_waypt ;
}

/* Gives drone new coordinates; stops current waypoint */
void drk_autonomous_set_waypoint( gps_waypoint_t in_waypoint )
{
	AUTO_LOADED_CONDITION ;
	//feeding the global var 'waypoint', used by the controller thread
	pthread_mutex_lock(&mutex_waypoint);
	global_waypoint = in_waypoint ;
	global_new_waypoint = TRUE ;
	pthread_mutex_unlock(&mutex_waypoint);
	//printf(PREFIX_AC "Captured new waypoint: (%3.6lf, %3.6lf)\n", in_waypoint.latitude, in_waypoint.longitude);
	
}

/* Distance in degrees from the spin 
 * POSITIVE VALUE: You are to the left, turn right
 * NEGATIVE VALUE: You are to the right, turn left */
/*
static double _drk_get_degree_error(double spin, double target)
{
	double err = target - spin;
	err = wrapTo360 ( err ) ;
	return err ;
}
*/

/* This function returns the altitude using pressure. It includes a LP filter.	*/
double drk_lpf_abs_altitude( void )
{
	/* static variable! (used for the LPF)*/
	static double last_altitude = 0 ; 
	last_altitude = 0.3*last_altitude + 0.7*drk_rel_altitude(); 
	return last_altitude; 
}





