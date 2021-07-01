/**
 * ar_configuration.c
 */
#include <stdio.h>
#include <string.h>


#include "drk/ar_configuration.h"
#include "drk/internal_actuator_api.h" /* send_at_command commands during Init */
#include "drk/internal_sensor_raw.h" /* get raw accelerometer values */
#include "drk/navdata.h" // get magnetometer status
#include "drk/utils.h"

#define PREFIX	"AR_CONF"
#define APP_ID

/******************************************
 * Globals
 *****************************************/
extern struct global_actuator_ *actuator_buf ;

/*******************************************************************************
						DRONE CONFIGURATION
*******************************************************************************/
char Session[15];
char Profile[15];
char App[15];

int call_config_ids( )
{

	return drk_send_at_command(
		"AT*CONFIG_IDS=%ld,\"%s\",\"%s\",\"%s\"\r", 
		actuator_buf->global_timestamp++, 
		Session, 
		Profile, 
		App ) ;

}

int set_session_profile_app( char* session, char* profile , char* app )
{
	strncpy( Session , session 	, sizeof(App)-1 ) ;
	strncpy( Profile , profile	, sizeof(App)-1 ) ;
	strncpy( App 	 , app  	, sizeof(App)-1 ) ;

	call_config_ids();
	drk_send_at_command(
		"AT*CONFIG=%ld,\"custom:session_id\",\"%s\"\r", 
		actuator_buf->global_timestamp++,
		session ) ; 
	microsleep(10000);
	
	call_config_ids();
	drk_send_at_command(
		"AT*CONFIG=%ld,\"custom:profile_id\",\"%s\"\r", 
		actuator_buf->global_timestamp++,
		profile );
	
	microsleep(10000);
	
	call_config_ids();
	drk_send_at_command(
		"AT*CONFIG=%ld,\"custom:application_id\",\"%s\"\r", 
		actuator_buf->global_timestamp++,
		app );

	return 1 ;
}
/* Maximum pitch / roll angle. Must be between 0 and 0.52 radians */
int drk_ar_change_max_angle(float radians)
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	radians = MAX(0, radians);
	radians = MIN(0.52, radians);	
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:euler_angle_max\",\"%f\"\r",
		actuator_buf->global_timestamp++, 
		radians );
	sem_post(actuator_buf->time_sem);
	return ret ;
}

/* min Altitude of the drone in millimeters. 50 - max */
int drk_ar_change_min_altitude(int altitude_mm)
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	altitude_mm = MAX(500, altitude_mm);
	//if (minimum_altitude > 5000) maximum_altitude = 10000;
	ret+=drk_send_at_command(
		"AT*CONFIG=%ld,\"control:altitude_min\",\"%d\"\r",
		actuator_buf->global_timestamp++, 
		altitude_mm);
	sem_post(actuator_buf->time_sem);
	return ret ;
}

/* Maximum Altitude of the drone in millimeters. 500 - 5000, or 10000 = no lim*/
int drk_ar_change_max_altitude(int altitude_mm)
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	altitude_mm = MAX(500, altitude_mm);
	if (altitude_mm > 5000) altitude_mm = 10000;
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:altitude_max\",\"%d\"\r",
		actuator_buf->global_timestamp++, altitude_mm);
	sem_post(actuator_buf->time_sem);
	return ret ;
}

/* Maximum Vertical Speed, in millimeters per second, 200-2000 */
int drk_ar_change_max_vertical_speed( int speed_mm_per_sec )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	
	speed_mm_per_sec = MAX( 200 , speed_mm_per_sec);
	speed_mm_per_sec = MIN( 2000, speed_mm_per_sec);
	
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:control_vz_max\",\"%d\"\r",
		actuator_buf->global_timestamp++, 
		speed_mm_per_sec);
	sem_post(actuator_buf->time_sem );
	return ret ; 
}

/* Maximum Yaw Speed, in radians per second, 0.7 - 6.11 */
int drk_ar_change_max_yaw_speed( float speed_rad_per_sec )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	speed_rad_per_sec = MAX(0.7, speed_rad_per_sec);
	//maximum_speed = MIN(6.11, maximum_speed); //TODO should be commented out?
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:control_yaw\",\"%f\"\r",
		actuator_buf->global_timestamp++,
		speed_rad_per_sec );
	sem_post(actuator_buf->time_sem);
	return ret ; 

}


//zeroes inertial sensors
int drk_ar_flat_trim( void )
{
	PRINTF_FL("Flat trim ..\n");
	sem_wait(actuator_buf->time_sem);
	int ret = drk_send_at_command(
		"AT*FTRIM=%ld\r",
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	usleep(15000);
	PRINTF_FL(
		"Acc > P[%+3.2f]\tR[%+3.2f]\tY[%+3.2f]\n",  
		drk_drone_pitch_get() , 
		drk_drone_roll_get() , 
		drk_drone_yaw_get() );

	return ret ;
}


int drk_ar_set_outdoor_flight( void )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:outdoor\",\"TRUE\"\r", 
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	return ret ;
}


int drk_ar_set_indoor_flight( void )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:outdoor\",\"FALSE\"\r", 
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	return ret ;
}

int drk_ar_set_outdoor_hull( void )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:flight_without_shell\",\"TRUE\"\r", 
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	return ret ;
}

int drk_ar_set_indoor_hull( void )
{
	sem_wait(actuator_buf->time_sem);
	int ret = call_config_ids();
	ret += drk_send_at_command(
		"AT*CONFIG=%ld,\"control:flight_without_shell\",\"FALSE\"\r", 
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	return ret ;
}

void drk_ar_calibrate_magnetometer(void)
{
	PRINTF_FL("Mag Calibration needed? %d\n", 
		getStateMagnetometer());
	//The flag is tracked by drk_actuator_update
	sem_wait(actuator_buf->semaphore);
	(actuator_buf->actuator).magneto_calib = 1 ;
	sem_post(actuator_buf->semaphore);

}

void drk_ar_set_hover_oriented_roundel(void)
{


	/* Activate oriented roundel detection */
	sem_wait(actuator_buf->time_sem);
	call_config_ids();
	drk_send_at_command("AT*CONFIG=%ld,\"detect:detect_type\",\"%d\"\r", 
		actuator_buf->global_timestamp++, 12);
	sem_post(actuator_buf->time_sem);


	/* Activate hover over roundel mode */
	sem_wait(actuator_buf->time_sem);
	call_config_ids();
	drk_send_at_command("AT*CONFIG=%ld,\"control:flying_mode\",\"%d\"\r", 
		actuator_buf->global_timestamp++, 1 << 1);
	sem_post(actuator_buf->time_sem);
	

	/* Set hover distance in millimeters */
	sem_wait(actuator_buf->time_sem);
	call_config_ids();
	drk_send_at_command("AT*CONFIG=%ld,\"control:hovering_range\",\"100\"\r", 
		actuator_buf->global_timestamp++);
	sem_post(actuator_buf->time_sem);
	

}
