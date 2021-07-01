/**
 * ar_configuration.h
 * use this functions to configure the drone , max speeds, etc..
 */

#ifndef _AR_CONFIGURATION_H_
#define _AR_CONFIGURATION_H_

#include <semaphore.h>

#include "drk/internal_sensor_api.h" /* drone_actuator */


#define ARDRONE_SESSION_ID			"00000000" //"d2e081a3"	// SessionID
#define ARDRONE_PROFILE_ID			"00000000" //"be27e2e4"	// Profile ID
#define ARDRONE_APPLICATION_ID 		"00000000" //"d87f7e0c"	// Application ID


/***************************************
 * Structs
 **************************************/

/****************************
 * Declarations
 ***************************/
/** Maximum pitch / roll angle. Must be between 0 and 0.52 radians **/
int drk_ar_change_max_angle( float radians );
/** min Altitude of the drone in millimeters. 50 - max **/
int drk_ar_change_min_altitude( int altitude_mm );
/** Maximum Altitude of the drone in millimeters. 500 - 5000, or 10000 = no lim **/
int drk_ar_change_max_altitude( int altitude_mm );
/** Maximum Vertical Speed, in millimeters per second, 200-2000 **/
int drk_ar_change_max_vertical_speed ( int speed_mm_per_sec);
/** Maximum Yaw Speed, in radians per second, 0.7 - 6.11 **/
int drk_ar_change_max_yaw_speed( float speed_rad_per_sec);
/** zero inertial sensors **/
int drk_ar_flat_trim(void);
int drk_ar_set_outdoor_flight(void);
int drk_ar_set_indoor_flight(void);
int drk_ar_set_outdoor_hull(void);
int drk_ar_set_indoor_hull(void);
void drk_ar_calibrate_magnetometer(void);
void drk_ar_set_hover_oriented_roundel(void);
int call_config_ids() ;
int set_session_profile_app( char* session, char* profile , char* app );

#endif // _AR_CONFIGURATION_H_
