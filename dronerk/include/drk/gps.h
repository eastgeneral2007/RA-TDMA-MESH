/*! \file gps.h
    \brief A Documented file.
    
    Details.
*/

#ifndef _GPS_H_
#define _GPS_H_

//#include <termios.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <fcntl.h>
//#include <pthread.h>
//#include <vector.h>
//#include <drk.h>
//#include <sys/syscall.h>
//#include <sched.h>
//#include <drk_sched.h>
#include <time.h>
#include <stdint.h>
#include "libsbp/read_piksi.h"

#define EARTH_RADIUS 6368913 // lets use the radius at 41.17º lat
#define LAT_RADIUS   6368913 // = R_lon,  lat = 41.17º . NORTH SOUTH deltas
#define LON_RADIUS   4801205 // = a.cos( lat ) ,  lat = 41.17º. WEST EAST deltas



/*
 * R_lon = SQRT( num / den )
 * 
 * num = ( a^2*cos(t) )^2 + ( b^2*sin(t) )^2 
 * den = ( a*cos(t) )^2 + ( b*sin(t) )^2
 * Rt = radius of earth at latitude t                
 * a = semi major radius of earth         = 6378.137 meters
 * b = semi minor radius of earth         = 6356.75231420 meters 
 */
 
 
/* Print error messages */
#define OSS 3









/* Accessors for GPS data */
//double GPS_latitude();
//double GPS_longitude();
//double GPS_altitude();



/* Functions to be used by everyone */

int16_t drk_wait_for_gps( void );
//void drk_set_basestation( void );
//void drk_get_basestation( double * blat, double * blon );


//void drk_calibration_data_print();

/* return lastest gps sample data received */
llh_t drk_gps_data( void );
//gps_t drk_gps_copy();

/* Check for a GPS fix on the current drone's GPS */
int drk_gps_myfix( void );
int drk_gps_get_numsats( void );

/* Calculate the distance between : */ 
/* two coordinate pairs */
//double drk_gps_coordinates_distance( double lat1, double long1, double lat2, double long2);

/* two LLH structs */
double drk_gps_distance_between( llh_t gpsA, llh_t gpsB );

/* current position and a coordinate pair */
double drk_gps_mydistance_to( llh_t target );

/* Get altitude from GPS */
double gps_get_altitude( void ) ;

//int drk_cartesian_heading( double my_lat, double my_long, double target_lat, double target_long);

/* current position and a GPS struct */
double drk_gps_true_bearing( llh_t target );

/* Returns GPS altitude of drone */
double drk_gps_altitude_get( void );
/* Returns ground speed of drone */
double drk_groundSpeed_get( void );
/* Returns VTG heading data, if groundSpeed > 0 */
double drk_vtg_heading( void );

/* Helper function definitions */
double drk_nmea_to_decimal(double coordinate);



/* Debugging methods */
void drk_gps_print();

#endif // _GPS_H_
