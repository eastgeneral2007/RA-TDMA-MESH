/**
 * gps.c
 * Nat Storer - 6/2/2011
 * Luis Pinto - since
 * Reads gps values from our external sensor board 
 */
#include <stdio.h>
#include <math.h> /* pow, cos, etc.. */
#include <string.h> /* memcpy */
#include <unistd.h> /* sleep */

#include "drk/gps.h"
//#include "drk/internal_sensor_raw.h" /* drk_calibration_get/set */
#include "drk/expansion_sensor_api.h" /* global_serial is defined there */
#include "drk/utils.h" /* printf_fl_err , etc */



#define PREFIX	"GPS"

/*************************
 * Externable Globals
 ***********************/
//double base_lat = BASELAT  ; //in gps.h
//double base_lon = BASELON ; // in gps.h

/******************
 * Externs
 *****************/
/* instantied in ESSH.c , all data from gps comes is here */
extern struct global_serial *serial_buf ; 



/*****************************+++ 
 * Static Prototypes 
 * ************************/
/* Check to see if a given GPS data point was valid */
static int16_t wait_for_gps( int16_t time_seconds ) ;



/***************************************************
 * CODE - Function definitions
 *******************************************************/


/*******************************************************************************
* GPS - useful functions
*******************************************************************************/


/* Wait for valid GPS data for 30 seconds 
 * Returns number of seconds waited
 */
int16_t drk_wait_for_gps()
{
	return wait_for_gps(30);
}



/*******************************************************************************
			Processed GPS Data
*******************************************************************************/

/* Get altitude from GPS */
double gps_get_altitude( void ) 
{
	llh_t temp_gps = drk_gps_data();
	return temp_gps.altitude ;
}

/* Calculate the distance between 
 * current position and a coordinate pair 
 * returns -1 if there is not a gps fix */
double drk_gps_mydistance_to( llh_t target )
{
	/* Check for fix validity, then pass off to calculation function */
	if ( !drk_gps_myfix() ) return -1 ;

	llh_t my_location = drk_gps_data();
	
	//double my_lat = temp_gps.latitude;
	//double my_long = temp_gps.longitude;

	double dist = drk_gps_distance_between( my_location , target );

	return dist ;
}


/* Calculate north-angle (CW) between 
 * current location & target pair of lat-long  (degrees) */
double drk_gps_true_bearing( llh_t target )
{
	llh_t temp_gps = drk_gps_data();
	
	
	double my_lat = temp_gps.latitude;
	double my_long = temp_gps.longitude;
	
	double target_lat	= degrees_to_radians( target.latitude - my_lat ) ;
	double target_long 	= degrees_to_radians( target.longitude - my_long ) ;
	
	// convert radians to arclen 
	double north_meters = target_lat 	* LAT_RADIUS ;
	double east_meters 	= target_long	* LON_RADIUS ;
	
	double true_bearing_deg = wrapTo360( 
		radians_to_degrees( 
			atan2( north_meters , east_meters ) ) ) ; /* true bearing degrees, in earth referential */
	return true_bearing_deg ;

}

/* Return whether or not the current drone's GPS has a valid fix */
int drk_gps_myfix( void )
{
	int n_sats = drk_gps_get_numsats() ;
	if ( n_sats <= 3 ) 
		return 0;
	else
		return n_sats ;

}

/* Returns the number of satellites used for locking */
int drk_gps_get_numsats( void )
{
	llh_t temp_gps = drk_gps_data();
	return temp_gps.num_sats ;
}


/*******************************************************************************
			Raw GPS Data
*******************************************************************************/
/* Return a thread-safe copy of the last sample of GPS data */
llh_t drk_gps_data(void)
{
	//PRINTF_FL_WARN("getting gps data --> spp\n");
	return drk_piksi_get_spp();
}



/* Returns VTG heading data if groundSpeed > 0
 * Returns -1 if groundSpeed <= 0
 */
double drk_vtg_heading(void)
{
	double trueTrack=0;	/* Bearing due North */
	double groundSpeed=0;	/* Speed of Drone relative to ground */

	/* Pull data from struct */
	//llh_t temp_gps = drk_gps_data();
	//trueTrack 	= temp_gps.trueTrack;
	//groundSpeed = temp_gps.groundSpeed;

	/* If speed <= 0, return -1 */
	if (groundSpeed <= 0) return -1.0;
	else return trueTrack;
}

/* Returns ground speed of drone */
double drk_groundSpeed_get(void)
{
	double groundSpeed=0;

	//pthread_mutex_lock( &(serial_buf->gps_mutex)  );
	//groundSpeed = (serial_buf->gps_buf).groundSpeed;
	//pthread_mutex_unlock( &(serial_buf->gps_mutex)  );

	/* If speed < 0, print error */
	if (groundSpeed < 0)
	{
		PRINTF_FL_ERR("Speed < 0\n");
	}
	return groundSpeed;
}

/* Returns GPS altitude of drone */
double drk_gps_altitude_get(void)
{
	
	llh_t gps = drk_gps_data();
	double altitude = gps.altitude ;

	/* If altitude < 0 print error */
	if (altitude < 0) {
		PRINTF_FL_ERR("Invalid Altitude\n");
		return 0;
	}
	else return altitude;
}

/* ***************************
 * GPS utils * 
 * *******************************/
/* NMEA format angle(latitude\longitude) into decimal */
double drk_nmea_to_decimal(double coordinate)
{
	double end = fmod(coordinate, (double)100.0);
	return (double)(((coordinate - end) / 100.0) + (end / 60.0));
}

/* calculate heading between 
 * some position and a target location - degrees */
//int drk_cartesian_heading(double my_lat, double my_long, 
	//double target_lat , double target_long )
//{
	//double latdiff = target_lat - my_lat;
	//double longdiff = target_long - my_long;
	//return ((int)(drk_radians_to_degrees(atan2(longdiff, latdiff)) + 360))%360;
//}



/* Calculate the distance (meters) 
 * between two coordinate pairs */
double drk_gps_distance_between( 
	llh_t gps1 , llh_t gps2 )
{
	const double earth_radius = 6371000; //TODO: why is this a variable? LP

	// Get the difference between our two points then convert the difference into
	// radians

	double lat_diff = degrees_to_radians(gps2.latitude - gps1.latitude);  
	double long_diff = degrees_to_radians(gps2.longitude - gps1.longitude); 

	double lat1 =  degrees_to_radians( gps1.latitude ) ;
	double lat2 =  degrees_to_radians( gps2.latitude ) ;

	double a = pow(sin(lat_diff / 2), 2) +
		cos(lat1) * cos(lat2) * pow(sin(long_diff/2), 2); 

	double c = 2 * atan2( sqrt(a), sqrt(1 - a));

	double dist = earth_radius * c;

	return dist;
}


static int16_t wait_for_gps( int16_t time_seconds )
{
	int16_t count = time_seconds ;

	if ( !drk_gps_myfix() )
	{
		PRINTF_FL("Waiting for a GPS fix for %ds\n" , time_seconds );
		while ( !drk_gps_myfix() && count > 0)
		{
			sleep(1);
			count--;
		}
		if ( !drk_gps_myfix() )
		{
			PRINTF_FL("Fix acquired.\n");
		}
		else
			PRINTF_FL_ERR("Failed to get a fix\n") ;

	}

	return count ;
}


/*******************************************************************************
              DEBUGGING METHODS
*******************************************************************************/

/* Print the most recent GPS data */
void drk_gps_print( void )
{
  /* Copy the data to make it thread-safe */
  //PRINTF_FL_WARN("printng gps...\n");
  llh_t gps_copy = drk_gps_data();
  //double age = getEpoch() - gps_copy.timestamp   ;
  /* Print the data */
  //printf("****GPS DATA****\n");
  PRINTF_FL_WARN("time [%ld]s\t#Sats [%u]\tLat [%3.6f]\tLon [%3.6f]\tAlt [%3.1f]\n",
    gps_copy.timestamp , 
    gps_copy.num_sats , 
    gps_copy.latitude, gps_copy.longitude, gps_copy.altitude );
    
   /* quality
	 * 1  NO CHANGE, 
	 * 2  NEW RTK, 
	 * 3  NEW SPP,
	 * 4  expired,
	 * -1 ERROR
	 */
  //printf("UTC [%f]\tQuality [%d]\tnum_sats [%d]\tidx [%d]\n\n",
  //  gps_copy.UTC, gps_copy.quality, gps_copy.num_sats, gps_copy.index);
}
