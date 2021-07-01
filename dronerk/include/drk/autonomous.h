/**
 * autonomous.h
 */

#ifndef _AUTONOMOUS_H_
#define _AUTONOMOUS_H_

//#include <inttypes.h>

#include "drk/gps.h" /* data types - gps_t */
#include "drk/utils.h"
/*******************************************************************************
 * Data types
 ******************************************************************************/
typedef struct
{
	llh_t 	llh ;
	double 	max_output ; /* "speed" on horizontal plane */
	double 	distance_tolerance ;
} gps_waypoint_t ;

typedef struct {
	double p ;
	double last_p ;
	double i ;
	double d ;
} controlpid_t; 

// speed of the drone 
// tangential component ~  pitch
// normal component ~  roll
// vertical component = gaz
typedef struct{
	double horizontal_magnitude ; /* > 0 */ 
	double horizontal_tangential ; /* normalized */ 
	double horizontal_normal ; /* normalized */ 
	double vertical ; /* >0 --> up . <0 --> down */
} actuation_t ; 

/**
 * Bit vector to be used with the translate function
 */
//enum TRANSLATE_OPTIONS
//{
  //DRK_CONTINUE  = 0x00000000, // Keep moving when done
  //DRK_HOVER     = 0x00000001, // Return to hover when done
  //DRK_STOP      = 0x00000002, // TODO: Aggressively stop when done (not in yet)
  //DRK_OVERRIDE  = 0x00000004  // Allow control in lockdown
//};

enum flight_status
{ 
	FLYING	= 0,
	PAUSED	= 1,
	UNLOADED = 2,
};
enum target_status
{ 
	T_REACHED		= 1,
	T_NOT_REACHED	= 0
};

/*******************************************************************************
                            AUTONOMOUS FLIGHT FUNCTIONS
*******************************************************************************/

/**
 * initiates autonmous controller
 * returns -1 if fails to init
 */
error_t 			drk_autonomous_init(void);

/** Clean up **/
void 				drk_autonomous_close(void);

/** Pause movement immediately (used by keyboard thread)**/
void				drk_autonomous_pause(void);


/** Resume flight (used by keyboard thread) **/
void 				drk_autonomous_resume(void);

/**
 * Returns the state of autonomous flight:
 * PAUSED (1) - no rotor output
 * FLYING (0) - flying to target
 */
enum flight_status drk_autonomous_get_state(void);


/** returns the current waypoint target**/
gps_waypoint_t 	drk_autonomous_get_waypoint(void) ;


/** Gives new waypoint to go; overwrites current waypoint **/
void drk_autonomous_set_waypoint( 
	gps_waypoint_t in_waypoint );


/** is drone at target position? (<= distance_tolerance) **/
enum target_status	drk_autonomous_target_reached(void);

/**
 * set PID controller.
 * Kp_? - porportional  
 * Ki_? - integral
 * Kd_? - differental
 * ?_h - horizontal 
 * ?_v - vertical
 */
void drk_autonomous_set_PID( 
	double _kp_h, double _ki_h, double _kd_h,  double _kp_v);

/** print a small ascii map with cur pos, BS, and cur target **/
void drk_print_map( void)  ; 

/** go to piksi base **/
void drk_autonomous_goBS( void );

/** go north, if dist_m > 0  
 * go south, if dist_m < 0 **/ 
void drk_autonomous_goNorth( double dist_m );

/** go east, if dist_m > 0  
 * go west, if dist_m < 0 **/ 
void drk_autonomous_goEast( double dist_m );


#endif // _AUTONOMOUS_H_
