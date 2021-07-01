/**
 *  internal_sensor_api.h
 * 	use these functions to: receive and parse navdata,  etc
 */
#ifndef _INTERNAL_ACTUATOR_API_H_
#define _INTERNAL_ACTUATOR_API_H_

#include <inttypes.h>
#include <semaphore.h>

#include "drk/utils.h"
typedef union _float_or_int_t {
	float f;
	int32_t i;
} float_or_int_t;


struct drone_actuator_
{
	int 			takeoff_flag;		/* 1 = take off, -1 = landing, 0 = flying */
	int 			emergency_flag; /* 1 = emergency */
	int 			pcmd_flag; /* 0 = hover, 1 = motion */
	float_or_int_t 	roll;	/* Left/Right angles */
	float_or_int_t 	pitch;	/* Forth/Back angles */   
	float_or_int_t 	gaz;	/* vertical speed, + means rise, - means go down */
	float_or_int_t 	spin;	/* angle speed, + spin CW, - spin CCW (top view) */ 
	int 			magneto_calib;		/* calibrate magnetometer ? */
};

struct global_actuator_ 
{
	sem_t* 					time_sem;
	long int 				global_timestamp;
	sem_t* 					semaphore;
	struct drone_actuator_	actuator;
};

/* List of LED animations */
enum LED_ANIMATION
{
	BLINK_GREEN_RED,
	BLINK_GREEN,
	BLINK_RED,
	BLINK_ORANGE,
	SNAKE_GREEN_RED,
	FIRE,
	STANDARD,
	RED,
	GREEN,
	RED_SNAKE,
	BLANK,
	RIGHT_MISSILE,
	LEFT_MISSILE,
	DOUBLE_MISSILE,
	FRONT_LEFT_GREEN_OTHERS_RED,
	FRONT_RIGHT_GREEN_OTHERS_RED,
	REAR_LEFT_GREEN_OTHERS_RED,
	REAR_RIGHT_GREEN_OTHERS_RED,
	LEFT_GREEN_RIGHT_RED,
	LEFT_RED_RIGHT_GREEN,
	BLINK_STANDARD,
	NB_LED_ANIM_MAYDAY 
};


#define LANDING		-1 
#define TAKINGOFF	1
#define FLYING		0

/**
 *  Prototypes, Declarations 
 * */

error_t drk_actuator_init( void ) ;
error_t drk_actuator_close( void ) ;

int drk_actuator_update( struct drone_actuator_* const p );

/* Send a command to the drone */
int drk_send_at_command( const char *send_command , ... );

/* Play an animated sequence on the LEDs */
int drk_play_LED_animation(enum LED_ANIMATION animation, float frequency, int duration);


//void drk_zero_altitude(uint16_t dft);



#endif //_INTERNAL_ACTUATOR_API_H_
