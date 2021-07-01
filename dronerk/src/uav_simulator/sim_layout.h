/** 
 * Layout Module 
 * takes care of managing UAV motion, receving actuation and providing sensor readings
 * **/
#ifndef LAYOUT_H_
#define LAYOUT_H_

#include <inttypes.h>
#define MILLION			1000000.0
#define MAX_NUM_DRONES 	100 /* number of nodes instanciated */


/* all typedefs */
typedef struct {
	float X ; /* 4 bytes */
	float Y ; /* 4 bytes */
	float Z ; /* 4 bytes */
} triplet_t ; /* 12 bytes */

typedef struct {
	float roll ; /* 4 bytes */
	float pitch ; /* 4 bytes */
	float gaz ; /* 4 bytes */
	float torque ; /* 4 bytes */
} actuator_t ;

typedef struct {
	int 		status; /* 4bytes -- */
	float 		battery; /* 4bytes - 0 to 100 pp */
	/* current physics */
	triplet_t	position ; /* 12 bytes - meters */ 
	triplet_t	velocity ; /* 12 bytes - m/s */
	float		yaw_degrees ;	 /* 4 bytes - absolute orientation regarding north */
	actuator_t	thrust ; /* 12 bytes */ 	/* current actuation - Newton */
} state_t ; /* 44 bytes */

typedef struct {
	int ndrones;
	state_t state[MAX_NUM_DRONES] ; // lets ignore node 0
} layout_t ; /* (10+1)*44 bytes = 484 bytes */



typedef struct {
	uint32_t	rtk_sample_id ;
	int8_t		rtk_num_sats;
	int32_t 	north; 
	int32_t		east;
	int32_t		down;
	int32_t		vel_north;
	int32_t		vel_east; 
	int32_t		vel_down;
	uint32_t	spp_sample_id; 
	int8_t 		spp_num_sats ;
	int32_t		latitude ; 
	int32_t		longitude  ;
} rtk_reading_t ;


typedef struct {
	uint32_t state ;
	int 	battery ;
	float 	heading_degrees ;
	float 	ultra_altitude ;
} isensor_reading_t ; 

/* update the phyisical world */
void layout_update(void) ;

/* init layout  */
int layout_init(int tndrones) ;

/* print stuff */
void layout_printPositions(void) ;
void layout_printState(int node);


/* get distance between two drones - meters */
float layout_getDistance( int node1, int node2 ) ;

/* get position from one drone - meters */
triplet_t layout_getPosition( int node );

/* get velocity from one drone - m/s */
triplet_t layout_getVelocity( int node );

/* get yaw from one drone  - degrees */
float layout_getYaw( int node );

/* get altitude from one drone - m */
float layout_getAltitude( int node );

/* get battery level 0 to 100 */
int layout_getBattery( int node);

/*takeoff before moving drone , wind won't affect landed drones */
void layout_takeoff( int node );

/* get a sensor reading */
isensor_reading_t layout_getIntReading( int node );
rtk_reading_t layout_getRTKReading( int node );

/* set drone thrust vector */
void layout_setActuation( actuator_t actuator , int node );

/******
 *  external defined functions 
 * ****/



#endif
