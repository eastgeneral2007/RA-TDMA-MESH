/** 
 * Layout Module 
 * takes care of managing UAV motion, receving actuation and providing sensor readings
 * **/
#include <math.h> /* sqrt */
#include <stdlib.h> /* rand */
#include <inttypes.h> /* int16_t etc */
#include <strings.h> /* memcpy */
#include <stdio.h> /* printf */
#include <sys/mman.h>	/* shm_open */
#include <sys/stat.h>	/* For mode constants */
#include <fcntl.h>		/* For O_* constants */
#include <semaphore.h>	/* semaphore */
#include <unistd.h>		/* ftruncate *//* close */

#include "sim_layout.h"
#include "return_codes.h"
#include "sim_clock.h" /*  getSimClock(); */

#include "utils.h"

#define DRAG_CONSTANT		0.1 /* air default 0.1; 0.5 for sidney tests */
#define UAV_MASS			0.200 // kg
#define MAX_THRUST			5 // Newtons 

enum flyingstatus { LANDED=0,FLYING=1};

#define GPS_ERROR			5 // getSensor + randn <GPS_ERROR> meters
#define YAW_ERROR			1.0 // getSensor + randn <YAW_ERROR> degrees
#define ULTRASOUND_ERROR	0.5 // meters 
#define ACTUATION_ERR  		0.01 // Newtons . Actuation + randnorm <ACTUATION_ERR> 

#define LAYOUT_INIT_POSITIONS_FILENAME "layout_init_positions.conf"
#define EARTH_RADIUS 6368913 // lets use the radius at 41.17º lat
#define LAT_RADIUS   6368913 // = R_lon,  lat = 41.17º . NORTH SOUTH deltas
#define LON_RADIUS   4801205 // = a.cos( lat ) ,  lat = 41.17º. WEST EAST deltas

#define	BSLAT 41.176752
#define	BSLON -8.596227
	
#define INITHEIGHT	10 /* meters */

#define PREFIX	"Layout"

static volatile int init_flag=0;

#define IF_INIT_RETURN(x)\
if (!init_flag)\
{\
	PRINTF_FL_ERR("Not init'ed. Call layout_init()\n");\
	return x;\
}

#define IF_NODE_RETURN(node,x)\
if (node > layout_ptr->ndrones)\
{\
	PRINTF_FL_ERR("invalid node number\n");\
	return x;\
}



/** 
 * evil static globals
 ***/
static uint64_t			last_timestamp_this_run = 0 ; /* timestamp when layout updates occur */
static const triplet_t 	triplet0 ; /* to be used to zero any triplet  */
static layout_t			*layout_ptr ; /* we save here the state of every dronem, for other apps to read*/


/*** prototypes -  only to be used locally ****/
inline static int layout_loadInitPosition(int ndrones);
inline static triplet_t get_wind_vel( float t ); /*grab wind speed at time <time_us> */

#define LAYOUT_MEM	"/layout_memspace"


/**
 * Initialize layout:
 * init positions and memshare for others to VIZUALIZE the network  
 * there is no semaphore. others only READ.
 * */
int layout_init(int t_ndrones) 
{
	PRINTF_FL("Initiating\n");
	if (t_ndrones > MAX_NUM_DRONES)
	{
		PRINTF_FL_ERR("trying to load more than maximum #drones\n");
		return NOTOKAY;
	}

	/* init memory share of the layout_ptr-> this should be used by Visualizers external apps */
	const int oflag = O_CREAT | O_EXCL | O_RDWR ; /* create and make sure it is *NEW*. R+W access  */
	const mode_t mode = S_IRUSR | S_IWUSR ;
	int fd;
	do
	{
		fd = shm_open(LAYOUT_MEM, oflag, mode );
		if (fd!=-1)break;
		 	
		PRINTF_FL_WARN("Layout SHM already exists..trying to unlink..\n");
		if (-1 == shm_unlink(LAYOUT_MEM))
		{
			PRINTF_FL_ERR(" - fail to unlink :(\n");
			return NOTOKAY;
		}
		//else
		PRINTF_FL("- unlinked :)\n");
	}	while(1);
	
	if (-1 == ftruncate(fd, sizeof(*layout_ptr))) // truncate memory space to layoutsize bytes. that's all we need
	{
		PRINTF_FL_ERR("ftruncate failed!\n");
		return -1;
	}
	layout_ptr = (typeof(layout_ptr))
		mmap(NULL, sizeof(*layout_ptr), 
		PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0 ) ;
	if (MAP_FAILED == layout_ptr)
	{
		PRINTF_FL_ERR("mmap - failed!\n") ;
		return NOTOKAY ;
	}
	close(fd) ; /* not needed anymore */ 


	layout_ptr->ndrones = t_ndrones ; /* save it */ 
	
	/* read init layout file , and set it */
	if ( layout_loadInitPosition(t_ndrones) == NOTOKAY )
	{
		//PRINTF_FL("Loading Init Positions: FAILED !\n") ;
		return NOTOKAY ;
	}
	PRINTF_FL("there are %d drones\n",layout_ptr->ndrones);
	init_flag=1; /* now it is safe to call any of the layout functions */
	for (int i = 0;i<layout_ptr->ndrones; i++)
		layout_printState(i);
	return 0 ; /* success */
}

void layout_takeoff(int node)
{
	PRINTF_FL_WARN("#%d taking off!\n",node);
	IF_INIT_RETURN();
	IF_NODE_RETURN(node, );
	layout_ptr->state[node].status = 1 ;//flying;
}

/* set a given actuation to the node. adds some  normaldistributed error */
void layout_setActuation( actuator_t actuator , int node )
{
	IF_INIT_RETURN();
	IF_NODE_RETURN(node,);
	float z1,z2;
	normalrand(&z1,&z2);
	layout_ptr->state[node].thrust.roll	 = BOUND( actuator.roll + z1*ACTUATION_ERR, MAX_THRUST ) ;
	layout_ptr->state[node].thrust.pitch = BOUND( actuator.pitch + z2*ACTUATION_ERR, MAX_THRUST ) ;
	layout_ptr->state[node].thrust.gaz	 = BOUND( actuator.gaz, MAX_THRUST ) ;
	layout_ptr->state[node].thrust.torque= BOUND( actuator.torque, MAX_THRUST ) ;
	//printf("t.X=%.2f ; t.y=%.1f ; torque=%.1f ", 
		//layout_ptr->state[node].thrust.roll , 
		//layout_ptr->state[node].thrust.pitch ,
		//layout_ptr->state[node].thrust.torque ) ;
}

/* update delta_us microseconds of the physical world
 * follow 2nd order dynamics */
void layout_update(void)
{
	IF_INIT_RETURN();
	uint64_t delta_us  = SimClock_get() - last_timestamp_this_run ;
	last_timestamp_this_run = SimClock_get() ;
	//printf(" delta_us: %" PRIu64 "us ", delta_us );
	
	for (int idx=0; idx<layout_ptr->ndrones; idx++)
	{
		if (layout_ptr->state[idx].status == LANDED)/*landed*/
			{PRINTF_FL_WARN("landed\n");
			continue;}
		/* z axis - linear */
		layout_ptr->state[idx].position.Z += layout_ptr->state[idx].thrust.gaz / UAV_MASS * delta_us / 1e6 ; 
		
		/* spin - linear */
		layout_ptr->state[idx].yaw_degrees += 25 * layout_ptr->state[idx].thrust.torque / UAV_MASS * delta_us / 1e6  ; 
		while ( layout_ptr->state[idx].yaw_degrees>360 ) layout_ptr->state[idx].yaw_degrees -= 360 ;
		while ( layout_ptr->state[idx].yaw_degrees<0 ) layout_ptr->state[idx].yaw_degrees += 360 ;
		//printf("layout@ yaw degrees %f: \n" ,  layout_ptr->state[idx].yaw_degrees );
		//if ( layout_ptr->state[idx].yaw_degrees>0 ) layout_ptr->state[idx].yaw_degrees %= 360 ;
		//if ( layout_ptr->state[idx].yaw_degrees<0 ) {-layout_ptr->state[idx].yaw_degrees %= 360 ; 
			//layout_ptr->state[idx].yaw_degrees=-layout_ptr->state[idx].yaw_degrees;}
		
		/* x,y axis is 2nd order -update POS */
		layout_ptr->state[idx].position.X += layout_ptr->state[idx].velocity.X * delta_us / 1e6 ;
		layout_ptr->state[idx].position.Y += layout_ptr->state[idx].velocity.Y * delta_us / 1e6 ;
	
		
		// air speed
		triplet_t wind = get_wind_vel( 1.0*SimClock_get()/1e6 ) ; 
		float v_air_x = layout_ptr->state[idx].velocity.X - wind.X ; 
		float v_air_y = layout_ptr->state[idx].velocity.Y - wind.Y ; 
		float v_air_mag = sqrt( v_air_x*v_air_x + v_air_y*v_air_y + 0.0001) ;
		
		/* air resistance */
		float drag_mag = DRAG_CONSTANT * ( v_air_x*v_air_x  + v_air_y*v_air_y  ) ; // 10m/s^2 when at 10m/s ,equals max thrust
		
		float drag_accel_x = -drag_mag * ( v_air_x ) / v_air_mag ;
		float drag_accel_y = -drag_mag * ( v_air_y ) / v_air_mag ;
		
		float angle_rad = (layout_ptr->state[idx].yaw_degrees)*M_PI/180 ;
		float thrust_global_x = -layout_ptr->state[idx].thrust.pitch * cos( angle_rad ) -
							-layout_ptr->state[idx].thrust.roll * sin( angle_rad ) ;
							 
		float thrust_global_y = -layout_ptr->state[idx].thrust.roll * cos( angle_rad ) +
			-layout_ptr->state[idx].thrust.pitch * sin( angle_rad ) ;

		float total_accel_x = drag_accel_x + thrust_global_x / UAV_MASS *5 ;
		float total_accel_y = drag_accel_y + thrust_global_y / UAV_MASS *5;
		
		/* update VElocity*/
    	layout_ptr->state[idx].velocity.X += total_accel_x * delta_us / 1e6 ; 
		layout_ptr->state[idx].velocity.Y += total_accel_y * delta_us / 1e6 ; 
		
		/* update battery*/
		if (layout_ptr->state[idx].status==FLYING)
			layout_ptr->state[idx].battery -= sqrt(
				ABS(total_accel_x)*ABS(total_accel_x) +
				ABS(total_accel_y)*ABS(total_accel_y)) / 1e7  ; 
	}
}


void layout_printState(int node)
{
	IF_INIT_RETURN();
	PRINTF_FL(
		"[%02d] "
		"P=(%.1f,%.1f)m\t "
		"V=(%.1f,%.1f)m\t" 
		"Bat=%.1f%%\n" , 
			node, 
			layout_ptr->state[node].position.X,
			layout_ptr->state[node].position.Y ,
			layout_ptr->state[node].velocity.X,
			layout_ptr->state[node].velocity.Y ,
			layout_ptr->state[node].battery 
			) ;
}


void layout_printPositions(void)
{
	IF_INIT_RETURN();
	for (int idx=0; idx < layout_ptr->ndrones; idx++)
		printf("Pos[%02d]=(%.3f,%.3f)m " , 
			idx, layout_ptr->state[idx].position.X , layout_ptr->state[idx].position.Y ) ;
	printf("\n");
}

/* GET STUFF: */

/**
 * get distance between two drones 
 * TODO: ignoring Z 
 * **/
float layout_getDistance( int node1,  int node2 )
{
	IF_INIT_RETURN(-1);
	IF_NODE_RETURN(node1,-1);
	IF_NODE_RETURN(node2,-1);
	float tmp = (layout_ptr->state[node1].position.X -
	layout_ptr->state[node2].position.X ) * 
	(layout_ptr->state[node1].position.X -
	layout_ptr->state[node2].position.X ) +
	(layout_ptr->state[node1].position.Y -
	layout_ptr->state[node2].position.Y ) *
	(layout_ptr->state[node1].position.Y -
	layout_ptr->state[node2].position.Y ) ;
	
	return sqrt(tmp) ;
}

/* get yaw from one drone - degrees */
float layout_getYaw( int node )
{
	IF_INIT_RETURN(-1);
	IF_NODE_RETURN(node,-1);
	return layout_ptr->state[node].yaw_degrees ;
}

/* get altitude from one drone - m */
float layout_getAltitude( int node )
{
	IF_INIT_RETURN(-1);
	IF_NODE_RETURN(node,-1);
	return layout_ptr->state[node].position.Z ;
}

/* get position from one drone */
triplet_t layout_getPosition( int node )
{
	IF_INIT_RETURN(triplet0);
	IF_NODE_RETURN(node,triplet0);
	return layout_ptr->state[node].position ;
}

/* get velocity from one drone */
triplet_t layout_getVelocity( int node )
{
	IF_INIT_RETURN(triplet0);
	IF_NODE_RETURN(node,triplet0);
	return layout_ptr->state[node].velocity ;
}


int layout_getBattery( int node )
{
	IF_INIT_RETURN(-1);
	IF_NODE_RETURN(node,-1);
	return layout_ptr->state[node].battery;
}


/* sensor related functions : */
isensor_reading_t layout_getIntReading( int node )
{
	isensor_reading_t tmp_reading ={0}; 
	IF_INIT_RETURN(tmp_reading);
	IF_NODE_RETURN(node,tmp_reading);
	

	tmp_reading.battery = layout_getBattery(node);
	tmp_reading.state = 0 ;

	float z1,z2;
	normalrand(&z1,&z2);
	tmp_reading.heading_degrees = layout_getYaw( node ) ; /* get the acurate value */
	tmp_reading.heading_degrees += z1 * YAW_ERROR ; /* add some gaussian error */
	
	tmp_reading.ultra_altitude = layout_getAltitude( node ) ; /* get the acurate value */
	tmp_reading.ultra_altitude += z2 * ULTRASOUND_ERROR ; /* add some gaussian error */
	
	return tmp_reading ;
}

/* return a NED gps position*/
rtk_reading_t layout_getRTKReading( int node )
{
	rtk_reading_t tmp_reading={0}; 
	IF_INIT_RETURN(tmp_reading);
	IF_NODE_RETURN(node,tmp_reading);
		
	/* positions are int32 - values in mm */
	static uint32_t sample_id = 0;
	sample_id++;
	float z1,z2;
	normalrand(&z1,&z2);
	triplet_t position = layout_getPosition( node ) ;
	tmp_reading.east		= (int32_t)(1000.0*(position.X + z1 * GPS_ERROR )) ;
	tmp_reading.north		= (int32_t)(1000.0*(position.Y + z2 * GPS_ERROR )) ;
	tmp_reading.down		= (int32_t)(1000.0*(position.Z )) ;
	
	triplet_t vel = layout_getVelocity( node ) ;
	normalrand(&z1,&z2);
	tmp_reading.vel_north	= (int32_t)(1000.0*(vel.Y + z1 * GPS_ERROR )) ;
	tmp_reading.vel_east	= (int32_t)(1000.0*(vel.X + z2 * GPS_ERROR )) ;
	tmp_reading.vel_down	= (int32_t)(1000.0*(vel.Z )) ;
	
	tmp_reading.rtk_sample_id = sample_id;
	tmp_reading.rtk_num_sats = 99;
	tmp_reading.spp_sample_id = sample_id;; 
	tmp_reading.spp_num_sats = 99;
	tmp_reading.latitude 	= 1e6*(BSLAT + position.Y / LAT_RADIUS/M_PI*180);
	tmp_reading.longitude 	= 1e6*(BSLON + position.X / LON_RADIUS/M_PI*180) ; // times 1million

	return tmp_reading ;
}

/* get the whole layout struct */
//layout_t getLayout()
//{
	//return layout ;
//}

/*get a wind speed given a seed (usually time) */
triplet_t get_wind_vel(float t) 
{
	float abs = 1.0 + 1.0*sinf(2.0*M_PI*t/30.0);
	float ang = M_PI/2.0 + M_PI/6.0*sinf(2.0*M_PI*t/60.0); // 90º + sin
	triplet_t wind;
	wind.X = abs*cosf(ang);
	wind.Y = abs*sinf(ang);
	//printf("wind (t=%.0fs)=%.1f.ang(%.1frad){%.1f,%.1f}\n",
		//t,
		//abs,ang,
		//wind.X,wind.Y);
	// constant wind:
	//wind.X = 3; //m/s
	//wind.Y = 0;
	return wind;
}

/**
 * Read init layout file , and set it. 
 * this runs once <=> inline 
 * **/
static int layout_loadInitPosition(int ndrones)
{
	char buf[16] ; 
	FILE *file ;
	float x, y ; /* save here the positions read from file */
	char *ret;
	file = fopen( LAYOUT_INIT_POSITIONS_FILENAME , "r" ) ;
	if (NULL == file)
	{
		PRINTF_FL_ERR( "Can't find file: "LAYOUT_INIT_POSITIONS_FILENAME", to load init pos\n" ) ;
		return NOTOKAY ;
	}


	for (int id=0; id<ndrones; id++)
	{
		
		ret = fgets(buf, sizeof(buf), file); /* read a line */
		if (NULL == ret)/* number of lines in the file less than num_drones */
		{
			PRINTF_FL_ERR("failed loading layout of uav #%d\n",id);
			return NOTOKAY ;
		}
			
		if (2 != sscanf(buf, "%f %f", &x, &y ) ) /* parse its data */
		{
			PRINTF_FL_ERR("failed loading layout of uav #%d\n",id);
			return NOTOKAY ;
		}
		
		layout_ptr->state[id].position.X = x ; 
		layout_ptr->state[id].position.Y = y ;
		layout_ptr->state[id].position.Z = INITHEIGHT ; /* height */
		layout_ptr->state[id].yaw_degrees = 0 ;
		layout_ptr->state[id].battery = 100 ;
		layout_ptr->state[id].velocity.Y = 1 ; /* m/s */
		layout_ptr->state[id].status= FLYING;

	}
	fclose(file);
	return OKAY;

}


