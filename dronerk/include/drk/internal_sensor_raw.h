/**
 * internal_sensor_raw.h
 */
#ifndef _INTERNAL_SENSOR_RAW_H_
#define _INTERNAL_SENSOR_RAW_H_

#include <stdint.h>
#include <semaphore.h>

//#include "internal_sensor_api.h"
#include "vector.h"

#define MAG_FILTER_SAMPLES (14)


struct offsets_
{
	float gyro ;
	float compass ;
	float fusion ;
	//double pitch 	= 0;
	//double roll 	= 0;
} ;



/* Logging unfiltered values for the paper */
typedef struct 
{
    int     raw_acc_x;
    int     raw_acc_y;
    int     raw_acc_z;
    float   phys_acc_x;
    float   phys_acc_y;
    float   phys_acc_z;
} raw_acc_sensors_t;



/* used for updating NAV_DEMO_data */
typedef struct 
{
	uint32_t 	state;  
	uint32_t 	sequence;
	int     	battery;
	float   	altitude;
	float   	absolute_altitude;  
	float   	pitch;      
	float   	roll;  
	float   	yaw;
	float   	vx; /* UAV’s estimated linear velocity */
	float   	vy; /* UAV’s estimated linear velocity */
	float   	vz; /* UAV’s estimated linear velocity */
	float   			mag_x;
	float   			mag_y;
	float  				mag_z;
	float   			mag_heading;
	float   			mag_heading_gyro;
	float   			mag_heading_fusion;
	struct offsets_		offset ;
	int					mag_filter_index;
	float				mag_heading_samples[MAG_FILTER_SAMPLES];
	raw_acc_sensors_t 	raw_vals;
} drone_sensor_t; 

struct vision_tags_
{
	int 	number;
	int 	type[4];
	int 	x[4];
	int 	y[4];
	int 	width[4];
	int 	height[4];
	int 	dist[4];
	float 	angle[4];
};

struct global_navdata_
{
	sem_t					*semaphore ;
	drone_sensor_t 	sensor ;
	struct vision_tags_ 	vision ;
};




/* Thread-safe pitch accessor */
float drk_drone_pitch_get(void);

/* Thread-safe roll accessor */
float drk_drone_roll_get(void);

/* Thread-safe yaw accessor. */
float drk_drone_yaw_get(void);





vector_t drk_compass_data(void); /* raw magnetic values */

/**********
 * HEADING functions 
 * *********/
double drk_mag_filtered_heading(void);


//void drk_magneto_raw(double* magx, double* magy, double* magz);

void drk_set_heading_offsets(double ogyro, double ofusion, double ocompass);

/* Tells the drone that its current heading is north, for calibration */
void drk_calibrate_north(void);

// Yaw value compensated for offset from compass, 
// Gyroscope , ABSOLUTE if calibrated 
float drk_gyro_heading(void);
float drk_fusion_heading(void);
float drk_compass_heading(void);

/* magnetic heading , compensated for the current pitch and roll */
float drk_tc_compass_heading(void) ;

// Zero the gyro by assuming your current heading is north 
void drk_gyro_zero(void);

// Calibrate the gyro offset, assuming the compass is correct 
void drk_gyro_calibrate(void);

/* COMPASS CALIBRATION stuff */
void drk_calibration_data_set(int xx, int xn, int yx, int yn, int zx, int zn, int offset, int g_offset);
int drk_calibration_data_get(void);

/*******************************************************************************
                                UTILITIES
*******************************************************************************/



/* Prints and shows a graph of the battery */
void 	drk_print_battery(void);
/* return % of bat left */
int 	drk_get_battery(void);

/* return struct with sensor data */
raw_acc_sensors_t drk_get_raw_acc_sensors(void);

int 	drk_sensor_data_update(
	drone_sensor_t* const p, 
	struct vision_tags_ * const vt ) ;

int 	drk_sensor_data_get_int(char *data, int offset);
short 	drk_sensor_data_get_short(char *data, int offset);
float 	drk_sensor_data_get_float(char *data, int offset);

#endif // _INTERNAL_SENSOR_RAW_H_
