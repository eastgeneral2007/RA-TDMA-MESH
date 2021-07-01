/**
 * internal_sensor_raw.c
 */


#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <math.h>


#include "drk/internal_sensor_raw.h"
#include "drk/vector.h"
#include "drk/navdata.h"
#include "drk/utils.h" /* printfl_err */
#include "drk/vector.h"


#ifndef SENSOR_LOADED_CONDITION
#define SENSOR_LOADED_CONDITION \
	if (Navdata_initiated == 0) return 
#endif

#define PREFIX 	"ISR"

#define MAXLINE					(3048) /* packet of navdata received */
#define PRESSURE_AT_SEA_LEVEL	101325 /* 101325 Pa - Pressure at sea level */


/* ************
 * Declarations  
 * ***************/
float tcHeading( drone_sensor_t * buf ) ;
/* Thread to read the incoming navdata from the drone */
//void* _drk_ar_nav_sensor_thread(void* args);
 
 
 
 
/******************
 * Externs
 *****************/
extern struct global_actuator_*	actuator_buf;
extern int 						Navdata_initiated ; /* flag module initated or not - in ISA */
extern uint8_t 					global_exit_all_threads ; /* drk.c */
extern int 						drone_sockfd_nav ;


/*******************************************************************************
 * Globals
 ******************************************************************************/
struct global_navdata_	 		*navdata_buf;
struct offsets_ 				Offset ={0,0,0};

/* globals for max min magnetometer */
int x_max=0;
int x_min=0;
int y_max=0;
int y_min=0;
int z_max=0;
int z_min=0;





/* Thread-safe pitch accessor */
float drk_drone_pitch_get(void )
{
	SENSOR_LOADED_CONDITION -1;
	sem_wait(navdata_buf->semaphore);
	float pitch =  (navdata_buf->sensor).pitch;
	sem_post(navdata_buf->semaphore);
	return pitch;// - Pitch_offset;
}

/* Thread-safe roll accessor */
float drk_drone_roll_get( void )
{
	SENSOR_LOADED_CONDITION -1;
	
	sem_wait(navdata_buf->semaphore);
	float roll =  (navdata_buf->sensor).roll;
	sem_post(navdata_buf->semaphore);
	return roll;// - Roll_offset;
}

/* Thread-safe yaw accessor. This is RELATIVE! TODO: this is the gyro output */
float drk_drone_yaw_get(void )
{
	SENSOR_LOADED_CONDITION -1;
	sem_wait(navdata_buf->semaphore);
	float yaw = (navdata_buf->sensor).yaw;
	sem_post(navdata_buf->semaphore);
	return yaw;
}





vector_t drk_compass_data(void)
{
	
	vector_t mag;
	SENSOR_LOADED_CONDITION mag;
	sem_wait(navdata_buf->semaphore);
	mag.x = (navdata_buf->sensor).mag_x;
	mag.y = (navdata_buf->sensor).mag_y;
	mag.z = (navdata_buf->sensor).mag_z;
	sem_post(navdata_buf->semaphore);
	return mag ;
}







///* get magnetic raw values */
//void drk_magneto_raw( double* const magx, double* const magy, double* const magz )
//{
	//sem_wait(navdata_buf->semaphore);
	//(*magx) = (double)((navdata_buf->sensor).mag_x);
	//(*magy) = (double)((navdata_buf->sensor).mag_y);
	//(*magz) = (double)((navdata_buf->sensor).mag_z);
	//sem_post(navdata_buf->semaphore);
//}




/* heading (degrees) Tilt Compensated  */
float tcHeading( drone_sensor_t * buf )
{
	if( x_max == 0 && x_min == 0 && 
		y_max == 0 && y_min ==0 && 
		z_max == 0 && z_min ==0 )
			drk_calibration_data_get() ;
	
	vector_t mag = { 
		(double)(buf->mag_x),
		(double)(buf->mag_y),
		(double)(buf->mag_z) } ;
	vector_t from = {0,-1,0};

	mag.x = (mag.x-x_min) / (x_max - x_min) * 2 - 1.0;
	mag.y = (mag.y-y_min) / (y_max - y_min) * 2 - 1.0;
	mag.z = (mag.z-z_min) / (z_max - z_min) * 2 - 1.0;

	raw_acc_sensors_t raw = buf->raw_vals ;
	vector_t acc = {raw.phys_acc_x, raw.phys_acc_y, raw.phys_acc_z};

	vector_t acc_norm = acc;
	vector_normalize(&acc_norm) ;

	vector_t E;
	vector_t N;
	vector_cross(&mag, &acc_norm, &E) ;
	vector_normalize(&E) ;
	vector_cross(&acc_norm, &E, &N) ;

 
	float heading = 
		atan2f( vector_dot(&E,&from), vector_dot(&N,&from) ) *
		180.0 / M_PI ;
	heading += buf->offset.compass ;
	heading = wrapTo360f( heading ) ;
	return heading ;
}


/* returns fusion heading + offset */
float drk_fusion_heading(void)
{
	SENSOR_LOADED_CONDITION -1;
	sem_wait(navdata_buf->semaphore);
	float heading = (navdata_buf->sensor).mag_heading_fusion + 
		(navdata_buf->sensor).offset.fusion  ;
	sem_post(navdata_buf->semaphore);
	return wrapTo360( heading );
}

/* magnetic */
float drk_compass_heading( void )
{
	SENSOR_LOADED_CONDITION -1;
	sem_wait(navdata_buf->semaphore);
	float heading = (navdata_buf->sensor).mag_heading + 
		(navdata_buf->sensor).offset.compass  ;
	sem_post(navdata_buf->semaphore);
	return wrapTo360( heading  );
}

/* magnetic heading , compensated for the current pitch and roll */
float drk_tc_compass_heading( void ) 
{
	SENSOR_LOADED_CONDITION -1;
	sem_wait(navdata_buf->semaphore);
	/* get heading , function uses accels and magnemoter */
	float heading = tcHeading( &(navdata_buf->sensor) ); 
	sem_post(navdata_buf->semaphore);
	return heading;
}

///* heading (degrees) compensated for tilt */
//double drk_compass_heading(void)
//{
	//if(	x_max == 0 && x_min == 0 && 
		//y_max == 0 && y_min == 0 && 
		//z_max == 0 && z_min == 0 )
			//drk_calibration_data_get();

	//sem_wait(navdata_buf->semaphore);
	//vector_t mag = {  (double)((navdata_buf->sensor).mag_x),
				  //(double)((navdata_buf->sensor).mag_y),
				  //(double)((navdata_buf->sensor).mag_z)};
	//Offset.compass = (navdata_buf->sensor).offset.compass;
	//sem_post(navdata_buf->semaphore);
	////vector_t from = {0,-1,0};

	//mag.x = (mag.x-x_min) / (x_max - x_min) ;//* 2 - 1.0;
	//mag.y = (mag.y-y_min) / (y_max - y_min) ;//* 2 - 1.0;
	//mag.z = (mag.z-z_min) / (z_max - z_min) ;//* 2 - 1.0;
	////vector_normalize(&mag);//luis

	//vector_t acc = {0,0,1}; // not the acelerometer

	//vector_t acc_norm = acc;
	//vector_normalize(&acc_norm);

	//vector_t E;
	//vector_t N;
	//vector_cross(&mag, &acc_norm, &E);
	//vector_normalize(&E);
	//vector_cross(&acc_norm, &E, &N);

	////double heading = atan2(vector_dot(&E,&from), vector_dot(&N,&from)) * 180.0/M_PI;
	//double heading = atan2(mag.x, mag.y) * 180.0/M_PI;
	////heading += Offset.compass;
	////if(heading < 0) heading += 360;
	////heading = fmod(heading,360);
	//return heading;
//}


/* todo: missing description how it works */
double drk_mag_filtered_heading(void)
{

	SENSOR_LOADED_CONDITION -1;
	/* Grab a copy of the sensor data */
	drone_sensor_t temp_sensor;
	sem_wait(navdata_buf->semaphore);
	memcpy(&temp_sensor, &(navdata_buf->sensor), sizeof(temp_sensor));
	sem_post(navdata_buf->semaphore);


	double max_heading = -1, min_heading = 400;

	/* find lowest and highest sample value */
	for (int i = 0; i < MAG_FILTER_SAMPLES; i++)
	{
		min_heading = MIN(min_heading, temp_sensor.mag_heading_samples[i]);
		max_heading = MAX(max_heading, temp_sensor.mag_heading_samples[i]);
	}

	
	int low_shift = 0;
	if (min_heading < 60 && max_heading > 300)
		low_shift = 1;

	double sum = 0, sample;
	for (int i = 0; i < MAG_FILTER_SAMPLES; i++)
	{
		sample = temp_sensor.mag_heading_samples[i];
		if (low_shift && sample < 180)
			sample += 360;
		sum += sample;
	}
	sum /= MAG_FILTER_SAMPLES;
	sum = wrapTo360( sum ) ;
	return sum;
}



// TODO remove globals and use the sensor struct 
/* Tells the drone that its current heading is north, for calibration */
void drk_calibrate_north(void)
{
	
	double samples = 0, n_samples = 500 ;
	double compass = 0.0, gyro = 0.0, fusion = 0.0;
	Offset.compass = 0.0;
	Offset.gyro = 0.0 ;
	Offset.fusion = 0.0 ;
	printf(
		"Averaging %lf samples to calibrate compass and gyro.\n", 
		n_samples ) ;
	sleep(1);
	while (samples < n_samples )
	{
		compass += drk_mag_filtered_heading();
		fusion += drk_fusion_heading();
		gyro += drk_gyro_heading();
		samples++;
		usleep(10000);
	}
	Offset.compass 	= -1 * compass 	/ samples;
	Offset.gyro 	= -1 * gyro 	/ samples;
	Offset.fusion 	= -1 * fusion 	/ samples;
	printf(
		"Offsets:\nGyro\t%5.1f\nCompass\t%5.1f\nFusion\t%5.1f\n",
		Offset.gyro, Offset.compass, Offset.fusion);
	drk_set_heading_offsets(Offset.gyro, Offset.fusion, Offset.compass);
	sleep(1); /* todo: why this */
	//drk_calibration_data_set(x_max, x_min, y_max, y_min, z_max, z_min,
	//  Offset.compass, Offset.gyro);
}

/* Gyro heading, value compensated with offset */
float drk_gyro_heading(void)
{
	return wrapTo360f( drk_drone_yaw_get() + Offset.gyro );
}

/* Zero the gyro by assuming your current heading is north  */
void drk_gyro_zero(void)
{
	Offset.gyro = -1.0 * drk_drone_yaw_get();
	printf("Gyro calibrated by current position. Offset: %lfº\n", 
		Offset.gyro);
}

/* Calibrate the gyro offset, assuming the compass is correct */
void drk_gyro_calibrate(void)
{
	int total = 0, samples = 0;

	printf("Sampling compass data for 5s to calibrate the rate gyro\n");    
	time_t start_time = time(NULL);
	time_t now_time = time(NULL);
	while ( difftime(now_time, start_time) < 5)
	{
		total += drk_compass_heading();
		samples++;
		usleep(100000);
		now_time = time(NULL);
	}

	Offset.gyro = ((total / samples) - drk_drone_yaw_get());
	printf("Gyro calibrated by compass. Offset: %lf\n", Offset.gyro);
}

/* COMPASS CALIBRATION stuff */
void drk_calibration_data_set(int xx, int xn, int yx, int yn, 
	int zx, int zn, int offset, int g_offset )
{
	FILE* fp;
	fp = fopen("compass_calibration.txt", "w");
	fprintf(fp, "x_max:%d\nx_min:%d\ny_max:%d\ny_min:%d\nz_max:%d\nz_min:%d\noffset:%d\ngoffset:%d\n",
		xx, xn, yx, yn, zx, zn, offset, g_offset);
	fclose(fp);
}

#define EXP_MATCHES		8
int drk_calibration_data_get(void)
{
	FILE *fp = NULL;
	fp = fopen("compass_calibration.txt", "r");
	if(fp == NULL)
	{
		PRINTF_FL_ERR( "Unable to open compass calbiration file\n");
		drk_calibration_data_set(0,0,0,0,0,0,0,0);
		return E_FILE_NOT_FOUND;
	}
	int matches = 0;
	int values[EXP_MATCHES];
	char buf[30];

	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "x_max:%d\n", &values[0]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "x_min:%d\n", &values[1]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "y_max:%d\n", &values[2]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "y_min:%d\n", &values[3]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "z_max:%d\n", &values[4]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "z_min:%d\n", &values[5]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "offset:%d\n", &values[6]);
	if(fgets(buf,sizeof(buf), fp) != NULL)
		matches += sscanf(buf, "goffset:%d\n", &values[7]);
	fclose(fp);

	if( matches != EXP_MATCHES )
	{
		PRINTF_FL_ERR("Incomplete file, only matched [%d/8] params\n", matches);
		return E_INVALID_INPUT ;
	}
	else
	{
		x_max = values[0];
		x_min = values[1];
		y_max = values[2];
		y_min = values[3];
		z_max = values[4];
		z_min = values[5];
	}
	return E_SUCCESS ;

}


raw_acc_sensors_t drk_get_raw_acc_sensors( void )
{
	
	/* Grab a copy of the sensor data */
	drone_sensor_t temp_sensor={0};
	SENSOR_LOADED_CONDITION temp_sensor.raw_vals;
	sem_wait(navdata_buf->semaphore);
	memcpy(&temp_sensor, &(navdata_buf->sensor), sizeof(temp_sensor));
	sem_post(navdata_buf->semaphore);

	return temp_sensor.raw_vals;
}

void drk_set_heading_offsets(double ogyro, double ofusion, double ocompass)
{
	SENSOR_LOADED_CONDITION;
	sem_wait(navdata_buf->semaphore);
	(navdata_buf->sensor).offset.compass 	= ocompass;
	(navdata_buf->sensor).offset.gyro 		= ogyro;
	(navdata_buf->sensor).offset.fusion 	= ofusion;
	sem_post(navdata_buf->semaphore);   
}


/* Grab an int from the navdata stream */
int drk_sensor_data_get_int(char* data, int offset)
{   
	if ( offset-3 >= MAXLINE ) /* data points to a array of MAXLINE chars */
		PRINTF_FL_ERR("Segmentation fault prevented! \n");
	int i, tmp = 0, n = 0;
	for (i = 3; i >= 0; i--)
	{ 
		n <<= 8;
		tmp = data[offset + i] & 0xFF;
		n |= tmp;
	}
	return n;
}

/* Grab a short from the navdata stream */
short drk_sensor_data_get_short(char* data, int offset)
{   
	short i, tmp = 0, n = 0;
	for (i = 1; i >= 0; i--)
	{
		n <<= 8;
		tmp = data[offset + i] & 0xFF;
		n |= tmp;
	}
	return n;   
}


/* Grab a float from the navdata stream */
float drk_sensor_data_get_float(char* data, int offset)
{       
	float result = 0.0;
	int bits = drk_sensor_data_get_int(data, offset);

	int s = ((bits >> 31) == 0) ? 1 : -1;
	int e = ((bits >> 23) & 0xff);
	int m = (e == 0) ?
	(bits & 0x7fffff) << 1 :
	(bits & 0x7fffff) | 0x800000; 

	int temp = e-150;
	int neg_temp = -temp;
	float neg = 1.0/(1<<(neg_temp));        
	float power = (temp < 0) ? neg : (1<<temp);

	result = s*m*power; 
	return result;
}

/*******************************************************************************
                STRUCT _UPDATERS_
*******************************************************************************/

/**
 * drk_sensor_data_update function, receiving the nav_data from the socket,
 * and save the value into a struct
 *      intput:  the pointer of the drone_sensor struct;
 *      output:  <0 for failure, >0 for success.
 **/
int drk_sensor_data_update( 
	drone_sensor_t* const p, 
	struct vision_tags_* const vt )
{
	
	char nav_data[MAXLINE];
#ifndef SIMULATOR
	int block_ptr;
	unsigned short block_tag ;
	unsigned short block_size;
	int32_t pressure ;
	const double p0 = PRESSURE_AT_SEA_LEVEL ; 
#endif

	if (NULL == p)
	{
		PRINTF_FL_ERR("Need a valid pointer\n");
		return -2 ;
	}


	p->state = 1 ;
	(void)(vt); /* suppress warning */



	
	CLEAR(nav_data) ;
	
	if ( global_exit_all_threads == 1 ) return -1 ;

	#ifndef SIMULATOR
	#ifdef VERBOSE
	int n = 
	#endif 
	#endif

	recvfrom( drone_sockfd_nav, nav_data, MAXLINE, 0, NULL, NULL);   
	//PRINTF_FL( "recvfrom %dB\n" ,n );
	if ( global_exit_all_threads == 1 ) return -1 ;
	
#ifndef SIMULATOR
	#ifdef VERBOSE
	/* Restart if bad navdata was received 
	 * This size is explicitely saying that we should 
	 * restart connections */
	if(n == 24) { //TODO: maybe this should be <= 24 ?
		PRINTF_FL_ERR( "Error: recvd 24 bytes NAV DATA\n" ) ;
		//printf("Reconnecting?\n");
		//close(drone_sockfd_nav);
		//close(drone_sockfd_at);       
		//drk_sensor_data_init();
		return -1;
	}
	#endif
	
	
	p->state	= drk_sensor_data_get_int(nav_data, NAV_HEADER_STATE);
	p->sequence	= drk_sensor_data_get_int(nav_data, NAV_HEADER_SEQUENCE);

	block_ptr = HEADER_OFFSET;
	block_tag  = drk_sensor_data_get_short(nav_data, block_ptr + TAG_OFFSET); 
	block_size = drk_sensor_data_get_short(nav_data, block_ptr + SIZE_OFFSET);



	//uint32_t up;
	//uint16_t ut;
	//int32_t temperature;
	
	//double altitude;


	//float phys_acc[3];
	//uint16_t raw_acc[3];

    /* Iterate through navdata blocks and parse data */
    // PRINTF_FL(  "line %d\n" , __LINE__ );
    //PRINTF_FL("NAV_CHECKSUM_TAG = %d \n", NAV_CHECKSUM_TAG);
   // PRINTF_FL("Block tag: %d, block_size %d \n", block_tag, block_size);
	while ( block_tag != NAV_CHECKSUM_TAG && block_size != 0)
	{
		
		switch ( block_tag )
		{
			case NAV_DEMO_TAG:     
				//PRINTF_FL("NAV_DEMO_TAG \n");
				if (block_size != NAV_DEMO_SIZE)
				{
					PRINTF_FL_ERR("Block NAV_DEMO (%d) didn't match expected (%d)\n",
							block_size, NAV_DEMO_SIZE);
							break;
				}
				/* Parse general nav data */
				p->battery  = drk_sensor_data_get_int(nav_data, block_ptr + NAV_DEMO_BATTERY);
				p->altitude = (((float)(drk_sensor_data_get_int(nav_data, block_ptr + NAV_DEMO_ALTITUDE)))/1000.0);
				p->pitch    = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_PITCH)/1000.0;
				p->roll     = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_ROLL)/1000.0;
				p->yaw      = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_YAW)/1000.0;
				p->vx       = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_VX)/1000.0;
				p->vy       = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_VY)/1000.0;
				p->vz       = drk_sensor_data_get_float(nav_data, block_ptr + NAV_DEMO_VZ)/1000.0;
				if (isnan(p->altitude) || isnan(p->pitch) ||
					isnan(p->roll) || isnan(p->yaw)) return -1 ;
				break;
				
			case NAV_VIS_DETECT_TAG:    // 16
				PRINTF_FL("NAV_VIS_DETECT_TAG\n");
				/*if (block_size != NAV_VIS_DETECT_SIZE) {
					printf("Block NAV_VIS_DETECT (%d) didn't match expected (%d)\n",
							block_size, NAV_VIS_DETECT_SIZE);
					break;
				}*/
				break;
				/* Vision variables */
				/*vt->number = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_NB_DETECTED);
				
				(vt->type)[0]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_TYPE1);
				(vt->x)[0]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_XC1);
				(vt->y)[0]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_YC1);
				(vt->height)[0] = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_WIDTH1);
				(vt->width)[0]  = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_HEIGHT1);
				(vt->dist)[0]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_DIST1);
				(vt->angle)[0]  = drk_sensor_data_get_float(nav_data, block_ptr + NAV_VIS_DETECT_ANGLE1);

				(vt->type)[1]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_TYPE2);
				(vt->x)[1]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_XC2);
				(vt->y)[1]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_YC2);
				(vt->height)[1] = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_WIDTH2);
				(vt->width)[1]  = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_HEIGHT2);
				(vt->dist)[1]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_DIST2);
				(vt->angle)[1]  = drk_sensor_data_get_float(nav_data, block_ptr + NAV_VIS_DETECT_ANGLE2);

				(vt->type)[2]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_TYPE3);
				(vt->x)[2]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_XC3);
				(vt->y)[2]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_YC3);
				(vt->height)[2] = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_WIDTH3);
				(vt->width)[2]  = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_HEIGHT3);
				(vt->dist)[2]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_DIST3);
				(vt->angle)[2]  = drk_sensor_data_get_float(nav_data, block_ptr + NAV_VIS_DETECT_ANGLE3);

				(vt->type)[3]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_TYPE4);
				(vt->x)[3]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_XC4);
				(vt->y)[3]      = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_YC4);
				(vt->height)[3] = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_WIDTH4);
				(vt->width)[3]  = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_HEIGHT4);
				(vt->dist)[3]   = drk_sensor_data_get_int(nav_data, block_ptr + NAV_VIS_DETECT_DIST4);
				(vt->angle)[3]  = drk_sensor_data_get_float(nav_data, block_ptr + NAV_VIS_DETECT_ANGLE4);
				*/
				//break;
			
			case NAV_MAGNETO_TAG: // 22
				//PRINTF_FL("NAV_MAGNETO_TAG \n"); // magnetic information
				p->mag_x = drk_sensor_data_get_float(nav_data, block_ptr + NAV_MAGNETO_MAG_X);
				p->mag_y = drk_sensor_data_get_float(nav_data, block_ptr + NAV_MAGNETO_MAG_Y);
				p->mag_z = drk_sensor_data_get_float(nav_data, block_ptr + NAV_MAGNETO_MAG_Z);
				p->mag_heading = 
					drk_sensor_data_get_float(nav_data, block_ptr 
					+ NAV_MAGNETO_HEADING);
				p->mag_heading_gyro = 
					drk_sensor_data_get_float(nav_data, block_ptr 
					+ NAV_MAGNETO_HEADING_GYRO);
				p->mag_heading_fusion = 
					drk_sensor_data_get_float(nav_data, block_ptr 
					+ NAV_MAGNETO_HEADING_FUSION);
				//printf("Heading [%6.2f]\tGyro [%6.2f]\tFusion [%6.2f]\n",
				//  p->mag_heading, p->mag_heading_gyro, p->mag_heading_fusion);
				break;

			case NAV_PRESSURE_RAW_TAG:
				//PRINTF_FL("NAV_PRESSURE_RAW_TAG \n");
				//up = drk_sensor_data_get_int(nav_data, block_ptr + NAV_PRESSURE_RAW_UP);
				//ut = drk_sensor_data_get_int(nav_data, block_ptr + NAV_PRESSURE_RAW_UT);
				//temperature = drk_sensor_data_get_int(nav_data, block_ptr + NAV_PRESSURE_RAW_TEMP);
				pressure = drk_sensor_data_get_int(nav_data, block_ptr + NAV_PRESSURE_RAW_PRESS);
				p->absolute_altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));
				break ;
				//printf("Pressure: %d\tTemperature: %d\tAltitude: %7.2f\n", pressure, temperature, altitude);

			case NAV_KALMAN_PRESSURE_TAG: // 24;
				PRINTF_FL("NAV_KALMAN_PRESSURE_TAG\n");
				/*kalman_pressure = drk_sensor_data_get_float(nav_data, block_ptr + NAV_KALMAN_OFFSET_PRESSURE);
				kalman_z = drk_sensor_data_get_float(nav_data, block_ptr + NAV_KALMAN_EST_Z);
				kalman_us = drk_sensor_data_get_float(nav_data, block_ptr + NAV_KALMAN_OFFSET_US);
				printf("Pressure [%6.2f]\tEst Z [%6.2f]\tUS [%6.2f]\n",
					kalman_pressure, kalman_z, kalman_us);*/
				break;
			
			case NAV_PHYS_MEAS_TAG:
				//PRINTF_FL("NAV_PHYS_MEAS_TAG\n");
				(p->raw_vals).phys_acc_x = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_X);
				(p->raw_vals).phys_acc_y = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_Y);
				(p->raw_vals).phys_acc_z = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_Z);

				/*
				phys_acc[0] = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_X);
				phys_acc[1] = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_Y);
				phys_acc[2] = drk_sensor_data_get_float(nav_data, block_ptr + NAV_PHYS_MEAS_ACC_Z);
				printf("Phys measures: %6.2f\t%6.2f\t%6.2f\n", phys_acc[0], phys_acc[1], phys_acc[2]);
				*/
				break;

			case NAV_RAW_MEAS_TAG:
				//PRINTF_FL("NAV_RAW_MEAS_TAG\n");
				(p->raw_vals).raw_acc_x = (int) drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_X);
				(p->raw_vals).raw_acc_y = (int) drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_Y);
				(p->raw_vals).raw_acc_z = (int) drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_Z);
				//printf("Test: %d\n", p->raw_vals.raw_acc_x);
				/*
				raw_acc[0] = drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_X);
				raw_acc[1] = drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_Y);
				raw_acc[2] = drk_sensor_data_get_short(nav_data, block_ptr + NAV_RAW_MEAS_ACC_Z);
				printf("Raw measures: %d\t%d\t%d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
				*/
				break;

			case NAV_CHECKSUM_TAG:      // 0xFF
				//PRINTF_FL("NAV_CHECKSUM_TAG\n") ;
				break;
				
			default:
				PRINTF_FL("Unrecognized block ID\n");
				break;
		}
		
		/* Advance to the next navdata struct */
		block_ptr += block_size;
		block_tag  = drk_sensor_data_get_short(nav_data, block_ptr + TAG_OFFSET); 
		block_size = drk_sensor_data_get_short(nav_data, block_ptr + SIZE_OFFSET);
		//PRINTF_FL("Block tag: %d, block_size %d \n", block_tag, block_size);
	} 
    
	//    fprintf(navfile, "7\t%f\n", drk_elapsed_time_secs());   
	
#else //#ifndef SIMULATOR:
	/* for simulator this is the code we get from UAVworld */
	//PRINTF_FL(  "Got %dB from Sensor-Simulator: <%s>\n", n, (char*)nav_data  );
	unsigned int a;
	//int b ;
	//float c, d ;
	sscanf(
		nav_data ,
		"state=%u,battery=%d,compass=%f,ultra=%f" , 
		&a,&(p->battery),&(p->mag_heading),&(p->altitude)  ) ;
	p->state=a;
	//p->battery  = 100 ;
	//p->altitude = 3 ; /* ultrasound */
	//p->mag_heading_gyro = p->mag_heading ;
	//p->mag_heading_fusion = p->mag_heading ;
	p->absolute_altitude = 103 ;
	p->pitch    = 0 ;
	p->roll     = 0 ;
	p->yaw      = 0 ;
	p->vx       = 0 ;
	p->vy       = 0 ;
	p->vz       = 0 ;
	(p->raw_vals).phys_acc_z = 0 ;
	(p->raw_vals).raw_acc_x = 0 ;
	#endif //#ifndef SIMULATOR

	return 1;
}

