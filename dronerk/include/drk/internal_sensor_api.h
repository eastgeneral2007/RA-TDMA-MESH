/**
 *  internal_sensor_api.h
 * 	use these functions to: receive and parse navdata,  etc
 */

#ifndef _INTERNAL_SENSOR_API_H_
#define _INTERNAL_SENSOR_API_H_
 
#include <semaphore.h>
#include <inttypes.h>


#include "drk/vector.h"



//struct compass_
//{
	//float mag_x;
	//float mag_y;
	//float mag_z;
//};



/*******************************************************************************
                                USEFUL ENUMS
*******************************************************************************/

/* Vision tag types */
enum TAG_TYPE
{
    TAG_TYPE_NONE             = 0,
    TAG_TYPE_SHELL_TAG        = 1,
    TAG_TYPE_ROUNDEL          = 2,
    TAG_TYPE_ORIENTED_ROUNDEL = 4,
    TAG_TYPE_STRIPE           = 8,
    TAG_TYPE_NUM
} TAG_TYPE;

/* Detection types */
enum CAD_TYPE
{
    CAD_TYPE_HORIZONTAL = 0,           /*<! Deprecated */
    CAD_TYPE_VERTICAL,                 /*<! Deprecated */
    CAD_TYPE_VISION,                   /*<! Detection of 2D horizontal tags on drone shells */
    CAD_TYPE_NONE,                     /*<! Detection disabled */
    CAD_TYPE_COCARDE,                  /*<! Detects a roundel under the drone */
    CAD_TYPE_ORIENTED_COCARDE,         /*<! Detects an oriented roundel under the drone */
    CAD_TYPE_STRIPE,                   /*<! Detects a uniform stripe on the ground */
    CAD_TYPE_H_COCARDE,                /*<! Detects a roundel in front of the drone */
    CAD_TYPE_H_ORIENTED_COCARDE,       /*<! Detects an oriented roundel in front of the drone */
    CAD_TYPE_STRIPE_V,
    CAD_TYPE_MULTIPLE_DETECTION_MODE,  /* The drone uses several detections at the same time */
    CAD_TYPE_NUM,                      /*<! Number of possible values for CAD_TYPE */
} CAD_TYPE;




/*******************************************************************************
                                FUNCTION PROTOTYPES
*******************************************************************************/
int 	drk_ctrl_config(void);

int 	drk_sensor_data_init(void);
int 	drk_sensor_data_close(void); 

void 	drk_ar_object_tracker_setup(void);

int 	drk_sensor_data_wait(void);

float 	drk_heading(void);/* BEST heading method found. If you find better, edit this */

vector_t drk_drone_speed_get(void);

void 	dump_sensors(void);
/* Thread-safe altitude accessor */

/* Ultrasound altitude reading compensated for tilt */
double 	drk_ultrasound_raw(void);
double 	drk_ultrasound_altitude(void);
double	drk_abs_altitude(void);
double 	drk_rel_altitude(void);
void 	drk_zero_altitude( uint16_t dft ) ;






#endif // _INTERNAL_SENSOR_API_H_
