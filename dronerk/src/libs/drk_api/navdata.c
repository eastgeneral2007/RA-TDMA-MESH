/**
 * navdata.c
 * GET STATE OF DRONE.
 */



#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <semaphore.h>

#include "drk/navdata.h"
#include "drk/internal_sensor_raw.h"
#include "drk/utils.h"

#define PREFIX "NAVDATA"

#ifndef SENSOR_LOADED_CONDITION
#define SENSOR_LOADED_CONDITION \
	if (Navdata_initiated == 0) return 
#endif
// From ARDroneLib/Soft/Common/config.h
// Define masks for ARDrone state
// 31                                                             0
//  x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x x -> state
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | FLY MASK : (0) ardrone is landed, (1) ardrone is flying
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VIDEO MASK : (0) video disable, (1) video enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | | VISION MASK : (0) vision disable, (1) vision enable
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | | CONTROL ALGO : (0) euler angles control, (1) angular speed control
//  | | | | | | | | | | | | | | | | | | | | | | | | | | | ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active
//  | | | | | | | | | | | | | | | | | | | | | | | | | | USER feedback : Start button state
//  | | | | | | | | | | | | | | | | | | | | | | | | | Control command ACK : (0) None, (1) one received
//  | | | | | | | | | | | | | | | | | | | | | | | | Camera enable : (0) Camera enable, (1) camera disable
//  | | | | | | | | | | | | | | | | | | | | | | | Travelling enable : (0) disable, (1) enable
//  | | | | | | | | | | | | | | | | | | | | | | USB key : (0) usb key not ready, (1) usb key ready
//  | | | | | | | | | | | | | | | | | | | | | Navdata demo : (0) All navdata, (1) only navdata demo
//  | | | | | | | | | | | | | | | | | | | | Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent
//  | | | | | | | | | | | | | | | | | | | Motors status : (0) Ok, (1) Motors Com is down
//  | | | | | | | | | | | | | | | | | | Communication Lost : (1) com problem, (0) Com is ok
//  | | | | | | | | | | | | | | | | | 
//  | | | | | | | | | | | | | | | | VBat low : (1) too low, (0) Ok
//  | | | | | | | | | | | | | | | User Emergency Landing : (1) User EL is ON, (0) User EL is OFF
//  | | | | | | | | | | | | | | Timer elapsed : (1) elapsed, (0) not elapsed
//  | | | | | | | | | | | | | Magnetometer calibration state : (0) Ok, no calibration needed, (1) not ok, calibration needed
//  | | | | | | | | | | | | Angles : (0) Ok, (1) out of range
//  | | | | | | | | | | | WIND MASK: (0) ok, (1) Too much wind
//  | | | | | | | | | | Ultrasonic sensor : (0) Ok, (1) deaf
//  | | | | | | | | | Cutout system detection : (0) Not detected, (1) detected
//  | | | | | | | | PIC Version number OK : (0) a bad version number, (1) version number is OK
//  | | | | | | | ATCodec thread ON : (0) thread OFF (1) thread ON
//  | | | | | | Navdata thread ON : (0) thread OFF (1) thread ON
//  | | | | | Video thread ON : (0) thread OFF (1) thread ON
//  | | | | Acquisition thread ON : (0) thread OFF (1) thread ON
//  | | | CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled // Check frequency of control loop
//  | | ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good // Check frequency of uart2 dsr (com with adc)
//  | Communication Watchdog : (1) com problem, (0) Com is ok // Check if we have an active connection with a client
//  Emergency landing : (0) no emergency, (1) emergency

/*****************
 * Externs
 ****************/
extern struct global_navdata_* 	navdata_buf ; /* flag module initated or not - in ISA */
extern int 						Navdata_initiated ; /* flag module initated or not - in ISA */

uint32_t getDroneState()
{
	SENSOR_LOADED_CONDITION 0;
	uint32_t droneState;
	if (sem_wait(navdata_buf->semaphore) < 0 )
	{
		PRINTF_FL_ERR("err sem\n") ;
		return 0 ;
	}
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	return droneState;
}

void print_navdata_state ()
{
	uint32_t droneState = getDroneState() ;

	char* data = (char*)(&droneState);
	unsigned int i;
	PRINTF_FL( "Drone state: [");
	for (i = 0; i < sizeof(droneState) ; ++i)
		printf("%02x", data[i]);
	printf("]\n");
}

/*
	Return value 0 - drone has landed
				 1 - drone is flying
*/
bool getStateFlyMask(void)
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_FLY_MASK) == ARDRONE_FLY_MASK)
		return 1;
	return 0;
}

/*
	Return value 
	0 - Video disabled
	1 - Video enabled			
*/
bool getStateVideoMask(void)
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_VIDEO_MASK) == ARDRONE_VIDEO_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Vision disabled
	1 - Vision enabled
*/
bool getStateVisionMask()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_VISION_MASK) == ARDRONE_VISION_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Euler angle control
	1 - Angular speed control
*/
bool getStateControlAlgo()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_CONTROL_MASK) == ARDRONE_CONTROL_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Altitude control inactive
	1 - Altitude control active
*/
bool getMaskAltitudeControlAlgo()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ALTITUDE_MASK) == ARDRONE_ALTITUDE_MASK)
		return 1;
	return 0;
}

/*
	Start button state
*/
bool getStateStartButton()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_USER_FEEDBACK_START ) == ARDRONE_USER_FEEDBACK_START)
		return 1;
	return 0;
}

/*
 Return value
	0 - None received
	1 - one received
*/
bool getStateContolACK()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_COMMAND_MASK) == ARDRONE_COMMAND_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Camera enabled
	1 - Camera disabled
*/
bool getStateCamera()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_CAMERA_MASK ) == ARDRONE_CAMERA_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Travelling enabled
	1 - Travelling disabled
*/
bool getStateTravelling()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_TRAVELLING_MASK) == ARDRONE_TRAVELLING_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - USB key not ready
	1 - USB key ready
*/
bool getStateUSBKey()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_USB_MASK) == ARDRONE_USB_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Get all navdata
	1 - Only navdata demo
*/
bool getStateNavdata()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state ;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_NAVDATA_DEMO_MASK) == ARDRONE_NAVDATA_DEMO_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Not in bootstrap mode(sending navdata)
	1 - Still in bootstrap mode
*/
bool getStateBootstrapMode()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_NAVDATA_BOOTSTRAP) == ARDRONE_NAVDATA_BOOTSTRAP)
		return 1;
	return 0;
}

/*
	Return value
	0 - Motor status ok
	1 - Motor communication lost
*/
bool getStateMotor()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_MOTORS_MASK) == ARDRONE_MOTORS_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Communication is ok
	1 - Communication Problem
*/
bool getStateCommunication()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_COM_LOST_MASK) == ARDRONE_COM_LOST_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - 
	1 - 
*/
bool getStateSoftware()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_SOFTWARE_FAULT) == ARDRONE_SOFTWARE_FAULT)
		return 1;
	return 0;
}

/*
	Return value
	1 - Battery is too low
	0 - Battery is ok
*/
bool getStateBattery()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_VBAT_LOW) == ARDRONE_VBAT_LOW)
		return 1;
	return 0;
}

/*
	Return value
	0 - User Emergency Landing is ON
	1 - User EL is OFF
*/
bool getStateUserEmergencyLanding()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_USER_EL) == ARDRONE_USER_EL)
		return 1;
	return 0;
}

/*
	Return value
	0 - Timer not elapsed
	1 - Timer elapsed
*/
bool getStateTimerElapsed()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_TIMER_ELAPSED) == ARDRONE_TIMER_ELAPSED)
		return 1;
	return 0;
}

/*
	Return value
	0 - No calibration needed
	1 - Calibration needed
*/
bool getStateMagnetometer()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_MAGNETO_NEEDS_CALIB) == ARDRONE_MAGNETO_NEEDS_CALIB)
		return 1;
	return 0;
}

/*
	Return value
	0 - Angles ok
	1 - Angles out of range
*/
bool getStateAngles()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ANGLES_OUT_OF_RANGE) == ARDRONE_ANGLES_OUT_OF_RANGE)
		return 1;
	return 0;
}

/*
	Return value
	0 - Wind ok
	1 - Too much wind to fly
*/
bool getStateWind()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_WIND_MASK) == ARDRONE_WIND_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Ultrasonic sensor ok
	1 - Ultrasonic sensor on working
*/
bool getStateUltrasonicSensor()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ULTRASOUND_MASK) == ARDRONE_ULTRASOUND_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Not detected
	1 - Detected
*/
bool getStateSystemCutout()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_CUTOUT_MASK) == ARDRONE_CUTOUT_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - Bad PIC version number
	1 - veersion number is OK
*/
bool getStatePICVersion()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_PIC_VERSION_MASK) == ARDRONE_PIC_VERSION_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - ATCodec thread is OFF
	1 - ATCodec thread is ON
*/
bool getStateATCodecThread()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ATCODEC_THREAD_ON) == ARDRONE_ATCODEC_THREAD_ON)
		return 1;
	return 0;
}

/*
	Return value
	0 - Navdata thread is OFF
	1 - Navdata thread is ON
*/
bool getStateNavdataThread()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_NAVDATA_THREAD_ON) == ARDRONE_NAVDATA_THREAD_ON)
		return 1;
	return 0;
}

/*
	Return value
	0 - Video thread is OFF
	1 - Video thread is ON
*/
bool getStateVideoThread()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_VIDEO_THREAD_ON) == ARDRONE_VIDEO_THREAD_ON)
		return 1;
  return 0;
}

/*
	Return value
	0 - Acquisition thread is OFF
	1 - Acquisition thread is ON
*/
bool getStateAcquisitionThread()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ACQ_THREAD_ON) == ARDRONE_ACQ_THREAD_ON)
		return 1;
  return 0;
}

/*
	Return value
	0 - Contol is well scheduled
	1 - Delay in control execution(>5ms)
*/
bool getStateCTRLWatchDog()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_CTRL_WATCHDOG_MASK) == ARDRONE_CTRL_WATCHDOG_MASK)
		return 1;
  return 0;
}

/*
	Return value
	0 - UART is good
	1 - Delay in UART2 DSR(>5ms)
*/
bool getStateADCWatchDog()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_ADC_WATCHDOG_MASK) == ARDRONE_ADC_WATCHDOG_MASK)
		return 1;
  return 0;
}

/*
	Return value
	0 - Communication is ok
	1 - Communication problem
*/
bool getStateCommWatchDog()
{
	SENSOR_LOADED_CONDITION 0;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_COM_WATCHDOG_MASK) == ARDRONE_COM_WATCHDOG_MASK)
		return 1;
	return 0;
}

/*
	Return value
	0 - No emergency
	1 - Emergency state
*/
bool getStateEmergencyLanding()
{
	SENSOR_LOADED_CONDITION 1;
	unsigned int droneState;
	sem_wait(navdata_buf->semaphore);
	droneState=(navdata_buf->sensor).state;
	sem_post(navdata_buf->semaphore);

	if((droneState & ARDRONE_EMERGENCY_MASK) == (unsigned int)ARDRONE_EMERGENCY_MASK)
		return 1;
	return 0;
}
