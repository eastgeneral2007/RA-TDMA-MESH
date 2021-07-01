/**
 * flight_control_api.c
 */



#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>


#include "drk/flight_control_api.h"
#include "drk/ar_configuration.h"
#include "drk/navdata.h"
#include "drk/internal_actuator_api.h"
#include "drk/utils.h"
/*******************************************
 * 						MACROS
 * ******************************************/
#define PREFIX		"FCA"

#ifndef ACTUATOR_LOADED_CONDITION
#define ACTUATOR_LOADED_CONDITION \
	if (Actuator_initiated == 0) return 
#endif

#ifndef SENSOR_LOADED_CONDITION
#define SENSOR_LOADED_CONDITION \
	if (Navdata_initiated == 0) return 
#endif

/***********
 * Externs
 **********/
extern int Actuator_initiated ; /* flag: module initiated or not */
extern int Navdata_initiated ; /* flag module initated or not - in ISA */
extern struct global_actuator_ *actuator_buf ;
#ifdef SIMULATOR
#include "drk/internal_sensor_raw.h"
extern struct global_navdata_ *navdata_buf ; /* we can fake some navdata here. todo: send state data from uavsim */
#endif

/*******************************************************************************
                            FLIGHT CONTROLS
*******************************************************************************/




/* Send a takeoff command, then block for five seconds to wait for completion */
void drk_takeoff( void )
{

	ACTUATOR_LOADED_CONDITION ;
	
	/* Set the actuator values */
	//puts ("entered drk_takeoff");
	PRINTF_FL( "Drone flying? %s\n", getStateFlyMask()?"YES":"NO");
	print_navdata_state();

	//Check if drone is in emergency state & Reset:   
	//drk_remove_emergency();
	//usleep(100000);
	//print_navdata_state ();

	// zeroes pitch roll sensors. USE FLAT surface
	//drk_ar_flat_trim();


	if( !getStateFlyMask() && !getStateBattery() ) 
	{ //not flying & battery is not low
		PRINTF_FL( "Taking off...\n");
		//Take off commands
		sem_wait(actuator_buf->semaphore);
		(actuator_buf->actuator).takeoff_flag = TAKINGOFF ;
		(actuator_buf->actuator).pitch.f=0.0;
		(actuator_buf->actuator).roll.f=0.0;
		(actuator_buf->actuator).gaz.f=0.0;
		(actuator_buf->actuator).spin.f=0.0;
		(actuator_buf->actuator).pcmd_flag=0;
		(actuator_buf->actuator).emergency_flag=0;
		sem_post(actuator_buf->semaphore);


	#ifdef SIMULATOR /* emulate we got flying flag*/
		SENSOR_LOADED_CONDITION ;
		sem_wait( navdata_buf->semaphore );
		(navdata_buf->sensor).state |= (0xFFFF && ARDRONE_FLY_MASK) ;
		sem_post( navdata_buf->semaphore );
		print_navdata_state();printf(":D\n");
	#endif
	
		/* Test to see if drone doesnt take-off */
		/* Quick & dirty implemetation */
		uint8_t i=50; /*try 50times */
		while( getStateFlyMask()==0 && i>0 )
		{
			i--;
			usleep(10000);
		}
		if (i==0) //Drone didn't take-off
		{
		  PRINTF_FL( "Error taking off.\n");
		  (actuator_buf->actuator).takeoff_flag = 0;
		}
		else //drone has taken-off
		{
			(actuator_buf->actuator).takeoff_flag = 0;
			PRINTF_FL( "...done.\n");
		} 
	}
	else if(getStateFlyMask())
	{
		PRINTF_FL( "Drone is already flying\n");
	}
	else if( getStateBattery() )
	{
		PRINTF_FL_WARN( "Can't take off. Battery LOW\n");
	}
	
	PRINTF_FL( "Drone flying?:%s\n", getStateFlyMask()?"YES":"NO" );
	

}

/* Send a land command, then read sensor data until landed */
void drk_land(void)
{
	ACTUATOR_LOADED_CONDITION ;
	
		
	if( getStateFlyMask() )
	{ 
		/* flying */
		PRINTF_FL( "Landing...\n");
		
		/* Set the actuator values  */
		sem_wait(actuator_buf->semaphore);
		(actuator_buf->actuator).takeoff_flag = -1;
		(actuator_buf->actuator).pitch.f=0.0;
		(actuator_buf->actuator).roll.f=0.0;
		(actuator_buf->actuator).gaz.f=0.0;
		(actuator_buf->actuator).spin.f=0.0;
		(actuator_buf->actuator).pcmd_flag=0; /* no motion */
		(actuator_buf->actuator).emergency_flag=0;
		sem_post(actuator_buf->semaphore);

	#ifdef SIMULATOR
		sem_wait( navdata_buf->semaphore );
		(navdata_buf->sensor).state &= (0xFFFF ^ ARDRONE_FLY_MASK ) ;
		sem_post( navdata_buf->semaphore );
		print_navdata_state();printf(":D\n");
	#endif
		//printf("sem ok\n");
		/* Wait for the drone to land */
		while( getStateFlyMask() )
			usleep(30000) ;
		sem_wait(actuator_buf->semaphore);
		(actuator_buf->actuator).takeoff_flag = 0 ;
		sem_post(actuator_buf->semaphore);
		
		PRINTF_FL( "Landing done\n");
	}
	else
	{
		PRINTF_FL( "Drone has already landed.\n");
	}
}

/* Trigger an emergency shutoff (kill the motors) */
#define WAIT_PERIOD_EMERGENCY_US	300000
void drk_emergency(void)
{

	ACTUATOR_LOADED_CONDITION ;

		
	PRINTF_FL_WARN( "*******************EMERGENCY SHUTDOWN TRIGGERED******************\n");


	/* Pass into the struct */
	sem_wait(actuator_buf->semaphore);
	(actuator_buf->actuator).takeoff_flag = -1;
	(actuator_buf->actuator).pitch.f=0.0;
	(actuator_buf->actuator).roll.f=0.0;
	(actuator_buf->actuator).gaz.f=0.0;
	(actuator_buf->actuator).spin.f=0.0;
	(actuator_buf->actuator).pcmd_flag=0;
	(actuator_buf->actuator).emergency_flag=1;
	sem_post(actuator_buf->semaphore);
	

	
	/* Wait until usr emergency is ON */
	/* 0 - User Emergency Landing is ON */
	print_navdata_state();
	
	
#ifdef SIMULATOR /* emulate we got emergency flag*/
	sem_wait( navdata_buf->semaphore );
	(navdata_buf->sensor).state = (0xFF & ARDRONE_EMERGENCY_MASK) ;
	sem_post( navdata_buf->semaphore );
	print_navdata_state();
	
	sem_wait( navdata_buf->semaphore );
	(navdata_buf->sensor).state = ARDRONE_EMERGENCY_MASK ;
	sem_post( navdata_buf->semaphore );
	print_navdata_state();
#endif
	
	//PRINTF_FL( "print_navdata_state - ok\n" ); 
	
	//while ( !getStateEmergencyLanding() )
	//{
		//PRINTF_FL( ":_:\n") ;
		microsleep( WAIT_PERIOD_EMERGENCY_US );
		print_navdata_state();
		
	//}

	//PRINTF_FL( "emergency() - ok\n" ); 

	//sem_wait(actuator_buf->semaphore);
	//(actuator_buf->actuator).emergency_flag=0;
	//sem_post(actuator_buf->semaphore);
}

/* Puts the drone in hover mode, using the camera to stabilize */
void drk_lockdown_hover(int time)
{
	ACTUATOR_LOADED_CONDITION ;
	




	/* Set the actuator values */
	struct drone_actuator_ temp_actuator;
	temp_actuator.takeoff_flag = 0 ;
	temp_actuator.pitch.f = 0.0;
	temp_actuator.roll.f = 0.0;
	temp_actuator.gaz.f = 0.0;
	temp_actuator.spin.f = 0.0;
	temp_actuator.pcmd_flag = 0 ; /* hover */
	temp_actuator.emergency_flag = 0;

	/* Pass them into the struct */
	sem_wait(actuator_buf->semaphore);
	memcpy(&(actuator_buf->actuator), &temp_actuator, sizeof(temp_actuator));
	sem_post(actuator_buf->semaphore);

	/* Let the command run for the specified duration */
	if (time > 0) microsleep(time * 1000);
}

/* Puts the drone in hover mode */
void drk_hover( int time )
{
	drk_lockdown_hover( time ) ; /* use camera . pcmd_flag is 0 */
}

/* Combined movement */
void drk_translate( float pitch, float roll, 
	float yaw, float gaz, int time_ms )
{
	ACTUATOR_LOADED_CONDITION ;
	//printf("translate**\n");
	/* Error checking */
  if (time_ms < 0) time_ms = 0;
  if (pitch < -1.0 || pitch > 1.0 || yaw < -1.0 || yaw > 1.0 ||
      roll < -1.0 || roll > 1.0 || gaz < -1.0 || gaz > 1.0)
  {
    PRINTF_FL( "Translate value out of range. Returning to HOVER mode.\n");
    drk_hover(0);
    return;
  } 
  
  /* Set the actuator values */
  sem_wait(actuator_buf->semaphore);
  (actuator_buf->actuator).takeoff_flag = 0 ; /* Flying */
  (actuator_buf->actuator).pitch.f=pitch;
  (actuator_buf->actuator).roll.f=roll;
  (actuator_buf->actuator).gaz.f=gaz;
  (actuator_buf->actuator).spin.f=yaw;
  (actuator_buf->actuator).pcmd_flag = 1 ; // no hover , MOVE!
  sem_post(actuator_buf->semaphore);

	/* Let the command run for the specified duration, i.e. block here some time */
	/* todo: make an alarm , then erase cmd */
	if ( time_ms > 1 )
		microsleep(time_ms*1000); 


	
	
  //Restore settings to hover mode
  //drk_hover(0);
}

/* Rotate to the left */
void drk_spin_left (float rate, int time)
{
	drk_translate(0.0, 0.0, -rate, 0.0, time);
}

/* Rotate to the right*/
void drk_spin_right (float rate, int time)
{
	drk_translate(0.0, 0.0, rate, 0.0, time);
}

/* Move forward */
void drk_move_forward (float rate, int time)
{
	drk_translate(-rate, 0.0, 0.0, 0.0, time);
}

/* Move backward */
void drk_move_backward(float rate, int time)
{
	drk_translate(rate, 0.0, 0.0, 0.0, time);
}

/* Move right */
void drk_move_right (float rate, int time)
{
	drk_translate(0.0, rate, 0.0, 0.0, time);
}

/* Move left */
void drk_move_left (float rate, int time)
{
	drk_translate(0.0, -rate, 0.0, 0.0, time);
}

/* Fly upward */
void drk_move_up (float rate, int time)
{
	drk_translate(0.0, 0.0, 0.0, rate, time);
}

/* Fly downward */
void drk_move_down (float rate, int time)
{
	drk_translate(0.0, 0.0, 0.0, -rate, time);
}

/*Removes the drone from emergency state
Send AT command to drone to remove it from emergency state
*/
int drk_remove_emergency()
{
	ACTUATOR_LOADED_CONDITION -1;

	
	if ( getStateUserEmergencyLanding() || getStateEmergencyLanding() )
	{
		
		uint8_t try = 0;
		PRINTF_FL( "Removing USER emergency...\n");
		while ( getStateUserEmergencyLanding() || getStateEmergencyLanding() ) // drone in emergency mode:
		{
			try++ ;
			if ( try == 5 ) 
			{
				PRINTF_FL_ERR("failed. still emergency\n");
				return -1 ;
			}
			
			
			/* emergency shutdown/reset cmd */
			sem_wait(actuator_buf->time_sem);
			int ret = drk_send_at_command( 
				"AT*REF=%ld,290717952\r", 
				actuator_buf->global_timestamp++);
			sem_post(actuator_buf->time_sem);
			
			if ( ret < 0 )
				PRINTF_FL_ERR("failed to send at command\n" );
			microsleep(300000);
		}
		PRINTF_FL("done!\n");
	}
	
	

	
	return 1; 
}
