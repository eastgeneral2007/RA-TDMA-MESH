/**
 * drkP.h
 */

#ifndef _DRKP_H_
#define _DRKP_H_

#include <stdint.h>
#include <sys/time.h>


//using drone 2.0 API to set some params
void _drk_config();

/* Create the navigation and control threads */
void _drk_controller_init();

//thread that deals with feeding the receiver buffer
void * _drk_serial_thread();

/* Initialize thread for keyboard control */
void * _drk_keyboard_init();

/* Runs constantly in the background. Assign actions to keys by adding a case */
void _drk_keyboard_handler_thread();

double _drk_time_diff(struct timeval end);

/*******************************************************************************
						SEND / RECIEVE THREADS
*******************************************************************************/

/* Read the incoming navdata from the drone */
void* _drk_ar_nav_sensor_thread(void* args);

/* Send a command to the drone */
void* _drk_actuate_thread(void* args);


#endif // _DRKP_H_
