/********* 
 * keyboard thread 
 * ***********/
#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "drk/utils.h"

/* motion-actuation related keys: */
#define KEY_HOVER 		' ' /* space */
#define KEY_FWD 		'w'
#define KEY_BCK 		's'
#define KEY_LFT 		'a'
#define KEY_RHT 		'd'
#define KEY_FWD_LFT		'q'
#define KEY_FWD_RHT		'e'
#define KEY_BCK_LFT		'z'
#define KEY_BCK_RHT		'x'
#define KEY_SPIN_ACC	'>'
#define KEY_SPIN_CC		'<'
#define KEY_UP			'-'
#define KEY_DOWN		'.'

/* autonomous related keys: */	
#define KEY_GO_NORTH		'i'
#define KEY_GO_SOUTH		'k'	
#define KEY_GO_EAST			'l'	
#define KEY_GO_WEST			'j'	
#define KEY_GO_BS			'0'	
#define KEY_GO_NOWHERE		'1'
#define KEY_PRINT_MAP		'n'
#define KEY_PAUSE			'p'
#define KEY_RESUME			'y'

/* internal sensors related keys: */
#define KEY_CALIBRATE	'm'
#define KEY_FLAT_TRIM	'f'
#define KEY_BATTERY		'b'



error_t	drk_keyboard_init( void ) ;
error_t	drk_keyboard_close( void ) ;
int 	drk_keyboard_lockDown( void ) ;
error_t	drk_keyboard_setKey( int(*action)(int) , int input , char key );
#endif
