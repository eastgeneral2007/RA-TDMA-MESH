/*! \file drk.h
    \brief A Documented file.
    
    Details.
*/

#ifndef _DRK_H_
#define _DRK_H_

#include <stdint.h>


/* ******************************************
 * Initialize all of the DroneRK components 
 * ******************************************/

/* usb port for serial data, my id = TDMA slot id */
int 	drk_init( uint8_t usbport, uint8_t my_id , void(*callback)() ); 
int 	drk_init_emergency();
void 	drk_exit( void ); /* recreate CTRL+Z > emergency landing */
void 	drk_error( char *msg ) ;



#endif // _DRK_H_

