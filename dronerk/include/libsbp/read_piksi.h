/* piksi H */
#ifndef _PIKSI_H_
#define _PIKSI_H_

#include <pthread.h>
#include <inttypes.h>

#include "drk/utils.h"

//// default BS 
//#define BASELAT 41.176752 // feup canteen
//#define BASELON -8.596227 // feup canteen


/************
 * **** TYPEDEFS *****
 * ****************/
typedef struct
{
	double latitude; /* degrees 1.234º */ 
	double longitude; /* degrees 1.234º */ 
	double altitude; /* meters */
	uint8_t num_sats; 
	time_t timestamp ; /* when rcvd sample */
	//int8_t quality;	
} llh_t ;


typedef struct
{
	double north ;  /* meters */
	double east ;  /* meters */
	double down ;   /* meters */
	uint8_t num_sats; 
	time_t timestamp ; /* when rcvd sample */
	
} ned_t ;

/* use to save all from piksi HW */
typedef struct 
{

	
	llh_t best ; /* using BS+NED or SPP in case NED is not available  */
	llh_t spp ; /* raw LLH , single point position */
	ned_t ned ; /* NED referential */
	llh_t basestation ; /* known llh coordinates of the BS */
	

	double trueTrack;   // from VTG data - Track, degrees
	double groundSpeed; //from VTG data - Speed, Km/hr

	
	//double UTC;
	//newly added on 06/02/13
	//int gps_date; 		    // from GPRMC
	//float hPrecise;     //from $GPGGA - horizontal dilution of precision 	

} gps_t;

/* load module */
error_t drk_piksi_init( int usb_port ) ;

/* close it */
error_t drk_piksi_close( void ) ;


/* get best LLH possible */
llh_t drk_piksi_get_best(void);
ned_t drk_piksi_get_ned(void);
llh_t drk_piksi_get_spp(void);

/* get BS LLH  */
llh_t drk_piksi_get_bs(void);

/* get BS LLH  */
error_t drk_piksi_set_bs( llh_t bs_llh );


#endif



