#ifndef RX_ISENSOR_H_
#define RX_ISENSOR_H_

#include "generic.h"
#include "p80211.h"

typedef struct {
	uint32_t state ;
	int 	battery ;
	float 	heading_degrees ;
	float 	ultra_altitude ;
} isensor_t ; 

void * thread_rx_isensor(void *arg);
isensor_t isensor_getReading(void) ;


#endif
