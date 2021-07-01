#ifndef RX_ESENSOR_H_
#define RX_ESENSOR_H_

#include "generic.h"
#include "p80211.h"

typedef struct {
	uint32_t	rtk_sample_id ;
	int8_t		rtk_num_sats;
	int32_t 	north; 
	int32_t		east;
	int32_t		down;
	int32_t		vel_north;
	int32_t		vel_east; 
	int32_t		vel_down;
	uint32_t	spp_sample_id; 
	int8_t 		spp_num_sats ;
	int32_t		latitude ; 
	int32_t		longitude  ;
} esensor_t ; /* simulator rtk sensor t */

void * thread_rx_esensor(void *arg);
esensor_t esensor_getReading(void) ;


#endif
