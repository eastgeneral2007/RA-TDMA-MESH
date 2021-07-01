#ifndef TX_ACTUATION_H_
#define TX_ACTUATION_H_

#include "generic.h"
#include "p80211.h"


typedef struct {
	triplet_t thrust ;
	float torque ;
} actuator_t ; 

void setActuation( actuator_t actuation_vector );

#endif

