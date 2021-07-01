/*! \file drk_network.h
    \brief A Documented file.
    
    Details.
*/
#ifndef _DRK_NETWORK_H_
#define _DRK_NETWORK_H_

#include <stdlib.h>
#include <stdio.h>

struct drk_rssi {
	double quality;
	int signal;
	int noise;
};

int drk_update_rssi(struct drk_rssi * ptr);

#endif // _DRK_NETWORK_H_
