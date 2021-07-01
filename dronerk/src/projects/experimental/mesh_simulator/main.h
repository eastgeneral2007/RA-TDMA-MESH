#ifndef MAIN_H_
#define MAIN_H_

#include <arpa/inet.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <inttypes.h> // int16_t etc
//#include <linux/sockios.h> // SIOCOUTQ
#include <math.h> /* cos sin */
#include <netinet/in.h> // udp packets
#include <pthread.h> // we have threads in the app
#include <stdlib.h> // exit()
#include <signal.h> // signal()
#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <string.h> // memcpy
//#include <sys/ioctl.h> // SIOCOUTQ
//#include <sys/socket.h> //recvfrom
#include <sys/types.h> // recvfrom
#include <unistd.h>
#include <errno.h>
#include <sys/shm.h>
#include <sys/ipc.h>

#include "drk/drk.h" //drk init
#include "drk/world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "drk/utils.h"
#include "drk/sim_clock.h"
//#include "drk/internal_sensor_api.h"
//#include "drk/flight_control_api.h" //move front , back , etc
//#include "drk/gps.h" //get gps 
//#include "libsbp/read_piksi.h"
//#include "drk/internal_sensor_raw.h" //battery
#include "drk/tdma.h"
#include "drk/tdma_utils.h"
#include "drk/pdr.h"

#include "drk/autonomous.h"

#include "drk/flight_control_api.h"


#define PREFIX	"main"

enum retu {
	OKAY = 1,
	NOTOKAY=-1
} ;
typedef struct {
	float X ; /* 4 bytes */
	float Y ; /* 4 bytes */
	float Z ; /* 4 bytes */
} triplet_t ; /* 12 bytes */
typedef struct {
	triplet_t thrust ;
	float torque ;
} actuator_t ; 

void setActuation( actuator_t actuation_vector );
void closeme();

#endif
