#ifndef MAIN_H_
#define MAIN_H_



#include <signal.h> // signal()

//#include <netinet/in.h> // udp packets
//#include <ifaddrs.h> // udp packets
//#include <arpa/inet.h> // udp packets

#include <linux/sockios.h> // SIOCOUTQ
#include <sys/ioctl.h> // SIOCOUTQ
#include <pthread.h> // we have threads in the app
#include <semaphore.h>	/* semaphore */

/* shm_open , unlink */
#include <sys/mman.h>
#include <sys/stat.h>  /* For mode constants */
#include <fcntl.h>  


#include <unistd.h> /* sleep */
#include <stdio.h> /* printf */
#include <stdlib.h> /* exit() */


  
#include "return_codes.h" /* handy return enum */


void sigint_handler(int sig);


#include "sim_network.h" // init UDP_Start
#include "event_scheduler.h" // EventScheduler_init
#include "sim_layout.h" // initLayout
#include "sim_clock.h" // SimClock_init


#endif
