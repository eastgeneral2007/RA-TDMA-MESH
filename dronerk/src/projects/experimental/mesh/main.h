#ifndef MAIN_H_
#define MAIN_H_

//#include <arpa/inet.h> // udp packets
//#include <ifaddrs.h> // udp packets
#include <inttypes.h> // int16_t etc
//#include <linux/sockios.h> // SIOCOUTQ
//#include <math.h> /* cos sin */
//#include <netinet/in.h> // udp packets
#include <pthread.h> // we have threads in the app
#include <stdlib.h> // exit()
//#include <signal.h> // signal()
#include <stdio.h> // printf
#include <stdlib.h> // atoi
#include <string.h> // memcpy
#include <sys/types.h> // recvfrom
#include <unistd.h>
#include <errno.h>

#include "drk/drk.h" //drk init
#include "drk/utils.h"

#include "drk/tdma.h"
#include "drk/tdma_utils.h"
#include "drk/pdr.h"
#define PREFIX	"main"

void closeme();

#endif
