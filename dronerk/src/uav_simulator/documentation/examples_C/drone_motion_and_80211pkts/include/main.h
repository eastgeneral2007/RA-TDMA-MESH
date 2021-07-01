#ifndef MAIN_H_
#define MAIN_H_


#include <stdlib.h> // atoi
#include <linux/sockios.h> // SIOCOUTQ
#include <sys/ioctl.h> // SIOCOUTQ
#include <pthread.h> // we have threads in the app
#include <stdlib.h> // exit()
#include <signal.h> // signal()
#include <stdio.h> // printf
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <string.h> // memcpy
#include <inttypes.h> // int16_t etc
#include <math.h> /* cos sin */

#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */
#include "generic.h" /* generic structs used everywhere */

#include "rx_isensor.h" /* to get last rcvd sensor reading */
#include "rx_esensor.h" /* to get last rcvd sensor reading */
#include "tx_actuation.h" /* to send a new actuation  */

void sigint_handler(int sig);

 
/**
 * app mission (main thread) needs 4 things:
 * 1)send actuation, 
 * 2)read sensors, 
 * 3)send 80211 pkts
 * 4)rcv 80211 pkts 
 * */   
void * thread_tx_actuation(void *arg);
void * thread_rx_sensor(void *arg);
void * thread_tx_80211(void *arg);
void * thread_rx_80211(void *arg);

#endif
