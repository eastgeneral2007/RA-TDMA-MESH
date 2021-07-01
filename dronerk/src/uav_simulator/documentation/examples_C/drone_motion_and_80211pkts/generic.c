/** generic.c **/
#include <unistd.h>
#include <inttypes.h>
#include <netinet/in.h> // udp packets
#include <ifaddrs.h> // udp packets
#include <arpa/inet.h> // udp packets
#include <sys/types.h> // recvfrom
#include <sys/socket.h> //recvfrom
#include <stdio.h> //sprintf 
#include <stdlib.h>  //exit
#include "world_simulator_api.h"


///* prepare sockaddr */
//void prepare_sockaddr(uint8_t id, uint16_t port, struct sockaddr_in * addr)
//{
	//addr->sin_family = AF_INET;
	//addr->sin_port = htons(port);
	
	//char addrstr[16];snprintf(addrstr,16, "127.0.0.%d", id);
	//int ret = inet_pton(AF_INET, addrstr, &(addr->sin_addr.s_addr));
	//if (ret != 1)
	//{fprintf(stderr, "Error on inet_pton!\n"); exit(1);}
	
	////printf("Prepared %s:%d\n", 
		////inet_ntoa(addr->sin_addr), 
		////ntohs(addr->sin_port) );
//}
/* end of sockaddr */	


/* sleep for <tmp_us> microseconds, return -1 if it fails */

//int microsleep( uint64_t tmp_us )
//{
	//uint64_t base_clock = SimClock_get() ;
	//while (SimClock_get() <= base_clock + tmp_us)
		//usleep(10);
	//return 1 ;
//}
