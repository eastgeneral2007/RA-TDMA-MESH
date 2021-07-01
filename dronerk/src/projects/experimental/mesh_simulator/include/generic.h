/* return codes used across the sim */

#ifndef GENERIC_H
#define GENERIC_H


#include <inttypes.h> // int16_t etc
#include <pthread.h> // we have threads 
#include <netinet/in.h> // udp packets

#define CLEAR(x)	memset((void*)(&(x)), 0, sizeof(x))
#define	SA 		const struct sockaddr *
#define SLEN	(socklen_t)sizeof(struct sockaddr_in)


typedef enum
{
	OKAY ,
	NOTOKAY
}
return_codes ;

typedef struct {    /* Used as argument to thread_start() */
	pthread_t	tid; /* ID returned by pthread_create() */
	int			in[5]; /* Application-defined input for each thread # */
} tinfo_t;

typedef struct {    /* Used as argument to thread_start() */
	tinfo_t 	t[10];
	int 		n; /* current count of threads in the struct */
} tinfo2_t;

typedef struct {
	float X ; /* 4 bytes */
	float Y ; /* 4 bytes */
	float Z ; /* 4 bytes */
} triplet_t ; /* 12 bytes */


//int microsleep( uint64_t tmp_us );


/** network related **/
//void prepare_sockaddr(uint8_t dst, uint16_t port, struct sockaddr_in * addr);





#endif
