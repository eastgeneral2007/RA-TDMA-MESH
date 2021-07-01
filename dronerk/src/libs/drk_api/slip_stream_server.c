/**
 * slip_stream_server.c
 * Created by David Chow on 7/11/13.
 * Modified by Fabian Okeke on 07/15/13.
 * Mod  by Luis Pinto since 29/sept/2013	
 **/

//#include "drk/slip_stream_server.h"

#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef SIMULATOR
#include <netinet/in.h> /* udp packets */
#include <ifaddrs.h> /* udp packets */
#include <arpa/inet.h> /* udp packets */
/* Simulator sends sensor data TO this port */
#define UDP_PORT_BASE_TX_SENSOR	62000
#endif

#include "drk/drk_udp.h"
#include "drk/sensor_defines.h"
#include "drk/expansion_sensor_api.h"
#include "drk/utils.h"

typedef struct {
	float X ; /* 4 bytes */
	float Y ; /* 4 bytes */
	float Z ; /* 4 bytes */
} truple_t ; /* 12 bytes */


typedef struct {
	truple_t position ;
	float yaw ;
} sensor_t ; 

#define MAX_80211_SIZE	1500

/* Modes */
enum serial_mode 
{
	ASCII = 1 ,
	SLIP = 0
};

/* SLIP CONTROL */
#define ESC		0xDB
#define END		0xC0
#define ESC_END	0xDC
#define ESC_ESC	0xDD
#define START   0xC1

/* others */
#define NRETRIES			1
//#define TIMEOUT         	3 //TODO: SECONDS?
#define LAST_CONNECTION		0
#define STATIC_CLIENT		1
#define MAX_SLIP_BUF    	254
#define ACK_BUF_SIZE		5

/*****************************
 *  FUNCTION MACROS 
 * ************************/
#define PREFIX "SSS"


#ifndef SERIAL_LOADED_CONDITION
#define SERIAL_LOADED_CONDITION \
	if ( Serial_initiated == 0 ) return 
#endif




/**********************
 * (Global) Externs
 *********************/
extern struct global_serial* serial_buf;
extern uint8_t global_exit_all_threads ;

/***************************************
 * Evil Static Globals. 
 ***********************************/
#ifdef SIMULATOR
static int sock_fd ; /* socket file descriptor */
#else
int port_fd; /* usb file descriptor. TODO: this is externable.. */
#endif

// static int	debug = 1 ;	//debug enabled	
//int rx_buf_len;														// Length of receiving buffer												


char tmp_buf[MAX_FRAGMENT_LENGTH];			// Temporary buffer
pthread_mutex_t mutex_tx = PTHREAD_MUTEX_INITIALIZER;	 // tx Mutex - prevents simultaneous modem_tx calls
pthread_mutex_t rx_buf_mutex = PTHREAD_MUTEX_INITIALIZER;//to protect radio rx buf of serial_buf
uint8_t g_usb_port = -1  ; 
uint16_t initseq = 187; //21331; //'SS'

//ACK BUFFER LIST
uint16_t ack_buf[ACK_BUF_SIZE];
uint8_t ack_buf_w_pointer = 0; //use to read slip_ack_buf
pthread_mutex_t rx_ack_buf_mutex = PTHREAD_MUTEX_INITIALIZER;	//use to read slip_ack_buf

int Serial_initiated = 0 ;


/* Open and configure the serial port, then start collecting data
 * This must be called before using any other functions */
int8_t expansion_serial_init( void );
int drk_open_serial_port( void ); /* get fd , open the serial port */
// void slip_server_thread(); /* while cycle that keeps reading serial messages */
void * serial_thread() ;

/* FUNCTION IMPLEMENTATION */
extern pthread_t ser_tid; /* in drk.c */



/****************** prototypes (static) ***************/
static int serial_close ( void ) ;


/** init external serial sensor service **/
int8_t drk_serial_init( uint8_t usb_port ) 
{
	g_usb_port = usb_port ;

	/* open serial port (nonblocking) */
	if ( drk_open_serial_port() < 0 )
	{
		PRINTF_FL_ERR( "\tNo USB stick detected\n");
		return -1 ;
	}
	else
	{
		PRINTF_FL( "\t[SERIAL] USB%d stick detected\n", g_usb_port );
	}
		
	/*  create serial buf */
	if ( (serial_buf = calloc( 1 , sizeof(struct global_serial)) ) == NULL )
	{
		PRINTF_FL_ERR( "\tError allocating <serial_buf>\n");
		return -1 ;
	}
	if (pthread_mutex_init( &(serial_buf->gps_mutex) , NULL )  != 0 )
	{
		PRINTF_FL_ERR( "\tGps_mutex - init failed");
		return -1 ;
	}
	
	
	//if ( expansion_serial_init() < 0 ) 
	//{
		////PRINTF_FL( "[SERIAL] Failed to init external sensor service\n");
		//return -1 ;
	//}
	
	
	
	/* start a thread that reads the serial port - incoming GPS data, radio packets from fireflynode ,etc */
	pthread_attr_t ser_attr;
	size_t ser_desired_stack_size = 4000000;
	pthread_attr_init( &ser_attr );
	pthread_attr_setstacksize(&ser_attr, ser_desired_stack_size);
	if (pthread_create(&ser_tid, &ser_attr, serial_thread , NULL) != 0) 
	{
		PRINTF_FL( "\t[SERIAL] thread- Error spawning!\n");
		return -1 ;
	}
	else
	{
		PRINTF_FL( "\t[SERIAL] thread - created with success\n");
	}
	
	Serial_initiated = 1 ;
	return 1 ;
}

/** 
 * Open serial port (non blocking!)
 **/
int drk_open_serial_port( void ) 
{
	
#ifdef SIMULATOR //simulator does not use SERIAL port, instead our expansion serial data comes from the simulator via UDP. TODO: simulator should send packets to a pty serial port. 
	struct sockaddr_in si_me ; /* sockaddr for myself and for the received packet*/
	/*prepare UDP socket*/ 
	PRINTF_FL( "\tpreparing to open fake serial /dev/ttyUSB<127.0.0.%hu>\n",g_usb_port);
	if ( ( sock_fd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP ) ) == -1 )
	{
		PRINTF_FL_ERR("\terror getting udp socket\n");
		return -1;
	}
	si_me.sin_family = AF_INET; /* Address Format (ip4) */
	si_me.sin_port = htons( UDP_PORT_BASE_TX_SENSOR + g_usb_port ) ; 
	si_me.sin_addr.s_addr = htonl( INADDR_ANY ) ; //TODO: this should be changed to 127.0.0.(myid)
	if ( bind( sock_fd , (struct sockaddr*)&si_me, (socklen_t)sizeof(si_me) ) == -1 ) //link si_me (port+address) to the socket
	{
		PRINTF_FL_ERR( "error binding sokt. local prt %d\n\n", UDP_PORT_BASE_TX_SENSOR + g_usb_port );
		return -1;
	}
	else
	{
		PRINTF_FL( "binding sokt OK! local prt %d\n\n", UDP_PORT_BASE_TX_SENSOR + g_usb_port );
	}
	
	
	
#else /* no simulation : */
	//struct termios serial_tio;
	//memset( &serial_tio,'\0', sizeof(serial_tio) );

	char usb_port_str[16];
	snprintf( usb_port_str ,15, "/dev/ttyUSB%hu", g_usb_port ) ;
	
	long baud, databits, stopbits, parityon, parity;
	//place for old and new port settings for serial port
	struct termios newtio ;

	//int got_connection = 0;
	//int reply_mode = LAST_CONNECTION;

	//PARITYON = PARENB;
	//PARITY = PARODD;
	/* open the device(com port) to be non-blocking (read will return immediately) */
	port_fd = open( usb_port_str , O_RDWR | O_NOCTTY | O_NONBLOCK ) ;
	if (port_fd < 0) 
	{
		PRINTF_FL_ERR( "Failed to open %s (%s)\n" , usb_port_str , strerror(errno) ) ;
		return -1;
	}
	
	/* set new port settings for canonical input processing */
	baud = B115200 ;
	databits = CS8 ;
	stopbits = 0 ;
	//STOPBITS = CSTOPB;
	parityon = 0 ;
	parity = 0 ;
	newtio.c_cflag = baud | databits | stopbits | parityon | parity | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;         //ICANON;
	newtio.c_cc[VMIN] = 1; /* return one byte at a time */
	newtio.c_cc[VTIME] = 0;
	tcsetattr (port_fd, TCSANOW, &newtio);

	tcflush (port_fd, TCIOFLUSH); //io flushes data not read & not written
	usleep(100000);
	tcflush (port_fd, TCIFLUSH); //input .flushes data not read & not written
	usleep(100000);
	tcflush (port_fd, TCOFLUSH); //out .flushes data not read & not written	
	
	#endif
	return 1 ;
}

/************
 * Check for ASCII or SLIP data
 * Receives the message and saves it to struct (ASCII) or global buffer (SLIP)
 * Global buffer used is (serial_buf->radio_buf).msg_buf
 * Note: This function runs in an infinite loop
 * If you want only Radio data, comment out when mode == ASCII
* *****/
void *serial_thread()
{
	int ret = 0 ;
	//printf("SSS@ getting mutex\n");

	/* radio SLIP messages */
	pthread_mutex_lock(&rx_buf_mutex);
	(serial_buf->radio_buf).ready_to_read = 0;
	memset((serial_buf->radio_buf).msg_buf, 0, MAX_FRAGMENT_LENGTH);
	pthread_mutex_unlock(&rx_buf_mutex);


	
	#ifdef SIMULATOR 
	char recv_data[MAX_80211_SIZE] ;
	#endif

	char line_buffer[SENSOR_LINE_LENGTH]; 	// Buffer for parse_line


	#ifndef SIMULATOR
	int	serial_index = 0; /* Index for lineBuffer */
	uint8_t c = 0 ; /* save here each char received from the serial port */
	int mode = ASCII; /* Assume ASCII to begin (GPS stuff), SLIP mode we get a stream of bytes */
	#endif
	
	// Lock&Unlock mutex 
	//mutex_check("_slip_rx_handler acquire", pthread_mutex_lock(&rx_buf_mutex));
	

	while (!global_exit_all_threads)
	{
		
		//printf("reading my first char\n");
		#ifdef SIMULATOR
			ret = recvfrom( sock_fd , recv_data , MAX_80211_SIZE , 0, NULL , NULL ) ; /* blocking */
			//strcpy( line_buffer, recv_data , strlen( recvfrom) );
			//printf("got %dB\n" , ret );
			memcpy( line_buffer , recv_data , ret ) ;
			line_buffer[ret] = '\0' ;
			_drk_parse_line(line_buffer, ret ); /* in : expansion_sensor_serial_handlers.c */
		#else
			c = 0 ;
			ret = read(port_fd, &c, 1) ; /* nonblocking */
			/* Correctly read the first byte  */
			if (ret > 0)
			{
				//printf("got something");
				//If receive the START character, it's SLIP 
				if (c == START)
				{
					//printf("START received\n");
					mode = SLIP;
				}
				
				// Non-radio packets (like gps, thermal , etc)
				switch (mode)
				{
					case ASCII :
						// printf("[%c]",c);
						line_buffer[serial_index++] = c;
						if (c == '\n')
						{ 
							line_buffer[serial_index++] = '\0';
							//printf("SSS@ \n\nascii: ==[%s]==\n",line_buffer); 
							_drk_parse_line( line_buffer , 0 ); // in :	expansion_sensor_serial_handlers.c
							serial_index = 0;
						}
						break ;
					
					// Radio Packets 
					case SLIP:
						//_slip_rx_handler(port_fd); /* handle the following bytes */
						mode = ASCII;
						break ;
						//sleep(1);
					}
			}
			else
			{
				usleep(100); //TODO: serial's non blocking to allow the thread to terminate whenever needed. another solution should be designed
				/* serial read() failed  to read stuff. this will happen a lot, since the port is non blocking */
			}
		#endif
	} 
	
	/* someone said we should terminate all threads! */
	
	if ( serial_close( ) < 0 )
		PRINTF_FL_ERR("Failed closing serial module\n"); // success
	
	return NULL ; /* failed */
}

int serial_close ( void ) 
{
	SERIAL_LOADED_CONDITION 2;
	/* close usb port */
	{
		int ret;
		#ifdef SIMULATOR
		ret = close( sock_fd ); 
		#else
		if ( port_fd == 0 )
			return -1 ;
		ret = close( port_fd ); 
		#endif
		if ( ret == 0 )
		{
			PRINTF_FL( "\tSerial port - Closed\n");
		}
		else
		{
			PRINTF_FL_ERR( "\tSerial port - close() FAILED \n");
			return -2 ;
		}
	}
	
	/* delete Serial mutex */
	{
		int ret = pthread_mutex_destroy( &(serial_buf->gps_mutex) );
		if ( ret == 0 )
		{
			PRINTF_FL( "\tGps_mutex destroyed\n");
		}
		else
		{
			PRINTF_FL( "\tGps_mutex failed destroy %d:\t errno(%d) %s\n", ret , errno, strerror(errno));
			return -2 ;
		}
	}
	
	return 1 ;
}
/**
 * Open serial USB port <usbportnum
 * this is called from drk.c @ drk_init() 
 ***/
//int8_t expansion_serial_init( void )
//{

	
	///*
	//if ((serial_buf->semaphore = sem_open("serial_sem", O_CREAT, 0777, 1)) == SEM_FAILED)
	//{
	//printf("ESSH@ Semaphore failed\n");
	//drk_exit(EXIT_FAILURE);
	//}
	//*/
	//// Initialize sensor structures:
	///*
	//gps_t temp_gps;
	//pthread_mutex_lock(&gps_mutex);
	//memcpy(&(serial_buf->gps_buf), &temp_gps , sizeof(temp_gps));
	//pthread_mutex_unlock(&gps_mutex);
	//*/
	////struct infrared temp_ir = {0,0,0,0,0};
	////memcpy(&(serial_buf->infrared_buf), &temp_ir, sizeof(temp_ir));

	////serial_buf->grideye_buf.index = 0;
	////serial_buf->grideye_buf.frame = 0;
	////(serial_buf->read_buf_pos) = 0;
	////(serial_buf->write_buf_pos) = 0;
	////memset(serial_buf->packet_udp_buf,'\0' ,sizeof(struct drk_udp_packet) * PACKET_BUF);
	////memset(serial_buf->packet_buf,'\0' ,sizeof(struct drk_packet) * PACKET_BUF);
	////sem_post(serial_buf->semaphore);
  
	//return 1;
//}

// Wrapper function for receiving data 
 // Uses mutex and conditional variables
 // TODO: ADD DEFRAGMENTATION
 //
/*
void _slip_rx_handler(int fd)
{

	uint8_t recv = 0; 
	
	// Lock mutex to prevent user from reading while receiving
	pthread_mutex_lock(&rx_buf_mutex);

	//you should only read more data if the previous msg was rEad.
	if ((serial_buf->radio_buf).ready_to_read == 0)
	{
		// Receive message 
		recv = slip_rx(fd); // read the usb port, feed the buffer ..radio_buf).msg_buf
		(serial_buf->radio_buf).msg_buf_len = recv; // feed the lenght_buffer
		//printf("recv bytes:%d\n",recv);
		// If nothing read, unlock then return 
		if (recv <= 0)
		{
			//printf("Nothing read or packet discarded!\n");
			pthread_mutex_unlock(&rx_buf_mutex);
			return;
		}

		// Send signal to User
		// Allows them to copy (serial_buf->radio_buf).msg_buf
		//printf("YOU GOT MAIL\n");
		(serial_buf->radio_buf).ready_to_read = 1;
	}
	else
	{
		//printf("user didnt read previous packet\n");
	}
	// Unlock 
	pthread_mutex_unlock(&rx_buf_mutex);

  //printf("leaving _slip_rx_handler\n");

	//                              //
	// TODO DEFRAG                  
	//  *Check missing fragments    
	//  *Reconstruct packet         
	//  *Total Checksum Check       
}
*/

/* checks mutex locks and unlocks for errors 
 * prints message if mutex function calls fail
 */ 


/* Receives radio message */
// Returns: Size of message received (max 8bits = 1B)
// call this using mutex for radio_buf.
/*
uint8_t slip_rx(int fd)
{
	uint8_t c=0;
	int res=0;
	uint8_t i, received=0, size=0;
	// Already received a SLIP start
	int mode = SLIP;

	//printf("SLIP:\n");
	// Fill in the buffer unless you see END or ESC
	while (mode == SLIP) {
		// get a character to process
		res = read (fd, &c, 1);
		
		if (res > 0)
		{
			//printf("<%d>",c);
			// handle byte-stuffing if necessary
			switch (c)
			{
				// if it's an END character then we're done with
				// the packet
				case END:
					// a minor optimization: if there is no
					// data in the packet, ignore it. This is
					// meant to avoid bothering IP with all
					// the empty packets generated by the
					// duplicate END characters which are in
					// turn sent to try to detect line noise.
					if (received)
					{
						uint8_t checksum;
					
						size = (serial_buf->radio_buf).msg_buf[0];
				
						if (received - 2 != size) {
							printf ("\n*** SLIP rx size mismatch %d vs %d\n", received - 2, size);
							mode = ASCII;
							size = 0;
							break;
						}
						checksum = 0;
	
						for (i = 1; i < received - 1; i++) {
							checksum += (serial_buf->radio_buf).msg_buf[i];
						} 
			
						checksum &= 0x7F;

		
						if (checksum != (serial_buf->radio_buf).msg_buf[received - 1])
						{
							printf ("\n*** SLIP rx checksum error %d != %d...\n", checksum,
									(serial_buf->radio_buf).msg_buf[received - 1]);
							mode = ASCII;
							size =0;
				
							break;
						}

					}
					//printf("<END>\n");
					mode = ASCII;
					break;

				// if it's the same code as an ESC character, wait
				// and get another character and then figure out
				// what to store in the packet based on that.
				case ESC:
					//printf("[ESC]\n");
					do {
						res = read (fd, &c, 1);
					} while (res < 1);
					// if "c" is not one of these two, then we
					// have a protocol violation.  The best bet
					// seems to be to leave the byte alone and
					// just stuff it into the packet
					switch (c) {
						case ESC_END:
							c = END;
							break;
						case ESC_ESC:
							c = ESC;
							break;
					}

				// here we fall into the default handler and let
				// it store the character for us
				default:
					if (received < MAX_SLIP_BUF)
					{
		
						(serial_buf->radio_buf).msg_buf[received] = c;
					
						received++;

						//if (received==1)
						//		printf("[%d]",c);
						//else
						//		printf("[%c]",c);
						
					}
			}
		}
		else //if not res>0
		{
			//printf("wntng fortherest\n");
		}
		
	}
	// Return the size of the full message
	// test if the received packet is an (START)-ACK-SEQNUM-CS-END
	if (size == 3 && (serial_buf->radio_buf).msg_buf[1] =='Z')
	{
		//printf("got a ack %d %d\n",(uint8_t)(serial_buf->radio_buf).msg_buf[2],(uint8_t)(serial_buf->radio_buf).msg_buf[3]);
		pthread_mutex_lock(&rx_ack_buf_mutex); //lock

		ack_buf[ack_buf_w_pointer] = (uint8_t)(serial_buf->radio_buf).msg_buf[3]   + ((uint8_t)(serial_buf->radio_buf).msg_buf[2] << 8);
		//printf("val %d",ack_buf[ack_buf_w_pointer]);

		ack_buf_w_pointer++;
		if (ack_buf_w_pointer>=ACK_BUF_SIZE) ack_buf_w_pointer=0; //correct the pointer if it is over the limit
		//printf("got an ACK %d\n", (((uint16_t)(serial_buf->radio_buf).msg_buf[2]) << 8) + (serial_buf->radio_buf).msg_buf[3]);
 		pthread_mutex_unlock(&rx_ack_buf_mutex); //unlock
		size=0;
	}

	return size;
}
*/
/*
//get a threadsafe copy of the radio_buf
// slip_server_cycle has to be running in a thread in drk.c! 
uint8_t drk_modem_rx(uint8_t * buf, HDR_S * hdr)
{
	uint8_t len=0;
	uint8_t temp_len=0;
	//lock rx_buf_mutex
	//printf("i am reading the buffer for new msgs\n");
	pthread_mutex_lock(&rx_buf_mutex);


	if ((serial_buf->radio_buf).ready_to_read == 1)
	{
		//copy global buffer
		memcpy(buf, (serial_buf->radio_buf).msg_buf+1, (serial_buf->radio_buf).msg_buf_len);
		(serial_buf->radio_buf).ready_to_read = 0; //i just read what you have to say
		len = (serial_buf->radio_buf).msg_buf_len;
		pthread_mutex_unlock(&rx_buf_mutex);
		//printf("unpacking %d\n",len);
		temp_len = unpack(hdr, len, buf);	
		//printf("%d done unpacking %dBytes\n",len,temp_len);
	}
	//unpack
	pthread_mutex_unlock(&rx_buf_mutex);

	//unlock 
	

	//and return
	return temp_len;
}
*/
// Function transmits message buf with length len through the radio
// Returns: 1 if success, in transferring message
//          0 if fail, in transferring message
// using mutex_tx to lock someone else to call this function simultaneously
//TODO: uint8_t
/*
int8_t drk_modem_tx(uint8_t * buf, uint8_t len, HDR_S * hdr)
{
  //uint8_t i;
	uint8_t tempbuf[len];
	memcpy(tempbuf, buf,len);

	//printf("\nIn drk_tx\n");		
	uint8_t p_len = 0;
	if ( pthread_mutex_trylock(&mutex_tx) == EBUSY ) //no one can use this function again, before returning
	{
		if (debug) printf("mutex_tx is in use. modem_tx locked!\n");
		return -1;
	}
	

	
	//pack the header with payload
	//return total number of bytes = hdr + payload
	p_len = pack(tempbuf, len, hdr);
	if (p_len<=0){
		printf("Fail to insert header\n");
	}

	//printf("packed %dB msg==%s==\n",p_len,buf);
	//printf("ready to send: ");

	//for (i=0;i<p_len;i++)
		//printf("{%d}",tempbuf[i]);
	//printf("\n");
	int8_t ret = _slip_tx_ack(tempbuf, p_len, NRETRIES); //returns 1=success or 0=fail
	if (ret == 1)
	{
		hdr->seq++;
		//printf("inc seq to %d\n",hdr->seq);
	}
	
	pthread_mutex_unlock(&mutex_tx);
	return ret;
	
}
*/
// Sends the message "new_buf" with size "size"
// Listens 5 times for ACK (acknowledgement from receiver)
// If it gets NACK, retries "retries" number of times
// Returns: 1 if successful in transferring message to the FIREFLY (not to the air)
//          0 if unsuccessful in transferring message to the FIREFLY (maybe serial bus was overloaded)


// send a slip msg and reads a ack from the ack_list
//1 success
//0 success
/*
int8_t _slip_tx_ack(uint8_t *new_buf, uint8_t size, uint8_t retries)
{
  (void)(retries); // suppress warnings

	uint8_t i=0;
	int8_t ret=0;
	uint16_t seq_num = ((uint16_t)new_buf[HDR_SEQ_0_INDEX] << 8) + new_buf[HDR_SEQ_1_INDEX] ;

	//printf("seq num in cause:%d\n",seq_num);




	_slip_tx(port_fd,new_buf,size); //trying to send again
	usleep(300000); //300 ms
	
	for (i=0;i<1;i++)
	{	
		//if (debug) printf("tx: reading ack\n");
		ret = slip_read_ack(seq_num); //read the list of threads and check if there is an ACK with the seq numb of the msg I just sent.
		//if (debug) printf("tx: slip_rx_ack returned %d\n",ret);
		
		if (ret == 1) break;
		
	}		

	return ret;
}
*/
/*
//this function just reads the ack buffer list.
int8_t slip_read_ack(uint16_t seq_num)
{
	uint8_t i = 0;
	int8_t ret =0;
	pthread_mutex_lock(&rx_ack_buf_mutex);
	for (i=0;i<ACK_BUF_SIZE;i++)
	{
		if (ack_buf[i] == seq_num)	
		{
			ret = 1;
			break;
		}
	}
	pthread_mutex_unlock(&rx_ack_buf_mutex);
	return ret;
}
*/
// 
/* Transmits radio messages */
// Message format:
// START-SIZE-MESSAGE-(ESC)-MESSAGE-CHECKSUM-END
// ESC could be there or not

//
/*
int8_t _slip_tx(int fd, uint8_t * tx_buf, uint8_t size)
{
	uint8_t i;
	uint8_t checksum;
	uint8_t c;
	// Make sure size is less than 128 so it doesn't act as a control
	// message
	if (size > 128)
		return 0;
	
	//printf("writing to serial port!\n");

	checksum = 0;
	// Send the start byte
	c = END;
	//TODO: this could block forever
	while(write (fd, &c, 1)<0);
	c = END;
	//TODO: this could block forever
	while(write (fd, &c, 1)<0);

	//printf("just send end\n");
	c = START;
	//TODO: this could block forever
	while(write (fd, &c, 1)<0);
	//printf("just send start\n");
	// Send size of message
	c = size;
	//TODO: this could block forever
	while(write (fd, &c, 1)<0);
	//printf("just send size\n");
	
	// Send payload and stuff bytes as needed
	for (i = 0; i < size; i++) {
		
		if (tx_buf[i] == END )
		{
			//printf("found an END. sending ESC ESCEND\n");
			c = ESC;
			while(write (fd, &c, 1)<0);
			checksum += END;
			c = ESC_END;
			while(write (fd, &c, 1)<0);
		
		}
		else if (tx_buf[i] == ESC )
		{
			//printf("found an ESC. sending ESC ESCESC\n");
			c = ESC;
			while(write (fd, &c, 1)<0);
			checksum += ESC;
			c = ESC_ESC;
			while(write (fd, &c, 1)<0);
			
		}
		else 
		{
			c = tx_buf[i];
			while(write (fd, &c, 1)<0);
			checksum += c;
		}
	}	
	// Make sure checksum is less than 128 so it doesn't act as a control
	// message
	checksum &= 0x7f;
	
	// Send checksum
	c = checksum;
	//printf("my checksum %d\n", c);
	while(write (fd, &c, 1)<0);

	// Send the end byte
	c = END;
	while(write (fd, &c, 1)<0);

  return 0;
}
*/
/*
HDR_S* conf_hdr(uint8_t version, uint16_t src_mac, uint8_t ttl)
{
	HDR_S *hdr_part = (HDR_S*) malloc(sizeof(HDR_S));
	hdr_part->version = version;
	hdr_part->seq = initseq;


	printf("Src IS %d\n",src_mac);
	hdr_part->src_mac = src_mac;

	//hdr_part->dst_mac = 59;
	hdr_part->ttl = MIN(ttl,30);
	//hdr_part->tdma_slot = 57;
	hdr_part->rssi = 'R';
	//hdr_part->checksum = 'C';

	return hdr_part;
}
*/
/*
uint8_t set_dst_tdma(HDR_S * hdr, uint16_t dst_mac, uint16_t tdma_slot)
{

	hdr->dst_mac = dst_mac; 
	hdr->tdma_slot = tdma_slot; 	

	printf("Settings: Dst %d, TDMAslot %d\n", dst_mac,tdma_slot);
	return 1;

}
*/
/*
// pack up the header into package
uint8_t pack(uint8_t * buf, uint8_t len, HDR_S * hdr){
	uint8_t p_len = len+HDR_SIZE;
	uint8_t tempbuf[p_len];
	memcpy(tempbuf+HDR_SIZE, buf,len);

	if (len>128-HDR_SIZE){
		printf("Too much payload\n");
		return 0;
	}

	//uint8_t i, checksum=0;	
	//TODO using size of HDR_S?
	

	//printf("Start packing .... p_len = %d\n", p_len);	
	if (buf == NULL || hdr == NULL ){
		return 0;
	}
	//shifting	

	for (i=0;i<len;i++ ){
		printf("[%c]", buf[i]);
	}
	printf("\n");
	
	for (i=HDR_SIZE;i<HDR_SIZE+len;i++ ){
		printf("[%c]", buf[i]);
	}
	printf("\n");

	tempbuf[HDR_VERSION_INDEX] = hdr -> version;
	tempbuf[HDR_SEQ_1_INDEX] = hdr -> seq & 0xff;
	tempbuf[HDR_SEQ_0_INDEX] = (hdr -> seq >> 8) & 0xff;
	tempbuf[HDR_SRC_MAC_1_INDEX] = hdr -> src_mac & 0xff;
	tempbuf[HDR_SRC_MAC_0_INDEX] = (hdr -> src_mac >> 8) & 0xff;
	tempbuf[HDR_DST_MAC_1_INDEX] = hdr -> dst_mac & 0xff;
	tempbuf[HDR_DST_MAC_0_INDEX] = (hdr -> dst_mac >> 8) & 0xff;
	tempbuf[HDR_TTL_INDEX] = hdr -> ttl;
	tempbuf[HDR_TDMA_SLOT_1_INDEX] = hdr -> tdma_slot & 0xff;
	tempbuf[HDR_TDMA_SLOT_0_INDEX] = (hdr -> tdma_slot >> 8) & 0xff;
	tempbuf[HDR_RSSI_INDEX] = hdr -> rssi;
	//calculate checksum
	//for (i=0; i<len; i++){
	//	checksum += tempbuf[i+HDR_SIZE];
	//}

	//uint8_t temp[200];
	//for (i=0;i<p_len;i++ ){
	//	printf("[%c]", buf[i]);
	//}
	//printf("\n");
	//memcpy(temp, buf, p_len);
	//sprintf(temp+p_len,"\0");	
	//printf("%s\n", temp+p_len);
	tempbuf[HDR_CHECKSUM_INDEX] = 'k';
	memcpy(buf,tempbuf,p_len);
	return p_len;	

	
}
*/
/*
// unpackage to remove the header
uint8_t unpack(HDR_S * hdr, uint8_t p_len, uint8_t * buf){
	//uint8_t i, sum=0;

	if (buf == NULL || hdr == NULL)
	{
		printf("null pointers\n");
		return 0;
	}
	if (p_len<HDR_SIZE)
	{
		printf("len non positve\n");
		return 0;
	}
	hdr -> version = buf[HDR_VERSION_INDEX];
	hdr -> seq = 
	((uint16_t)buf[HDR_SEQ_1_INDEX])|
	((uint16_t)buf[HDR_SEQ_0_INDEX] << 8);

	hdr -> src_mac = 
	((uint16_t)buf[HDR_SRC_MAC_1_INDEX]) |
	((uint16_t)buf[HDR_SRC_MAC_0_INDEX] << 8) ;

	hdr -> dst_mac = 
	((uint16_t)buf[HDR_DST_MAC_1_INDEX]) |
	((uint16_t)buf[HDR_DST_MAC_0_INDEX] << 8) ;

	hdr -> ttl = buf[HDR_TTL_INDEX];
	hdr -> tdma_slot = 
	((uint16_t)buf[HDR_TDMA_SLOT_1_INDEX]) |
	((uint16_t)buf[HDR_TDMA_SLOT_0_INDEX] << 8) ;


	//TODO find rssi
	hdr -> rssi = ((uint16_t)buf[HDR_RSSI_INDEX]);
	hdr -> checksum = buf[HDR_CHECKSUM_INDEX];
	//calculate checksum of the header
	//for(i=0;i<hdr_size-1;i++){
	//	sum += buf[i];
	//}	

	//if(sum == hdr->checksum){
	
	//for (i=hdr_size;i<p_len;i++)
	//	printf("[%c]",buf[i]);
	//printf("\n");

	//printf("msg %s, #bytes %d\n",buf+hdr_size, p_len-hdr_size);

	//copy the buf
	uint8_t tempmsg[p_len-HDR_SIZE]; //
	memcpy(tempmsg, buf+HDR_SIZE, p_len-HDR_SIZE);

	//printf("msg %s\n",buf);
	//	return p_len-hdr_size;
	//}
	//else
	//{
	//	printf("HDR Checksum Error\n");
		//return 0;
	//}

	//pasting the buf
	buf[p_len-HDR_SIZE]='\0';
	memcpy(buf,tempmsg,p_len-HDR_SIZE);
	//TODO still return the correct now for debug
	return p_len-HDR_SIZE;
}
*/
