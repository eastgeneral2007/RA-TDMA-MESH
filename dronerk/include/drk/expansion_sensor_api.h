/**
 * expansion_sensor_api.h
 * Nat Storer - 6/2/2011
 * Read values from our external sensor board
 * aka ESA.h
 */
#ifndef EXPANSION_SENSOR_API_H
#define EXPANSION_SENSOR_API_H

#include <stdint.h>
#include <time.h>
#include <semaphore.h>
#include <pthread.h>

#include "drk/drk_udp.h"
#include "drk/gps.h"

/* Serial constants */
//#define BAUDRATE	B115200
//#define DATABITS	CS8	
//#define STOPBITS	CSTOPB
//#define PARITYON	0
//#define PARITY	0


#define SERIAL_STATE_FILE "/serial.shared"

/* Print error messages */
#define DEBUG 0
#define SERIAL_PRINT 1
#define OSS 3

/* Device to use */
//#define	USB_PORT	"/dev/ttyUSB0"	// For use on the drone

/* Other useful constants */
#define	SENSOR_LINE_LENGTH		(160)
#define PACKET_LENGTH			(80)
#define PACKET_BUF              (50)
#define STACKSIZE 				(512000)

#define MAX_PAYLOAD				(64)
#define HEADER_BYTES	2

#define DRK_ACK			2
#define DRK_STATE		3

#define PIKSI_FLOAT 0
#define PIKSI_FIXED 1



struct infrared
{
	unsigned int index;
	int front;
	int left;
	int back;
	int right;
};

struct grideye
{
	unsigned int index;
	unsigned int frame;
	double temperature[8][8];
};

struct drk_packet
{
	uint8_t 	        protocol_id;
	uint8_t 	        priority;
	uint32_t 	        src_mac;
	uint32_t 	        dest_mac;
	uint8_t		        seq_num;
	uint8_t		        ttl;
	uint32_t	        time_secs;
	uint32_t	        time_nsecs;
	uint32_t	        last_hop;
	uint32_t	        next_hop;
	uint8_t 	        payload_len;
	char 		        payload[MAX_PAYLOAD];
	char 		        rssi;
	uint8_t 	        checksum;
	struct timespec   receive_time;
};

struct drk_udp_packet
{
  uint8_t           packet_id;
  uint8_t           ttl;
  uint8_t           seq_num;
  uint8_t           seq_total;
  uint8_t           rssi;
  uint8_t           payload_len;
  char 		        data[MAX_FRAGMENT_LENGTH];
  struct timespec   receive_time;
};

//this one is being used
struct drk_radio_packet
{
	uint8_t 	ready_to_read;
	uint8_t 	packet_id;
	uint8_t 	ttl;
	uint8_t 	src;
	uint8_t 	dest;
	uint8_t 	seq_num;
	uint8_t 	seq_total;
	char  		msg_buf[MAX_FRAGMENT_LENGTH];
	uint16_t	msg_buf_len;
	uint8_t		checksum;
};

struct global_serial
{
	pthread_mutex_t			gps_mutex;
	struct infrared			infrared_buf;
	gps_t					gps_buf;
	struct drk_packet		packet_buf[PACKET_BUF];
	struct drk_udp_packet	packet_udp_buf[PACKET_BUF];
	struct drk_radio_packet	radio_buf;
	struct grideye			grideye_buf;
	uint32_t 				read_buf_pos;
	uint32_t 				write_buf_pos; 
};


/* Struct needed to pass in all parameters for thread safe
 * version of goto_coordinate
 */
/*
struct goto_coordinate_param
{
    double t_lat;        		// Target Latitude            
    double tlon;        		// Target Longitude           
    double talt;        		// Target Altitued above ground (meters)  
    double max_speed;   		// Maximum Speed (rate from 0.0-1.0)      
    double distance_error;  // Drone lands within this distance of target
};
*/

/* Global variable to stop goto_coordinate at any time.
 * Usage:
 *     Calls to drk_gps_goto_coordinate return thread id or 0
 *     Store this thread id (tid)
 *     Set stop = 1 to stop moving toward target
 *     Call pthread_join(tid, NULL); // This waits for thread to end
 *     Set stop = 0 to reset stop
 */
extern int stop;


extern struct global_serial *serial_buf; //TODO: instantiated in ESSH.c


/* initiate serial reader */
int8_t drk_serial_init( uint8_t usb_port ) ;

/* Expansions serial handlers function prototypes */
void drk_packet_create(struct drk_packet *temp_packet);
void drk_packet_set_header(struct drk_packet *packet, int header);
int drk_packet_get_header(struct drk_packet *packet);
void drk_packet_set_data(struct drk_packet *packet, char *data, int bytes);
int drk_packet_get(struct drk_packet *packet);
int drk_packet_send(struct drk_packet *packet);

void _drk_expansion_sensor_thread();
int drk_expansion_send_packet(char line[PACKET_LENGTH], int length);
void _drk_parse_line (char line[SENSOR_LINE_LENGTH], int bytesRead);
int _drk_radio_packet_handler(char line[PACKET_LENGTH]);
int _drk_udp_radio_packet_handler(char line[PACKET_LENGTH], int bytesRead);
int _drk_validate_packet (char packet[SENSOR_LINE_LENGTH]); 

//void _drk_gga_handler(char line[SENSOR_LINE_LENGTH]);
//void _drk_ggp_handler(char line[SENSOR_LINE_LENGTH]);
//void _drk_vtg_handler(char line[SENSOR_LINE_LENGTH]);
//void _drk_rmc_handler(char line[SENSOR_LINE_LENGTH]);

void _drk_grideye_handler(char line[SENSOR_LINE_LENGTH]);
void _drk_infrared_handler(char line[SENSOR_LINE_LENGTH]);
int _drk_expansion_gather_data();



/* Accessors for IR / Ultrasound data */
int _drk_dist_raw_front();
int _drk_dist_raw_back();
int _drk_dist_raw_left();
int _drk_dist_raw_right();

int drk_dist_IR_front();
int drk_dist_IR_back();
int drk_dist_IR_left();
int drk_dist_IR_right();

double drk_dist_US_front();
double drk_dist_US_back();
double drk_dist_US_left();
double drk_dist_US_right();



#endif
