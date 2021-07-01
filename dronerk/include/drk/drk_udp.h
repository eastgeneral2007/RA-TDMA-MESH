/**
 * drk_udp.h
 */

#ifndef _DRK_UDP_H_
#define _DRK_UDP_H_

#include <stdint.h>

#define UDP_SERVER_PORT 8000
#define BROADCAST_TRIES 1
#define MAX_PACKET_LENGTH 1024
#define MAX_FRAGMENT_LENGTH 80
#define PACKET_IDENT "$RAD:"
#define PACKET_IDENT_LEN 5
#define DEFAULT_TTL 5
#define HEADER_LEN 5


/*Defined Control Packets*/
#define TAKEOFF_PACKET          "TaKeOfF"
#define LAND_PACKET             "LaNd"
#define MOVE_FORWARD_PACKET     "MoVeFoRwArD"
#define MOVE_BACKWARD_PACKET    "MoVeBaCkWaRd"
#define MOVE_LEFT_PACKET        "MoVeLeFt"
#define MOVE_RIGHT_PACKET       "MoVeRiGhT"
#define SPIN_LEFT_PACKET        "SpInLeFt"
#define SPIN_RIGHT_PACKET       "SpInRiGhT"
#define MOVE_UP_PACKET          "MoVeUp"
#define MOVE_DOWN_PACKET        "MoVeDoWn"   

#define LAND_PACKET_LEN             4
#define TAKEOFF_PACKET_LEN          7
#define MOVE_FORWARD_PACKET_LEN     11
#define MOVE_BACKWARD_PACKET_LEN    12
#define MOVE_LEFT_PACKET_LEN        8
#define MOVE_RIGHT_PACKET_LEN       9
#define SPIN_LEFT_PACKET_LEN        8
#define SPIN_RIGHT_PACKET_LEN       9
#define MOVE_UP_PACKET_LEN          6
#define MOVE_DOWN_PACKET_LEN        8   

extern int rec_seq;
extern uint8_t curr_id;

/* Transmits the last received udp packet by the udp_server 
 * BROADCAST_TRIES times over serial to the firefly.
 * If packet length > packet fragment size then send
 * fragmented packets over serial */

/*PACKET STRUCTURE:
    $RAD_PACKET:[PACKET ID][TTL][SEQ_NUM][SEQ_TOTAL][RSSI][DATA][\r][\n]
 */ 

/*Checks if drone received control packet and sends command to drone internally*/
int drk_check_control_packet(uint8_t *buf,int size);

#endif // _DRK_UDP_H_
