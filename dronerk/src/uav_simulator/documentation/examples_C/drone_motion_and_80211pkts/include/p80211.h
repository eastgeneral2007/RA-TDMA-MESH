#ifndef P80211_H_
#define P80211_H_


#include <inttypes.h> // int16_t etc

#define PDR_ 		1
#define GENERIC_ 	2
typedef struct {
	uint16_t payload_length ; /* how many more bytes are part of this packet besides header. */
	uint16_t seq_num ; /* unique identifier of a packet */
	uint8_t origin ; /* ID whihc originated this packet */
	uint8_t sink ; /* final destination ID of this packet */
	uint8_t src; /* previous hop ID */
	uint8_t dst; /* next hop ID for this packet */
} p80211_header_t ;

typedef struct {
	p80211_header_t header;
	uint8_t type; /* = PDR_*/
	uint8_t pdr; /* pdr value in % (from 0 to 100) of link src-->dst*/
	uint8_t src; /* link in question*/
	uint8_t dst; /*link in question*/
} p80211_pdrprobe_t ; 


typedef struct {
	p80211_header_t header;
	uint8_t type; /* = GENERIC_ */
	uint8_t payload[200];
} p80211_traffic_t ; 


typedef struct {
	uint16_t seq_num;
	uint8_t payload[200];
} p80211_simple_t ; 



#endif
