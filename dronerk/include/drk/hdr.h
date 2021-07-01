/**
 * hdr.h
 */

#ifndef _HDR_H_
#define _HDR_H_

#include<stdint.h>

//data length

#define HDR_VERSION		1
#define HDR_SEQ			2
#define HDR_SRC_MAC		2
#define HDR_DST_MAC		2
#define HDR_TTL			1
#define HDR_TDMA_SLOT		2
#define HDR_RSSI		1
#define HDR_CHECKSUM		1


//index

#define HDR_VERSION_INDEX		0
#define HDR_SEQ_0_INDEX			1
#define HDR_SEQ_1_INDEX			2
#define HDR_SRC_MAC_0_INDEX		3
#define HDR_SRC_MAC_1_INDEX		4
#define HDR_DST_MAC_0_INDEX		5
#define HDR_DST_MAC_1_INDEX		6
#define HDR_TTL_INDEX			7
#define HDR_TDMA_SLOT_0_INDEX		8
#define HDR_TDMA_SLOT_1_INDEX		9
#define HDR_RSSI_INDEX			10
#define HDR_CHECKSUM_INDEX		11
#define HDR_SIZE			12


typedef struct {
	uint8_t version;
	uint16_t seq;
	uint16_t src_mac;
	uint16_t dst_mac;
	uint8_t ttl;
	uint16_t tdma_slot;
	uint8_t rssi;
	uint8_t checksum;
} HDR_S;

#endif // _HDR_H_
