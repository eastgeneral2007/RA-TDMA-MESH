/********************
TDMA slot - changes in slot length,position,etc are done by this guy
*******************/
/**
 * Created by : Luis Pinto
 * date: 16 nov 2017 18:40 GMT
 */
#ifndef _TDMA_SLOT_H_
#define _TDMA_SLOT_H_

#define HIDE	__attribute__((visibility ("hidden")))

#include <inttypes.h>
#include "utils.h"
#include "tdma_types.h" /* all structs are here */





/***************************
 * PUBLIC functions 
 * <TDMA_> prefix
 * **/
uint8_t 		TDMA_dwStream( void ) ;
uint8_t 		TDMA_upStream( void ) ;



#if VSP
/** 
 * @brief set slot width to any neighbor IP **/
void TDMA_reqSlotWidth( uint16_t req_ms, uint8_t dst_ip ) ; 

/** 
 * @brief provide a new slot list
 * 
 * @param slot_list , pointer to array of 
 * node ids that are active on the round 
 * @param len , size of the list
 */
error_t	TDMA_newSlotList( 
	const uint8_t * const slot_list, 
	const int len );
#endif	
	
	
/*************
 * FUNCTIONS TO be used by <TDMA_> MODULES 
 * style: all prefixed by tdma_ (smallcaps) 
 * semiprivate
 * ***********/
#if defined(TDMA_IMPORT) || \
defined(TDMA_SLOT_IMPORT) || \
defined(TDMA_UTILS_IMPORT)
	
HIDE int16_t 		tdma_getRequestedWidth( uint8_t ip );


HIDE slot_limits_t 	tdma_getOwnSlot_unsafe( void ) ; /* no mutex */
HIDE slot_limits_t 	tdma_getOwnSlot_guarded( void ) ; /* safe slot, but stops earlier: some guard period */

HIDE void 			tdma_clearStuff( void );
HIDE error_t		tdma_initSlotLimits( void ) ; /* init slot limits */
HIDE error_t		tdma_recordPktDelay( tdma_header_t tdma_header ) ; //old: int8_t saveDeltas 
HIDE int			tdma_syncronizeSlot( int32_t *delta ); /* set new TDMA slot limits based on saved delays, returns delta */
HIDE void			tdma_setIp2SlotId( uint8_t ip, uint8_t id ); /* Set slot id associated with a given IP */
//HIDE uint16_t 	tdma_getIp2SlotWidth( uint8_t ip );/* get Slot-width (ms), given the IP of a node */

HIDE uint8_t		tdma_getIp_givenId( uint8_t slot_id ); /* get IP from slot id */
HIDE int32_t		tdma_getLastRecordedPktDelay(void);

HIDE void 			tdma_slot_setstd();

#if VSP
HIDE void 		tdma_runRequests( tdma_header_t tdma_header ) ;
HIDE error_t		tdma_makeRequest( void ) ;
#endif




#endif /* <tdma*> imports */




/***************** 
 * <TDMA_SLOT> Private Stuff 
 ***************** **/
#ifdef TDMA_SLOT_IMPORT

/** set my new slot, safely  **/
static void setOwnSlot_safe( slot_limits_t tmpslot );
static void shiftSlot( slot_limits_t *tmpslot, int32_t delta);

/** Table get IP - slotID - Slot IP **/
static uint8_t 	getId_givenIp( uint8_t ip );
static uint16_t getSlotWidth_givenIp( uint8_t ip );/** update knowledge **/
static uint16_t getSlotWidth_givenId( uint8_t slot_id );/** update knowledge **/
static void setSlotWidth( uint8_t id, uint16_t width_ms );

#if VSP

static int 	underOutgoingRequest(void);
static int 	underIncomingRequest(void);
static void	clearRequests(void);
static void processIncomingRequest(tdma_header_t tdma_header);
static void processOutgoingRequest(tdma_header_t tdma_header);
#endif //if VSP

#endif //ifdef TDMA_SLOT_IMPORT













#endif
