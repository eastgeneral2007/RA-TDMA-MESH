#ifndef _TDMA_UTILS_H_
#define _TDMA_UTILS_H_
/********************
 * TDMA utils
 *******************/
/**
 * Created by : Luis Pinto
 * date: 16 nov 2017 18:40 GMT
 */
#define HIDE	__attribute__((visibility ("hidden")))


#include "utils.h"
#include "tdma_types.h"

#include <inttypes.h>
#include <unistd.h>




#ifdef VERBOSE
#define PRINTSTATS
#endif



/***************** 
 * externs to be used only by <tdma*> modules 
 * *****/
#if defined(TDMA_IMPORT) || \
defined(TDMA_SLOT_IMPORT) || \
defined(TDMA_UTILS_IMPORT)
extern volatile bandwidth_t 	Bandwidth ; /* estimation of bandiwdth to/from other neighbor nodes */
extern volatile param_tdma_t 	Param_tdma ; /* all TDMA *constants* are saved here, loaded from a file */
extern volatile tdma_stats_t	Tdma_statistics ; /* save stats here  */
extern volatile tdma_header_t Header; /*header from a packet */
extern volatile int				Tdma_initiated; 
#endif





/*************************************
 *  Function prototypes here. 
 ********************************/

/***** 
 * Public functions to everyone. 
 * start with TDMA_ 
 * ****/
 
 
#ifdef VERBOSE
void 	TDMA_printSettings( void ) ; 
#endif 


param_tdma_t 	TDMA_getParams( void ) ; /* get current TDMA param struct */


bandwidth_t TDMA_getBandwidth( void ) ;

uint16_t 	TDMA_getOwnSlotWidth( void ) ; /*  get our own slot length (ms) */
slot_limits_t TDMA_getOwnSlot( void ) ; /* get a safe copy of current slot */

/*
 *  Adicionado Dino
 */
uint32_t TDMA_getSeqNum( void );
int TDMA_getRoundTime( void );
void sleepToStart ( void );
/*----------------------------------*/

/** public stuff ends here **/

/************
 *  FUNCTIONS TO BE USED ONLY BY TDMA* MODULES 
 * semi-private
 * ***********/
#if defined(TDMA_IMPORT) ||\
 defined(TDMA_SLOT_IMPORT) ||\
 defined(TDMA_UTILS_IMPORT)

#ifdef VERBOSE
char*	tdma_printHeader( tdma_header_t *tmp_h_pkt );
#endif
uint16_t tdma_getSlotWidth( slot_limits_t slot ) ; /* get slot width (ms) based on its limits and PERIOD */

/* calls loadThroughputSettings() and loadSlotSettings(): */
HIDE error_t tdma_loadSettings( uint8_t my_id ) ; 
/* computes position of a given pkt within its slot*/
HIDE float 	tdma_getMsgPosition(tdma_header_t tdma_header);

HIDE uint16_t tdma_getType(const tdma_header_t * const header);
HIDE uint16_t tdma_getPrio(const tdma_header_t * const header);
// static int isRxBufferEmpty( void  ) ;
/** Log data **/
#ifdef LOGDATA
HIDE void tdma_logPacket( 
	FILE *logfile, 
	tdma_header_t *header, 
	double timestamp , 
	uint8_t src_ip, uint8_t dest_ip, 
	uint16_t tdma_pkt_len, 
	uint16_t buf_use, 
	float bandwidth ,
	int32_t delay ) ;/* log a rx or tx packet to a file */
#endif

/** gets **/
HIDE double 		tdma_getCurrentRoundTimeD( void ); /* ms , double ; negative=error */
HIDE int32_t 		tdma_getCurrentRoundTime( void ) ; /* ms ; negative=error*/
HIDE int32_t 		tdma_getDelay( tdma_header_t tdma_header ) ; /* ms */
HIDE useconds_t 	tdma_getTimeTillSlotBegin(void);
HIDE useconds_t 	tdma_getTimeTillSlotEnd(void);

HIDE void 			tdma_sleepTillSlotBegin(void); /* sleep */
HIDE int 			tdma_waitTillBufFree( int socket, int timeout ); /* microsleep while buffer is not empty again */



/** prints **/
HIDE int	tdma_debugInsideSlot(void);
#ifdef VERBOSE
HIDE void 	tdma_printSlot( const slot_limits_t slot ) ; /* Print slot limits */
#endif
#ifdef PRINTSTATS
HIDE void 	tdma_printStats( void ); /* print current statistics */
HIDE void 	tdma_printBout( uint8_t ip ) ;
HIDE void 	tdma_printBin( uint8_t ip ) ;
#endif

HIDE void 	tdma_clearStats( void );

/* update stats when sending a packet */
HIDE error_t tdma_updateTxStats( 
	uint16_t bytes_sent, 
	uint8_t dest_IP, 
	uint16_t timestamp ) ;
	



/***************
 * 
 * statistics ,. bandwidth , etc 
 * maybe this should join pdr and be named "tdma etrics module"
 */
/* get current bandiwdith estimates */
HIDE float 		tdma_getBandwidthOut( uint8_t ip ) ;
HIDE float 		tdma_getBandwidthIn( uint8_t ip ) ;
 
/* */ 
HIDE void 		tdma_updateRxStats( void );
HIDE int 		tdma_updateClockStat2( void );
HIDE tdma_stats_t tdma_getStats( void ) ;


HIDE int 	tdma_insideSlot(void) ; /* true/false : check if we are inside timeslot*/
HIDE int 	tdma_insideSlot_guarded(void) ; /* true/false : check if we are inside timeslot, some guard left*/


#endif /* if any tdma* import */








/*************************************************************************************** 
 * Prototypes - Internal Local Functions - to be used only by this unit
 * PRIVATE
 *************************************************************************************/
#ifdef TDMA_UTILS_IMPORT

//static HIDE int64_t 	getMsgPosition( tdma_header_t tdma_header ) ; /* timespan (ms) from slot init */



static error_t 	loadThroughputSettings( void ) ;
static error_t 	loadSlotSettings( void ) ; 




#if LOGDATA
/* read pm layer seqnum to make log analysis simpler task */
static void		buildPMSeqnumString( void* tdmapkt_ptr, char* other_string ) ;
#endif

#endif /* ifdef TDMA_UTILS_IMPORT */












#endif
