//event_scheduler_priv.h
/* 
 * to be included ONLY by event_scheduler.c 
 * */
#define PREFIX			"Schdlr"


//struct periodic_info
//{
	//int timer_fd;
	//unsigned long long wakeups_missed;
//};

enum event_types {
	PACKET_80211 	= 1 ,
	ISENSOR_READING	= 2 ,
	ESENSOR_READING	= 3 ,
	ACTUATION		= 4 ,
	UNKNOWN			= -1
} ;

static const char event_types_strings[4][8]={
	"n/a",
	"80211",
	"ESENSOR",
	"ISENSOR" 
}; //todo use define 

/************* 
 * evil static globals 
 * *********/
static event_t 			event_list[LIST_SIZE]; /* buffer with all the events registered. reading/writting random acess */
static qidx_t 			w_ptr = 0 ; /* writer porinter to the event list */
static pthread_mutex_t 	mutex_list; /* mutex to read/write from the event list */

//typedef struct {
	//static event_t 			event_list[LIST_SIZE];
	//static qidx_t 			w_ptr = 0 ;
	//static pthread_mutex_t 	mutex_list = PTHREAD_MUTEX_INITIALIZER; 
//} list_t;

static volatile uint8_t g_num_threads = 0; /* store total number of threads = uavs */
static FILE 			*file_log_out;
static FILE 			*file_log_in;

/************** 
 * internal prototypes
 * ***********/
//static void wait_period (struct periodic_info *info) ;
//static int 	make_periodic (unsigned int period, struct periodic_info *info) ;

static void			*scheduler_reader_cycle() ; /* cycle (thread) to deal with all current scheduled events */
static void			*UDP_thread_start() ; /* thread that will read the incoming UDP pckts - top part */
static int 			UDP_thread_cycle(void) ; /* thread that will read the incoming UDP pckts - body of while(1) */

/* Manage **EventList**: add , remove,  print, etc, events from the list */
static void 		event_clearlist(void);
static qidx_t 		event_schedule(event_t event); /* add an event to the list thread safe  */
static void 		event_delete(qidx_t qidx); /* remove an event from the list - thread unsafe */
static void 		event_log(qidx_t qidx); /* log an event to logfile - thread unsafe */
static void 		event_postpone( qidx_t qidx, uint64_t time_us); /* - thread unsafe */
static uint64_t 	event_getAddTime(qidx_t qidx); /* - thread unsafe */
static uint64_t 	event_getTrigTime(qidx_t qidx); /* - thread unsafe */
static qidx_t 		event_getnext(void); /* get next event IDX - find which event is to be triggered at the current time (if there's one) */
static enum event_types event_gettype( uint16_t psrc ) ; /* get type of newly received event by checking its port */
static int	 		isPacketCorrupted(qidx_t idx1,qidx_t *idx2);

/* plot events */
static void plot_events(int8_t type);
static void plot_80211events(void);
static void event_print(event_t *event_ptr); /* print a given event */

/* send packets */
static int sendPacket( int src, int dst , int port , void *ptr, int len);/* send a string to dst on a given port */
static void send80211Packet(	event_t *event_ptr, qidx_t qevent, uint8_t dst);
static void sendESensorPacket(	event_t *event_ptr, qidx_t qevent, uint8_t dst);
static void sendISensorPacket(	event_t *event_ptr, qidx_t qevent, uint8_t dst);

/* recved message from outside */
static int parser_actuation(packet_t pkt );
static int parser_80211(packet_t pkt);


#ifdef DEBUG_ACT
static void printf_cmd( char *string, int maxlen ) ; /* print rcvd actuation command */
#endif

//static int openLogFile( char * prefix , char * sufix , FILE ** file );

/* keep track of the last event added */
qidx_t last_event_idx = 0 ;
