/*  scheduler events */
/* this scheduler allows events to be added .
 * 
 * Each event can be set to trigger/ be used in a future moment
 * Simultaneous events can collide (such as packet transmissions)
 * in that case, packets are discared or postponed using CSMA backoff
 * 
 * times are all in microseconds (that's our resolution = 1us)
 * */
#include <errno.h>
#include <sys/timerfd.h>
#include <time.h> //nanosleep

#include "event_scheduler.h"
#include "utils.h"
#include "event_scheduler_priv.h"
#include "world_simulator_api.h" //get all ports
#include "sim_clock.h"		/* get time, etc */
#include "sim_network.h" 	/* get unicast tx time */
#include "sim_layout.h" 	/* updateLayout */


/* globals */
static int sendsockfd[MAX_NUAVS+1]; /* socket used to SEND packets */
static int recvsockfd ; /*rx-socket */

pthread_t reader_tid;
pthread_t udp_tid;

const struct timespec onenap = {0, 200};//0s, 200ns

static unsigned int g_ndrones = 0 ;

/***********************************************************
 ************************ CODE ******************************
 ************************************************************/
 
/* printf some data from this event */
/* depends on the its type */
static void event_print(event_t *event_ptr)
{
	switch (event_ptr->type)
	{
		case PACKET_80211:
		{
			PRINTF_FL(
				"<%03"PRIu32"> 80211pkt to #%02d (pyld:%zuB). "
				"TimeW [%"PRIu64"-%"PRIu64"]us\n", 
				event_ptr->id,
				event_ptr->dst, 
				event_ptr->length,
				event_ptr->added_timestamp, 
				event_ptr->when_to_trigger ) ;
			break ;
		}
		default:
			PRINTF_FL(
				"<%03"PRIu32">, "
				"Timestamp@ %"PRIu64"us-"
				"Trigger@ %"PRIu64"us, %s -- [%d][%d]\n", 
				event_ptr->id, 
				event_ptr->added_timestamp,
				event_ptr->when_to_trigger, 
				event_types_strings[event_ptr->type],
				event_ptr->src,
				event_ptr->dst ) ;
	}
}

/** 
 * find if there is an event at the current time,
 * return: -1, if there's none
 * return: event idx, if there is one.
 **/
static qidx_t event_getnext(void)
{
	qidx_t idx ;
	uint64_t current_time = SimClock_get() ; 
	pthread_mutex_lock(&mutex_list); // Lock - make sure we read a whole event without being modified along the way
	//PRINTF_FL("idx: %d min_when_to_trigger %" PRIu64 "us " , min_idx, min_when_to_trigger); 

	/* search for the earlier event */
	for (idx = 0; idx < LIST_SIZE; idx++) 
	{
		if (event_list[idx].id != 0 &&
			event_list[idx].when_to_trigger == current_time )
		{
			pthread_mutex_unlock(&mutex_list) ; // U
			//PRINTF_FL("something found *%" PRIu8 "* [%" PRIu64 "us]\n" , tmp_event.id , current_time ); 
			/* return event */
			return idx ;
		}
	}
	pthread_mutex_unlock(&mutex_list) ; // U
	return NONE ;
}

/* check if there is at least *one* transmission for the same moment of <idx1>*/
/* not thread safe - protect the queue */
/* return -1 -> there's no problem */
/* return idx of the packet that is also using the medium */
static int isPacketCorrupted( qidx_t idx1 , qidx_t *idx2)
{

	//qidx_t	 idx2 ;
	uint64_t end2 ;
	uint64_t begin2; 

	pthread_mutex_lock(&mutex_list);
	uint64_t begin1 = event_list[idx1].added_timestamp ;
	//uint64_t end1 	= event_list[idx1].when_to_trigger ;
	//PRINTF_FL("Len1 %.0f " ,end1-begin1  ) ; 
	
	/* search for conflicting events */
	for (*idx2 = 0; *idx2 < LIST_SIZE ; (*idx2)++)
	{
		begin2 	= event_list[*idx2].added_timestamp;
		end2 	= event_list[*idx2].when_to_trigger;

		//check if event 1 does not overlap event2
		
		/* same ID? ignore */
		if (event_list[idx1].id == event_list[*idx2].id)
			continue;
			
		/* only 80211 packtes can collide */
		if (event_list[*idx2].type != PACKET_80211 )
			continue;
			
		//PRINTF_FL(
			//"b1 e1 %ld %ld ; b2 e2 %ld %ld\n" , 
			//begin1, end1, begin2 , end2 ) ;
		//b1 e1 351 459 ; b2 e2 313 421

		if ((begin1 >= begin2) && (begin1 <= end2))
		{
			//printEvent ( event_list[idx] ) ;
			PRINTF_FL("mutual interference!!\n");
			//PRINTF_FL("\t\tmyself {Id *%d* (idx%d)} with {id *%d* (idx%d)}\n", 
				//	event_list[idx1].id , idx1 , event_list[idx2].id , idx2 );
			pthread_mutex_unlock(&mutex_list);
			return 1 ; /* mutual interf. */
		}
	}
	pthread_mutex_unlock(&mutex_list) ;
	return NONE;
}

static void event_log( qidx_t qidx )
{
	/* log it before deletion */
	fprintf( file_log_out, 
		"<id%03"PRIu32"> "
		"#%"PRIu8"->#%"PRIu8". "
		"Wnd[%"PRIu64 "-%"PRIu64 "]us. "
		"Type %s\n", 
		event_list[qidx].id ,
		event_list[qidx].src ,
		event_list[qidx].dst ,
		event_list[qidx].added_timestamp,
		event_list[qidx].when_to_trigger,
		event_types_strings[ event_list[qidx].type ] ) ;
	fflush(file_log_out) ;
}


/** 
 * Delete event from queue, setting simply the ID = 0 
 * in case there is payload, free that space
 * when_to_trigger should be kept to check interference with future other events
 * this is NOT thread safe - call mutex_lock ( &mutex_list ) before **/
static void event_delete( qidx_t qidx ) /* index */
{
	/* delete it */
	if ( event_list[qidx].id != 0 )
	{
		event_list[qidx].id = 0 ; /*null id means null event*/
		if ( event_list[qidx].pkt_ptr != NULL )
		{
			free( event_list[qidx].pkt_ptr ) ;
			event_list[qidx].pkt_ptr = NULL ;
		}
	}
	//PRINTF_FL("\tDeleted idx %d\n" , qidx ) ;
}

/* schedule a given event, return the idx where it was saved 
 * circular buffer: event_list 
 * thread safe 
 * input: a full event struct 
 * return: idx in the buffer */
qidx_t event_schedule(event_t event)
{
	static uint32_t event_seq_num = 0 ;
	//TODO: memcpy
	pthread_mutex_lock(&mutex_list); //L
	event_seq_num++ ;
	qidx_t idx=w_ptr;
	
	event_list[w_ptr].id = event_seq_num ;
	event_list[w_ptr].dst = event.dst ;
	event_list[w_ptr].src = event.src ;
	event_list[w_ptr].type = event.type ;
	event_list[w_ptr].when_to_trigger = event.when_to_trigger ;
	event_list[w_ptr].added_timestamp = event.added_timestamp ;
	
	switch (event.type)
	{
	case PACKET_80211:
	{
		event_list[w_ptr].CW = CWMIN/2 ;
		event_list[w_ptr].backoff = 0 ;
		event_list[w_ptr].length = event.length ;
		event_list[w_ptr].pkt_ptr = malloc( (size_t)event.length ) ; // alloc some memory for the 80211 payload
		memcpy( event_list[w_ptr].pkt_ptr, (const void *)event.pkt_ptr, (size_t)event.length ) ;
		break;
	}
	case ISENSOR_READING:
	{
		event_list[w_ptr].length = 0 ; // no need for this
		event_list[w_ptr].pkt_ptr = NULL ; // no need for this
		#if DEBUG_ISENSOR
		event_print( &event_list[w_ptr] ) ;
		#endif
		break;
	}
	case ESENSOR_READING:
	{
		event_list[w_ptr].length = 0 ; // no need for this
		event_list[w_ptr].pkt_ptr = NULL ; // no need for this
		#if DEBUG_ESENSOR
		event_print( &event_list[w_ptr] ) ;
		#endif
		break;
	}
	default:
		PRINTF_FL_WARN("Unknown TYPE\n");
	}
	
	//PRINTF_FL("tX event_list[%d].id = %d, event_list[%d].when_to_trigger = %" PRIu64 "\n" , 
	//	w_ptr , event_list[w_ptr].id , w_ptr ,	event_list[w_ptr].when_to_trigger ) ;
	
	/* find next index to write -- <w_ptr> */
	w_ptr++; w_ptr = w_ptr % LIST_SIZE ;
	
	if (event_list[w_ptr].id == 0) /* free index , all good */
	{
		pthread_mutex_unlock( &mutex_list ) ; // U
		return idx ; /* return newly written index */
	}
	
	
	/* if <w_ptr> is occupied then */
	PRINTF_FL("queue is becoming packed. wrote to qidx=%"QIDX_FMT ". next item is occupied\n\n" , idx );

	/* search for a free spot in the queue */
	while ( event_list[w_ptr].id != 0 ) 
	{
		w_ptr++ ; w_ptr = w_ptr % LIST_SIZE ;
		if ( idx == w_ptr ) /* he did a full round :S */
		{
			PRINTF_FL_WARN("QUEUE 100%% full. All ids are full. w_ptr =%" QIDX_FMT "\n\n" , w_ptr );
			pthread_mutex_unlock(&mutex_list) ; // U
			event_dumpall();
			exit(1) ;
		}
	}
	pthread_mutex_unlock( &mutex_list ) ; // U
	return idx ; /* free index */

}

/* print all events on the screen */
void event_dumpall(void)
{	
	PRINTF_FL_WARN("---dump-starts---\n");
	pthread_mutex_lock ( &mutex_list ) ;  /* Lock - read atomically */
	for (qidx_t idx = 0 ; idx < LIST_SIZE ; idx++ )
		event_print(&event_list[idx]) ;
	pthread_mutex_unlock( &mutex_list ) ; // U
	PRINTF_FL_WARN("---dump-ends---\n");
}

#ifdef DEBUG_ACT
void printf_cmd(char *string, int maxlen)
{
	int string_len;//how much to read
	
	if (maxlen==0)
		string_len = strlen(string);
	else
		string_len = maxlen;
	PRINTF_FL_WARN("Reading %d\n", string_len);
	
	PRINTF_FL("cmd: {" ) ;
	for (int i = 0 ; i<string_len; i++)
		{
			switch ( string[i] )
			{
			case '\r': 
				printf("[\\r]") ;
				break;
			case '\n': 
				printf("[\\n]") ;
				break;
			case '\0': 
				printf("[\\0]") ;
				break;
			default:
				printf("%c", string[i]);
			}
		}
		printf("} (%dB)\n", string_len );
}
#endif


/* if we get a packet 802.11 to distribute,
 * trigger its action to unicasttime() after current time */
int parser_80211(packet_t pkt)
{
	
	PRINTF_FL_WARN( 
		"Rcvd a 80211msg. "
		"Src #%u, Destinated to #%u.\n",
		pkt.src,
		pkt.dst ) ;
	dumpData( pkt.data , pkt.lendata);
	PRINTF_FL("payload [");
	//sizeof(tdma_header_t) ; 
	for (unsigned int i = 0; i < pkt.lendata; i++ )
	{
		char curchar = *((char*)pkt.data+i);
		switch ( curchar )
		{
			case '\r': 
			{
				printf("[\\r]") ;
				break;
			}
			case '\n': 
			{
				printf("[\\n]") ;
				break;
			}
			case '\0': 
			{
				printf("[\\0]") ;
				break;
			}
			default:
			{
				if (curchar < 32 || curchar > 126 ) // not visible.
					printf("{%hhu}", curchar );
				else
					printf("%c", curchar ); 
			}
		}
	}
	printf("]\n");

	event_t event ; 
	event.id 		= 0;//UINT32_MAX ; /* not in the queue yet, so this number doesnt matter */
	event.type 		= PACKET_80211 ; /* 802.11 */
	event.dst 		= pkt.dst ;
	event.src 		= pkt.src ;
	event.length 	= pkt.lendata ;
	event.pkt_ptr 	= pkt.data ;
	
	 /* new event should be added TDIFS after current time. 
	  * that's when the packet will start being sent */  				
	event.added_timestamp = SimClock_pause();// + (uint64_t)(1e6*T_DIFS);
	//if ( last_event_idx != 0 ) /* new event should be added TDIFS after latest triggertime */
	//{
		//pthread_mutex_lock ( &mutex_list ) ; //L
		//event.added_timestamp = MAX( 
			//event.added_timestamp , 
			//event_list[ last_event_idx ].when_to_trigger + (uint64_t)(1e6*T_DIFS) );
		//pthread_mutex_unlock ( &mutex_list ) ; // U
	//}
	/*  whentotrigger is addedtime + airtime (us) */
	uint64_t airtime =  (uint64_t)(1e6*T_DIFS) + (uint64_t)( MILLION * network_getUnicastTime( event.length, 0 ) );
	event.when_to_trigger = event.added_timestamp + airtime ;
	/* insert in queue- set global <lasteventidx > */
	last_event_idx = event_schedule(event);
	
	//uint16_t pdst = (uint16_t)rx_port_number ;
	pthread_mutex_lock(&mutex_list) ;//L
	PRINTF_FL(
		"[CLOCK %07"PRIu64"us] "
		"Got new event:\n",
		event.added_timestamp);
	event_print( &event_list[last_event_idx]) ;
	event_log(last_event_idx);
	pthread_mutex_unlock(&mutex_list) ;// U
	SimClock_resume();

	return 1;
}



int parser_actuation(packet_t pkt)
{
	#if DEBUG_ACT
	PRINTF_FL(
		"[CLOCK %.4fs] "
		"Got command (%dB): [p%" PRIu16 "]->[p%" PRIu16 "]\n" , 
		SimClock_get()/1e6, 
		(int)pkt.lendata , 
		pkt.src ,  
		pkt.dst )  ;
	printf_cmd((char *)pkt.data, pkt.lendata); 
	#endif 

	long int timestamp;
	int flag ; 
	int ret;
	actuator_t actuation ;
	// AT*PCMD=54,1,0.000000,-0.900000,0.000000,-0.000000[\r][\r][\0]
	ret = sscanf( pkt.data , 
		"AT*PCMD=%ld,%d,%d,%d,%d,%d\r",
		&timestamp ,
		&flag , 
		(int*)&(actuation.roll) , 
		(int*)&(actuation.pitch),
		(int*)&(actuation.gaz) ,
		(int*)&(actuation.torque) ) ;
	
	
	if (6 == ret)
	{
		#if DEBUG_ACT
		PRINTF_FL("Setting actuation[#%d] => u=(%.1f,%.1f,%.1f) :: w=%.1fº :: timestmp: %ld\n" , 
			pkt.src , 
			actuation.roll , 
			actuation.pitch, 
			actuation.gaz , 
			actuation.torque , 
			timestamp ) ;
		#endif
		layout_setActuation( actuation, pkt.src-1 ) ;//layout starts at #0
		return 1;
	}

	
	
	int dummy;
	ret = sscanf( pkt.data , 
		"AT*REF=%d,290718208\r",&dummy) ;
	if (ret == 1)
	{
		layout_takeoff(pkt.src);
		return 1;
	}
	
	#if DEBUG_ACT
	printf_cmd((char *)pkt.data, pkt.lendata);
	#endif
	dumpData((uint8_t*)pkt.data, pkt.lendata);
	return -1;
}


/* thread cyle - read incoming UDP packets , actuation and 80211pkts */
int UDP_thread_cycle(void)
{
	//UDP stuff
	ssize_t 			num_bytes_read ; //for recvfrom()
	struct sockaddr_in 	si_me ;
	
	//get UDP socket 
	if ((recvsockfd = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP)) ==-1)
		PRINTF_FL_ERR("error getting udp socket\n");

	si_me.sin_family = AF_INET; //address format (ip4)
	si_me.sin_port = htons(SIM_RX_ALL); //simulator receives everything at this port
	si_me.sin_addr.s_addr = htonl(INADDR_ANY); //packets are destinated to any 127.0.0.X IP.

	//PRINTF_FL("[Recv Socket act bind]\n");
	int yes = 1 ; /* when app closes unexpectedly, this is useful */
	if (setsockopt(recvsockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1)
		PRINTF_FL_ERR( "fail to reuse\n");
		
	//link si_me (port+address) to the socket
	if (-1==bind( recvsockfd, (struct sockaddr*)&si_me, sizeof(si_me))) 
	{
		PRINTF_FL_ERR(
		"UAVSIM listening - bind error %s:%d\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port) );
		return -1;
	}
	PRINTF_FL(
		"UAVSIM Listening at %s:%d\n", 
		inet_ntoa(si_me.sin_addr), 
		ntohs(si_me.sin_port) );

	//allocate some bytes. ; // where received data will be. 
	void *recv_data = malloc(MAX_PKT_SIZE) ;
	if (NULL == recv_data){PRINTF_FL_ERR("Erro calloc()\n");return -1;}
	
	/* Anciliary data
	 * we need this to retrieve ORIGINAL destination of packet (127.0.0.X)
	 * sock : AF_INET socket, usually SOCK_DGRAM
	 * include struct in_pktinfo in the message "ancilliary" control data
	 * */
	const void *opt = malloc(0x100) ;
	setsockopt( recvsockfd, IPPROTO_IP, IP_PKTINFO, &opt, sizeof(opt));
	// the control data is dumped here
	char cmbuf[0x100];
	// the remote/source sockaddr is put here
	struct sockaddr_in peeraddr;
	// if you want access to the data you need to init the msg_iovec fields
	struct iovec iovec[1];
	iovec[0].iov_base = recv_data;
	iovec[0].iov_len = MAX_PKT_SIZE;
	
	struct msghdr mh = {
		.msg_name = &peeraddr,
		.msg_namelen = sizeof(peeraddr),
		.msg_iov = iovec,
		.msg_iovlen = sizeof(iovec) / sizeof(*iovec),
		.msg_control = cmbuf,
		.msg_controllen = sizeof(cmbuf),
	};
					
	
	uint16_t port_src = 0; /* to store src and dst of received pkts. */
	uint16_t dst,src ;
	char dst_string[100];
	
	while(1) // keep reading packets from <recvsockfd> socket 
	{
		//num_bytes_read = recvfrom( recvsockfd , recv_data , MAX_PACKET_SIZE , 0, (struct sockaddr*)&si_other , &slen) ;

		num_bytes_read = recvmsg( recvsockfd, &mh , 0);
		for ( // iterate through all the control headers
			struct cmsghdr *cmsg = CMSG_FIRSTHDR(&mh);
			cmsg != NULL;
			cmsg = CMSG_NXTHDR(&mh, cmsg))
		{
			// ignore the control headers that don't match what we want
			if (cmsg->cmsg_level != IPPROTO_IP ||
				cmsg->cmsg_type != IP_PKTINFO)
			{
				continue;
			}
			
			/* msg by now has IP info */
			
			/* 
			 * If a UAV Y instance wants to send packets to UAV X,
			 * they should send packets to localhost, namely:
			 * 127.0.0.[X] 
			 * destination of the packet 
			 * destination port should be [Y] */
			 /* here we have the header destip addr */
			struct in_pktinfo *pi = (struct in_pktinfo *)CMSG_DATA(cmsg) ; 
			strcpy( dst_string, inet_ntoa(pi->ipi_addr)) ;
			sscanf( dst_string, "%*u.%*u.%*u.%"SCNu16, &dst ) ;
			/* packet sent to 127.0.0.1 is to uav #0*/
			//dst--;
			
			/* <src> port tells us who is sending data 
			 * original destination tells us the <dst> */
			port_src = ntohs(peeraddr.sin_port);
			break;
			
			// other stuff:
			// at this point, peeraddr is the source sockaddr
			// pi->ipi_spec_dst is the destination in_addr
			// pi->ipi_addr is the receiving interface in_addr
		}
					
		if (0 >= num_bytes_read) // receiving
		{
			/* weird case , recvvfrom returned error?. there is no timeout now. */
			perror(__func__);
			continue;
		}

		/* prepare this payload to be added to event Queue */
		enum event_types type = event_gettype( port_src ) ;
		packet_t packet;
		packet.data 	= recv_data,  // void * 
		packet.lendata 	= num_bytes_read; //size_t
		packet.dst 		= dst ;
		switch (type)
		{
			case ACTUATION: /* we got an actuation packet */
			{
				src = port_src - UDP_PORT_BASE_SRC_ACT ; /* */
				packet.src 		= src ;
				parser_actuation(packet);
				break ;
			}
			
			case PACKET_80211: /* we got an 80211 packet */
			{
				src = port_src - UDP_PORT_BASE_SRC_80211 ;
				packet.src 		= src ;
				parser_80211(packet);
				break;
			}
			default:
				PRINTF_FL("Port Src [%d] is unknown\n\n", port_src ) ;
		} /* end of switch type */
	
		
	} /* while 1 */
	return 1 ;
}

/* get event type based on src port 
 * */
enum event_types event_gettype( uint16_t psrc )
{
	if ( ( psrc >= UDP_PORT_BASE_SRC_ACT)  &&  (psrc <= UDP_PORT_BASE_SRC_ACT + MAX_NUM_DRONES  ) )
		return ACTUATION; /* we got an actuation packet */
	
	if ( psrc >= UDP_PORT_BASE_SRC_80211  &&  psrc <= UDP_PORT_BASE_SRC_80211 + MAX_NUM_DRONES )
		return PACKET_80211 ; /* we got an 80211 packet */
	
	return UNKNOWN ; /*unknown */

}

/* this thread responds to the app , poll server */
void *UDP_thread_start()
{
	if (0 < UDP_thread_cycle())
		PRINTF_FL("---------terminating OK---------\n" ) ;
	else
		PRINTF_FL_ERR("-------Terminating with errors---\n" ) ;

	return NULL ;
}

void event_clearlist(void)
{
	pthread_mutex_lock(&mutex_list) ;//L
	for (qidx_t idx = 0; idx < LIST_SIZE; idx++ )
	{
		event_list[idx].id = 0 ;
		event_list[idx].when_to_trigger = 0 ; 
		event_list[idx].pkt_ptr = NULL ;
		//PRINTF_FL("IIdx: %d when_to_trigger %" PRIu64 "us\n" , idx, event_list[idx].when_to_trigger ); 
	}
	PRINTF_FL("Erased Queue\n");
	
	pthread_mutex_unlock(&mutex_list) ;//U
	
	//event_dumpall();
}


int eventScheduler_init(tinfo_t *tinfo, int num_drones)
{
	/* init state of the list elements, rcv and tx threads */
	PRINTF_FL("Initiating Scheduler with %d drones\n", num_drones);
	if ( MAX_NUAVS-1 < num_drones )
	{
		PRINTF_FL_ERR("num of drones greater than defined max.\n");
		return NOTOKAY ;
	}
	
	g_ndrones = num_drones ;
	
	/* clear the list */
	pthread_mutex_init( &mutex_list, NULL);
	event_clearlist();


	size_t desired_stack_size = 1000000 ; /* 1MB */
	pthread_attr_t thread_attr;
	pthread_attr_init(&thread_attr);
	pthread_attr_setstacksize(&thread_attr, desired_stack_size);
	
	/* two threads will be spawnd */	
	//tinf->inputs[tinf->n] ={0,0,0,0,0}//nothing needed!
	if ( pthread_create(&(tinfo->tid[tinfo->n]), &thread_attr, &scheduler_reader_cycle, tinfo->inputs[tinfo->n] ) != 0 )
	{
		PRINTF_FL_ERR("Error spawning 'scheduler' thread.\n");
		return NOTOKAY;
	}
	tinfo->n++;
	
	/* The pthread_create() call stores the thread ID into corresponding element of tinfo[] */                                
	//tinfo->inputs[tinfo->n] ={0,0,0,0,0}//nothing needed!
	if ( pthread_create(&(tinfo->tid[tinfo->n]), &thread_attr, &UDP_thread_start, tinfo->inputs[tinfo->n] ) != 0 )
	{
		PRINTF_FL_ERR("Error spawning 'UDP' thread.\n") ;
		return NOTOKAY ;
	}
	tinfo->n++;

	/* Destroy the thread attributes object, since it is no longer needed */
    if (pthread_attr_destroy(&thread_attr) != 0)
		PRINTF_FL_ERR("error destroying attr\n") ;
	
	return OKAY ;
}



/* cycle (thread) that is _reading_ events and
 * "distributing" them */
static void *scheduler_reader_cycle()
{

	qidx_t qidx = -1 ;
	event_t *event_ptr  = malloc( sizeof( event_t ) ) ; /* use this to get a shorter alias for an event */ ;
	struct sockaddr_in 	si_me ;

	/* initiate sockets (source port + id)*/
	for (unsigned int i=0; i<=g_ndrones; i++)
	{
	
		sendsockfd[i] = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP) ; /* open outgoing socket */
		if (-1==sendsockfd[i]) 
		{
			PRINTF_FL_ERR("Sendsock error %s", 
				strerror(errno) ) ;
			continue;
		}
		si_me.sin_family = AF_INET; //address format (ip4)
		si_me.sin_port = htons(PORT_BASE_SIM_TX80211 + i); //simulator sends 8021pkts from this port-base.
		si_me.sin_addr.s_addr = htonl(INADDR_ANY); //packets are destinated to any 127.0.0.X IP.

		if (-1==bind(sendsockfd[i], (struct sockaddr*)&si_me, sizeof(si_me))) 
		{
			PRINTF_FL_ERR(
			"UAVSIM send srcport  - bind error %s:%d\n", 
			inet_ntoa(si_me.sin_addr), 
			ntohs(si_me.sin_port) );
			continue;
		}
		PRINTF_FL(
			"UAVSIM send[%d] bind %s:%d\n", 
			i,
			inet_ntoa(si_me.sin_addr), 
			ntohs(si_me.sin_port) );
	}
	
	
	//struct periodic_info info;
	//make_periodic(0, READER_PERIOD, &info); /* in us */


	open_log_file("sim", "out", &file_log_out);
	open_log_file("sim", "in", &file_log_in);
	fprintf( file_log_out, "eventid init_us end_us type\n") ;
	fprintf( file_log_in, "eventid init_us end_us type\n") ;

	while (1)
	{
		qidx = event_getnext() ; /* grab event that is to trigger at current time */
		
		if (qidx >= 0) /* there is one */
		{
			
			pthread_mutex_lock(&mutex_list) ; // L
			memcpy(event_ptr, &(event_list[qidx]) , sizeof(event_t) ) ;
			pthread_mutex_unlock(&mutex_list) ; // L

			//event_ptr = &(event_list[ qidx ]) ; /* use this to get a shorter alias for the event -noone changes an nonzero id event */
			//PRINTF_FL("next event idx<%03" QIDX_FMT ">\n" , qidx );
			//event_print( event_ptr ) ;
			//fflush( stdout ) ;
		
			if (event_ptr->id != 0) /* if it is zero, it has been deleted? */
			{
				switch (event_ptr->type)
				{
				case PACKET_80211:
					PRINTF_FL(
						"[CLOCK %07" PRIu64 "us] "
						"Serving event <%" PRIu32 ">:\n",
						SimClock_get(), event_ptr->id );
					event_print(event_ptr) ;
					send80211Packet(event_ptr, qidx, event_ptr->dst  ) ;
					break ;
				case ISENSOR_READING:
					sendISensorPacket(event_ptr, qidx, event_ptr->dst  ) ;
					break ;
				case ESENSOR_READING:
					sendESensorPacket(event_ptr, qidx, event_ptr->dst ) ;
					break ;
				default:
					PRINTF_FL_ERR("Event to be processed has wrong type!\n" );
					PRINTF_FL_ERR("---------Terminating ------\n" );
					return NULL ;
				}
				
			}
			else
			{ 
				//tmp_time_us = 0 ;
				//move_sim_clock( tmp_time_us ) ;
			}
			
		}
		
		/* move clock by 1 us */
		SimClock_move(1);//in us
		
		/* update layout once every 10ms */
		if ( 0 == (SimClock_get() % LAYOUT_UPD_PERIOD) )
		{
			//SimClock_print();
			//layout_printState(0);
			/* update drone network layout */
			layout_update();
		}
		if ( 0 == (SimClock_get() % PLOT_PERIOD ))
		{
			//layout_printState(0);
			SimClock_print();
			//event_dumpall();
			//plot_events(PACKET_80211);

		}
		
		/* schedule Internal sensor readings transmissions for all uavs, every <SENSOR_READINGS_PERIOD> us, all wellspaced */
		if (SimClock_get() % ISENSOR_READINGS_PERIOD == 0)
		{
			uint8_t i ;
			for (i = 1 ; i < g_ndrones ; i++ )
			{
				event_t event;
				event.type = ISENSOR_READING ; /* sensor */
				event.dst = i ; /* round robin */
				event.src = event.dst ; 
				event.added_timestamp = SimClock_get() ;
				event.when_to_trigger = event.added_timestamp + (ISENSOR_READINGS_PERIOD/g_ndrones) * i;
				event_schedule(event) ;
			}
		}
		
		/* schedule External (gps) sensor readings transmissions for all uavs, every <SENSOR_READINGS_PERIOD> us */
		if (SimClock_get() % ESENSOR_READINGS_PERIOD == 0 )
		{
			uint8_t i ;
			for (i = 1 ; i < g_ndrones ; i++ )
			{
				event_t event;
				CLEAR(event);
				event.type = ESENSOR_READING ; /* sensor */
				event.dst = i ; /* round robin */
				event.src = event.dst ; 
				event.added_timestamp = SimClock_get() ;
				event.when_to_trigger = event.added_timestamp + (ESENSOR_READINGS_PERIOD/g_ndrones) * i + 100; // +100us 
				event_schedule(event) ;
			}
			/* 
			 * typedef struct {
	uint32_t	id; // unique id
	uint8_t		type; // 80211, sensor, actuat
	uint64_t 	added_timestamp; // us
	uint64_t 	when_to_trigger; // us
	double		backoff; //csma - time to wait till next try. each packet will backoff n times, backoff is a random time = random_n() * SLOTTIME 
	int 		CW; //csma - current contention window
	uint8_t 	dst; // routing of this event
	uint8_t 	src; // routing of this event
	ssize_t 	length; // payload of event
	void 		*pkt_ptr; // payload of event
} event_t ;
			 * */
		}
		
		//if ( retardant % 100 == 1 )
		
		//PRINTF_FL("#");
		//while ( 1 )
		//{
			//if( clock_gettime( CLOCK_REALTIME, &temp_time ) == -1 )
			//{
				//perror( "\nEventSch-Srvr@ clock gettime" );
				//return NULL ;
			//}
			//if ( (double)( temp_time.tv_sec )  + (double)( temp_time.tv_nsec )/1e9 - last_run_sec  > 2e-6 ) 
				//break ;
				
			////temp_time.tv_sec = 0 ;
			////temp_time.tv_nsec = 10 ;
			////nanosleep( &temp_time , 0 ) ;
			//PRINTF_FL(".");
		//}
		
		//last_run_sec = (double)( temp_time.tv_sec )  + (double)( temp_time.tv_nsec )/1e9 ;
		/* designed this to "sleep" sometime here. usleep was not precise enough */
		//for ( retardant=0;retardant<3000;retardant++) asm("nop");
		//nanosleep( &req , 0 ) ;
		//SimClock_print();
		//nanosleep(&onenap,NULL);
	}
	
	return NULL ;

}

/* plot the last PLOT_PERIOD seconds of 80211events */
void plot_80211events(void)
{
	const float char_p_line=80;
	uint64_t time = SimClock_get() - PLOT_PERIOD ;
	qidx_t idx ;
	int i ;
	PRINTF_FL("Last %.0fms of 80211events [time %lu-%luus]:\n", 
		0.001*PLOT_PERIOD, 
		time, 
		time+PLOT_PERIOD );
	//float tmptime =1;
	//float lastslotbar =1;
	for ( i = 1 ; i < (int)char_p_line ; i++ ) 
	{
		
		//if ( i - lastslotbar > 1.0*PLOT_PERIOD/TDMAPERIOD )
		//{
			//PRINTF_FL("|"); 
			//lastslotbar=i;
			//i++;
		//}
		printf("_");
	}
	printf("|");
	pthread_mutex_lock (&mutex_list) ; //Lock - read atomically
	for (idx = 0 ; idx<LIST_SIZE ; idx++ )
	{
		if (event_list[idx].type != PACKET_80211) 
			continue;
		if (event_list[idx].when_to_trigger > time+PLOT_PERIOD)
			continue;
		if (event_list[idx].added_timestamp < time)
			continue;
		
		
		printf("\r");
		float begin = (float)(1.0*(event_list[idx].added_timestamp - time) / PLOT_PERIOD*char_p_line ); // each char is <PLOT_PERIOD/char_p_line>us
		float end = (float)(1.0*(event_list[idx].when_to_trigger - time) / PLOT_PERIOD*char_p_line ); // each char is <PLOT_PERIOD/char_p_line>us
		end = MIN( end , char_p_line );
		//float width= (float)(event_list[idx].when_to_trigger-event_list[idx].added_timestamp);
		float width = ceil(end-begin) ;
		//PRINTF_FL("begin end width %s > %.1f %.1f %.1f", 
		//	event_types_strings[event_list[idx].type], begin, end, width);
		//for ( i = 0 ; i < (int)begin ; i++ ) 
		//	PRINTF_FL("_");
		printf("\033[%dC", (int)begin);
		for ( i = 0 ; i < (int)width ; i++ ) 
			printf("x");
		//PRINTF_FL("\033[%dC", (int)(15+char_p_line-width-begin));
		//for ( i = end+1 ; i < char_p_line ; i++ ) 
		//	PRINTF_FL("_");
		//PRINTF_FL("\n");
		
	} 
	pthread_mutex_unlock( &mutex_list ) ; //L
	printf("%.1fms\n", .001*PLOT_PERIOD );
	//fflush( stdout ); 
	
	return ;
}

/* plot events on the screen, depending on the type */
static void plot_events(int8_t type)
{
	//event_dumpall();
	//int TDMAPERIOD=400000;
	switch (type)
	{
		case PACKET_80211:
		{
			plot_80211events();
			return;
		}
	}
	PRINTF_FL_WARN("invalid type\n");
}


uint64_t event_getTrigTime( qidx_t qidx )
{
	return event_list[qidx].when_to_trigger;
}

uint64_t event_getAddTime( qidx_t qidx )
{
	return event_list[qidx].added_timestamp;
}

/* postpone a given event <time_us> into the future */
void event_postpone(qidx_t qidx, uint64_t time_us)
{
	event_list[qidx].added_timestamp += time_us ; 
	event_list[qidx].when_to_trigger += time_us ;
	PRINTF_FL("Postponing <%"PRIu32">\n", 
		event_list[qidx].id ) ;
}

//------------------------------------


/* send_80211_packet 
 * time to deliver this event_ptr (80211pkt) to another uav
 * unprotected <event_ptr> 
 * use qidx to delete in case it has been sent 
 * it may be postponed due to backoffs */
static void send80211Packet( event_t *event_ptr, qidx_t qidx, uint8_t dst)
{
	if ( event_ptr->when_to_trigger < SimClock_get() )
	{
		#ifdef DEBUG_80211
		PRINTF_FL("[CLOCK %fs] *****DISCARDED*****\n\n", 
				1.0*SimClock_get() / MILLION ) ;
		#endif
		event_delete(qidx) ;
		return ;
	}


	/* Send packet! */
	//tmp_time_us = ( uint64_t )( MILLION * network_getUnicastTime( event1->length , DATARATE ) ) ;
	//move_sim_clock( tmp_time_us ) ;

	/* Check if the packet gets to the receiver (simulate wireless channel) */		
	qidx_t idx2;
	if (isPacketCorrupted(qidx,&idx2) > 0)
	{
		//PRINTF_FL("[CLOCK %07" PRIu64 "us]", SimClock_get() );
		//#ifdef DEBUG_80211
		PRINTF_FL("*Simultaneous transmissions**\n" ) ;
		event_print( &event_list[qidx] );
		event_print( &event_list[idx2] );
		//PRINTF_FL("deleting idxs: %" QIDX_FMT " and %" QIDX_FMT "\n" , qidx, idx2);
		//#endif 
		
		/* if packets are added exactly at the same time, 
		 * then both are discarded.
		 * this is unlikely to occur */
		if ( event_getAddTime(idx2) == event_getAddTime(qidx) )
		{
			#if DEBUG_80211
				PRINTF_FL("**Both discarded**\n");
			#endif
			//event_log( qidx ) ;
			//event_log( idx2 ) ;
			event_delete( idx2 ) ;
			event_delete( qidx ) ;
			return ;
		}
		
		#if CSMA /* CSMA-CA = backoff */
		if ( event_list[qidx].backoff <= 0 ) /* medium is still busy, and backoff expired: */
		{
			event_list[qidx].CW *= 2 ;
			event_list[qidx].CW = MIN(event_list[qidx].CW, CWMAX ) ; /* not more than CWMAX */
			/* compute random backoff*/
			event_list[qidx].backoff =  1e6*T_SLOTTIME * (double)( rand() % event_list[qidx].CW  )  ;
			PRINTF_FL("CW -> [0,%u] slots\n" , event_list[qidx].CW ) ;
			PRINTF_FL("backoff: %.0f slots = %.1fus\n" , 
				event_list[qidx].backoff /(1e6*T_SLOTTIME) ,/*n slots*/
				event_list[qidx].backoff  ) ; /* useconds */
			
			uint64_t blocking = event_getTrigTime( idx2 ) - event_getAddTime( qidx ) ;
			event_postpone( qidx , (uint64_t)(1e6*T_DIFS) + blocking + (uint64_t)(event_list[qidx].backoff)   );
			PRINTF_FL("Delayed by: blocking(%" PRIu64 ") + DIFS (%" PRIu64 ") + backoff(%" PRIu64 ") us\n" , 
				blocking , 
				(uint64_t)(1e6*T_DIFS), 
				(uint64_t)(event_list[qidx].backoff) ) ;
		}
		else
		{
			uint64_t remaining_backoff = event_getAddTime( qidx ) - event_getAddTime( idx2 ) ;
			uint64_t blocking = event_getTrigTime( idx2 ) - event_getAddTime( qidx ) ;
			event_list[qidx].backoff = (double)remaining_backoff ; /* keeps track of the remaining backoff time */
			PRINTF_FL("Remaining backoff: %fus\n" , event_list[qidx].backoff  ) ;
			event_postpone( qidx , blocking + (uint64_t)(1e6*T_DIFS) + remaining_backoff  );
			PRINTF_FL("Delayed by: blocking(%" PRIu64 ") + DIFS (%" PRIu64 ") + rem_backoff(%" PRIu64 ") us\n" , 
				blocking , 
				(uint64_t)(1e6*T_DIFS),
				remaining_backoff ) ;
			if ( ( blocking > 1000000 ) || ( remaining_backoff > 1000000 ) ) {PRINTF_FL_ERR("terminating\n");exit(1);}

		}
		return ;
			//if ( event_getAddTime( qidx ) < event_getAddTime( idx2 ) )
			//{
				////event_postpone( idx2 , 1000 );
				//goto actual_sendto ;
			//}
			//else
			//{
				////event_postpone( qidx , event_getTrigTime( idx2 ) - event_getAddTime( qidx ) + 1  ) ;
				//uint64_t backoff = (uint64_t)( T_SLOTTIME * (rand() % 31 ) ) ;
				//event_list[qidx].added_timestamp += backoff ;
				//event_list[qidx].when_to_trigger += backoff ;
				//return ;
			//}
		#else /* no CS, so delete both - none is transmitted */
			ad ad
			#ifdef DEBUG_80211
			
			PRINTF_FL("**Both discarded**\n" ) ;
			#endif
			//event_log( qidx ) ;
			//event_log( idx2 ) ;
			event_delete( idx2 ) ;
			event_delete( qidx ) ;
		#endif
		return ;
	}
	
	/* packet is not corrupted */
	
	/* check if it will be successfully transmitted */
	#if 1
	if ( network_isPacketDelivered(event_ptr->src, event_ptr->dst) != 1 )
	{
		#ifdef DEBUG_80211
		PRINTF_FL("[CLOCK %.4fs] **DISCARDED: packet lost in the medium **\n\n", 
			SimClock_get()/1e6 ) ;
		#endif
		event_log( qidx ) ;
		event_delete( qidx ) ;
		return ;
	}
	#endif
	
	//else if ( (isPacketCorrupted( qidx ) < 0) && (network_isPacketDelivered( event_ptr->src , event_ptr->dst ) == 1) )
	//{
	#if CSMA
	//actual_sendto:
	#if DEBUG_80211
	PRINTF_FL("Event id <%03"PRIu32"> sending\n", event_list[qidx].id ) ;
	#endif
	#endif
	
	

	if (0 < sendPacket(event_ptr->src, dst, PORT_UAV_RX80211, event_ptr->pkt_ptr, event_ptr->length))
		event_log(qidx);	
	event_delete(qidx) ;
	

}



/* OPEN LOG FILE */
//int openLogFile( char *prefix , char *sufix , FILE **file )
//{
	///* string => TX or RX */

	//char logfilename[100] ;
	//time_t rawtime ;
	//time( &rawtime ) ;
 	//struct tm todays_date = *localtime( &rawtime ) ;
	//sprintf( logfilename , "%s_%02d-%02d-%02d_%02dh%02dm%02ds_%s.log" , 
		//prefix ,
		//todays_date.tm_year+1900 ,
		//todays_date.tm_mon+1 ,
		//todays_date.tm_mday ,
		//todays_date.tm_hour ,
		//todays_date.tm_min ,
		//todays_date.tm_sec ,
		//sufix ) ; 
	//PRINTF_FL("%s\t\t@logging into %s\n" , __func__ , logfilename );
	//*file = fopen( logfilename , "w" ) ; /* create and open the file to log stuff */
	//if (*file == NULL )
		//return -1 ;
	
	//return 1;
//}

/* send a string to dst on a given port */
int sendPacket( int src, int dst , int port , void *string, int len)
{
	/* prepare sockaddr */ 
	struct sockaddr_in send_addr;
	prepare_sockaddr(dst, port, &send_addr);
	/* we are sending packet to 127.0.0.IP:port_type_pkt , from source port BASE+src*/
	/* end of sockaddr */	

	int ret = (int)sendto( sendsockfd[src], string , len , 0 ,
			(const struct sockaddr *)&send_addr , (socklen_t)sizeof(send_addr) ) ;

	if (ret < 0)
	{
		PRINTF_FL_ERR("[CLOCK %.4fs] "
			"Error delivering to {%s:%d}\n\n", 
			SimClock_get()/1e6 ,
			inet_ntoa( send_addr.sin_addr ) ,
			ntohs(send_addr.sin_port) ) ;
		return -1 ;
	}
	
	#if DEBUG_ESENSOR
	PRINTF_FL("[CLOCK %.4fs] Delivered %dB to {%s:%d}\n", 
		SimClock_get()/1e6,
		len,
		inet_ntoa(send_addr.sin_addr),
		ntohs(send_addr.sin_port) ) ;
	#endif
	//PRINTF_FL("\t"); 
	//PRINTF_FL("\t"); printPositions();
	
	return 1; /*ok  */
}

 static void sendESensorPacket( event_t *event_ptr , qidx_t qidx, uint8_t dst )
{
	#if DEBUG_ESENSOR
	PRINTF_FL("[CLOCK %.4fs] "
		"Serving event<%"PRIu8">: "
		"Sensor Ext Reading to [#%02d].\n", 
		SimClock_get()/1e6, event_ptr->id, event_ptr->dst) ;
	#endif

	if ( event_ptr->src != event_ptr->dst )
	{
		PRINTF_FL_ERR("Error: some node is trying to access sensor readings from another node\n");
		return;
	}
	
	//sensor_reading_t reading = layout_getSensor( event_ptr->src ) ;
	rtk_reading_t reading = layout_getRTKReading( event_ptr->src-1 ) ;
	char string[150] ;
	/***********************id,sats, N, E, D,vN,vE,vD,ID,NSAT,la,lo**************************************/
	snprintf( string ,sizeof(string),
		"GPGGP,%u,%hhd,%d,%d,%d,%d,%d,%d,%u,%hhd,%x,%x" , 
		reading.rtk_sample_id , reading.rtk_num_sats, 
		reading.north, reading.east, reading.down, 
		reading.vel_north, reading.vel_east, reading.vel_down,
		reading.spp_sample_id, reading.spp_num_sats ,
		reading.latitude, reading.longitude ) ;

	unsigned int computed_checksum = 0 ;
	for (int index = 0; index < (int)strlen(string) ; index++)
		computed_checksum ^= string[index] ;
	//PRINTF_FL("\t\treading: X[%d]=(%f %f %f)\n" , event_ptr->src , reading.X, reading.Y, reading.Z );
	//PRINTF_FL("\t\treading: [%s] - len %d\n" , string, (int)strlen( string) );
	//PRINTF_FL("\t\tchecksum: [%02x]\n" , computed_checksum );
	char string1[150] ;
	snprintf(string1, sizeof(string1), "$%s*%02x" , string , computed_checksum ) ;
	
	#if DEBUG_ESENSOR
	PRINTF_FL("Reading [%s] - %dB\n", string1, (int)strlen(string1) );
	#endif
	CLEAR(string);
	sprintf(string, "%s\r\n", string1  ) ;
	
	if (0<sendPacket( 0, dst, PORT_ESENSOR, string, strlen(string)))
		event_log(qidx);
		
	event_delete(qidx) ;


	//pthread_mutex_unlock ( &mutex_list ) ; // U


	
}


static void sendISensorPacket( event_t *event_ptr , qidx_t qidx , uint8_t dst )
{


	//pthread_mutex_lock ( &mutex_list ) ; //L
	#if DEBUG_ISENSOR
	PRINTF_FL(
		"[CLOCK %07"PRIu64"us] "
		"Serving event<%" PRIu8 ">: Sensor Int Reading to [#%02d].\n", 
		SimClock_get() , event_ptr->id , event_ptr->dst  ) ;
	#endif

	if (event_ptr->src != event_ptr->dst)
	{
		PRINTF_FL_ERR("some node is trying to access "
			"sensor readings from another node\n");
		return;
	}
	
	
	isensor_reading_t reading = layout_getIntReading(event_ptr->src-1);
	
	//PRINTF_FL("\t\treading: X[%d]=(%f %f %f)\n" , event_ptr->src , reading.X, reading.Y, reading.Z );
	//PRINTF_FL("\t\treading: [%s] - len %d\n" , string, (int)strlen( string) );
	//PRINTF_FL("\t\tchecksum: [%02x]\n" , computed_checksum );
	char string[150];
	float compass = 90.0-reading.heading_degrees;
	while (compass<0) compass+=360;
	
	snprintf(string , sizeof(string), "state=%08x,battery=%d,compass=%0.3f,ultra=%0.3f" , 
		reading.state, reading.battery, compass, reading.ultra_altitude) ;

	#if DEBUG_ISENSOR
	PRINTF_FL("Reading: [%s] - %dB\n" , string , (int)strlen( string ) );
	#endif
	
	if (0<sendPacket(0, dst, PORT_ISENSOR, string,strlen(string)))
			event_log(qidx);	
	event_delete(qidx) ;
}




//static int make_periodic (unsigned int period_us, struct periodic_info *info)
//{
	//int ret;
	//unsigned int ns;
	//unsigned int sec;
	//int fd;
	//struct itimerspec itval;

	///* Create the timer */
	//fd = timerfd_create(CLOCK_MONOTONIC, 0);
	//info->wakeups_missed = 0;
	//info->timer_fd = fd;
	//if (fd == -1)
		//return fd;

	///* Make the timer periodic */
	//sec = period_us / 1000000 ;
	//ns = (period_us - (sec * 1000000)) * 1000;
	//itval.it_interval.tv_sec = sec; /* periodic part */
	//itval.it_interval.tv_nsec = ns; /* periodic part */
	//itval.it_value.tv_sec = sec; /* first time */
	//itval.it_value.tv_nsec = ns; /* first time */
	//ret = timerfd_settime (fd, 0, &itval, NULL);
	//return ret;
//}

//static void wait_period(struct periodic_info *info)
//{
	//uint64_t missed;
	//int ret;

	///* Wait for the next timer event. If we have missed any the
	   //number is written to "missed" */
	//ret = read(info->timer_fd, &missed, sizeof (missed));
	//if (ret == -1)
	//{
		//perror ("read timer");
		//return;
	//}

	///* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	//if (missed > 0)
		//info->wakeups_missed += (missed - 1);
//}
