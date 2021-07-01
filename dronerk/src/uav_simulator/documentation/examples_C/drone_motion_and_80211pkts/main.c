	/* program that simulates the behavior a single drone */
#include "main.h" 
#include "world_simulator_api.h"  /* to interact directly with the simulator , and get ports */


#include "utils.h"
#define PREFIX	"main"

/*****
 * evil globals 
 * */
/* none! hurray */

#include <unistd.h>
#define STRINGME(x)		#x
/**
 *  spawn some threads : 
 * 1)send actuation, 
 * 2)read Esensors, (gps) 
 * 3)send 80211 pkts
 * 4)rcv 80211 pkts 
 * */
typedef void* (*ft)(void*);       
 
 
static const ft ft_toload[] = {
			&thread_rx_esensor,	///* thread to get incoming isensor readings *///
			&thread_rx_isensor,		///* thread to get incoming esensor readings *///
			&thread_rx_80211,  ///* thread to read incoming 802.11 pkts *///
			&thread_tx_actuation, ///* thread to send actuation *///
			&thread_tx_80211};
static const char names[][20] = { 
			"thread_rx_esensor",
			"thread_rx_isensor",
			"thread_rx_80211",
			"thread_tx_actuation", 
			"thread_tx_80211" };
 
int main(int argc, char *argv[])
{
	PRINTF_FL("initiating\n");
	signal(SIGINT, sigint_handler);

	/* start by "connecting" to the simulator clock: readtime ,pause, resume , etc */
	if (NOTOKAY == SimClock_init())
	{
		PRINTF_FL("Main@\t\tclock failed to init\n") ;
		return NOTOKAY ;
	}
	

	/* setup thread attr for all 4 threads */
	size_t desired_stack_size = 40000 ; /* 40KB for stack */
	pthread_attr_t thread_attr ;
	pthread_attr_init( &thread_attr );
	pthread_attr_setstacksize( &thread_attr, desired_stack_size ) ;
	
	

	if ( argc <3)
	{	 
		fprintf( stderr , "Usage: %s <myid> [<sender?> <dst]\n", argv[0] );
		return -1 ;
	}

	int my_id = (int)strtol( argv[1] , NULL, 0 ) ;
	PRINTF_FL("my id: #%" PRIu8 ".\n" , my_id );
	int sender = (int)strtol( argv[2] , NULL, 0 ) ;
	PRINTF_FL("Am I sender? %s.\n" , sender==0? "no":"yes" );
	if (argc == 4)
		PRINTF_FL("Dst = #%d.\n" , (int)strtol( argv[3], NULL, 0 ) ); 
	
	//uint8_t idx=0 ; /*thread idx */


		
			
	/* thread to get incoming isensor readings */
	tinfo2_t tinfo={0} ; /* struct to save arguments for all threads */
	
	for (tinfo.n=0 ;tinfo.n<4;//ARRAYSIZE(ft_toload); 
		tinfo.n++)
	{
		tinfo.t[tinfo.n].in[0] = my_id ;     
		if (ft_toload[tinfo.n]==&thread_tx_80211)  
		{
			tinfo.t[tinfo.n].in[1] = sender ;     
			tinfo.t[tinfo.n].in[2] = (int)strtol(argv[3] , NULL, 0 ) ; /* dst */                  
		}
		if (0 != pthread_create( 
			&tinfo.t[tinfo.n].tid, &thread_attr, 
			ft_toload[tinfo.n], 
			tinfo.t[tinfo.n].in ))
		{
			PRINTF_FL_ERR("Error spawning '%s' thread.\n", names[tinfo.n]);
			exit(1) ;
		}
		PRINTF_FL( "spawning '%s' thread.\n",  names[tinfo.n]);
	}
	PRINTF_FL_WARN("spawned %d threads\n", tinfo.n);
	
	


	/* thread to send 802.11 pkts to other drone */       
	//idx++;
	//tinfo[idx].arg1 = my_id ;                         

	////printf("SIM@ Destination #%" PRIu8 ".\n" , dst );
	//if ( pthread_create( &tinfo[idx].thread_id, &thread_attr, &thread_tx_80211, &tinfo[idx] ) != 0 )
	//{
		//fprintf( stderr , "Main@ Error spawning 'thread_tx_80211' thread.\n" ) ;
		//fflush( stdout )  ;
		//exit( 1 ) ;
	//}
	
	/* Destroy the thread attributes object, since it is no longer needed */
    if ( pthread_attr_destroy( &thread_attr ) != 0 )
		fprintf( stderr , "error destroying attr\n") ;
	
	

	/******************
	 * my basic mission: 
	 * 		1) move in a circle 
	 * 		2) send 100pkts to all other nodes, every 5 seconds
	 ****************************/
	actuator_t tmp_actuation ; 
	esensor_t tmp_ereading ;
	isensor_t tmp_ireading ;
	tmp_actuation.thrust.Z=0;
	float tx, ty; /*  target coordinates are tx,ty  */
	while( 1 )
	{
		/* periodic, parametric function with time 
		 * move in a circle R=60m */
		tx = 70*cos( (my_id*5 + 1.0*SimClock_get()/1e6) /100.0 * 6.28 ) ; 
		ty = 90*sin( (my_id*5 + 1.0*SimClock_get()/1e6) /100.0 * 6.28 ) ; 
		
		PRINTF_FL("Target=[x%0.1f,y%0.1f,%.0fº] -- \n", tx,ty,0.0 );
		tmp_ereading = esensor_getReading();
		tmp_ireading = isensor_getReading();
		float cmp = tmp_ireading.heading_degrees;
		if (cmp>180) cmp-=360; /* get heading between -180 and 180º */
		tmp_actuation.thrust.Y = 0.01 * ( tx - tmp_ereading.east/1000.0 ) ; // (+) goes right , (-) goes left 
		tmp_actuation.thrust.X = -0.02 * ( ty - tmp_ereading.north/1000.0 ) ;// (-) goes FOWARD, (+) goes BACKWARDS
		tmp_actuation.torque = 0.005 * (cmp) ; //(+) turns CCW, (-) turn ClockWise
		PRINTF_FL(" Act =[x %0.2fN ,y %0.2fN, cmp%.0fº >> w %0.2fN.m]\n", 
			tmp_actuation.thrust.X  , 
			tmp_actuation.thrust.Y ,
			cmp,
			tmp_actuation.torque);
		SimClock_print();
		//tmp_actuation.thrust.X = 0 ;
		//tmp_actuation.thrust.Y = -.6  ;
		//tmp_actuation.torque = 10 ;
		
		setActuation(tmp_actuation) ; /* "send" actuation thread my lastest desired actuation vector */
		microsleep(500000) ;
	}

	/* wait here until all threads safely terminate */
	for (int idx = 0 ; idx<tinfo.n ; idx++)
		pthread_join( tinfo.t[idx].tid , NULL );
	printf("**MAIN - END**\n");
	return OKAY ;
}



/* ctrl+c signals this: */
void sigint_handler(int sig)
{
  (void)(sig); // suppress warning 
  exit(1);
}


