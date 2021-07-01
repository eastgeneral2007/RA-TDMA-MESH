/* this is the simulator world code 
 * //define WORLDSIM! -D WORLDSIM
 * this app is independent of any other app
 * it creates shared memory spaces where other apps can read ORACLE information time and uav status
 * it uses UDP packets to comunicate with UAV code 
 * ARDrone2.0 apps can be compiled directly and they will talk with the simulator */
 
#include "main.h" 

#define PREFIX	"MAIN"
#include "utils.h"

//define WORLDSIM

/** Globals **/
/* no globals :D hurray */


/** simualator starts here **/
int main(int argc, char *argv[])
{
	PRINTF_FL("\n\n\n -------------------- UAV World Simulator --------------------\n") ;
	signal(SIGINT, sigint_handler); // set ctrl+c handler
	
	
	//printf("time %f\n",network_getUnicastTime(150, 0 )); 
	//exit(1);
	
	/* check number of threads */
	if (argc <= 1)
	{	 
		PRINTF_FL_ERR( "Terminating: Usage: %s <num_threads>\n\n\n\n", argv[0] );
		return NOTOKAY ;
	}
	
	int n_uavs = strtoul(argv[1], NULL, 0 ) ;
	PRINTF_FL("Number of threads/uavs: %d.\n", n_uavs );
	
	/* start by initating simulator clock */
	if (NOTOKAY == SimClock_init())
	{
		PRINTF_FL("[SimClock] Failed to Init\n\n\n\n") ;
		return NOTOKAY ;
	}

	//PRINTF_FL("Info: sending UDP:100B  takes: %0.4fms\n" , 1e3*network_getUnicastTime( 100 , 0 ) ) ;
	//PRINTF_FL("Info: sending UDP:1000B takes: %0.4fms\n" , 1e3*network_getUnicastTime( 1000 , 0 ) ) ;
	
	/* 80211 channel simulator thread*/


	if (NOTOKAY==layout_init(n_uavs)) 
	{
		PRINTF_FL_ERR("[layout] failed to init\n\n") ;
		return NOTOKAY;
	}
	PRINTF_FL("layout OK\n\n") ;
	
	/* init event scheduler. it will start listening to UDP packets from "outside" */

	tinfo_t tinfo; /* write down here the number of threads iniated and tids */
	tinfo.n=0; /* there are zero threads in the beggin. */
	if (NOTOKAY == eventScheduler_init(&tinfo, n_uavs)) 
	{
		PRINTF_FL_ERR("scheduler failed to init\n\n\n\n") ;
		return NOTOKAY;
	}
	PRINTF_FL("Scheduler OK\n") ;

	sleep(1); /* all threads should be spinning by now */
	for (int i = 0; i<tinfo.n; i++)
		pthread_join(tinfo.tid[i],NULL);
	

	return OKAY ;
}




void sigint_handler(int sig)
{
  (void)(sig); // suppress warning
  shm_unlink( "clock_memspace" ) ;
  shm_unlink( "layout_memspace" ) ;
  sem_unlink( "clock_semaphore" ) ;  
  exit(1);
}

