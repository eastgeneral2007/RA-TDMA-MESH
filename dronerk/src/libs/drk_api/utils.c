/* this file is in drklib */
#include <arpa/inet.h> // udp packets
#include <errno.h> // strerror
#include <fcntl.h>           /* For O_* constants */
#include <ifaddrs.h> // udp packets
#include <math.h>
#include <net/if.h> /* IFNAMSIZ */
#include <pthread.h> /* mutex */
#include <semaphore.h>
#include <signal.h>
#include <stdio.h> /* printf() */
#include <stdlib.h> /* atoi*/
#include <string.h> /* strncpy() */
#include <sys/time.h>
#include <sys/timerfd.h> /* timerfd */
#include <sys/ioctl.h> /* ioctl() */
#include <sys/stat.h> /* For mode constants */
#include <termios.h>
#include <unistd.h> /* close() */

#ifdef SIMULATOR // only include this if we are compiling simulated code
#include "drk/world_simulator_api.h"
#include "drk/sim_clock.h"
#endif 



#ifdef OUTSIDEDRK // only include this if we are running examples
#include "sim_clock.h"
#include "utils.h"
#else
#include "drk/utils.h"
#endif 



#define PREFIX	"UTILS"

/* save here the last part of our ip (1--254) */
static uint16_t ip_last_segment = 0; 
	

///** under simulation, we need to set IP last segment **/
#ifdef SIMULATOR
void setMyIP(uint8_t my_id)
{
	ip_last_segment= my_id ;
}
#endif


///** Get my IP's last segment **/
int16_t getMyIP(void) 
{
	//#ifdef SIMULATOR
		////printf(PREFIX_DRK "[SIM STUFF]\n");
		//FILE * my_id_file = fopen( "myid.conf" , "r" ) ;
		//if ( my_id_file == NULL )
		//{
			//PRINTF_FL("myid.conf file non-existent. Can't resolve my own ID\n" ) ;
			//return 0 ;
		//}
		//char buf[3] ;
		//fgets( buf, sizeof(buf), my_id_file ) ; // read a line
		//fclose( my_id_file ) ;
		//return atoi( buf ) ;
	//#else

	if ( ip_last_segment != 0 )
		return ip_last_segment ;
		
#ifdef SIMULATOR
		PRINTF_FL_ERR("ip should be set before any getMyip call\n");
		exit(1);
#else
		
	/* Get the pc/drone IP address searching at "BASEIP.X" */
	struct ifaddrs* ifAddrStruct = NULL;
	struct ifaddrs* ifa = NULL;
	
	if (-1 == getifaddrs(&ifAddrStruct))
	{
		PRINTF_FL_ERR("Error on getifaddrs()\n");
		return E_OTHER ;
	}
	/* Desktop: lo,eth2, eth1, wlan1 , wlan2, mon0 */
	/* Drone: lo , ath0 */
	ifa = ifAddrStruct;
	while (NULL != ifa)
	{

		char 	ip_addr_string[INET_ADDRSTRLEN];
		void	*tmp_addr_ptr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
		inet_ntop( AF_INET, tmp_addr_ptr, ip_addr_string, INET_ADDRSTRLEN ) ;
		
		if (1 == sscanf( ip_addr_string, ""BASEIP"%"SCNu16, &ip_last_segment ) ) 
		{
			/* we have read a valid IP */
			PRINTF_FL("Interface %s. IP [%s]\n" , ifa->ifa_name, ip_addr_string );
			freeifaddrs(ifAddrStruct); 
			
			/* return the answer */
			return ip_last_segment ;
		}
		
		ifa = ifa->ifa_next ; /* get next one */
		
		
	}
	PRINTF_FL("No interface found with an IP of type "BASEIP"*\n") ;
	return E_OTHER ;
	//ifa = ifAddrStruct;
	//char * nic_name = ifa->ifa_name ;
	//sscanf( ip_addr_string, "%*d.%*d.%*d.%hhu", &last_ip_chunk );
	//PRINTF_FL( "My IP: %s (NIC %s)\n" , ip_addr_string , nic_name ) ;
	
	
#endif //simulator
}

 
/** get last segment ip from a struct si **/
uint8_t getOtherIP( struct sockaddr_in si_other )
{
	
#ifdef SIMULATOR 
	//PRINTF_FL_WARN("Source port %u\n", ntohs(si_other.sin_port) );
	/* under simulated code, the source port is the ID of the uav */
	return ntohs(si_other.sin_port) - PORT_BASE_SIM_TX80211;//PORT_BASE_SIM_TX80211	 ;
#else
	uint8_t other_ip ;
	//printf("%s\n",inet_ntoa( si_other.sin_addr ) );
	sscanf( 
		inet_ntoa( si_other.sin_addr ) , 
		""BASEIP"%" SCNu8 "", 
		&other_ip  ) ;
	//printf("other ip %" PRIu8 "\n", other_ip );
	return other_ip ;
#endif
	
}


/* OPEN LOG FILE */
error_t open_log_file( const char* const prefix , const char* const sufix , FILE** file_p )
{
	/* prefix => TX or RX */
	#define LOGFOLDER "./usb/"
	char logfilename[100] ;
	
	volatile int first_time = 1;
	volatile struct tm todays_date;
	if (1 == first_time)
	{
		time_t rawtime ;
		time( &rawtime ) ;
		todays_date = *localtime( &rawtime ) ;
		first_time = 0;
	}
	
	snprintf( logfilename , sizeof(logfilename)-1, 
		LOGFOLDER "%02d-%02d-%02d_%02dh%02dm%02ds_%s_%s.log" , 
		todays_date.tm_year+1900 ,
		todays_date.tm_mon+1 ,
		todays_date.tm_mday ,
		todays_date.tm_hour ,
		todays_date.tm_min ,
		todays_date.tm_sec ,
		prefix ,
		sufix ) ; 
	PRINTF_FL("Creating file %s\n" , logfilename );
	*file_p = fopen( logfilename , "w" ) ; /* create and open the file to log stuff */
	if ( *file_p == NULL )
		return E_FILE_CREATION ;

	return E_SUCCESS ;
}


/* sleep for <tmp_us> microseconds, return -1 if it fails */
int microsleep( uint64_t tmp_us )
{
	#if defined(SIMULATOR) || defined(OUTSIDEDRK)
	
	simclock_t base_clock = SimClock_get() ; //get current simtime,
	while (SimClock_get() <= base_clock + tmp_us)
		usleep(10); /* sleep realtime 10us until simulator advances enough */
	
	//if (tmp_us>1000)
		//PRINTF_FL_WARN("slept %"PRIu64"ms\n", tmp_us/1000);
	//else
		//PRINTF_FL_WARN("slept %"PRIu64"us\n", tmp_us);
		
	#else
	usleep( tmp_us ) ;
	#endif
	return E_SUCCESS ;
	
	#if 0
	/* Create the timer */
	int fd = timerfd_create( CLOCK_REALTIME , 0 ) ;
	if (fd == -1)
	{
		PRINTF_FL_ERR( "Timerfd_create - ERROR! (%s)\n" ,strerror( errno) );
		return -1;
	}
	
	
	/* Make the timer fire once in <tmp_us> mciroseconds*/
	time_t sec = (time_t)(tmp_us / 1000000) ;
	long ns = (long)((tmp_us - (sec * 1000000)) * 1000);
	struct itimerspec itval;
	itval.it_value.tv_sec = sec ; /* first time */
	itval.it_value.tv_nsec = ns ; /* first time */
	itval.it_interval.tv_sec = 0 ; /* periodic part */
	itval.it_interval.tv_nsec = 0 ; /* periodic part */
	

	if (timerfd_settime(fd, 0, &itval, NULL )  == -1 ) 
	{
		PRINTF_FL_ERR("Timerfd_settime - ERROR ! (%s)\n", strerror( errno) );
	}
	


	/* it will sleep here until time has passed */
	if ( fd == -1 )
	{
		PRINTF_FL_ERR("Prevented a Read of fd=-1\n") ;
		return E_READING ;
	}
	uint64_t missed ;
	if ( read( fd , &missed , sizeof(missed) ) < 0 )
	{
		if (errno != EINTR )
		PRINTF_FL_ERR("Unexpected error read() - %s\n", strerror(errno) ) ;
		return -1 ;
	}

	if (missed == 0)
	{
		PRINTF_FL_ERR( "Weird case ! missed %" PRIu64 "\n" , missed ) ;
	}
	//PRINTF_FL_ERR( "missed %" PRIu64 "\n", missed ) ;
	if ( fd == -1 ) 
	{
		PRINTF_FL_ERR("Prevented a closure of fd=-1\n") ;
		return -1 ;
	}
	
	if ( close ( fd ) < 0  )
	{
		PRINTF_FL_ERR("Error closing file - %s" , strerror( errno ) ) ;
		return -1 ;
	}
	#endif
	return E_SUCCESS ;
}

#if 0
int wait_period(struct periodic_info *info)
{
	PRINTF_FL( "info->timer_fd = %d\n", info->timer_fd ); 
	uint64_t missed;
	int ret;
	#ifdef SIMULATOR
	simclock_t last_runtime = SimClock_get(), cur_clock = last_runtime ; /* us - microseconds */
	while ( cur_clock - last_runtime < info->period_us ) /* 1)sleep for a bit. 2)check sim clock */
	{
	#endif
		/* Wait for the next timer event. If we have missed any the
		   number is written to "missed" */
		ret = read( info->timer_fd , &missed , sizeof(missed) ) ;
		if (ret == -1)
		{
			PRINTF_FL_ERR("Error read timer (%s)\n", strerror( errno) );
			return -1 ;
		}

		/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
		if (missed == 0)
		{
			PRINTF_FL_ERR( "WEIRD missed %" PRIu64 "\n", missed ) ;
			return -1 ;
		}
		info->wakeups_missed += (missed - 1);
	
	#ifdef SIMULATOR
		/* check if simulator has advanced ACT_PERIOD (us)*/
		cur_clock = getSimClock() ;
	}
	#endif
	
	return E_SUCCESS ;
}


int make_periodic( uint64_t init_us , uint64_t period_us , struct periodic_info *info )
{
	int ret;
	
	
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create( CLOCK_MONOTONIC, 0 ) ;
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	info->period_us = period_us;
	if (fd == -1)
	{
		PRINTF_FL_ERR( "timerfd_create - ERROR! (%s)\n" ,strerror( errno) );
		return fd;
	}
	
	/* define when it starts from the moment now */
	unsigned int sec = (unsigned int)(init_us / 1000000) ;
	unsigned int ns = (unsigned int)((init_us - (sec * 1000000)) * 1000);
	itval.it_value.tv_sec = sec; /* first time */
	itval.it_value.tv_nsec = ns; /* first time */
	
	/* Make the timer periodic  */
	sec = (unsigned int)(period_us / 1000000) ;
	ns = (unsigned int)((period_us - (sec * 1000000)) * 1000);
	itval.it_interval.tv_sec = sec; /* periodic part */
	itval.it_interval.tv_nsec = ns; /* periodic part */
	
	
	ret = timerfd_settime(fd, 0, &itval, NULL);
	if (ret == -1 ) 
	{
		PRINTF_FL_ERR(
			"timerfd_settime - ERROR ! (%s)\n", 
			strerror(errno) );
	}
	return ret;
}
#endif

// TODO: rename this to getCurrentEpoch
int getCurrentTime( struct timespec *temp_time )
{
	#ifdef SIMULATOR
	simclock_t tmp = SimClock_get(); 
	temp_time->tv_sec = tmp/1e6 ;
	temp_time->tv_nsec = (tmp % 1000000)*1000 ;
	#else
	if ( clock_gettime( CLOCK_REALTIME, temp_time) == -1 )
	{
		PRINTF_FL_ERR( "clock gettime (%s)\n" ,
			strerror(errno) );
		return E_OTHER ;
	}
	#endif
	return E_SUCCESS ;
}

double getEpoch( void )
{
	struct timespec time_temporary ; /* to timestamp_ms pkts */
	int ret = getCurrentTime( &time_temporary );
	if ( ret < 0 ) /* get current epoch time */
	{	
		PRINTF_FL_ERR("Time ERROR!\n");
		return (double)ret ;
	}
	return 
		(double)time_temporary.tv_sec +
		(double)time_temporary.tv_nsec / BILLION ;
}

int32_t computeMax(float array[], int64_t n )
{
	if ( n == 0 )
	{
		return 0 ;
	}

	int32_t max = INT32_MIN ; /* init max with lowest possible value */
	for ( 	int64_t idx = 0 ; 
			idx < n; 
			idx++
		)
	{
		if (array[idx] > max)
		{
			max = array[idx] ;
		}
	}
	if (INT32_MIN == max)
	{
		PRINTF_FL_ERR("max = %" PRId32 "\n" , max);	
		max = 0 ;
	}
	return max ;

}


int32_t computeMin( float array[], int64_t n )
{
	if (0 == n)
	{
		return 0 ;
	}

	int32_t min = INT32_MAX ; /* init max with highest possible value */
	for ( 	int64_t idx = 0 ; 
			idx < n; 
			idx++
		)
	{
		if (array[idx] < min)
		{
			min = array[idx] ;
		}
	}
	if (INT32_MAX == min)
	{
		PRINTF_FL_ERR("min = %" PRId32 "\n" , min);	
		min = 0 ;
	}	

	return min ;

}

int cmpfunc (const void * a, const void * b)
{
   return ( *(float*)a - *(float*)b );
}

int32_t computeMedian( float array[], int64_t n )
{
	if (0 == n)
	{
		return 0 ;
	}

	
	qsort(array, (size_t)n, sizeof(float), cmpfunc);
	int32_t median = array[n/2];
	
	return (int32_t)(median) ;

}


int32_t computeMean( float array[], int64_t n )
{
	if (0 == n)
	{
		return 0 ;
	}

	
	int64_t sum = 0  ;
	int64_t counter = 0 ;
	//PRINTF_FL("[");
	for ( 	int64_t idx = 0 ; 
			idx < n; 
			idx++
		)
	{
		/* only compute average with positive values */
		//if ( array[ idx ] > 0 ) 
		//{
			sum += array[idx] ;
			counter++ ;
		//}
		//printf("%" PRId32 " ", array[ idx ] );
	}
	//printf("]\n");
	//PRINTF_FL("mean = %" PRId32 "\n" , (int32_t)(sum/counter) );
	if ( counter == 0 )
	{
		return 0 ; /* nothing found */
	}
	
	return (int32_t)(sum/counter) ;

}

int32_t computeStd( float *array , int64_t n , int32_t average )
{
	if ( n == 0 )
	{
		return 0 ;
	}

	
	int64_t sum = 0  ;
	int64_t counter = 0 ;
	
	for ( 	int64_t idx = 0 ; 
			idx < n; 
			idx++
		)
	{
		/* only compute average with positive values */
		//if ( array[ idx ] > 0 ) 
		//{
			sum += ((int64_t)array[ idx ] - (int64_t)average ) * ((int64_t)array[ idx ] - (int64_t)average ) ;
			counter++ ;
		//}
		//printf("%" PRId32 " ", array[ idx ] );
	}
	//PRINTF_FL("Std = %" PRId32 "\n" , (int32_t)sqrt(1.0*sum/counter) );
	if ( counter == 0 )
	{
		return 0 ; /* nothing found */
	}
	
	return (int32_t)sqrt(1.0*sum/counter) ;

}



/* Turn keyboard echo back on */
//kind of : system("reset")
void drk_restore_termios(void)
{
	struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag |= (ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
	PRINTF_FL( "Terminal settings restored.\n");
}

/* setup terminal */
void drk_setup_termios(void)
{
	/* Set up the terminal to disable buffering and echo */
	struct termios oldt, newt;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	//newt.c_cc[VMIN] = 0; //1
	//newt.c_cc[VTIME] = 3; //wait 0.3 seconds to read a char.
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  
	PRINTF_FL( "Terminal set up.\n");
}

/************** mathematical stuff ****************/

/* produce 2random normal distributed numbers - box muller transform*/
void normalrand(float *z1, float *z2)
{
	float u = 1.0*rand()/RAND_MAX ; /* uniform distributed random variable */
	float v = 1.0*rand()/RAND_MAX ; /* uniform distributed random variable */
	*z1 = sqrt( -2 * log( u ) ) * cos( 2.0*M_PI*v ) ; /* NORMAL(0,1)*/
	*z2 = sqrt( -2 * log( u ) ) * sin( 2.0*M_PI*v ) ; /* NORMAL(0,1)*/
	//printf("\nu;%f v:%f z1:%f z2:%f\n", u,v ,z1,z2);
}

/* Convert any angle to an angle between 0º and 359.(9)º */
double wrapTo360( double angle_in_degrees )
{
	if( angle_in_degrees < 0 ) angle_in_degrees += 720;
	angle_in_degrees = fmod( angle_in_degrees , 360 ) ;
	return angle_in_degrees ; 
}

/* Convert any angle to an angle between 0º and 359.(9)º */
float wrapTo360f( float angle_in_degrees )
{
	if( angle_in_degrees < 0 ) angle_in_degrees += 720;
	angle_in_degrees = fmodf( angle_in_degrees , 360 ) ;
	return angle_in_degrees ; 
}

/* Convert angle degrees to radians */
double degrees_to_radians( double degrees )
{
	return degrees * (M_PI/180.0);
}

/* Convert angle radians to degrees */
double radians_to_degrees( double radians )
{
	return radians * (180.0/M_PI);
}

/* get time since drk initiated */
double drk_elapsedTimeSecs( void )
{
	static struct timeval start = {0,0} ; /* persistent */
	
	if ( start.tv_sec == 0 )
		gettimeofday( &start, NULL ) ;
	
	/* current time : */
	struct timeval end;
	gettimeofday(&end, NULL);

	double seconds 	= end.tv_sec	- start.tv_sec ;
	double useconds = end.tv_usec 	- start.tv_usec ;
	return seconds + (useconds/MILLION);

}

char* getError(  error_t status )
{
	static char msg[100];
	switch ( status )
	{
		case E_TERMINATING :
			snprintf( msg,  sizeof(msg),"External request to terminate");
			break ;
		case E_SUCCESS:
			snprintf( msg,   sizeof(msg),"Success\n");
			break ;
		case E_FILE_CREATION:
			snprintf( msg,  sizeof(msg),"Failed to open log file\n") ;
			break ;
		case E_NO_MEM:
			snprintf( msg,  sizeof(msg), "Failed to malloc\n") ;
			break ;
		case E_NO_INIT:
			snprintf( msg,  sizeof(msg), "Module has not been initiated\n") ;
			break ;
		case E_ROUTE_UNKNOWN:
			snprintf( msg,  sizeof(msg), "No route to given destination\n") ;
			break ;
		case E_FULL_QUEUE:
			snprintf( msg,  sizeof(msg), "Queue is full\n" ) ;
			break;
		case E_INVALID_INPUT:
			snprintf( msg,  sizeof(msg), "Input given is invalid\n" ) ;
			break;
		default: 
			snprintf( msg,  sizeof(msg), "Some other error (%d)\n", status ) ;
	}
	return msg;
}

void printError2( error_t status )
{
	switch ( status )
	{
		case E_TERMINATING :
			PRINTF_FL_WARN( "%s\n",getError(status) );
			break ;
		case E_SUCCESS:
			PRINTF_FL(  "%s\n",getError(status) );
			break ;
		default :
			PRINTF_FL_ERR( "%s\n",getError(status));
	}
}

void printError( error_t status )
{
	switch ( status )
	{
		case E_TERMINATING :
			PRINTF_FL_WARN( "External request to terminate...\n");
			break ;
		case E_SUCCESS:
			PRINTF_FL( "Success\n");
			break ;
		case E_FILE_CREATION:
			PRINTF_FL_ERR( "Failed to open log file\n") ;
			break ;
		case E_NO_MEM:
			PRINTF_FL_ERR( "Failed to malloc\n") ;
			break ;
		case E_NO_INIT:
			PRINTF_FL_ERR( "Module has not been initiated\n") ;
			break ;
		case E_ROUTE_UNKNOWN:
			PRINTF_FL_ERR( "No route to given destination\n") ;
			break ;
		case E_FULL_QUEUE:
			PRINTF_FL_ERR( "Queue is full\n" ) ;
			break;
		case E_INVALID_INPUT:
			PRINTF_FL_ERR( "Input given is invalid\n" ) ;
			break;
		default: 
			PRINTF_FL_ERR( "Some other error (%d)\n", status ) ;
	}
}

/** print array of data, byte by byte **/
void dumpData( uint8_t *data, uint16_t len )
{
	PRINTF_FL("pkt: ");
	for (int i = 0 ; i < len ; i++ )
	{
		printf("[%03"PRIu8"]" , *(data + i) ) ;
	}
	printf("\n");
}

/*********************************** 
 * *****************SIGNALS!! 
 * ***************************/

void unblock_all_signals(void)
{
	struct sigaction sa;
	sigemptyset( &sa.sa_mask ); /* BLOCK ALL ! */
	pthread_sigmask(SIG_SETMASK, &sa.sa_mask, NULL);
}


void block_all_signals(void)
{
	struct sigaction sa;
	sigfillset( &sa.sa_mask ); /* BLOCK ALL ! */
	pthread_sigmask(SIG_BLOCK, &sa.sa_mask, NULL); 
}

/** do not change current mask, 
 * just unblock this one **/
void unblock_one_signal(int sig)
{
	struct sigaction sa;
	sigemptyset( &sa.sa_mask ); 
	sigaddset( &sa.sa_mask, sig );
	pthread_sigmask(SIG_UNBLOCK, &sa.sa_mask, NULL);
}

/** do not change current mask, 
 * just Block this one **/
void block_one_signal(int sig)
{
	struct sigaction sa;
	sigemptyset( &sa.sa_mask ); 
	sigaddset( &sa.sa_mask, sig );
	pthread_sigmask(SIG_BLOCK, &sa.sa_mask, NULL);
}

error_t sig_set_action( void(*cb)(int), int sig )
{
	struct sigaction sa ;
	CLEAR(sa);
	sa.sa_handler = cb ;
	sigemptyset( &sa.sa_mask );
	if ( -1 == sigaction( sig, &sa, NULL) )
	{
		PRINTF_FL_ERR( "Failed to set SIGTSTP handler\n") ;
		return E_OTHER ;
	}
	PRINTF_FL( "SIGTSTP handler - OK\n") ;
	return E_SUCCESS;
}


/***************** 
 * SEMAPHORES !! 
 * ************/
 
/** **/
error_t sem_wait_safe(sem_t *semaphore_ptr, char *str, volatile int *exit_flag)
{
	int debugger=0;
	int ret ;
	semwait_label:
	if (*exit_flag)
	{
		PRINTF_FL_WARN("semwait %s returned; Terminating\n",str);
		return E_TERMINATING ;
	}
	ret = sem_wait(semaphore_ptr); /* sleep thread until free spot - decrease value */
	/* error handling */
	if (*exit_flag)
	{
		PRINTF_FL_WARN("semwait %s returned; Terminating\n",str);
		return E_TERMINATING ;
	}
	if ( (0 > ret) && (EINTR == errno) )
	{
		PRINTF_FL_WARN("sem_wait %s interrupted. retrying\n",str);
		
		debugger++;
		if (debugger == 3)
			return E_TERMINATING;
			
		goto semwait_label;
	}
	if (0 > ret) 
	{
		PRINTF_FL_ERR( 
			"sem_timedwait %s failed: %s\n", 
			str,
			strerror(errno) ) ;
		return E_SEMAPHORE ;
	}
	
	return E_SUCCESS ; /* sem wait returned after a sem_post */
}	

/** **/
error_t sem_timedwait_safe( sem_t *semaphore_ptr, char *str, 
	volatile int *exit_flag, useconds_t t_us )
{
	/* wait here until txthread tells us there's a packet*/
	struct timespec abs_timeout ;
	int ret ;
	tr_sem_timedwait:
	getCurrentTime( &abs_timeout ) ;
	
	//PRINTF_FL(
	//"abs for %lu+%luns\n", 
	//abs_timeout.tv_sec,
	//abs_timeout.tv_nsec );
	
	/*take care of seconds */
	while (t_us>=1000000)
	{
		abs_timeout.tv_sec++;
		t_us-=1000000;
	}
	
	if (999999999UL < (uint64_t)abs_timeout.tv_nsec + t_us*1000 )
	{
		abs_timeout.tv_sec++;
		abs_timeout.tv_nsec = (long)t_us*1000 - (999999999UL- (uint64_t)abs_timeout.tv_nsec ) ;
		
		//PRINTF_FL_WARN("overflow nsec\n");
	}
	else
	{
		abs_timeout.tv_nsec += (long)t_us*1000 ; /* abs_timeout is the end of the slot  */	
	}
	
	if ( abs_timeout.tv_nsec >= 1000000000L )
	{
		PRINTF_FL_ERR("IMPOSSIBLE - tvnsec > 999999. %ld\n", 
			abs_timeout.tv_nsec);
		abs_timeout.tv_nsec = 999999999L;
	}
	//PRINTF_FL(
		//"abs for %lu+%luns\n", 
		//abs_timeout.tv_sec,
		//abs_timeout.tv_nsec );
	
	ret = sem_timedwait(semaphore_ptr, &abs_timeout); 
	/* error handling */
	if (*exit_flag)
	{
		PRINTF_FL_WARN("semwait %s returned; Terminating\n",str);
		return E_TERMINATING ;
	}
	if ( (-1 == ret) && (EINTR == errno) )
	{
		PRINTF_FL_WARN("semwait %s interrupted\n",str);
		goto tr_sem_timedwait;
	}
	if ( (-1 == ret) && (ETIMEDOUT == errno) )
	{
		//PRINTF_FL_WARN("time OUT!\n") ;
		//PRINTF_FL_WARN(".................\n");semaphoreDebugger(0);
		return E_TIMEDOUT;
	}
	if (-1 == ret)
	{
		PRINTF_FL_ERR( 
			"sem_timedwait %s failed: %s. usecs %u\n", 
			str,
			strerror(errno) ,
			t_us ) ;
		return E_SEMAPHORE ;
	}
	
	return E_SUCCESS;
}

/** create a named semaphore and check errors **/
int drk_sem_create( char *sem_name , sem_t **sem_ptr )
{

	char sem_name1[100];
	snprintf( sem_name1, sizeof(sem_name1)-1, 
		"%s_%02d", sem_name, getMyIP() );
	
	sem_t * ptr = sem_open( sem_name1 , O_CREAT|O_EXCL, S_IRWXU , 1 ) ;
	while ( ptr == SEM_FAILED )
	{
		PRINTF_FL(  "Semaphore [%s] - already exists!\n", sem_name1 ) ;
		if ( sem_unlink( sem_name1 ) == -1 )
		{
			PRINTF_FL(  "Semaphore [%s] - fail to unlink!\n", sem_name1 );
			return -1 ;
		}
		else
		{
			PRINTF_FL(  "Semaphore [%s] - unlinked!\n", sem_name1 );
			ptr = sem_open( sem_name1 , O_CREAT|O_EXCL, S_IRWXU , 1 ) ;
			//printf("[%p]\n", (void*) ptr );
		}
		
	}
	/* testing semaphore */
	sem_wait( ptr );
	PRINTF_FL(  "Semaphore [%s] - created n tested successfully!\n", sem_name1 ); 
	sem_post( ptr );
	*sem_ptr = ptr ;
	return E_SUCCESS ;

	
}


/** **/
int drk_sem_cleanup( char *sem_name, sem_t *sem_ptr )
{

	char sem_name1[100];
	snprintf( sem_name1, 99,"%s_%02d" ,sem_name, getMyIP() );
	
	if( sem_ptr != SEM_FAILED )
	{
		sem_post( sem_ptr );

		/* Close the Semaphore */
		if ( -1 == sem_close( sem_ptr ) )
		{
			PRINTF_FL_ERR(  "%s - failed to close (%s)\n", sem_name1, strerror( errno ) ) ;
			return -1 ;
		}
		else
		{
			//PRINTF_FL( "Navdata Sem Closed\n");			
			/* Delete the shared memory object */
			if ( sem_unlink( sem_name1 ) == -1 )
			{
				PRINTF_FL_ERR(  "%s - failed to unlink (%s)\n", sem_name1, strerror( errno ) ) ;
				return -1;
			}
			//else
			//	PRINTF_FL( "Navdata Sem Unlinked\n");
		}
		
	}
	sem_ptr = NULL;
	//PRINTF_FL(  "%s semaphore cleaned\n", sem_name1 );
	return E_SUCCESS ;
}


int my_mutex_lock(pthread_mutex_t *mutex)
{
	if (0 != pthread_mutex_lock(mutex) )
	{
		PRINTF_FL_ERR("mutex lock FAILED\n");
		return -1 ;
	}
	//PRINTF_FL("lockd\n");
	return 1 ;

}


int my_mutex_unlock(pthread_mutex_t *mutex)
{
	if (0 != pthread_mutex_unlock(mutex) )
	{
		PRINTF_FL_ERR("mutex unlock FAILED\n");
		return -1 ;
	}
	//PRINTF_FL("unlockd\n");
	return 1 ;
	

}


/* prepare sockaddr */
void prepare_sockaddr(uint8_t dst, uint16_t port, struct sockaddr_in * addr)
{
	addr->sin_family = AF_INET;
	addr->sin_port = htons(port);
	
	char addrstr[16];
	snprintf(addrstr, sizeof(addrstr), BASEIP"%d", dst);
	int ret = inet_pton(AF_INET, addrstr, &(addr->sin_addr.s_addr));
	if (ret != 1)
	{PRINTF_FL_ERR("Error on inet_pton!\n"); exit(1);}
	
	//PRINTF_FL("Prepared %s:%d\n", 
		//inet_ntoa(addr->sin_addr), 
		//ntohs(addr->sin_port) );
}
/* end of sockaddr */	
