/***************************
* GENERIC application stuff
*******************************/
/* this is in includes of drklib */


#ifndef _UTILS_H
#define _UTILS_H

#include <inttypes.h>	/* int8_t etc */
#include <netinet/in.h> /* addrss */
#include <stdio.h>
//#include <time.h>
#include <unistd.h> /* useconds_t */
#include <stdlib.h> /* memset */
#include <string.h>
/********************
 * very generic constants
 * ******************/
#define KDFL  "\x1B[0m"
#define KRED  "\x1B[31;4m"
#define KGRN  "\x1B[32;1m"
#define KYEL  "\x1B[33;1m"
#define KBLU  "\x1B[34;1m"
#define KMAG  "\x1B[35;1m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

/* filename, and functionname */
#define KFILE KBLU
#define KFUNC KGRN
/* give colors for ERROR, WARNING and NORMAL msgs */
#define KERRO KRED
#define KWARN KYEL
#define KNORM KWHT


#define BILLION				1000000000.0
#define MILLION				1000000.0
#define NONE 				-1

#define CLEAR(x)			memset((void*)(&(x)), 0, sizeof(x))

#define MIN(a,b)			(((a)<(b))?(a):(b))
#define MAX(a,b)			(((a)<(b))?(b):(a))    
#define BOUND(a,L)			MAX(MIN( (a), (L) ), -(L) )  
#define ABS(a)				(((a)<(0))?(-a):(a))

#define ARRAYSIZE(arr) 		(sizeof(arr) / sizeof(arr[0]))
#define LASTINDEX(arr)		(ARRAYSIZE(arr)-1)
#define MEMBER_TYPE(type, member, p) typeof(p(((type *)0)->member))
#define MEMBER_SIZE(type, member, p) sizeof(p(((type *)0)->member))



#if VERBOSE 
	#define PRINTF_FL(...) \
    do{ \
		char __string__[200]; \
		snprintf(__string__, 199 , __VA_ARGS__ ) ; \
		fprintf(\
			stdout, \
			"%.3f " \
			KFILE "%-9.9s" \
			KFUNC "[%-4.4d] %-10.10s " \
			KNORM  "%s" KDFL , \
			drk_elapsedTimeSecs(), \
			PREFIX, \
			__LINE__, \
			__func__, \
			__string__); \
		fflush(stdout); } while (0)
#else
	#define PRINTF_FL(...) /*nothing*/
#endif

/* macro to print errors */
#define PRINTF_FL_ERR(...) \
    do{ \
		char __string__[200]; \
		snprintf( __string__ , 199 , __VA_ARGS__ ) ; \
		fprintf( \
			stderr, \
			"%.3f " \
			KFILE "%-9.9s" \
			KFUNC "[%-4.4d] %-10.10s " \
			KERRO "%s" KDFL, \
			drk_elapsedTimeSecs(), \
			PREFIX , \
			__LINE__,__func__,\
			__string__); \
		fflush(stdout); } while (0)

#define PRINTF_FL_WARN(...) \
    do{ \
		char __string__[200]; \
		snprintf( __string__, 199 , __VA_ARGS__ ) ; \
		fprintf( \
			stderr, \
			"%.3f " \
			KFILE "%-9.9s" \
			KFUNC "[%-4.4d] %-10.10s " \
			KWARN "%s" KDFL, \
			drk_elapsedTimeSecs(), \
			PREFIX, \
			__LINE__, \
			__func__, \
			__string__); \
		fflush(stdout); } while (0)
		
		
#define BEE			while( !global_exit_all_threads ) sleep(1); 

#if defined(SIMULATOR) || defined(WORLDSIM)
#define BASEIP 		"127.0.0."
#else
#define BASEIP 		"192.168.2."
#endif

#ifndef SIMULATOR
//#define NIC 		"ath0" // name of the NIC of the drone
#define NIC 		"wlan0" // name of the NIC of the drone
#else
#define NIC 		"lo" //"wlan1" // name of the NIC of my computer
#endif

//typedef struct {
	//uint8_t debug_level ;
	//uint8_t other_stuff ;
//} param_app_t ;

///* save here all threads initiated, and the current count */
typedef struct {    /* Used as argument to thread_start() */
	pthread_t	tid[10]; /* save info up to 10 threads */
	int 		inputs[10][5]; /* allow up to 5 inputs to each thread */
	int 		n; /* current count of threads in the struct */
} tinfo_t;



struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
	unsigned long period_us ;
};


enum errors {
	E_SUCCESS 			= +1 ,
	E_TIMEDOUT			= +2 ,
	E_IGNORE			= +3 ,
	E_TERMINATING		= -1 , /* keep -1 . everything lower than -1 is an actual (unexpected) error */
	E_FILE_NOT_FOUND 	= -2 , 
	E_FILE_CREATION 	= -3 ,
	E_INVALID_INPUT 	= -4 ,
	E_NOTHING_DELETABLE = -5 ,
	E_FULL_QUEUE 		= -6 ,
	E_ROUTE_UNKNOWN 	= -7 ,
	E_NO_MEM			= -8 ,
	E_NO_INIT 			= -10 , /* TODO>confirm this doesn-t change - inband error -> has to be outside [-1,1] . TODO: change this stupid design */
	E_INVALID_ITEM 		= -11 ,
	E_PTHREAD 			= -12 ,
	E_BIND				= -13 ,
	E_SOCKET 			= -14 ,
	E_READING			= -15 ,
	E_INVALID_FORMAT	= -16 ,
	E_SEMAPHORE			= -18 ,
	E_TOOBIG 			= -19 ,
	E_UNK_DST 			= -20 ,
	E_EMPTYQUEUE		= -21 ,
	E_OTHER				= -100 ,
	

};

typedef enum errors error_t ;


/**********************
 * Generic functions
 ********************/
/* network socket stuff */
uint8_t getOtherIP(struct sockaddr_in si_other); /* get last part of ip, from a struct sockaddr_in */
int16_t getMyIP(void); /* Get my IP's last part */
#ifdef SIMULATOR
void setMyIP(uint8_t my_id); // only simulator mode needs IP to be set, since every instance runs at a local machine.
#endif
//param_app_t getParamApp(); /* retrieve app params from file */
//int8_t loadParamApp(); /* load app params from file */
int open_log_file(const char* const prefix , 
	const char * const sufix , FILE **file_p); /* open a log file */
int 	microsleep(uint64_t tmp_us); /* sleep for <tmp_us> microseconds, return -1 if it fails */
int 	wait_period(struct periodic_info *info); /* waits for an alarm to go off */
int 	make_periodic(uint64_t init_us, uint64_t period_us, struct periodic_info *info);
int 	getCurrentTime(struct timespec *temp_time); /* get current time */
double 	getEpoch(void); /* same but in seconds */

/* statistics */
int32_t computeMax( 	float array[], int64_t n);
int32_t computeMedian( 	float array[], int64_t n);
int32_t computeMean( 	float array[], int64_t n);
int32_t computeStd( 	float array[], int64_t n, int32_t average);
int32_t computeMin( 	float array[], int64_t n);

/* math, trigonometry , etc */
void normalrand(float *z1, float *z2);

double 	wrapTo360(double angle_in_degrees); /* convert any angle-in-degrees to range 0-360º */
float 	wrapTo360f(float angle_in_degrees); /* convert any angle-in-degrees to range 0-360º */


double 	degrees_to_radians(double degrees);
double 	radians_to_degrees(double radians);

/** sem related **/
#include <semaphore.h>
int 	drk_sem_create(char *sem_name, sem_t **sem_ptr); /* force creation of semaphore by name */
int 	drk_sem_cleanup(char *sem_name, sem_t *sem_ptr); /* clean semaphore by name */
int 	my_mutex_lock(pthread_mutex_t *mutex);
int 	my_mutex_unlock(pthread_mutex_t *mutex);

/** terminal **/
void 	drk_restore_termios(void);
void 	drk_setup_termios(void); 

double 	drk_elapsedTimeSecs(void);

/** error handling **/
void  printError(error_t status);
char* getError(error_t status);

void dumpData(uint8_t *data, uint16_t len);

void unblock_all_signals(void);
void block_all_signals(void);
void unblock_one_signal(int sig);
void block_one_signal(int sig);
error_t sem_wait_safe(sem_t *semaphore_ptr, char *str, volatile int *exit_flag);
error_t sem_timedwait_safe(sem_t *semaphore_ptr, char *str, volatile int *exit_flag, useconds_t t_us );
error_t sig_set_action(void(*cb)(int), int sig);

/** network related **/
void prepare_sockaddr(uint8_t dst, uint16_t port, struct sockaddr_in * addr);
#endif


