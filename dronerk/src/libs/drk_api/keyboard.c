/*************
 * grab key presses and run some manual commands
 * 
 * **********/ 
#include <pthread.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/timerfd.h> /* makeperiodic , wait_period */

//#include "drk/keyboard.h"
#ifndef NOTADRONE
#include "drk/internal_sensor_raw.h" /* print battery */
#include "drk/gps.h" /* gps_t */
#include "drk/autonomous.h" /* waypoint_t,, goto */
#include "drk/flight_control_api.h"
#include "drk/ar_configuration.h"
#endif

#ifdef SIMULATOR
#include "drk/sim_clock.h"  /* getClock , etc ... */
#endif
#include "drk/tdma.h"
#include "drk/utils.h"
#include "drk/keyboard.h"

#define PREFIX "KBD "
#define KBD_PERIOD 10000

#ifndef KBD_LOADED_CONDITION
#define KBD_LOADED_CONDITION \
	if (Kyb_initiated == 0) return 
#endif

/* prototypes */
void* keyboard_handler_thread(); /* Runs constantly in the background. Assign actions to keys */
error_t standard_key_handler( int c ) ;
error_t special_key_handler( int c ) ;


/** 
 * evil globals 
 * **/
static int (*Global_action[37])(int) = {NULL} ; /* pointer to a function with a int input */
static int Global_action_inputs[37]={-1} ;
//char 			Global_key[37] = '5' ; /* pair a Key with a given action */

static int 		Exit_module = 0 ; /* flags closure of this module */
static int 		Kyb_initiated = 0 ;
static int	 	Lockdown = 0 ; /* manual control ON(1) / OFF(0) */
static pthread_t 	key_tid ;



/* externed , get Lockdown status. TODO: use mutex */
int drk_keyboard_lockDown( void ) 
{
	return Lockdown; 
}



/**********
 * Add a new key shortcut
 * **************
 * action = function pointer to be called (single input parameter - int)
 * input = value input of the function - int 
 * key = key to assign (only letters and digits)
 * */
error_t drk_keyboard_setKey( int(*action)(int) , int input , char key )
{
	int id =-1;
	
	
	/* handle digits */
	if ( key>='0' && key<='9' )
	{
		id = key - '0' ; /* get a unique ID ,given they key */
		goto define_callback;
	}

	/* handle letters */
	if ( key>='A' && key<='Z' ) /* between A and Z (caps)*/
		key = (key - 'A') + 'a';  /* convert to smallcaps */		
	if ( key >= 'a' && key <= 'z' ) /* it's a letter */
	{
		id = (key - 'a') + 10 ;
		goto define_callback;
	}
	
	/* key is invalid: */
	printError( E_INVALID_INPUT ) ;
	return E_INVALID_INPUT;
	
	/* save both callback function and parameter to use when calling*/
define_callback :
	PRINTF_FL("Setting key [%c] (priv id %d) \n", key ,id);
	Global_action[id] = action ;
	Global_action_inputs[id]= input ;
	return E_SUCCESS;
		
	

	
}

/* Initialize thread for keyboard control */
/* return a ptr to thread_tid */
error_t drk_keyboard_init( void )
{
	//unblock_one_signal(SIGUSR1);
	//sig_set_action( sugusr_hdlr, SIGUSR1);
	
	
	drk_setup_termios();
	pthread_attr_t key_attr;
	pthread_attr_init(&key_attr) ;
	size_t key_desired_stack_size = 400000 ;
	pthread_attr_setstacksize(&key_attr, key_desired_stack_size);
	if (pthread_create(&key_tid, &key_attr, keyboard_handler_thread ,NULL) != 0) // calls Hover 
	{
		PRINTF_FL(
			"["PREFIX"] thread - Error spawning\n");
		return E_PTHREAD ;
	}
	
	if ( pthread_attr_destroy(&key_attr) != 0 )
	{
		PRINTF_FL_ERR( 
			"["PREFIX"] thread - Error destroying attr: %s\n",
			strerror(errno));
		return E_PTHREAD;
	}
	PRINTF_FL(
		"["PREFIX"]thread created with success\n");
	
	Kyb_initiated = 1 ;
	return E_SUCCESS ;
}


error_t drk_keyboard_close( void )
{
	KBD_LOADED_CONDITION E_NO_INIT;
	Kyb_initiated = 0 ;
	Exit_module = 1 ;
	//pthread_kill( key_tid, SIGTSTP ) ; /* to interrupt all blocking calls */
	//PRINTF_FL_WARN("joinin\n");
	
	pthread_kill( key_tid, SIGUSR1	);
	pthread_join( key_tid, NULL) ; /* wait for termination of thread */
	//PRINTF_FL_WARN("finished\n");
	/* restore terminal */
	drk_restore_termios();
	//PRINTF_FL( "["PREFIX"] Closed\n");
	
	
	return E_SUCCESS ;
}

error_t call_callbacks( int idx )
{
	if ( Global_action[idx] == NULL )
		return E_INVALID_ITEM ;
		
	return Global_action[idx]( Global_action_inputs[idx] )  ;
	
}

void print_instructions(void)
{
	printf("\n");
	printf("\t["PREFIX"] control enabled.\n");
	printf("\t[q-e] -> Spin left/right\n");
	printf("\t[w-a-s-d] -> Move Horizontally\n");
	printf("\t[z-x] -> Up/Down\n");
	printf("\t[b] -> Bat status\n");
	printf("\t[up-down arrow] -> TakeOff/Land\n");
	printf("\t[y-p] -> Resume/Pause Flight\n");
	printf("\t[f-v-c] -> CalibrateIMU/Basestation/Magnetometer\n");
	printf("\t[i-l-k-j] -> Set waypoint to the North/East/South/West\n");
	printf("\t[0] -> Go to BS\n");
	printf("\n");
}

int toupper(int c)
{
	if ( c>='A' && c<='Z' ) /* between A and Z (caps)*/
		c = c -'A' + 'a';  /* convert to small */
	return c ;
}




/* Runs constantly in the background. Assign actions to keys by adding a case */
void *keyboard_handler_thread()
{
	
	
	int c = '0';
	while( !Exit_module )
	{
		c = getchar(); /* wait for a char for 0.3s , then timeout (cf. drk_setup_termios(), drk.c )*/
		
		c = toupper(c); /* ignore caps */
		
		/* first check special keys overwrites */
		if ( special_key_handler( c ) > 0 )
			continue ;

		/* standard key actions: */
		if ( standard_key_handler( c ) > 0)  
			continue ;
		
		
		microsleep( KBD_PERIOD ); /* Pause for KBD_PERIOD us*/
		
    	/* Flush any input received while processing, prevent queuing */
		tcflush(STDIN_FILENO, TCIFLUSH);
	}

	PRINTF_FL( "["PREFIX"] Thread terminated normally\t\t[%08x] *******\n", 
		(unsigned int)pthread_self() );  

	
	return NULL ;
}


error_t standard_key_handler( int c ) 
{
	
	#ifndef BS
	int fly_time = 0; // timespan to return from drk_translate()
	const double spin_rate = 0.5, angle = 0.9, gaz_rate = 0.7; // this can all go up to 1.
	#endif 
	
	switch (c)
	{
		case '\\': 
			/* backslash:
			 * Enable or disable keyboard keys. 
			 * show sinstructions */
			if (Lockdown == 0) 
				print_instructions();
			else
				printf("\t["PREFIX"] Manual control disabled.\n");
			Lockdown = !Lockdown ;
			break;
			
	#ifndef BS
	case KEY_HOVER: 
		drk_autonomous_pause();
		drk_hover(0);
		PRINTF_FL_WARN("HALT\n");
		break;
		
	/* movement W-A-S-D-Q-E-Z-X: */
	case KEY_FWD:
		if (Lockdown) {
			printf("FWD\n");
			drk_move_forward( angle, fly_time ) ;}
		break;

	case KEY_LFT:
		if (Lockdown)  {
			printf("LFT\n");
			drk_move_left(angle, fly_time); }
		break;

	case KEY_BCK:
		if (Lockdown) {
			printf("BCK\n");
			 drk_move_backward(angle, fly_time);}
		break;

	case KEY_RHT:
		if (Lockdown)  {
			printf("RHT\n"); 
			drk_move_right(angle, fly_time);}
		break;

	case KEY_FWD_LFT://float pitch, float roll, float yaw, float gaz, int time_ms)
		if (Lockdown)  {
			printf("FWD+LFT\n"); 
			drk_translate( -angle, -angle , 0 ,0 ,0);} 
		break;

	case KEY_FWD_RHT:
		if (Lockdown)  {
			printf("FWD+RHT\n"); 
			drk_translate( -angle, angle , 0 ,0 ,0);} 
		break;

	case KEY_BCK_LFT:
		if (Lockdown)  {
			printf("BKW+LFT\n"); 
			drk_translate( angle, -angle , 0 ,0 ,0);} 
		break;

	case KEY_BCK_RHT:
		if (Lockdown)  {
			printf("BKW+RHT\n");
			drk_translate( angle, angle , 0 ,0 ,0);} 
		break;

	case KEY_SPIN_ACC:
		if (Lockdown)  {
			printf("SP+TOPVIEW+ANTICLK\n");
			drk_spin_left(spin_rate, fly_time);}
		break;

	case KEY_SPIN_CC:
		if (Lockdown)  {
			printf("SP+TOPVIEW+CLK\n");
			drk_spin_right(spin_rate, fly_time);}
		break;

	case KEY_UP:
		if (Lockdown)  
		{
			printf("UP\n"); 
			drk_move_up( gaz_rate , fly_time);}
		break;

	case KEY_DOWN:
		if (Lockdown)  {
			printf("DOWN\n"); 
			drk_move_up( -gaz_rate , fly_time);}
		break;
		
	case KEY_CALIBRATE:
		drk_ar_calibrate_magnetometer();
		break;	

	case KEY_FLAT_TRIM:				
		drk_ar_flat_trim(); /* zeroes accelerometers */
		break;

	case KEY_BATTERY:
		drk_print_battery();  /* print battery level */
		break;
		
		
	/* autonomous related keys */	

	case KEY_PAUSE:
		drk_autonomous_pause();
		break;
	case KEY_RESUME:
		drk_autonomous_resume();
		break;


	/* manual set waypoint (i-j-k-l) */
	#define MOVE_METERS	10.0
	case KEY_GO_NORTH: // NORTH
		drk_autonomous_goNorth( MOVE_METERS );
		break;

	case KEY_GO_SOUTH: // SOUTH
		drk_autonomous_goNorth( -MOVE_METERS );
		break;
	case KEY_GO_EAST: // EAST
		drk_autonomous_goEast( MOVE_METERS );
		break;

	case KEY_GO_WEST: // WEST
		drk_autonomous_goEast( -MOVE_METERS );
		break;
	#undef MOVE_METERS
	case KEY_GO_BS: /* Go to BS */
		drk_autonomous_goBS();
		break;
			
	case KEY_GO_NOWHERE: /* Set current location as current waypoint (this anchors the uav where it is) */
	{
		gps_waypoint_t tmpw ;
		tmpw.llh = drk_gps_data() ;
		tmpw.llh.altitude = 10;
		//tmpw.llh.altitude = 15 ;
		tmpw.max_output = 1 ;
		tmpw.distance_tolerance = 1 ; 
		drk_autonomous_set_waypoint	( tmpw ) ;
		break;
	}
	case KEY_PRINT_MAP: /* show ascii map with: BS, cur_pos, target_pos */
		drk_print_map() ;
		break ;



	/* Takeof-land: Arrows */
	case '\033' :
		getchar(); // skip middle char
		c = getchar(); // wait for a char
		switch (c)
		{
			//drk_autonomous_pause(); drk_land();
			case 'A': /* up arrow, former 't'*/
				if (Lockdown)
				{
					PRINTF_FL_WARN( "TAKING OFF..!\n" );
					drk_autonomous_pause(); 
					drk_takeoff();
				}
				PRINTF_FL_WARN(" TAKE OFF - Blocked\n") ;
				break;
			case 'B':  /* down arrow, former 'l' */
				drk_autonomous_pause(); 
				drk_land(); // land whenever. no Lockdown required
				break;
			case 'C':
				//PRINTF_FL_WARN("N/A\n"); /* right arrow */
				break;
			case 'D':
				//PRINTF_FL_WARN("N/A\n"); /* left arrow */
				break;
			default :
				PRINTF_FL_ERR("\nweird\n");
				break;
		}
		break ;
	#endif /* #ifndef BS */


	/* TDMA related keys */
	case '7':
		PRINTF_FL_WARN("TDMA OFF\n");
		TDMA_off();
		break;
	case '8':
		PRINTF_FL_WARN("TDMA RIGID\n");
		TDMA_rigid();
		break;
	case '9':
		//PRINTF_FL_WARN("TDMA DYNAMIC\n");
		//TDMA_dynamic();
		PRINTF_FL_WARN("TDMA Nosynch\n");
		TDMA_nosync();
		break;
		
	//~ case '+':
	//~ {
		//~ uint16_t slot = TDMA_ip2SlotWidth( 0 ) ;
		//~ slot -= 2 ;
		//~ PRINTF_FL_ERR("Slot %d\n", slot ) ;
		//~ TDMA_reqSlotWidth( slot , 0 ) ;
		//~ break ;
	//~ }
	//~ case '*':
	//~ {
		//~ uint16_t slot = TDMA_ip2SlotWidth( 0 ) ;
		//~ slot +=2 ;
		//~ PRINTF_FL_ERR("Slot %d\n", slot) ;
		//~ TDMA_reqSlotWidth( slot , 0 ) ;
		//~ break ;
	//~ }
	//~ case '5':
	//~ {
		//~ uint16_t slot = TDMA_ip2SlotWidth( 10 ) ;
		//~ slot -= 2 ;
		//~ PRINTF_FL_ERR("Slot %d\n", slot) ;
		//~ TDMA_reqSlotWidth( slot , 10 ) ;
		//~ break ;
	//~ }
	//~ case '6':
	//~ {
		//~ uint16_t slot = TDMA_ip2SlotWidth( 10 ) ;
		//~ slot += 2 ;
		//~ PRINTF_FL_ERR("Slot %d\n", slot) ;
		//~ TDMA_reqSlotWidth( slot , 10 ) ;
		//~ break ;
	//~ }
		
	default :
		return E_INVALID_INPUT ; /* key is unknown */
	}
	return E_SUCCESS ; /* key has been processed */
}

error_t special_key_handler ( int c ) 
{
	if ( c>='a' && c<='z' )
	{
		int idx = (c-'a') + 10 ; 
		if ( call_callbacks( idx ) > 0 ) 
			return E_SUCCESS ; /* Callback has been called */
	}
	else if ( c>='0' && c<='9' )
	{
		int idx = (c-'0') ; 
		if ( call_callbacks( idx ) > 0 ) 
			return E_SUCCESS ; /* Callback has been called */
	}
	
	return E_INVALID_INPUT ; /* no callback registered for this key */
}
