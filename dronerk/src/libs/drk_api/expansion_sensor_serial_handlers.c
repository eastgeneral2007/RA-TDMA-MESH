/**
 * expansion_sensor_serial_handlers.c
 * aka ESSH
 * TODO: revise ESSH and ESA . theyre diff files, how should this be organized 
 */


#include <pthread.h>
#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <math.h>
#include <inttypes.h>

#include "drk/expansion_sensor_serial_handlers.h"
#include "drk/expansion_sensor_api.h"
#include "drk/gps.h"
#include "drk/drk.h"
#include "drk/utils.h"

#define GPS_SAMPLE_SECONDS_TOLERANCE 2LL /* TODO: explain this */
#define PREFIX_ESSH "ESSH" 

enum ggp_return_code
{
	NO_CHANGE=1, 
	NEW_RTK=2, 
	NEW_SPP=3,
	EXPIRED=4,
	GGP_ERROR=-1
};


/****************
 * Externs
 ***************/
//extern pthread_mutex_t gps_mutex ; // instatiated in gps.c
extern int last_compass;
extern int last_GPS;
extern int last_radio;
extern double base_lat ;
extern double base_lon ;
extern struct global_serial * serial_buf ; //struct defined in ESA.h




/**********
 * internal prototypes 
 ******/
int _drk_ggp_handler(char line[SENSOR_LINE_LENGTH]) ;


/*******************************************************************************
            SETUP FUNCTIONS
*******************************************************************************/


/* Dispatch a thread to continuously receive data */
/*
int _drk_expansion_gather_data()
{
    if (!(serial_pid = fork()))
    {
        printf("****** SERIAL PID [%d] *******\n", getpid());
        signal(SIGTSTP, ignore_signal);
        signal(SIGINT, ignore_signal);
        
        struct sched_param p;
    
        p.sched_priority = 6;
    
        sched_setscheduler(getpid(),SCHED_FIFO,&p);
        
        //_drk_expansion_sensor_thread();
        drk_exit(EXIT_SUCCESS);
    }
    return 1;
}
*/
/* Continuously receive data and process it */
/*
void _drk_expansion_sensor_thread()
{   

    char lineBuffer[SENSOR_LINE_LENGTH];
    int bytesRead = 0;
    int offset=0;
    int i=0;
    // Make sure the port has been initialized 
    if(port_fd == -1)
    {
        perror("Error: Port not initialized!");
        return;
    }
    
    FILE *serfile;
    serfile = fopen("./timestamps/serial.txt", "w");
  
    // Receiving loop 
    while (1)
    {
      
      //fprintf(serfile, "5\t%f\n", drk_elapsed_time_secs());
      //bytesRead = read(port_fd, lineBuffer, SENSOR_LINE_LENGTH-1); // Save 1 for \0 
      //fprintf(serfile, "6\t%f\n", drk_elapsed_time_secs());
      
      // Since read is in blocking mode, we don't need a sleep
      bytesRead=0;
      if (bytesRead > 0)
      {
          _drk_parse_line(lineBuffer, bytesRead);
      }
      usleep(100000);
      
      sleep(1000);
    }
}
*/

/*******************************************************************************
	PARSING
*******************************************************************************/
/* Process one line of serial data */
void _drk_parse_line( char *line, int bytesRead)
{
	//for (int i = 0 ; i<=bytesRead ; i++)
	//{
		//switch ( line[i] )
		//{
		//case '\r': 
			//printf("[\\r]") ;
			//break;
		//case '\n': 
			//printf("[\\n]") ;
			//break;
		//case '\0': 
			//printf("[\\0]") ;
			//break;
		//default:
			//printf("[%c]", line[i]);
		//}
	//}
	//printf("\n") ;
	//printf("SSS@\t\t <%s> \n" ,line );
	(void)(bytesRead); // suppress warnings. TODO: this should be changed. 
	//printf("got a line\n") ;
	//printf("%s\n" , line ) ;
	
	/* Compare checksum to ensure intact packets */ //todo:explain this name
	if (_drk_1ate_packet(line) != 1)
		return ;

	/* Dispatch the proper handler for each line */ 
	if (!strncmp(line, "$GPGGA", 6))  /* GPS: GGA packet */
		_drk_gga_handler(line);

	else if (!strncmp(line, "$GPGLL", 6)) {} /* ignore GPS: GLL packet */
	else if (!strncmp(line, "$GPGSA", 6)) {} /* ignore GPS: GSA packet */
	else if (!strncmp(line, "$GPGSV", 6)) {}   /* GPS: GSV packet */
	else if (!strncmp(line, "$GPRMC", 6)) /* GPS: RMC packet */
		_drk_rmc_handler(line);
	else if (!strncmp(line, "$GPVTG", 6)) /* GPS: VTG packet */
		_drk_vtg_handler(line);
	// else if (!strncmp(line, "$GRIDEYE", 8)) /* Grideye pixel row */
	//   _drk_grideye_handler(line);
	// else if (!strncmp(line, "$INFRARED", 9)) /* Infrared distance */
	//   _drk_infrared_handler(line);
	else if(!strncmp(line, "$GPGGP", 6)) 
	{
		//printf("ggp! piksi relative data\n");
		//printf("GGP ret=>%d\n" ,_drk_ggp_handler(line) ) ;
		_drk_ggp_handler(line) ;
	}
	else if(!strncmp(line, "$RAD", 4)) /* Radio (802.15.4) */
		printf("ESSH@ RADIO DATA! this shoud not happen\n");
	else {} /* Unrecognized */
}

/* Compute the NMEA checksum (XOR of chars between '$' and '*')*/
int _drk_1ate_packet (char * packet)
{
	//int dbg = 1 ;
	int start_chars = 1;    // Truncate '$'
	int end_chars = 5;  // Truncate '*XX\r\n'
	unsigned int computed_checksum = 0, read_checksum = 0;
	int packet_length = strlen(packet);
	int index = 0;

	if(!strncmp(packet, "nanork@", 7)) // temporarily accept all radio packets
		return 1;

	if (packet_length < 10) // Quick sanity check
		return -1;

	for (index = start_chars; index < packet_length - end_chars; index++)
		computed_checksum ^= packet[index];

	char *packetEnd = packet + (packet_length - (end_chars-1)); // Get 'XX\r\n'
	sscanf(packetEnd,"%2x\r\n", &read_checksum);

	
	if (computed_checksum == read_checksum)
		return 1;
	else
	{
		printf(PREFIX_ESSH "Computed [%02x] Read [%02x]\n", computed_checksum, read_checksum);
		printf(PREFIX_ESSH "FAILURE: packet ={");
		for (int i = 0 ; i<=packet_length ; i++)
		{
			switch ( packet[i] )
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
				printf("%c", packet[i]);
			}
		}
		printf("} (%d)\n", packet_length );

		//packet[strlen(packet)-2] = '\0';
		//printf("\n\nFAILURE: packet = <%s>\n", packet);
		//printf("\n\nEnd:<%c,%c>", packet[strlen(packet)-5], packet[strlen(packet)-4] );
			
		
		if(!strncmp(packet, "$RAD", 4)) //temporarily accept all radio packets
			return 1;
		else
			return -1;
	}
}

/*******************************************************************************
	SERIAL PACKET HANDLERS
*******************************************************************************/
/** 
 * Handle Piksi data, relative GPS data return code.
 **/
int _drk_ggp_handler (char line[SENSOR_LINE_LENGTH])
{

	int ret; 
	
	// printf("ESSH@ %s",line) ;
	// update gps struct
	int north, east, down;
	int vel_north, vel_east, vel_down;
	double spp_lat=-1 , spp_lon=-1 , rtk_lat=-1 , rtk_lon=-1 , best_lat=-1 , best_lon=-1;
	double altitude=-1, trueTrack=-1, groundSpeed=-1;
	int8_t spp_num_sats=-1, rtk_num_sats =-1, best_num_sats=-1 ;
	uint32_t rtk_sample_id = 0  , spp_sample_id = 0 ;
	int matches; // tmp var for sscanf return 
	uint64_t hexlatitude, hexlongitude; // tmp var to save SPP lat,lon 
	
	
	

	/* Parse the GGP line for data values  */
	matches = sscanf(line,
		"$GPGGP,%u,%hhd,\
		%d,%d,%d,\
		%d,%d,%d,\
		%u,%hhd,\
		%" PRIx64 ",%" PRIx64 "*",
		&rtk_sample_id, &rtk_num_sats, 
		&north, &east, &down, 
		&vel_north, &vel_east, &vel_down,
		&spp_sample_id, &spp_num_sats ,
		&hexlatitude, &hexlongitude ) ;


	
	
	// old stuff:
	//matches = sscanf(line, "$GPGGP,%d,%d,%d,%d,%d,%d,%d,%d,%lf,%lf",
	//  &quality, &num_sats, &north, &east, &down, &vel_north, &vel_east, &vel_down, &latitude, &longitude);
	//puts(line);
	//printf("ESSH@ dbase = %d\n" , east*east+north*north);
	//printf("lat lng ::  %lx %lx\n",hexlatitude,hexlongitude);
	




	// Check that the line was scanned properly
	if ( matches != 12 )
	{
		printf("ESSH@ Parsing error in GGP handler");
		return GGP_ERROR ; 
	}

	pthread_mutex_lock( &(serial_buf->gps_mutex) );
	time_t last_gps_timestamp = serial_buf->gps_buf.timestamp ; // get last sample timestamp
	uint32_t last_spp_sample_id = serial_buf->gps_buf.spp_sample_id ; // get last sample id SPP
	uint32_t last_rtk_sample_id = serial_buf->gps_buf.rtk_sample_id ; // get last sample id RTK
	pthread_mutex_unlock( &(serial_buf->gps_mutex) );

	time_t seconds_past_epoch = time(0) ; // get current time
  
	/* check if RTK-sampleid increased: */
	if ( rtk_sample_id > last_rtk_sample_id )
	{

		/* update serial_buf */
		// Convert altitude from down
		altitude = down / 1000.0;

		// calcualte lat lon from NED + base
		/**
		 * note: this uses a simple arc length formula, probably not very accurate,
		 *       particularly with regards to latitude, as LAT_RADIUS is the average
		 *       radius between equator and pole, which differ by about .33%
		 */

		// convert coords to meters from mm
		double north_meters = north / 1000.0;
		double east_meters = east / 1000.0;

		// convert arclen to radians
		double north_radians = north_meters / LAT_RADIUS;
		double east_radians = east_meters / LON_RADIUS;

		// convert radians to degrees
		double north_degrees = radians_to_degrees(north_radians);
		double east_degrees = radians_to_degrees(east_radians);

		// lat = base + n, lon = base + e
		rtk_lat = base_lat + north_degrees;
		rtk_lon = base_lon + east_degrees;
		/*
		printf("ESSH@\t\tnorth east ::  %d %d\n\t\tbase_lat base_lat %0.6f %0.6f\n\t\tlat lon %.6f %.6f\n",
			north, east, 
			base_lat, base_lon ,
			rtk_lat , rtk_lon);
		*/
		/* calculate VTG from NED */
		// convert vector to meters from mm
		double vel_north_meters = vel_north / 1000.0;
		double vel_east_meters = vel_east / 1000.0;

		// calculate heading from vector
		double heading_rads = atan2(vel_east*1.0, vel_north*1.0);
		trueTrack = heading_rads * (180 / M_PI);

		//convert: +/- 180 ---> 0-360
		if(vel_east < 0){
			trueTrack = 360+trueTrack;
		}

		// calculate velocity from vector
		groundSpeed = sqrt(pow(vel_north_meters, 2.0) + pow(vel_east_meters, 2.0));

		// convert to km/h
		groundSpeed = (groundSpeed * 3600.0) / 1000.0;


		//spp_lat = *((double *)&hexlatitude) ; //dont.. this is not strict-aliasing
		//spp_lon = *((double *)&hexlongitude) ;
		memcpy( &spp_lat ,  &hexlatitude , sizeof(hexlatitude) ) ;
		memcpy( &spp_lon ,  &hexlongitude , sizeof(hexlongitude) ) ;
		ret = NEW_RTK ; //new rtk
		
	} // end of RTK received
	else
	{
		// RTK is not updating. maybe we have SPP solution?
		if ( spp_sample_id > last_spp_sample_id )
		{
			memcpy( &spp_lat ,  &hexlatitude , sizeof(hexlatitude) ) ;
			memcpy( &spp_lon ,  &hexlongitude , sizeof(hexlongitude) ) ;
			ret = NEW_SPP ; // new SPP
			
		} // end of SPP update
		else 
		{ 
			// check expiration date of data
			//printf("NOW: %ld, LAST timestamp %ld. ", seconds_past_epoch, last_gps_timestamp ) ;
			
			if ( seconds_past_epoch > GPS_SAMPLE_SECONDS_TOLERANCE + last_gps_timestamp ) // TRUE -> expired, FALSE -> ok
			{
				//printf("GPS@ expired!") ;
				// set buffer to all the default values (-1)
				seconds_past_epoch = 0;
				ret = EXPIRED;
				
			}
			else
			{
				ret = NO_CHANGE ;
				pthread_mutex_lock( &(serial_buf->gps_mutex) ) ;
				serial_buf->gps_buf.quality = ret;
				pthread_mutex_unlock( &(serial_buf->gps_mutex) ) ;
				return ret ; // return without changin the sample, expect its quality.
			}
		} 
	}
	
	switch (ret)
	{
		case NEW_RTK:
		{
			/**new rtk**/
			best_lat = rtk_lat ;
			best_lon = rtk_lon ;
			best_num_sats = rtk_num_sats;
			break;
		}
		case NEW_SPP:
		{
			/**new rtk**/
			best_lat = spp_lat ; 
			best_lon = spp_lon ;
			best_num_sats = spp_num_sats ;
			break ;
		}
		case EXPIRED:
		{
			/**expired**/
			// keep everything set to -1 ;
			break;
		}
		
	}


  /* Write out the values to the struct */
	pthread_mutex_lock( &(serial_buf->gps_mutex) );

	serial_buf->gps_buf.latitude = best_lat;
	serial_buf->gps_buf.longitude = best_lon;
	serial_buf->gps_buf.altitude = altitude ; // rtk altitude
	serial_buf->gps_buf.quality = ret ; // 4- expired. 2- new rtk , 3-new spp
 	serial_buf->gps_buf.num_sats = best_num_sats;
	serial_buf->gps_buf.trueTrack = trueTrack;
	serial_buf->gps_buf.groundSpeed = groundSpeed;
	serial_buf->gps_buf.spp_lat = spp_lat ;
	serial_buf->gps_buf.spp_lon = spp_lon ;
	serial_buf->gps_buf.rtk_lat = rtk_lat ;
	serial_buf->gps_buf.rtk_lon = rtk_lon ;
	serial_buf->gps_buf.spp_sample_id = spp_sample_id ;
	serial_buf->gps_buf.rtk_sample_id = rtk_sample_id ;
	serial_buf->gps_buf.timestamp = seconds_past_epoch ; // timestamp the new sample

	pthread_mutex_unlock(&(serial_buf->gps_mutex));

	return ret; // return code
}

/* Handle the infrared packet */
void _drk_infrared_handler(char line[SENSOR_LINE_LENGTH])
{
  (void)(line); // suppress warning
/*
    int left, right, front, back;
    
    int matches = 0;
    // Parse the line for data values 
    matches = sscanf(line, "$INFRARED,%d,%d,%d,%d*%*s",
                    &front, &left, &back, &right);
                    
    if (matches != 4) return;
    
    struct infrared temp_ir = {0, front, left, back, right};
    sem_wait(serial_buf->semaphore);
    temp_ir.index = (serial_buf->infrared_buf).index + 1;
    memcpy(&(serial_buf->infrared_buf), &temp_ir, sizeof(temp_ir));
    sem_post(serial_buf->semaphore); 
*/
}

/* Handle the grideye packets */
void _drk_grideye_handler(char line[SENSOR_LINE_LENGTH])
{
  (void)(line); // suppress warning
/*
  uint16_t temprow[8];
  double temperature[8];
  int row = -1;
  int is_corrupted = 0; // Flag used to check if data is corrupted.
  
  int matches = 0;
  matches = sscanf(line, "$GRIDEYE,%d,%hX,%hX,%hX,%hX,%hX,%hX,%hX,%hX*%*s",
      &row, &(temprow[0]), &(temprow[1]), &(temprow[2]), &(temprow[3]),
      &(temprow[4]),&(temprow[5]),&(temprow[6]),&(temprow[7]));
                  
  // Detect sensor data corruption.
  if (matches != 9) is_corrupted = 1;
  if (row < 0 || row > 7) is_corrupted = 1;
      
  // Temperature conversion if no corruption detected. 
  if (is_corrupted == 0)
  {
    int i;
    for (i = 0; i < 8; i++) 
      temperature[i] = (temprow[i]*0.25); //to get Celsius
  }

  
  
	sem_wait(serial_buf->semaphore);
  int prev_index = (serial_buf->grideye_buf).index;
  
  if (is_corrupted == 0)
  {
    // If not corrupted then fill the grideye_buf row with read temperature. 
    memcpy(&(serial_buf->grideye_buf).temperature[row], temperature, sizeof(temperature));
  }
  else
  {
    if (row >= 0 && row <= 7)
    {
      // If corrupted but has valid row then fill the grideye_buf row with 0.
      memset(&(serial_buf->grideye_buf).temperature[row], 0, sizeof(temperature));
    }
    else
    {
      // If corrupted with invalid row then check previous index and fill appropriate grideye_buf row with 0.
      if (prev_index != 7)
        memset(&(serial_buf->grideye_buf).temperature[prev_index+1], 0, sizeof(temperature));
      else
        memset(&(serial_buf->grideye_buf).temperature[0], 0, sizeof(temperature));
    }
  }

  // Always increment the index counter.
  (serial_buf->grideye_buf).index++;    

  // Increase the frame counter only if we have received all 8 rows.
  // Once all 8 rows received, reset index counter. 
  if (row == 7)
  {
    (serial_buf->grideye_buf).frame++;
    (serial_buf->grideye_buf).index = 0;
  }
  sem_post(serial_buf->semaphore); 
*/
}

/* Handle the $GPGGA data packet. Contains most of the essential data */
void _drk_gga_handler(char line[SENSOR_LINE_LENGTH])
{

	printf("Parsing GGA");

  double latitude, longitude, altitude, UTC;
  int quality, num_sats;
  float hPrecise;
  latitude = longitude = altitude = UTC = -1;
  quality = 0;
  num_sats = 0;
  char N_or_S, E_or_W;
  int matches = 0;
  
  /* Parse the GGA line for data values */
  matches = sscanf(line, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%f,%lf,%*s", 
                  &UTC, &latitude, &N_or_S, &longitude, &E_or_W,
                  &quality, &num_sats, &hPrecise, &altitude);
            
  /* Check that the line was scanned properly */
  if (matches != 9)
  {
    printf("Parsing error in GGA Handler\n");       
    return;
  }

  /* Convert the NMEA degree values to decimal for easy calculations */
  latitude = drk_nmea_to_decimal(latitude);
  longitude = drk_nmea_to_decimal(longitude);
  
  /* Get the right sign */
  if (N_or_S == 'S')
    latitude *= -1;
  if (E_or_W == 'W')
    longitude *= -1;
  
  /* Write the parsed values into the struct */
  //gps_t temp_GPS = {latitude, longitude, altitude, UTC, quality, num_sats, 0};
  pthread_mutex_lock(&(serial_buf->gps_mutex));
  /*temp_GPS.index = (serial_buf->gps_buf).index + 1;
  memcpy(&(serial_buf->gps_buf), &temp_GPS, sizeof(temp_GPS));*/
  (serial_buf->gps_buf).latitude=latitude;
  (serial_buf->gps_buf).longitude=longitude;
  (serial_buf->gps_buf).altitude=altitude;
  (serial_buf->gps_buf).UTC=UTC;
  (serial_buf->gps_buf).quality=quality;
  (serial_buf->gps_buf).num_sats=num_sats;
  (serial_buf->gps_buf).hPrecise=hPrecise;
  (serial_buf->gps_buf).spp_sample_id  = (serial_buf->gps_buf).spp_sample_id + 1;
  pthread_mutex_unlock(&(serial_buf->gps_mutex));
}

/* Handle the $GPVTG data packet. Speed & track*/
void _drk_vtg_handler(char line[SENSOR_LINE_LENGTH])
{
  double trueTrack; // from VTG data-Track, degrees Magnetic
  double groundSpeed; //from VTG data - Speed, Km/hr

  int matches = 0;

  //$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
  //$GPVTG,000.0,T,,     M,000.0,N,000.0,K,N*02
  //      %lf,   T,%*lf, M,%*lf, N,%lf,  K,%*s
  //matches = sscanf(line, "$GPVTG,%lf,T,%*lf,M,%*lf,N,%lf,K,%*s", 

  //this produces warning matches = sscanf(line, "$GPVTG,%lf,T,,M,%*lf,N,%lf,K,%*s", &trueTrack,&groundSpeed);
  matches = sscanf(line, "$GPVTG,%lf,T,,M,%*f,N,%lf,K,%*s", &trueTrack,&groundSpeed);
  
  //printf("VTG Data: Track:%lf Speed in kmph:%lf",trueTrack,groundSpeed);
  if (matches != 2)
  {
    printf("VTG handler parsing error\n");
    printf("%d %lf %lf \n",matches,trueTrack,groundSpeed );
    return;
  } 

  pthread_mutex_lock(&(serial_buf->gps_mutex));
  (serial_buf->gps_buf).trueTrack=trueTrack;
  (serial_buf->gps_buf).groundSpeed=groundSpeed;
  pthread_mutex_unlock(&(serial_buf->gps_mutex));
}

/* Handle the $GPRMC data packet. Current date*/
void _drk_rmc_handler(char line[SENSOR_LINE_LENGTH])
{
  int date;

  int matches = 0;

  //$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  //$GPRMC,221147.132,A,4026.6412,N,07956.8179,W,000.0,000.0,180613,,,A*79 <= Taken from Drone-rk
  //       %d,      AorM,%*lf,   %*c,%*lf,   %*c,%*lf,%*lf  , %d,%*s
  //produces warning matches = sscanf(line, "$GPRMC,%*lf,%*c,%*lf,%*c,%*lf,%*c,%*lf,%*lf,%d,%*s",  &date);
  matches = sscanf(line, "$GPRMC,%*f,%*c,%*f,%*c,%*f,%*c,%*f,%*f,%d,%*s",  &date);
  
  //printf("%s RMC Date:%d\n",line,date);
  if (matches != 1)
  {
    printf("RMC Handler parsing error\n");
    return;
  } 

  pthread_mutex_lock(&(serial_buf->gps_mutex));
  (serial_buf->gps_buf).date=date;
  pthread_mutex_unlock(&(serial_buf->gps_mutex));
}

