/**
 * localization.c
 * Anton Dukeman, Oliver Shih
 *
 * RSSI localization functions
 */

#include "drk/rssi_localization.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "drk/drk.h"
#include "drk/expansion_sensor_api.h"

/* Define Macros */
#define MAX_BUF_SIZE	128
#define MAX_BEACON_NUM	10
#define MIN_BNUM	3
#define WINDOW_SIZE	3
#define LIVES		15

/* private function prototypes */
uint8_t load_coords();
uint8_t cla();
double r2d(uint8_t);
uint8_t check_beacons();
void* rssi_thread(void* args);

// buffer for messages 
uint8_t msg_buf[MAX_BUF_SIZE];
uint8_t rec_buf[MAX_BUF_SIZE];
uint8_t win_pt[MAX_BEACON_NUM+1]={0};
int8_t avail[MAX_BEACON_NUM+1]={-1};
double COORDS[MAX_BEACON_NUM+1][2];
uint8_t RSSIS[MAX_BEACON_NUM+1][2][WINDOW_SIZE]; //[beacon][0] for out, [beacon][1] for in

double my_lat=0, my_long=0;
pthread_mutex_t rssi_mutex;


void drk_get_rssi_location(double* x, double* y)
{
  pthread_mutex_lock(&rssi_mutex);
  *x = my_lat;
  *y = my_long;
  pthread_mutex_unlock(&rssi_mutex);
}

void drk_init_rssi()
{
  pthread_t rssi_tid;
  pthread_attr_t rssi_attr;
  pthread_attr_init(&rssi_attr);

	if(load_coords() <= 0)
    puts("drk_init_rssi() failed due to bad beacon coord load");
  else if (pthread_create(&rssi_tid, &rssi_attr, rssi_thread, NULL) != 0)
    puts("drk_init_rssi() error spawning 'rssi_thread' thread");
  else
    puts("drk_init_rssi() successfully spawned RSSI thread");
}

void* rssi_thread(void* args)
{
  (void)(args); // suppress warnings

	uint8_t bnum=0;
	uint8_t rssi_in=0;
	uint8_t rssi_out=0;
	uint8_t i=0;

  while(1)
  {
    memset(rec_buf, '\0', MAX_BUF_SIZE);
    if((serial_buf->radio_buf).ready_to_read == 1)
    {
      //copy global buffer
      memcpy(rec_buf, (serial_buf->radio_buf).msg_buf+1, (serial_buf->radio_buf).msg_buf_len);
      (serial_buf->radio_buf).ready_to_read = 0; //i just read what you have to say
      uint8_t len = (serial_buf->radio_buf).msg_buf_len;
      if(len != 3 || rec_buf[0] <= 0 || rec_buf[0] > MAX_BEACON_NUM)
				continue;

      //parsing & update RSSI table
      bnum = rec_buf[0];
      rssi_out = rec_buf[1]; //use this for CLA
      rssi_in = rec_buf[2];

      //insert with window pointer
      RSSIS[bnum][0][win_pt[bnum]] = rssi_out;
      RSSIS[bnum][1][win_pt[bnum]] = rssi_in;
      win_pt[bnum]++;
      if(win_pt[bnum] >= WINDOW_SIZE)
        win_pt[bnum] = 0;
      if(avail[bnum] >= 0)
        avail[bnum] = LIVES;
      for(i = 1; i <= MAX_BEACON_NUM; i++)
      {
        if(i==bnum)
          continue;
				if (avail[i] > 0)
					avail[i]--;
			}
			if (check_beacons() >= MIN_BNUM && cla() == 1) // update coords
      {
        printf("Updated current position: [%lf][%lf]\n", my_lat, my_long);
      }
    }
		usleep(100);
	}
}

/* Simple conversion from  rssi to distance */ 
double r2d(uint8_t rssi)
{
  double dis=0;
  dis = 50 - 2 * ((double)rssi - 23);
  
  if(dis <= 5)
    dis = 5 - ((double)rssi - 45) / 4;
  
  if(dis <= 0)
    dis = 0.2;

  return dis;
}

/* Load beacon coordinates */
uint8_t load_coords()
{
  double tlat=0;
  double tlong=0;
  uint32_t bnum=0;
  uint8_t count=0;
  FILE* fname = fopen("BEACON_COORDINATES.txt", "r");
  if(fname == NULL)
    return 0;

  while(fscanf(fname, "%u %lf %lf", &bnum, &tlat, &tlong) > 0)
  {
    printf("READ #%u: (%lf, %lf)\r\n", bnum, tlat, tlong);
    if(bnum >= 1 && bnum <= MAX_BEACON_NUM)
    {
      COORDS[bnum][0] = tlat;
      COORDS[bnum][1] = tlong;	
      count++;
    }
  }
  
  fclose(fname);
  return count;
}

/* Centroid localization Algorithm */
uint8_t cla()
{
  uint8_t i=0, j=0;
  double lat_sum=0, long_sum=0, temp_sum=0;
  double temp=0;
  double avg_rssi=0;
  for(i = 1; i <= MAX_BEACON_NUM; i++)
  {
    if (avail[i] <= 0)
      continue;
    else
    {
      // Use incoming RSSI only
      avg_rssi = 0;
      for (j = 0; j < WINDOW_SIZE; j++)
        avg_rssi += RSSIS[i][0][j] / WINDOW_SIZE;
      temp = 1 / r2d(avg_rssi);
      temp_sum += temp;
      lat_sum += temp*(double)COORDS[i][0];
      long_sum += temp*(double)COORDS[i][1];
    }
  }

  // Calulate current coords
  pthread_mutex_lock(&rssi_mutex);
  my_lat = lat_sum/temp_sum;
  my_long = long_sum/temp_sum;
  pthread_mutex_unlock(&rssi_mutex);

  return 1;
}

/* Check for the number of available beacons and clean the dead*/
uint8_t check_beacons()
{
  uint8_t i = 0, j = 0;
  uint8_t temp = 0;
  uint8_t count = 0;
  for(i = 1; i <= MAX_BEACON_NUM; i++)
  {
    temp = 0;

    // If dead, clean up data
    if(avail[i] == 0)
    {
      for(j = 0; j < WINDOW_SIZE; j++)
      {
        RSSIS[i][0][j] = 0;
        RSSIS[i][1][j] = 0;
      }
      win_pt[i] = 0;
      avail[i] = -1;
      continue;
    }
    else if(avail[i] == -1)// see if enough data
    {
      for (j = 0; j < WINDOW_SIZE; j++)
      {
        if(RSSIS[i][0][j] > 0)
          ++temp;
        if(temp >= WINDOW_SIZE)
        {
          ++count;
          avail[i] = LIVES;
        }	
      }
    }
    else if (avail[i])
    {
      ++count;
    }	
  }
  return count;
}
