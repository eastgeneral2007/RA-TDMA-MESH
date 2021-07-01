/**
 * drk_udp.c
 */

#include "drk/drk_udp.h"

#include <stdio.h>
#include <string.h>

#include "drk/autonomous.h"
#include "drk/flight_control_api.h"

int drk_check_control_packet(uint8_t *buf, int size)
{
  int fly_time = 200;
  double spin_rate = 0.7, angle = 0.2, gaz_rate = 0.5;
  
  if(size == 0)
      return -1;
  if(!strncmp((const char*)buf, TAKEOFF_PACKET,TAKEOFF_PACKET_LEN))
  {
      printf("Received Takeoff Packet\n");
      drk_takeoff();
      return 0;
  }
  else if(!strncmp((const char*)buf, LAND_PACKET,LAND_PACKET_LEN))
  {
      printf("Received Land Packet\n");
      drk_land();
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_FORWARD_PACKET,MOVE_FORWARD_PACKET_LEN))
  {
      printf("Received Move Forward Packet\n");
      drk_move_forward(angle, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_BACKWARD_PACKET,MOVE_BACKWARD_PACKET_LEN))
  {
      printf("Received Move Backward Packet\n");
      drk_move_backward(angle, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_LEFT_PACKET,MOVE_LEFT_PACKET_LEN))
  {
      printf("Received Move Left Packet\n");
      drk_move_left(angle, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_RIGHT_PACKET,MOVE_RIGHT_PACKET_LEN))
  {
      printf("Received Move Right Packet\n");
      drk_move_right(angle, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_UP_PACKET,MOVE_UP_PACKET_LEN))
  {
      printf("Received Move Up Packet\n");
      drk_move_up(gaz_rate, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, MOVE_DOWN_PACKET, MOVE_DOWN_PACKET_LEN))
  {
      printf("Received Move Down Packet\n");
      drk_move_down(gaz_rate, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, SPIN_LEFT_PACKET,SPIN_LEFT_PACKET_LEN))
  {
      printf("Received Spin Left Packet\n");
      drk_spin_left(spin_rate, fly_time);
      return 0;
  }
  else if(!strncmp((const char*)buf, SPIN_RIGHT_PACKET,SPIN_RIGHT_PACKET_LEN))
  {
      printf("Received Spin Right Packet\n");
      drk_spin_right(spin_rate, fly_time);
      return 0;
  }
  return -1;
}
