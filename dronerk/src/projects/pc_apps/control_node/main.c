#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <slipstream.h>
#include <drk_udp.h>

#define NONBLOCKING  1
#define BLOCKING     0


int main (int argc, char *argv[])
{
  char buffer[128];
  int v,cnt,i;
  char buf[128];

  printf("Drone Commands Usage:\n");
  printf("Addr - Change Address Mode\n");
  printf("t - Takeoff\n");
  printf("l - Land\n");
  printf("w - Move Forward\n");
  printf("s - Move Backward\n");
  printf("a - Move Left\n");
  printf("d - Move Right\n");
  printf("q - Spin Left\n");
  printf("e - Spin Right\n"); 
  printf("1 - Move Up\n");
  printf("2 - Move Down\n");

  v=slipstream_open("localhost",UDP_SERVER_PORT,BLOCKING);
 
 
  
  while (1) {
    printf(">");
    fflush(stdout);
    
    v=scanf("%s",buf);  

//    printf("SCANF:%c\n",buf[0]);
    if(!strncmp(buf,"quit",4)) {
        exit(0);
    }
    if(!strncmp(buf,"exit",4)) {
        exit(0);
    }

    bzero(buffer,128);    
    if(v == 1) {
        switch(buf[0]) {
            case 't':
                sprintf(buffer,TAKEOFF_PACKET);
            break;
            case 'l':
                sprintf(buffer,LAND_PACKET);
            break;
            case 'w':
                sprintf(buffer,MOVE_FORWARD_PACKET);
            break;
            case 's':
                sprintf(buffer,MOVE_BACKWARD_PACKET);
            break;
            case 'a':
                sprintf(buffer,MOVE_LEFT_PACKET);
            break;
            case 'd':
                sprintf(buffer,MOVE_RIGHT_PACKET);
            break;
            case 'q':
                sprintf(buffer,SPIN_LEFT_PACKET);
            break;
            case 'e':
                sprintf(buffer,SPIN_RIGHT_PACKET);
            break;
            case '1':
                sprintf(buffer,MOVE_UP_PACKET);
            break;
            case '2':
                sprintf(buffer,MOVE_DOWN_PACKET);
            break;
            default:
                printf("Invalid Command\n");
        }
        printf("Sending: %s\n",buffer);
        v=slipstream_send(buffer,strlen(buffer));
        if (v == 0) printf( "Error sending\n" );
    }
    
    v=slipstream_receive( buffer );
    /*if (v > 0) {
      printf ("Got: ");
      for (i = 0; i < v; i++)
        printf ("%c", buffer[i]);
      printf ("\n");
    }*/

    usleep (10000);
  }



}

