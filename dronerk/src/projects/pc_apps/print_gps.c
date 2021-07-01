#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <math.h>

#define BUF_LEN 160
#define BAUDRATE B115200
#define DATABITS CS8
#define STOPBITS CSTOPB
#define PARITYON 0
#define PARITY 0
#define USB_PORT "/dev/ttyUSB0"


int port_fd = -1;
char lineBuffer[BUF_LEN];
char lastGPS[BUF_LEN];
struct termios serial_tio;
FILE * file;
void read_packets();
double nmea_to_decimal(double coordinate);

void main() {

	
	bzero(&serial_tio, sizeof(serial_tio));

	port_fd = open(USB_PORT, O_RDWR | O_NOCTTY);
	if (port_fd < 0) {
		perror("Failed to open serial port");
		return;
	}

	serial_tio.c_cflag = BAUDRATE | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL| CREAD;
	serial_tio.c_iflag = IGNPAR;
	serial_tio.c_lflag = ICANON;
	serial_tio.c_cc[VMIN]= 1;
	serial_tio.c_cc[VTIME] = 0;
	tcflush(port_fd, TCIFLUSH);
	tcsetattr (port_fd, TCSANOW, &serial_tio);

	file = fopen("PacketLog.txt","w+");
	if(file < 0) {
	  perror("Failed to open file for writing");
	  return;
	}
	read_packets();
}



void read_packets() {
	int bytes = 0;
	double latitude, longitude, altitude, UTC;
	int quality, num_sats;
	int matches = 0;
	char N_or_S, E_or_W;

	while(1){
		bytes = read(port_fd,lineBuffer, BUF_LEN-1);
		if(bytes > 0){
			lineBuffer[bytes]= '\0';
			if(strncmp(lineBuffer, "$GPGGA",6) == 0) {
				
				matches = sscanf(lineBuffer, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%*f,%lf,%*s",
						&UTC, &latitude, &N_or_S, &longitude, &E_or_W, &quality, &num_sats, &altitude);
				if(matches != 8) {
			       printf("Not enough matches\n");  
				   continue;
				}

				latitude = nmea_to_decimal(latitude);
				longitude = nmea_to_decimal(longitude);
				
				if(N_or_S == 'S') latitude *= -1;
				if(E_or_W == 'W') longitude *= -1;

		    	printf("Longitude:%lf Latitude:%lf\n",longitude,latitude);
			}
		}
	}
}

double nmea_to_decimal(double coordinate) {
	double end = fmod(coordinate, (double)100.0);
	return (double)(((coordinate - end) / 100.0) + (end/60.0));
}
