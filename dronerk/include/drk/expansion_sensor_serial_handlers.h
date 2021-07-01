/**
 * expansion_sensor_serial_handlers.h
 */

#ifndef _EXPANSION_SENSOR_SERIAL_HANDLERS_H_
#define _EXPANSION_SENSOR_SERIAL_HANDLERS_H_

#include <stdint.h>

#include "sensor_defines.h"

int _drk_expansion_serial_init( uint8_t usb_port_num );

/* Process one line of serial data */
void _drk_parse_line(char line[SENSOR_LINE_LENGTH], int bytesRead);

/* Compute the NMEA checksum (XOR of chars between '$' and '*')*/
int _drk_1ate_packet (char packet[SENSOR_LINE_LENGTH]);

/*******************************************************************************
                SERIAL PACKET HANDLERS
*******************************************************************************/
/* Handle the infrared packet */
void _drk_infrared_handler(char line[SENSOR_LINE_LENGTH]);

/* Handle the grideye packets */
void _drk_grideye_handler(char line[SENSOR_LINE_LENGTH]);

/* Handle the $GPGGA data packet. Contains most of the essential data */
void _drk_gga_handler(char line[SENSOR_LINE_LENGTH]);

/* Handle the $GPVTG data packet. Speed & track*/
void _drk_vtg_handler(char line[SENSOR_LINE_LENGTH]);

/* Handle the $GPRMC data packet. Current date*/
void _drk_rmc_handler(char line[SENSOR_LINE_LENGTH]);

#endif // _EXPANSION_SENSOR_SERIAL_HANDLERS_H_
