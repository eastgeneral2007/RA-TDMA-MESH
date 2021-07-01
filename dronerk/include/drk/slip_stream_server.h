/**
 * slip_stream_server.h
 * Created by David Chow on 7/11/13.
 * Modified by Fabian Okeke on 07/15/13.
 * Mod by Luis Pinto 29/sept/2013	
 **/

#ifndef _SLIP_STREAM_SERVER_H_
#define _SLIP_STREAM_SERVER_H_

//#include <termios.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <unistd.h>
//#include <string.h>
#include <stdint.h>
//#include <fcntl.h>
//#include <sys/signal.h>
//#include <sys/types.h>
//#include <errno.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>

//#include <expansion_sensor_api.h>
#include "hdr.h"
//#include <drk.h>

/******************************
 * Function Declarations
 *****************************/
/**
 * Open serial port
 * Define port_fd
 */
int drk_open_serial_port(uint8_t usbport);

// Check for ASCII or SLIP data
// Receives the message and saves it to struct (ASCII) or global buffer (SLIP)
// Global buffer used is (serial_buf->radio_buf).msg_buf
// this function is called by a thread in drk.c (_drk_radio_thread)
int serial_thread(uint8_t usbport) ;

// Wrapper function for receiving data 
// Uses mutex and conditional variables
void _slip_rx_handler(int fd);

/* checks mutex locks and unlocks for errors 
 * prints message if mutex function calls fail
 */ 
/* Receives radio message */
// Returns: Size of message received (max 8bits = 1B)
// call this using mutex for radio_buf.
uint8_t slip_rx(int fd);

//get a threadsafe copy of the radio_buf
// slip_server_cycle has to be running in a thread in drk.c! 
uint8_t drk_modem_rx(uint8_t * buf, HDR_S * hdr);

// Function transmits message buf with length len through the radio
// Returns: 1 if success, in transferring message
//          0 if fail, in transferring message
// using mutex_tx to lock someone else to call this function simultaneously
//TODO: uint8_t
int8_t drk_modem_tx(uint8_t * buf, uint8_t len, HDR_S * hdr);

// Sends the message "new_buf" with size "size"
// Listens 5 times for ACK (acknowledgement from receiver)
// If it gets NACK, retries "retries" number of times
// Returns: 1 if successful in transferring message to the FIREFLY (not to the air)
//          0 if unsuccessful in transferring message to the FIREFLY (maybe serial bus was overloaded)
// send a slip msg and reads a ack from the ack_list
//1 success
//0 success
int8_t _slip_tx_ack(uint8_t *new_buf, uint8_t size, uint8_t retries);

//this function just reads the ack buffer list.
int8_t slip_read_ack(uint16_t seq_num);

// 
/* Transmits radio messages */
// Message format:
// START-SIZE-MESSAGE-(ESC)-MESSAGE-CHECKSUM-END
// ESC could be there or not
int8_t _slip_tx(int fd, uint8_t * tx_buf, uint8_t size);

HDR_S* conf_hdr(uint8_t version, uint16_t src_mac, uint8_t ttl);

uint8_t set_dst_tdma(HDR_S * hdr, uint16_t dst_mac, uint16_t tdma_slot);

// pack up the header into package
uint8_t pack(uint8_t * buf, uint8_t len, HDR_S * hdr);

// unpackage to remove the header
uint8_t unpack(HDR_S * hdr, uint8_t p_len, uint8_t * buf);

#endif // _SLIP_STREAM_SERVER_H_
