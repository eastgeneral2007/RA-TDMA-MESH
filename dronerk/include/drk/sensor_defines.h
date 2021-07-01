/**
 * sensor_defines.h
 */

#ifndef _SENSOR_DEFINES_H_
#define _SENSOR_DEFINES_H_

#define VALID					(1)
#define INVALID					(-1)

/* Other useful constants */
#define SENSOR_LINE_LENGTH		(160)
#define STACKSIZE 				(512000)

#define SERIAL_STATE_FILE "/serial.shared"
#define DEVICE	"/dev/ttyUSB0"	// For use on the drone

#endif // _SENSOR_DEFINES_H_
