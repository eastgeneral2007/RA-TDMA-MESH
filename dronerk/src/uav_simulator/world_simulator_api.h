#ifndef WORLD_SIMULATOR_API_H_
#define WORLD_SIMULATOR_API_H_



/* drone payloads to/from world_simulator */
#define MAX_SENSOR_SIZE			(30)
#define MAX_PKT_SIZE			(1500) /* udp packets not bigger than this */
#define MAX_80211_SIZE			(1500)


/** UDP ports to comunicate to/from **/
/* Drones send data FROM these ports */
/* we detect intented-destination 127.0.0.X, where X is #uav-destination. 
 * we detect source port to check packettype/source  */
/* SIM sends isensor data to uav . uavs should listen to this: */ 
#define PORT_ISENSOR			61000U 

/* SIM sends esensor data to uav . uavs should listen to this: */
#define PORT_ESENSOR			62000U 
 
/* SIM sends data to drone TO this port.uavs should listen to this: */
#define PORT_UAV_RX80211		60100U


/* Simulator expects to receive stuff FROM these ports: */
#define UDP_PORT_BASE_SRC_80211	40000U /* drone Bport from where 80211pkt data comes  */
#define UDP_PORT_BASE_SRC_ACT	41000U /* drone Bport from where actuation data comes  */

/* Simulator receives data sent TO this port */
/* SIM is listening to EVERYHING at this port */
/* Simulator listens to this only exact port. */
#define SIM_RX_ALL				50000U 

/* simulator sends data FROM this port + ID */
#define PORT_BASE_SIM_TX80211	51000U

#endif

