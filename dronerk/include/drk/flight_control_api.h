/**
 * flight_control_api.h
 */

#ifndef _FLIGHT_CONTROL_API_H_
#define _FLIGHT_CONTROL_API_H_


/** Send a takeoff command, then block for five seconds to wait for completion **/
void drk_takeoff(void);
/** Send a land command, then read sensor data until landed  **/
void drk_land(void);
/** Trigger an emergency shutoff (kill the motors)  **/
void drk_emergency(void);
/** Removes the drone from emergency state  **/
int drk_remove_emergency(void);
/** Puts the drone in hover mode, using the camera to stabilize **/
void drk_lockdown_hover(int time);
/** Puts the drone in hover mode, using the camera to stabilize **/
void drk_hover(int time);
/** Combined movement **/
void drk_translate(float pitch, float roll, float yaw, float gaz, int time_ms);
/**Rotate to the left **/
void drk_spin_left(float rate, int time);
/**Rotate to the right **/
void drk_spin_right(float rate, int time);
/**Move forward **/
void drk_move_forward(float rate, int time);
/** Move backward **/
void drk_move_backward(float rate, int time);
/** Move right **/
void drk_move_right(float rate, int time);
/** Move left **/
void drk_move_left(float rate, int time);
/**Fly upward **/
void drk_move_up(float rate, int time);
/** Fly downward **/
void drk_move_down(float rate, int time);





#endif // _FLIGHT_CONTROL_API_H_
