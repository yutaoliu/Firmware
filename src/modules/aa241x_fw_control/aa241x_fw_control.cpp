/*
 * aa241x_fw_control.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: adrienp
 */

// include header file
#include "aa241x_fw_control.h"
#include "aa241x_fw_aux.h"


void flight_control() {

	// TODO: write all of your flight control here...






	// ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
	// outputs should be set to values between -1..1 (except throttle is 0..1)
	// where zero is no actuation, and -1,1 are full throw in either the + or - directions
	roll_servo_out = man_roll_in;		// as an example, just passing through manual control
	pitch_servo_out = man_pitch_in;
	yaw_servo_out = man_yaw_in;
	throttle_servo_out = man_throttle_in;

	/* to take a picture */
	take_picture();

}
