/*
 * aa241x_fw_aux.h
 *
 *  Created on: Feb 9, 2015
 *      Author: adrienp
 */

#ifndef AA241X_FW_AUX_H_
#define AA241X_FW_AUX_H_

// Tait-Bryan Euler angles in radian
// TODO: figure out range on these... (-pi to pi or something different)
float roll;
float pitch;
float yaw;

// angular rates, Tait-Bryan NED [rad/s]
float roll_rate;
float pitch_rate;
float yaw_rate;

// body velocities [m/s]
float speed_body_u;
float speed_body_v;
float speed_body_w;

// body accelerations [m/s^2]






// user config parameters structure
struct aa_params aa_parameters;		// struct containing all of the user editable parameters (via ground station)






#endif /* AA241X_FW_AUX_H_ */
