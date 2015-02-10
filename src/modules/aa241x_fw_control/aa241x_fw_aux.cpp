/*
 * aa241x_fw_aux.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: Adrien
 */
#include "aa241x_fw_aux.h"

// set these variables for help in debugging (these will be sent to the ground station)
float roll_desired = 0.0f;
float pitch_desired = 0.0f;
float yaw_desired = 0.0f;
float throttle_desired = 0.0f;


// Tait-Bryan Euler angles in radian
// TODO: figure out range on these... (-pi to pi or something different)
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// angular rates, Tait-Bryan NED [rad/s]
float roll_rate = 0.0f;
float pitch_rate = 0.0f;
float yaw_rate = 0.0f;

// body velocities [m/s]
float speed_body_u = 0.0f;
float speed_body_v = 0.0f;
float speed_body_w = 0.0f;

// body accelerations [m/s^2]
float accel_body_x = 0.0f;
float accel_body_y = 0.0f;
float accel_body_z = 0.0f;

// velocities in the NED frame [m/s]
float vel_N = 0.0f;
float vel_E = 0.0f;
float vel_D = 0.0f;

// local position in NED frame [m] from center of lake lag
float position_N = 0.0f;
float position_E = 0.0f;
float position_D = 0.0f; 		// note: this is down, so altitude above ground is a negative value

// ground course and speed
float ground_speed = 0.0f;		// speed relative to ground in [m/s]
float ground_course = 0.0f; 	// this is course over ground (direction of velocity relative to North in [rad])

// airspeed [m/s]
float air_speed = 0.0f;		// speed relative to air in [m/s] (measured by pitot tube)

// status check
bool gps_ok = 0.0f; 			// boolean as to whether or not the gps data coming in is valid

// battery info
float battery_voltage = 0.0f;			// current voltage across the battery 	[V]
float battery_current = 0.0f;			// current current being consumed		[A]
float battery_energy_consumed = 0.0f;	// battery energy consumed since last boot [J] = [VAs]
float mission_energy_consumed = 0.0f;	// battery energy consumed since the start of the mission [J]


// RC info
// output for each of the controls ranging from -1..1 except for throttle which ranges from 0..1
float roll_servo_out = 0.0f;
float pitch_servo_out = 0.0f;
float yaw_servo_out = 0.0f;
float throttle_servo_out = 0.0f;

// manual control inputs
// input for each of the controls from the remote control, ranging from -1..1 except for throttle, which ranges from 0..1
float man_roll_in = 0.0f;
float man_pitch_in = 0.0f;
float man_yaw_in = 0.0f;
float man_throttle_in = 0.0f;

// trim conditions (from remote control)
float roll_trim = 0.0f;
float pitch_trim = 0.0f;
float yaw_trim = 0.0f;

// time information
uint64_t timestamp = 0; 	// timestamp of microseconds since boot
uint64_t utc_timestamp = 0; // GPS UTC timestamp in microseconds



// user config parameters structure
struct aa_params aa_parameters = {};		// struct containing all of the user editable parameters (via ground station)


