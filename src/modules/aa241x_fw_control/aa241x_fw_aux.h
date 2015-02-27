/*
 * @file aa241x_fw_aux.h
 *
 * Auxiliary header file containing all the variables accessible for
 * the control law in the aa241x_fw_control.cpp file.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_AUX_H_
#define AA241X_FW_AUX_H_

#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include "aa241x_fw_control_params.h"
#include "../aa241x_mission/aa241x_mission_namespace.h"

// set these variables for help in debugging (these will be sent to the ground station)
extern float roll_desired;
extern float pitch_desired;
extern float yaw_desired;
extern float throttle_desired;


// Tait-Bryan Euler angles in radian
// TODO: figure out range on these... (-pi to pi or something different)
extern float roll;
extern float pitch;
extern float yaw;

// angular rates, Tait-Bryan NED [rad/s]
extern float roll_rate;
extern float pitch_rate;
extern float yaw_rate;

// body velocities [m/s]
extern float speed_body_u;
extern float speed_body_v;
extern float speed_body_w;

// body accelerations [m/s^2]
extern float accel_body_x;
extern float accel_body_y;
extern float accel_body_z;

// velocities in the NED frame [m/s]
extern float vel_N;
extern float vel_E;
extern float vel_D;

// local position in NED frame [m] from center of lake lag
extern float position_N;
extern float position_E;
extern float position_D; 		// note: this is down, so altitude above ground is a negative value

// ground course and speed
extern float ground_speed;		// speed relative to ground in [m/s]
extern float ground_course; 	// this is course over ground (direction of velocity relative to North in [rad])

// airspeed [m/s]
extern float air_speed;		// speed relative to air in [m/s] (measured by pitot tube)

// status check
extern bool gps_ok; 			// boolean as to whether or not the gps data coming in is valid

// battery info
extern float battery_voltage;			// current voltage across the battery 	[V]
extern float battery_current;			// current current being consumed		[A]
extern float battery_energy_consumed;	// battery energy consumed since last boot [J] = [VAs]
extern float mission_energy_consumed;	// battery energy consumed since the start of the mission [J]


// RC info
// output for each of the controls ranging from -1..1 except for throttle which ranges from 0..1
extern float roll_servo_out;
extern float pitch_servo_out;
extern float yaw_servo_out;
extern float throttle_servo_out;

// manual control inputs
// input for each of the controls from the remote control, ranging from -1..1 except for throttle, which ranges from 0..1
extern float man_roll_in;
extern float man_pitch_in;
extern float man_yaw_in;
extern float man_throttle_in;

// trim conditions (from remote control)
extern float roll_trim;
extern float pitch_trim;
extern float yaw_trim;

// time information
extern uint64_t timestamp; 	// timestamp of microseconds since boot
extern uint64_t utc_timestamp; // GPS UTC timestamp in microseconds



// user config parameters structure
extern struct aa_params aa_parameters;		// struct containing all of the user editable parameters (via ground station)


struct snapshot_s {
	bool pic_taken;			/**< if true, a picture was successfully taken */
	uint64_t time_us;		/**< timestamp in microseconds since boot at which the picture was taken */
	float center_n;			/**< North coordinate (North distance from center in [m]) of the center of the picture */
	float center_e;			/**< East coordinate (East distance from center in [m]) of the center of the picture */
	float pic_d;			/**< diameter in [m] of the picture taken */
	int num_cells;			/**< number of grid cells in view (length of vectors i, j, and state) */
	std::vector<int> i;		/**< list of i grid coordinates in view */
	std::vector<int> j;		/**< list of j grid coordinates in view */
	std::vector<int> state; /**< state of each grid coordinate in view (-1 = water, 0 = nothing, 1 = on fire) */
};

snapshot_s take_picture();



#endif /* AA241X_FW_AUX_H_ */
