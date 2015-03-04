/*
 * aa241x_fw_aux.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: Adrien
 */

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/aa241x_picture_request.h>
#include <uORB/topics/aa241x_water_drop_request.h>
#include <drivers/drv_hrt.h>

#include "aa241x_fw_aux.h"

// set these variables for help in debugging (these will be sent to the ground station)
float roll_desired = 0.0f;
float pitch_desired = 0.0f;
float yaw_desired = 0.0f;
float throttle_desired = 0.0f;


// Tait-Bryan Euler angles in radian
float roll = 0.0f;		/**< range from -pi .. pi */
float pitch = 0.0f;		/**< range from -pi/2 .. pi/2 */
float yaw = 0.0f;		/**< range from -pi .. pi */

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
bool gps_ok = false; 				// boolean as to whether or not the gps data coming in is valid
bool local_pos_ne_valid = false;	// boolean as to whether or not the horizontal local position values are valid
bool local_pos_d_valid = false;		// boolean as to whether or not the vertical local position value is valid


// battery info
float battery_voltage = 0.0f;			// current voltage across the battery 	[V]
float battery_current = 0.0f;			// current current being consumed		[A]

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
uint64_t timestamp = 0; 				// timestamp of microseconds since boot (set at the beginning of loop)
uint64_t utc_timestamp = 0; 			// GPS UTC timestamp in microseconds
uint64_t previous_loop_timestamp = 0;	// timestamp of start of previous loop

// picture result
bool new_pic = false;
picture_result_s pic_result = {};

// data from low priority thread
struct aa241x_low_data_s low_data = {};
struct aa241x_high_data_s high_data = {};


// user config parameters structure
struct aa_params aa_parameters = {};		// struct containing all of the user editable parameters (via ground station)

struct mis_params mission_parameters = {};


orb_advert_t	_attitude_sp_pub = -1;
orb_advert_t	_picture_request_pub = -1;
orb_advert_t	_water_drop_request_pub = -1;


/* functions */

void take_picture()
{
	picture_request_s pic_request;
	pic_request.time_us = hrt_absolute_time();
	pic_request.pos_N = 30.0f; // position_N;	// DEBUG
	pic_request.pos_E = 10.0f; // position_E;	// DEBUG
	pic_request.pos_D = 100.0f; // position_D;	// DEBUG

	/* publish the picture request */
	if (_picture_request_pub > 0) {
		orb_publish(ORB_ID(aa241x_picture_request), _picture_request_pub, &pic_request);
	} else {
		_picture_request_pub = orb_advertise(ORB_ID(aa241x_picture_request), &pic_request);
	}
}


void drop_water()
{
	water_drop_request_s water_drop_request;
	water_drop_request.time_us = hrt_absolute_time();
	water_drop_request.pos_N = position_N;
	water_drop_request.pos_E = position_E;
	water_drop_request.pos_D = position_D;

	/* publish the picture request */
	if (_picture_request_pub > 0) {
		orb_publish(ORB_ID(aa241x_water_drop_request), _water_drop_request_pub, &water_drop_request);
	} else {
		_water_drop_request_pub = orb_advertise(ORB_ID(aa241x_water_drop_request), &water_drop_request);
	}
}

/*
void publish_desired_attitude(const float &roll_d, const float &pitch_d, const float &yaw_d, const float &throttle_d)
{
	vehicle_attitude_setpoint_s att_sp;
	att_sp.roll_body = roll_d;
	att_sp.pitch_body = pitch_d;
	att_sp.yaw_body = yaw_d;
	att_sp.thrust = throttle_d;

	// publish the desired attitude
	if (_attitude_sp_pub > 0) {
		orb_publish(ORB_ID(aa241x_mission_status), _attitude_sp_pub, &att_sp);
	} else {
		_attitude_sp_pub = orb_advertise(ORB_ID(aa241x_mission_status), &att_sp);
	}
}
*/


/*
picture_result_s take_picture()
{

	if (aa241x_mission::g_aa241x_mission) {
		return aa241x_mission::g_aa241x_mission->take_picture();
	} else {
		picture_result_s pic_result;
		pic_result.pic_taken = false;
		return pic_result;
	}
}

water_drop_result_s drop_water()
{

	if (aa241x_mission::g_aa241x_mission) {
		return aa241x_mission::g_aa241x_mission->drop_water();
	} else {
		water_drop_result_s water_drop;
		water_drop.success = false;
		return water_drop;
	}
} */


