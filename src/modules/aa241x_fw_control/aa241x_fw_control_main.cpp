/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aa241x_fw_control_main.cpp
 *
 * Wrapper for the flight controller to be designed for
 * Stanford's AA241X course.  Fixed wing controller based on
 * the fw_att_control file in the original PX4 source code.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <platforms/px4_defines.h>


#include "aa241x_fw_aux.h"
#include "aa241x_fw_control.h"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_fw_control_main(int argc, char *argv[]);

class FixedwingControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingControl();

	/**
	 * Destructor, also kills the main task.
	 */
	~FixedwingControl();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool	_task_should_exit;		/**< if true, attitude control task should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_loop_counter;			/**< count of iterations of main loop */

	// handles to the subscriptions needed
	int		_att_sub;				/**< vehicle attitude subscription */
	int		_accel_sub;				/**< accelerometer subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_sensor_combined_sub;	/**< sensor data subscription */
	int		_battery_status_sub;	/**< battery status subscription */

	// the data that will be published from this controller
	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure


	// structures of data that comes in from the uORB subscriptions
	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct accel_report					_accel;				/**< body frame accelerations */
	struct vehicle_rates_setpoint_s		_rates_sp;			/**< attitude rates setpoint TODO: potentially remove */
	struct vehicle_attitude_setpoint_s	_att_sp;			/**< attitude setpoint (for debugging help */
	struct manual_control_setpoint_s	_manual;			/**< r/c channel data */
	struct airspeed_s					_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator control inputs */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct sensor_combined_s			_sensor_combined;	/**< raw / minimal filtered sensor data (for some accelerations) */
	struct battery_status_s				_battery_status;	/**< battery status */

	// some flags
	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;					/**< if set to true, print debug output */


	// general RC parameters
	struct {
		float trim_roll;
		float trim_pitch;
		float trim_yaw;
	}		_parameters;			/**< local copies of interesting parameters */

	// handles for general RC parameters
	struct {
		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
	}		_parameter_handles;		/**< handles for interesting parameters */


	// handles for custom parameters
	// NOTE: the struct for the parameters themselves can be found in the aa241x_fw_aux file
	struct aa_param_handles		_aa_parameter_handles;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	void		vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for local position updates.
	 */
	void		local_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for combined sensor data updates.
	 */
	void		sensor_combined_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Set all the aux variables needed for control law.
	 */
	void 		set_aux_values();

	/**
	 * Set the actuator output values from the control law.
	 */
	void		set_actuators();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};


namespace att_control
{

// oddly, ERROR is not defined for c++
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingControl	*g_control = nullptr;
}


FixedwingControl::FixedwingControl() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_sensor_combined_sub(-1),
	_battery_status_sub(-1),

/* publications */
	_rate_sp_pub(-1),
	_attitude_sp_pub(-1),
	_actuators_0_pub(-1),

	_rates_sp_id(0),
	_actuators_id(0),

/* states */
	_setpoint_valid(false),
	_debug(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_rates_sp = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_sensor_combined = {};
	_battery_status = {};

	// initialize the global remote parameters
	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");

	// initialize the aa241x control parameters
	aa_parameters_init(&_aa_parameter_handles);


	// fetch initial remote parameters
	parameters_update();

	// fetch initial aa241x control parameters
	aa_parameters_update(&_aa_parameter_handles, &aa_parameters);
}

FixedwingControl::~FixedwingControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	att_control::g_control = nullptr;
}

int
FixedwingControl::parameters_update()
{
	// TODO: add custom paramters for updating here

	// update the remote control parameters
	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));

	// update the aa241x control parameters
	aa_parameters_update(&_aa_parameter_handles, &aa_parameters);

	return OK;
}

void
FixedwingControl::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
FixedwingControl::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
FixedwingControl::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

void
FixedwingControl::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
FixedwingControl::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
FixedwingControl::local_pos_poll()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
FixedwingControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
			_actuators_id = ORB_ID(actuator_controls_0);
		}
	}
}

void
FixedwingControl::sensor_combined_poll()
{
	/* check if there is new sensor combined data */
	bool sensor_combined_updated;
	orb_check(_sensor_combined_sub, &sensor_combined_updated);

	if (sensor_combined_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}


void
FixedwingControl::battery_status_poll()
{
	/* check if there is a new battery status */
	bool battery_status_updated;
	orb_check(_battery_status_sub, &battery_status_updated);

	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}


void
FixedwingControl::set_aux_values()
{

	// set the euler angles and rates
	roll = _att.roll;
	pitch = _att.pitch;
	yaw = _att.yaw;

	// set the angular rates
	roll_rate = _att.rollspeed;
	pitch_rate = _att.pitchspeed;
	yaw_rate = _att.yawspeed;


	// calculate the body velocities
	speed_body_u = 0.0f;		// set them to 0 just in case unable to do calculation
	speed_body_v = 0.0f;
	speed_body_w = 0.0f;
	if(_att.R_valid) 	{
		speed_body_u = PX4_R(_att.R, 0, 0) * _global_pos.vel_n + PX4_R(_att.R, 1, 0) * _global_pos.vel_e + PX4_R(_att.R, 2, 0) * _global_pos.vel_d;
		speed_body_v = PX4_R(_att.R, 0, 1) * _global_pos.vel_n + PX4_R(_att.R, 1, 1) * _global_pos.vel_e + PX4_R(_att.R, 2, 1) * _global_pos.vel_d;
		speed_body_w = PX4_R(_att.R, 0, 2) * _global_pos.vel_n + PX4_R(_att.R, 1, 2) * _global_pos.vel_e + PX4_R(_att.R, 2, 2) * _global_pos.vel_d;
	} else	{
		if (_debug && _loop_counter % 10 == 0) {
			warnx("Did not get a valid R\n");
		}
	}

	// body accelerations [m/s^2]
	accel_body_x = _sensor_combined.accelerometer_m_s2[0];
	accel_body_y = _sensor_combined.accelerometer_m_s2[1];
	accel_body_z = _sensor_combined.accelerometer_m_s2[2];

	// velocities in the NED frame [m/s]
	// TODO: maybe use local position...
	vel_N = _global_pos.vel_n;
	vel_E = _global_pos.vel_e;
	vel_D = _global_pos.vel_d;

	// local position in NED frame [m] from center of lake lag
	position_N = 0.0f;
	position_E = 0.0f;
	position_D = 0.0f;
	if (_local_pos.xy_valid) {		// only copy the data if it is valid
		position_N = _local_pos.x;
		position_E = _local_pos.y;
	}
	if (_local_pos.z_valid) {
		position_D = _local_pos.z;
	}

	// TODO: set the reference point of the local position information to be the center of the lake


	// ground course and speed
	// TODO: maybe use local position....
	ground_speed = sqrtf(_global_pos.vel_n * _global_pos.vel_n + _global_pos.vel_e * _global_pos.vel_e);		// speed relative to ground in [m/s]
	ground_course = _global_pos.yaw; 	// this is course over ground (direction of velocity relative to North in [rad])

	// airspeed [m/s]
	air_speed = _airspeed.true_airspeed_m_s;		// speed relative to air in [m/s] (measured by pitot tube)

	// status check
	//TODO: gps_ok; 			// boolean as to whether or not the gps data coming in is valid

	// battery info
	battery_voltage = _battery_status.voltage_filtered_v;
	battery_current = _battery_status.current_a;
	//TODO: battery_energy_consumed;	// battery energy consumed since last boot [J] = [VAs]
	//TODO: mission_energy_consumed;	// battery energy consumed since the start of the mission [J]


	// manual control inputs
	// input for each of the controls from the remote control, ranging from TODO: figure out range
	man_roll_in = _manual.y;
	man_pitch_in = _manual.x;
	man_yaw_in = _manual.r;
	man_throttle_in = _manual.z;

	// trim conditions (from remote control)
	roll_trim = _parameters.trim_roll;
	pitch_trim = _parameters.trim_pitch;
	yaw_trim = _parameters.trim_yaw;

	// time information
	timestamp = hrt_absolute_time();
	utc_timestamp = _global_pos.time_utc_usec;

}

void
FixedwingControl::set_actuators()
{
	// do some safety checking to ensure that all the values are within the required bounds of -1..1 or 0..1
	// check roll
	if (roll_servo_out > 1) {
		roll_servo_out = 1.0f;
	}
	if (roll_servo_out < -1) {
		roll_servo_out = -1.0f;
	}

	// check pitch
	if (pitch_servo_out > 1) {
		pitch_servo_out = 1.0f;
	}
	if (pitch_servo_out < -1) {
		pitch_servo_out = -1.0f;
	}

	// check yaw
	if (yaw_servo_out > 1) {
		yaw_servo_out = 1.0f;
	}
	if (yaw_servo_out < -1) {
		yaw_servo_out = -1.0f;
	}

	// check throttle
	if (throttle_servo_out > 1) {
		throttle_servo_out = 1.0f;
	}
	if (throttle_servo_out < 0) {
		throttle_servo_out = 0.0f;
	}


	// set the actuators
	_actuators.control[0] = (isfinite(roll_servo_out)) ? roll_servo_out : roll_trim;
	_actuators.control[1] = (isfinite(roll_servo_out)) ? pitch_servo_out : pitch_trim;
	_actuators.control[2] = (isfinite(roll_servo_out)) ? yaw_servo_out : yaw_trim;
	_actuators.control[3] = (isfinite(roll_servo_out)) ? throttle_servo_out : 0.0f;
	_actuators.control[4] = _manual.flaps;
}


void
FixedwingControl::task_main_trampoline(int argc, char *argv[])
{
	att_control::g_control->task_main();
}

void
FixedwingControl::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit attitude control to 50 Hz (with some margin, so 17 ms) */
	orb_set_interval(_att_sub, 17);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	global_pos_poll();	// TODO: might remove this....
	local_pos_poll();
	vehicle_status_poll();
	sensor_combined_poll();
	battery_status_poll();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		_loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);


			// update vehicle information structs as needed
			vehicle_airspeed_poll();
			vehicle_accel_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			local_pos_poll();
			vehicle_status_poll();
			sensor_combined_poll();
			battery_status_poll();


			if (_vcontrol_mode.flag_control_auto_enabled) {

				// set all the variables needed for the control law
				set_aux_values();

				// TODO: add function for students to fill out

				// TODO: potentially add stabilize and other modes back in....
				flight_control();

				// set the user desired servo positions (that were set in the flight control function)
				set_actuators();

				// set the attitude setpoint values
				_att_sp.roll_body = (isfinite(roll_desired)) ? roll_desired : 0.0f;
				_att_sp.pitch_body = (isfinite(pitch_desired)) ? pitch_desired : 0.0f;
				_att_sp.yaw_body = (isfinite(yaw_desired)) ? yaw_desired : 0.0f;
				_att_sp.thrust = (isfinite(throttle_desired)) ? throttle_desired : 0.0f;


			} else { // have manual control of the plane

				/* manual/direct control */
				_actuators.control[0] = _manual.y;
				_actuators.control[1] = -_manual.x;
				_actuators.control[2] = _manual.r;
				_actuators.control[3] = _manual.z;
				_actuators.control[4] = _manual.flaps;

			}

			// TODO: maybe remove these?? (they aren't needed)
			_actuators.control[5] = _manual.aux1;
			_actuators.control[6] = _manual.aux2;
			_actuators.control[7] = _manual.aux3;

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;


			/* publish the actuator controls */
			if (_actuators_0_pub > 0) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
			} else if (_actuators_id) {
				_actuators_0_pub= orb_advertise(_actuators_id, &_actuators);
			}

			/* publish the attitude setpoint (the targeted roll, pitch and yaw angles) */
			if (_attitude_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_pub, &_att_sp);
			} else {
				_attitude_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}

		}

		_loop_counter++;
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
FixedwingControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("fw_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedwingControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int aa241x_fw_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: aa241x_fw_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (att_control::g_control != nullptr)
			errx(1, "already running");

		att_control::g_control = new FixedwingControl;

		if (att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != att_control::g_control->start()) {
			delete att_control::g_control;
			att_control::g_control = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (att_control::g_control == nullptr || !att_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (att_control::g_control == nullptr)
			errx(1, "not running");

		delete att_control::g_control;
		att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
