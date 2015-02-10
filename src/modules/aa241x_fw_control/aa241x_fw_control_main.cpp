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
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
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
#include <uORB/topics/vehicle_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <platforms/px4_defines.h>

#include "aa241x_fw_control_params.h"
#include "aa241x_fw_control.h"
#include "aa241x_fw_aux.h"

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
	int		_vehicle_status_sub;	/**< vehicle status subscription */

	// the data that will be published from this controller
	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure


	// structures of data that comes in from the uORB subscriptions
	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct accel_report					_accel;			/**< body frame accelerations */
	struct vehicle_rates_setpoint_s		_rates_sp;		/**< attitude rates setpoint TODO: potentially remove*/
	struct manual_control_setpoint_s	_manual;		/**< r/c channel data */
	struct airspeed_s					_airspeed;		/**< airspeed */
	struct vehicle_control_mode_s		_vcontrol_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct vehicle_global_position_s	_global_pos;	/**< global position */
	struct vehicle_status_s				_vehicle_status;/**< vehicle status */

	// some flags
	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;					/**< if set to true, print debug output */

	struct {
		// TODO: add custom parameter values here (will all be floats)


		float trim_roll;
		float trim_pitch;
		float trim_yaw;


	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		// TODO: add custom parameter handles here

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;

	}		_parameter_handles;		/**< handles for interesting parameters */


	// TODO: these structs should maybe belong to a header file...
	struct aa_params			_aa_parameters;
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
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Set all the aux variables needed for control law.
	 */
	void 		set_aux_values();

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

/* oddly, ERROR is not defined for c++ */
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
	_vehicle_status_sub(-1),

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
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_global_pos = {};
	_vehicle_status = {};

	// initialize the global remote parameters
	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");

	// initialize the aa241x control parameters
	aa_parameters_init(&_aa_parameter_handles);


	// fetch initial remote parameters
	parameters_update();

	// fetch initial aa241x control parameters
	aa_parameters_update(&_aa_parameter_handles, &_aa_parameters);
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

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

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
	aa_parameters_update(&_aa_parameter_handles, &_aa_parameters);

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
FixedwingControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}



void
FixedwingControl::set_aux_values()
{

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

	// set the euler angles and rates
	roll = _att.roll;
	pitch = _att.pitch;
	yaw = _att.yaw;

	roll_rate = _att.rollspeed;
	pitch_rate = _att.pitchspeed;
	yaw_rate = _att.yawspeed;


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
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

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
	vehicle_status_poll();

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

		perf_begin(_loop_perf);

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
			vehicle_status_poll();



			if (_vcontrol_mode.flag_control_auto_enabled) {

				// set all the variables needed for the control law
				set_aux_values();

				// TODO: add function for students to fill out

				// TODO: potentially add stabilize and other modes back in....
				flight_control();



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
