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
 * @file aa241x_mission_main.cpp
 *
 * Mission logic for Stanford's Spring 2015 AA241x class.  This handles
 * all safety checks, mission checks, runs the mission (fire propagation),
 * and handles all picture and water dropping requests.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */
//#include <nuttx/config.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>

#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>


#include "AA241xMission.h"
#include "missions.h"


/**
 * aa241x_mission app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_mission_main(int argc, char *argv[]);


namespace aa241x_mission
{

AA241xMission	*g_aa241x_mission = nullptr;
}

AA241xMission::AA241xMission() :
	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),
	_mavlink_fd(-1),
	_buzzer(-1),
	_vcontrol_mode_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_local_data_sub(-1),
	_battery_status_sub(-1),
	_mission_status_pub(nullptr),
	_mission_start_time(-1),
	_mission_start_battery(0),
	_in_mission(false),
	_can_start(true),
	_early_termination(false),
	_mission_failed(false),
	_score(0.0f),
	_cross_min(false)
{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_batt_stat = {};

	_parameter_handles.min_alt = param_find("AAMIS_ALT_MIN");
	_parameter_handles.max_alt = param_find("AAMIS_ALT_MAX");
	_parameter_handles.auto_alt = param_find("AAMIS_ALT_AUTO");
	_parameter_handles.duration = param_find("AAMIS_DURATION");
	_parameter_handles.max_radius = param_find("AAMIS_RAD_MAX");
	_parameter_handles.index = param_find("AA_MIS_INDEX");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");
	_parameter_handles.max_discharge = param_find("AAMIS_BATT_MAX");
	_parameter_handles.team_num = param_find("AA_TEAM");

	parameters_update();

}

AA241xMission::~AA241xMission() {
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

	aa241x_mission::g_aa241x_mission = nullptr;
}

int
AA241xMission::parameters_update()
{
	param_get(_parameter_handles.min_alt, &(_parameters.min_alt));
	param_get(_parameter_handles.max_alt, &(_parameters.max_alt));
	param_get(_parameter_handles.auto_alt, &(_parameters.auto_alt));
	param_get(_parameter_handles.duration, &(_parameters.duration));
	param_get(_parameter_handles.max_radius, &(_parameters.max_radius));
	param_get(_parameter_handles.index, &(_parameters.index));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));
	param_get(_parameter_handles.max_discharge, &(_parameters.max_discharge));
	param_get(_parameter_handles.team_num, &(_parameters.team_num));

	// TODO: HANDLE ADDITIONAL PARAMETERS HERE

	return OK;
}


void
AA241xMission::vehicle_control_mode_update()
{
	/* Check if vehicle control mode has changed */
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
AA241xMission::global_pos_update()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
AA241xMission::local_pos_update()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
AA241xMission::vehicle_status_update()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
AA241xMission::local_data_update()
{
	/* check if there is new status information */
	bool local_data_updated;
	orb_check(_local_data_sub, &local_data_updated);

	if (local_data_updated) {
		orb_copy(ORB_ID(aa241x_local_data), _local_data_sub, &_local_data);
	}
}

void
AA241xMission::battery_status_update()
{
	/* check if there is new status information */
	bool battery_status_updated;
	orb_check(_battery_status_sub, &battery_status_updated);

	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_batt_stat);
	}
}


void
AA241xMission::publish_mission_status()
{
	aa241x_mission_status_s mis_stat;
	mis_stat.can_start = _can_start;
	mis_stat.in_mission = _in_mission;
	mis_stat.score = _score;
	mis_stat.mission_index = _parameters.index;

	if (_in_mission) {
		mis_stat.mission_time = (hrt_absolute_time() - _mission_start_time)/(1000000.0f*60.0f);
		mis_stat.battery_used = _batt_stat.discharged_mah - _mission_start_battery;
	} else {
		/*
		// don't necessarily need to set to 0?
		mis_stat.mission_time = 0.0f;
		mis_stat.battery_used = 0.0f;
		*/
	}

	/* publish the mission status */
	if (_mission_status_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_mission_status), _mission_status_pub, &mis_stat);
	} else {
		_mission_status_pub = orb_advertise(ORB_ID(aa241x_mission_status), &mis_stat);
	}
}


void
AA241xMission::initialize_mission()
{

	if (_parameters.index >= NUM_MISSIONS) {
		// TODO: throw a system warning
		_can_start = false;
		mavlink_log_info(_mavlink_fd, "#audio: AA241x invalid mission index");
		return;
	}

	// send message that mission has started
	mavlink_log_info(_mavlink_fd, "#audio: AA241x mission started");

	// trigger the buzzer audio for mission start
	/*
	switch(_parameters.team_num){
	case 1:

		break;
	case 2:

		break;
	case 3:
		ioctl(_buzzer, TONE_SET_ALARM, TONE_TRAINER_BATTLE_TUNE);
		break;
	case 4:

		break;
	default:

		break;
	} */

	_in_mission = true;
	_can_start = false;	// don't allow restarting of a mission
	_mission_start_time = hrt_absolute_time();
	_mission_start_battery = _batt_stat.discharged_mah;

	// TODO: ADD ANY ADDITIONAL INITIALIZATION REQUIRED
}



void
AA241xMission::calculate_score()
{
	
	// TODO: WRITE YOUR SCORE FUNCTION HERE

	_score = 0.0f;
}

void
AA241xMission::task_main_trampoline(int argc, char **argv)
{
	 aa241x_mission::g_aa241x_mission->task_main();
}


void
AA241xMission::task_main()
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/* open connection to mavlink logging */
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* open buzzer */
	_buzzer = open(TONEALARM0_DEVICE_PATH, O_WRONLY);

	/*
	 * do subscriptions
	 */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_local_data_sub = orb_subscribe(ORB_ID(aa241x_local_data));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* rate limit vehicle status updates and local data updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	orb_set_interval(_local_data_sub, 200);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_update();
	global_pos_update();
	local_pos_update();
	vehicle_status_update();
	local_data_update();
	battery_status_update();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2] = {};

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

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

		/* global position updated */
		if (fds[1].revents & POLLIN) {
			global_pos_update();
		}

		/* check all other subscriptions */
		vehicle_control_mode_update();
		local_pos_update();
		vehicle_status_update();
		local_data_update();
		battery_status_update();

		/* check auto start requirements */
		if (_can_start && !_in_mission) {

			if (!_vehicle_status.gps_failure && -_local_data.D_gps >= _parameters.auto_alt && !_vcontrol_mode.flag_control_auto_enabled) {
				// not allowed to start is above auto alt and not in auto mode
				_can_start = false;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission start conditions violated");
			}
		}

		/* check mission start requirements */
		if (_can_start && !_in_mission) {

			if (-_local_data.D_gps >= _parameters.auto_alt && _vcontrol_mode.flag_control_auto_enabled) {
				/* start the mission once have crossed over the minimum altitude */
				initialize_mission();
			}
		}

		/* ensure abiding by mission rules */
		if (_in_mission) {

			/* check to see if we have crossed min alt for first time */
			if (!_cross_min && -_local_data.D_gps >= _parameters.min_alt) {
				_cross_min = true;
			}

			// ---- Check soft terminations first ---- //

			/* check auto requirements */
			if (!_vcontrol_mode.flag_control_auto_enabled) {
				// end mission and set score to 0 if switch to manual mode
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: control mode violation");
			}

			/* check min altitude requirements (with 10m buffer) only if plane has gotten above it already */
			if (_cross_min && -_local_data.D_gps <= (_parameters.min_alt - 10.0f)) {
				// end mission, but let fire propagate for rest of time
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: below min alt");
			}

			/* check battery requirements */
			if ((_batt_stat.discharged_mah - _mission_start_battery) > _parameters.max_discharge) {
				_in_mission = false;
				_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: max battery discharge reached");
			}

			// propagate through the rest of the fire as needed
			if (_early_termination) {

				// TODO: DO ANYTHING YOU WANT FOR A VIOLATION

				calculate_score();
			}

			// ---- Check hard failures ---- //

			/* check strict requirements (max alt and radius) (with 10 and 5 m buffers, respectively) */
			float r2 = _local_data.N*_local_data.N + _local_data.E*_local_data.E;
			float max_r2 = (_parameters.max_radius + 5.0f)*(_parameters.max_radius + 5.0f); // with additional 5 meter buffer
			if (-_local_data.D_gps >= (_parameters.max_alt + 10.0f) || r2 > max_r2) {
				// end mission and set score to 0 if violate max altitude
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission failed: boundary violation");
			}


			// --- Time related checks --- //

			/* get the current time needed for further calculations */
			hrt_abstime current_time = hrt_absolute_time();


			/* check to see if mission time has elapsed */
			if (_in_mission && (current_time - _mission_start_time)/1000000.0f >= _parameters.duration*60.0f) {
				_in_mission = false;
				_can_start = false;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission completed");

				// TODO: ADD ANY END OF TIME CODE

				// calculate the final score
				calculate_score();
			}


		}

		/* publish the mission status as the last thing to do each loop */
		publish_mission_status();

	}

	warnx("exiting.\n");

	// close the buzzer connection
	close(_buzzer);

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
AA241xMission::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("aa241x_mission",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT + 20,
			1800,
			(main_t)&AA241xMission::task_main_trampoline,
			nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


int aa241x_mission_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: aa241x_mission_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (aa241x_mission::g_aa241x_mission != nullptr)
			errx(1, "already running");

		aa241x_mission::g_aa241x_mission = new AA241xMission;

		if (aa241x_mission::g_aa241x_mission == nullptr)
			errx(1, "alloc failed");

		if (OK != aa241x_mission::g_aa241x_mission->start()) {
			delete aa241x_mission::g_aa241x_mission;
			aa241x_mission::g_aa241x_mission = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (aa241x_mission::g_aa241x_mission == nullptr || !aa241x_mission::g_aa241x_mission->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (aa241x_mission::g_aa241x_mission == nullptr)
			errx(1, "not running");

		delete aa241x_mission::g_aa241x_mission;
		aa241x_mission::g_aa241x_mission = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (aa241x_mission::g_aa241x_mission) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
