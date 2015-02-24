/*
 * aa241x_mission_main.cpp
 *
 *  Created on: Feb 22, 2015
 *      Author: Adrien
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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include "LakeFire.h"


/**
 * aa241x_mission app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_mission_main(int argc, char *argv[]);


namespace aa241x_mission
{

LakeFire	*g_aa241x_mission;
}

LakeFire::LakeFire() :
	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),
	_vcontrol_mode_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_params_sub(-1),
	_mission_start_time(-1),
	_last_propagation_time(-1),
	_in_mission(false),
	_can_start(true),
	_early_termination(false),
	_mission_failed(true),
	_score(0.0f),
	_wind_direction(WIND_OTHER)
{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_grid = {{0}};

	_parameter_handles.min_alt = param_find("AAMIS_MIN_ALT");
	_parameter_handles.max_alt = param_find("AAMIS_MAX_ALT");
	_parameter_handles.auto_alt = param_find("AAMIS_AUTO_ALT");
	_parameter_handles.duration = param_find("AAMIS_DURATION");
	_parameter_handles.max_radius = param_find("AAMIS_MAX_RAD");
	_parameter_handles.timestep = param_find("AAMIS_TSTEP");
	_parameter_handles.std = param_find("AAMIS_STD");
	_parameter_handles.ctr_lat = param_find("PE_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("PE_CTR_LON");
	_parameter_handles.ctr_alt = param_find("PE_CTR_ALT");

	parameters_update();

}

LakeFire::~LakeFire() {
	// TODO Auto-generated destructor stub
}

int
LakeFire::parameters_update()
{
	param_get(_parameter_handles.min_alt, &(_parameters.min_alt));
	param_get(_parameter_handles.max_alt, &(_parameters.max_alt));
	param_get(_parameter_handles.auto_alt, &(_parameters.auto_alt));
	param_get(_parameter_handles.duration, &(_parameters.duration));
	param_get(_parameter_handles.max_radius, &(_parameters.max_radius));
	param_get(_parameter_handles.timestep, &(_parameters.timestep));
	param_get(_parameter_handles.std, &(_parameters.std));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));

	return OK;
}


void
LakeFire::vehicle_control_mode_poll()
{
	/* Check if vehicle control mode has changed */
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
LakeFire::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
LakeFire::local_pos_poll()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
LakeFire::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
LakeFire::initialize_mission()
{
	// TODO: load up the initial locations for the fire
	// TODO: load up the wind direction
}


void
LakeFire::propagate_fire()
{
	// TODO: write propagation script, and somehow need to be able
	// to save the list of new fire locations to publish via uORB and
	// then via mavlink to the ground
}

void
LakeFire::task_main_trampoline(int argc, char **argv)
{
	aa241x_mission::g_aa241x_mission->task_main();
}

void
LakeFire::task_main()
{
	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_poll();
	global_pos_poll();
	local_pos_poll();
	vehicle_status_poll();

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

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

		/* check auto start requirements */
		if (_can_start && !_in_mission) {

			if (-_local_pos.z >= _parameters.auto_alt && !_vcontrol_mode.flag_control_auto_enabled) {
				/* not allowed to start is above auto alt and not in auto mode */
				_can_start = false;
			}
		}

		/* check mission start requirements */
		if (_can_start && !_in_mission) {

			if (-_local_pos.z >= _parameters.min_alt && _vcontrol_mode.flag_control_auto_enabled) {
				/* start the mission once have crossed over the minimum altitude */
				_in_mission = true;
				_mission_start_time = hrt_absolute_time();
				_last_propagation_time = hrt_absolute_time();
			}
		}

		/* ensure abiding by mission rules */
		if (_in_mission) {

			/* check auto requirements */
			if (!_vcontrol_mode.flag_control_auto_enabled) {
				/* end mission and set score to 0 if switch to manual mode */
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;  // TODO: check with Robbie about this
			}

			/* check strict requirements (max alt and radius) */
			float r2 = _local_pos.x*_local_pos.x + _local_pos.y*_local_pos.y;
			float max_r2 = _parameters.max_radius*_parameters.max_radius;
			if (-_local_pos.z >= _parameters.max_alt || r2 > max_r2) {
				/* end mission and set score to 0 if violate max altitude */
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;
			}

			/* check min altitude requirements */
			if (-_local_pos.z <= _parameters.min_alt) {
				/* end mission, but let fire propagate for rest of time */
				_in_mission = false;
				_early_termination = true;
			}

			/* check to see if mission time has elapsed */
			hrt_abstime current_time = hrt_absolute_time();
			if ((current_time - _mission_start_time) >= _parameters.duration*1E6) {
				_in_mission = false;
				// TODO: end mission gracefully and report final score
			}

			/* check if timestep has advanced */
			if ((current_time - _last_propagation_time) >= _parameters.timestep*1E6) {
				_last_propagation_time = current_time;
				propagate_fire();
			}


		}


	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
LakeFire::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("aa241x_mission",
			SCHED_DEFAULT,
			SCHED_PRIORITY_DEFAULT + 20,
			1800,
			(main_t)&LakeFire::task_main_trampoline,
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

		aa241x_mission::g_aa241x_mission = new LakeFire;

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
