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
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include "LakeFire.h"
#include "fires.h"
#include "aa241x_mission_namespace.h"


/**
 * aa241x_mission app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_mission_main(int argc, char *argv[]);


namespace aa241x_mission
{

LakeFire	*g_aa241x_mission = nullptr;
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
	_mission_status_pub(-1),
	_new_fire_pub(-1),
	_mission_start_time(-1),
	_last_propagation_time(-1),
	_in_mission(false),
	_can_start(true),
	_early_termination(false),
	_mission_failed(true),
	_score(0.0f),
	_last_picture(0),
	_wind_direction(WIND_OTHER),
	_grid{{0}}
{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};

	_parameter_handles.min_alt = param_find("AAMIS_MIN_ALT");
	_parameter_handles.max_alt = param_find("AAMIS_MAX_ALT");
	_parameter_handles.auto_alt = param_find("AAMIS_AUTO_ALT");
	_parameter_handles.duration = param_find("AAMIS_DURATION");
	_parameter_handles.max_radius = param_find("AAMIS_MAX_RAD");
	_parameter_handles.timestep = param_find("AAMIS_TSTEP");
	_parameter_handles.std = param_find("AAMIS_STD");
	_parameter_handles.t_pic = param_find("AAMIS_TPIC");
	_parameter_handles.index = param_find("AAMIS_INDEX");
	_parameter_handles.ctr_lat = param_find("PE_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("PE_CTR_LON");
	_parameter_handles.ctr_alt = param_find("PE_CTR_ALT");

	parameters_update();

}

LakeFire::~LakeFire() {
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


void
LakeFire::take_picture()
{
	hrt_abstime curr_time = hrt_absolute_time();
	float time_diff = (curr_time - _last_picture)/1000000.0f;

	if (time_diff <= _parameters.t_pic) {
		/* do not take a picture */
		return;
	}

	// TODO: convert from x,y to i,j coords and determine the fov



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
	param_get(_parameter_handles.t_pic, &(_parameters.t_pic));
	param_get(_parameter_handles.index, &(_parameters.index));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));

	return OK;
}


void
LakeFire::vehicle_control_mode_update()
{
	/* Check if vehicle control mode has changed */
	bool vcontrol_mode_updated;
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
LakeFire::global_pos_update()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
LakeFire::local_pos_update()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
LakeFire::vehicle_status_update()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
LakeFire::publish_mission_status()
{
	aa241x_mission_status_s mis_stat;
	mis_stat.can_start = _can_start;
	mis_stat.in_mission = _in_mission;
	mis_stat.score = _score;
	mis_stat.mission_time = (hrt_absolute_time() - _mission_start_time)/(1000000.0f*60.0f);

	/* publish the mission status */
	if (_mission_status_pub > 0) {
		orb_publish(ORB_ID(aa241x_mission_status), _mission_status_pub, &mis_stat);
	} else {
		_mission_status_pub = orb_advertise(ORB_ID(aa241x_mission_status), &mis_stat);
	}
}

void
LakeFire::publish_new_fire(const std::vector<int> &i_new, const std::vector<int> &j_new)
{
	aa241x_new_fire_s new_fire;
	new_fire.mission_time = _last_propagation_time;
	new_fire.i_new = i_new;
	new_fire.j_new = j_new;

	/* publish the new fire cells */
	if (_new_fire_pub > 0) {
		orb_publish(ORB_ID(aa241x_new_fire), _new_fire_pub, &new_fire);
	} else {
		_new_fire_pub = orb_advertise(ORB_ID(aa241x_new_fire), &new_fire);
	}
}


float
LakeFire::generate_normal_random(const float &mean)
{
	// TODO: it is never actually using the spare, would need to make params global
	bool have_spare = false;

	/* get uniform random values on interval (0,1) */
	float rand1 = (float) rand() / ((float) MAX_RAND);
	float rand2 = (float) rand() / ((float) MAX_RAND);

	float R;
	float theta;

	/* generate 2 random numbers at a time, but only use 1, check for leftover from previous request */
	if (have_spare) {
		have_spare = false;
		return (_parameters.std * sqrtf(R) * sinf(theta)) + mean;
	}

	have_spare = true;
	R = -2*logf(rand1);
	theta = rand2 * 2.0f * (float) M_PI;

	/* return Box-Muller transform for normal random */
	return (_parameters.std * sqrtf(R) * cosf(theta)) + mean;
}


void
LakeFire::get_prop_coords(int *i_prop, int *j_prop, const int &prop_dir)
{

	/* adjust new fire cell coords based on propagation direction */
	switch (WIND_DIRECTION(prop_dir)) {
	case NORTH:
		(*i_prop)--;
		break;
	case NORTH_EAST:
		(*i_prop)--;
		(*j_prop)++;
		break;
	case EAST:
		(*j_prop)++;
		break;
	case SOUTH_EAST:
		(*i_prop)++;
		(*j_prop)++;
		break;
	case SOUTH:
		(*i_prop)++;
		break;
	case SOUTH_WEST:
		(*i_prop)++;
		(*j_prop)--;
		break;
	case WEST:
		(*j_prop)--;
		break;
	case NORTH_WEST:
		(*i_prop)--;
		(*j_prop)--;
		break;
	default:
		break;
	}
}

void
LakeFire::initialize_mission()
{
	// TODO: load up the initial locations for the fire
	// TODO: load up the wind direction

	if (_parameters.index >= NUM_FIRES) {
		// TODO: throw a system warning
		_can_start = false;
		return;
	}

	_wind_direction = WIND_DIRECTION(fire_wind_dir[_parameters.index]);
	srand(seed_start[_parameters.index]); // TODO: check to see if this works beyond the scope of this function

	int i_s;
	int j_s;
	for (int i = 0; i < NUM_STARTS; i++) {
		i_s = i_start[_parameters.index][i];
		j_s = j_start[_parameters.index][i];

		/* check to see if reached the end of valid locations */
		if (i_s < 0 || j_s < 0) {
			return;
		}

		/* set this starting grid on fire */
		_grid[i_s][j_s] = ON_FIRE;
	}

}


void
LakeFire::propagate_fire()
{
	// TODO: write propagation script, and somehow need to be able
	// to save the list of new fire locations to publish via uORB and
	// then via mavlink to the ground
	float prop_dir;
	int8_t cell_val;
	int i_prop;
	int j_prop;

	std::vector<int> i_new;
	std::vector<int> j_new;

	int count = 0;

	for (int i = 0; i < 21; i++) {
		for (int j = 0; j < 21; j++) {

			/* make sure this cell isn't a new fire cell */
			if (std::find(i_new.begin(), i_new.end(), i) != i_new.end() && std::find(j_new.begin(), j_new.end(), i) != j_new.end()) {
				continue;
			}

			/* check for fire in cell, continue if no fire in cell */
			cell_val = _grid[i][j];
			if (cell_val < ON_FIRE) continue;
			count++;

			prop_dir = roundf(generate_normal_random(_wind_direction));

			/* wrap the propagation direction to be within (0,8) */
			if (prop_dir < 0) prop_dir += 8;
			if (prop_dir > 7) prop_dir -= 8;

			i_prop = i;
			j_prop = j;

			get_prop_coords(&i_prop, &j_prop, (int) prop_dir);

			// printf("(%i,%i) : %i  ->  (%i,%i)\n", i, j, (int) prop_dir, i_prop, j_prop);  // DEBUG

			/* check to make sure new fire cell is a valid location */
			if (i_prop >= 21 || i_prop < 0 || j_prop >= 21 || j_prop < 0) continue;

			/* check for new fire cell value */
			cell_val = _grid[i_prop][j_prop];
			if (cell_val == OPEN_LAND) {
				/* add fire to this cell */
				_grid[i_prop][j_prop] = ON_FIRE;
				i_new.push_back(i_prop);
				j_new.push_back(j_prop);
				// TODO: add this cell to a list of newly on fire cells
			}
		}
	}

	// printf("on fire propagated: %i\n", count);  // DEBUG

	/* clear vectors after having published the info */
	i_new.clear();
	j_new.clear();

}

void
LakeFire::calculate_score()
{
	int count = 0;
	int8_t cell_val;

	for (int i = 0; i < 21; i++) {
		for (int j = 0; j < 21; j++) {

			/* check for fire in cell */
			cell_val = _grid[i][j];
			if (cell_val < ON_FIRE) {
				count++;
			}
		}
	}

	_score = (1.0f - (float) count / (float) worst_case_score[_parameters.index])*100.0f;
}


void
LakeFire::task_main_trampoline(int argc, char **argv)
{
	// aa241x_mission::g_aa241x_mission->task_main();
	aa241x_mission::g_aa241x_mission->testing(); // DEBUG
}

void
LakeFire::print_grid() {
	printf("\n");
	for (int i = 0; i < 21; i++) {
		for (int j = 0;j < 21; j++) {
			printf("%d ", _grid[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

void
LakeFire::testing()
{
	const int nrolls=10000;  // number of experiments
	const int nstars=100;    // maximum number of stars to distribute
	int mean = 5;

	int p[10]={};

	for (int i=0; i<nrolls; ++i) {
		float number = roundf(generate_normal_random(mean));

		if ((number>=0.0f)&&(number<10.0f)) ++p[int(number)];
	}

	printf("normal_distribution (5):\n");

	for (int i=0; i<10; ++i) {
		printf("%i: ", i);
		int num_stars = p[i]*nstars/nrolls;
		for (int j= 0; j < num_stars; j++) {
			printf("*");
		}
		printf("\n");
	}


	initialize_mission();
	print_grid();

	for (int i = 0; i < 60; i++) {
		propagate_fire();
		// usleep(500000);
	}

	print_grid();



	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
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
	vehicle_control_mode_update();
	global_pos_update();
	local_pos_update();
	vehicle_status_update();

	/* wakeup source(s) */
	struct pollfd fds[5];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _vcontrol_mode_sub;
	fds[1].events = POLLIN;
	fds[2].fd = _global_pos_sub;
	fds[2].events = POLLIN;
	fds[3].fd = _local_pos_sub;
	fds[3].events = POLLIN;
	fds[4].fd = _vehicle_status_sub;
	fds[4].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

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

		/* vehicle control mode updated */
		if (fds[1].revents & POLLIN) {
			vehicle_control_mode_update();
		}

		/* global position updated */
		if (fds[2].revents & POLLIN) {
			global_pos_update();
		}

		/* local position updated */
		if (fds[3].revents & POLLIN) {
			local_pos_update();
		}

		/* vehicle status updated */
		if (fds[4].revents & POLLIN) {
			vehicle_status_update();
		}

		/* check auto start requirements */
		if (_can_start && !_in_mission) {

			if (-_local_pos.z >= _parameters.auto_alt && !_vcontrol_mode.flag_control_auto_enabled) {
				// not allowed to start is above auto alt and not in auto mode
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
				initialize_mission();
			}
		}

		/* ensure abiding by mission rules */
		if (_in_mission) {

			/* check auto requirements */
			if (!_vcontrol_mode.flag_control_auto_enabled) {
				// end mission and set score to 0 if switch to manual mode
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;  // TODO: check with Robbie about this
			}

			/* check strict requirements (max alt and radius) */
			float r2 = _local_pos.x*_local_pos.x + _local_pos.y*_local_pos.y;
			float max_r2 = _parameters.max_radius*_parameters.max_radius;
			if (-_local_pos.z >= _parameters.max_alt || r2 > max_r2) {
				// end mission and set score to 0 if violate max altitude
				_in_mission = false;
				_mission_failed = true;
				_score = 0.0f;
			}

			/* check min altitude requirements */
			if (-_local_pos.z <= _parameters.min_alt) {
				// end mission, but let fire propagate for rest of time
				_in_mission = false;
				_early_termination = true;
			}

			/* check to see if mission time has elapsed */
			hrt_abstime current_time = hrt_absolute_time();
			if ((current_time - _mission_start_time)/1000000.0f >= _parameters.duration*60.0f) {
				// TODO: would be nice to send a message that mission is over
				_in_mission = false;
				_can_start = false;
				// TODO: end mission gracefully and report final score
			}

			/* check if timestep has advanced */
			if ((current_time - _last_propagation_time) >= _parameters.timestep*1E6f) {
				_last_propagation_time = current_time;

				/* propagate the fire for this next timestep */
				propagate_fire();

				/* calculate the new score */
				calculate_score();
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
