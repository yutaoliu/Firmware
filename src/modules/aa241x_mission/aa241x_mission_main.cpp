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
	_aa241x_local_data_sub(-1),
	_battery_status_sub(-1),
	_mission_status_pub(nullptr),
	_mission_start_time(-1),

	_in_mission(false),
	_mission_failed(false),
	_start_time(-1),
	_current_time(0.0f),
	_final_time(0.0f),

	_in_turn(false),
	_just_started_turn(false),
	_turn_num(-1),
	_turn_radians(0.0f),
	_turn_degrees(0.0f),
	_req_turn_degrees(-840.0f),
	_num_of_turns(2),
	_num_violations(0),
	_in_violation(false),
	_out_of_bounds(false),

	_timestamp(0),
	_previous_loop_timestamp(0)
{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_batt_stat = {};
	_cur_pos.N = 0.0f; _cur_pos.E = 0.0f; _cur_pos.D = 0.0f;
	_prev_pos = _cur_pos;

	_parameter_handles.min_alt = param_find("AA_ALT_MIN");
	_parameter_handles.max_alt = param_find("AA_ALT_MAX");
	_parameter_handles.start_pos_N = param_find("AAMIS_SPOS_N");
	_parameter_handles.start_pos_E = param_find("AAMIS_SPOS_E");
	_parameter_handles.keepout_radius = param_find("AAMIS_RAD_KPT");
	_parameter_handles.tilt = param_find("AAMIS_TILT");
	_parameter_handles.leg_length = param_find("AAMIS_LEG_LEN");
	_parameter_handles.gate_width = param_find("AAMIS_GTE_WID");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");
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
	param_get(_parameter_handles.start_pos_N, &(_parameters.start_pos_N));
	param_get(_parameter_handles.start_pos_E, &(_parameters.start_pos_E));
	param_get(_parameter_handles.keepout_radius, &(_parameters.keepout_radius));
	param_get(_parameter_handles.tilt, &(_parameters.tilt));
	param_get(_parameter_handles.leg_length, &(_parameters.leg_length));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));
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
AA241xMission::aa241x_local_data_update()
{
	/* check if there is new status information */
	bool aa241x_local_data_updated;
	orb_check(_aa241x_local_data_sub, &aa241x_local_data_updated);

	if (aa241x_local_data_updated) {
		orb_copy(ORB_ID(aa241x_local_data), _aa241x_local_data_sub, &_aa241x_local_data);
	}
	// Set current position
	_cur_pos.N = _aa241x_local_data.N;
	_cur_pos.E = _aa241x_local_data.E;
	_cur_pos.D = _aa241x_local_data.D_baro;
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
	
	mis_stat.in_mission 	= _in_mission;
	mis_stat.start_time 	= _start_time;
	mis_stat.current_time 	= _current_time;
	mis_stat.final_time 	= _final_time;
	mis_stat.mission_failed = _mission_failed;
	mis_stat.in_turn		= _in_turn;
	mis_stat.turn_num		= _turn_num;
	mis_stat.turn_degrees	= _turn_degrees;
	mis_stat.num_violations = _num_violations;
	mis_stat.in_violation	= _in_violation;
	mis_stat.out_of_bounds	= _out_of_bounds;

	if (_in_mission) {
		//mis_stat.mission_time = (hrt_absolute_time() - _mission_start_time)/(1000000.0f*60.0f);
		//mis_stat.battery_used = _batt_stat.discharged_mah - _mission_start_battery;
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
	//_can_start = false;	// don't allow restarting of a mission
	_start_time = hrt_absolute_time();
	//_mission_start_battery = _batt_stat.discharged_mah;

	// TODO: ADD ANY ADDITIONAL INITIALIZATION REQUIRED
}

//
/* ALL OF THE AA241x FUNCTIONS LIVE HERE!! */
//
//

// SGNF quick signum function
int8_t AA241xMission::sgnf(const float &val)
{
	return   (0.0f < val) - (0.0f > val) ;
}

//ANGDIFF find the difference in two angles, accounting for a discontinuity
	// at +/- pi
float AA241xMission::angular_difference(const float &theta1, const float &theta0)
{
	

	float deltaTheta = theta1 - theta0;

	// Check bounds
	if (deltaTheta > pi) {deltaTheta = deltaTheta - 2*pi;}
	else if (deltaTheta < -pi) {deltaTheta = deltaTheta + 2*pi;}
	
	return deltaTheta;
}

//// LINESIDE, check if left (+1) or right (-1) of line
	// a, start of line
	// b, end of line
	// c, query point
int8_t AA241xMission::line_side(const _land_pos &a, 
								const _land_pos &b, 
								const _airplane_pos &c)
{
	

	return sgnf((b.E - a.E)*(c.N - a.N) - (b.N - a.N)*(c.E - a.E));
}


// build the racecourse from parameters
void AA241xMission::build_racecourse()
{
	// populate the start pylon with the appropriate values
	_start_pylon.N 	= _parameters.start_pos_N;
	_start_pylon.E 	= _parameters.start_pos_E;
	float tiltrad 	= _parameters.tilt * deg2rad;

	// set up pylons
	_pylon[0].E  = roundf(_start_pylon.E  + _parameters.leg_length*cosf(tiltrad));
	_pylon[0].N  = roundf(_start_pylon.N  + _parameters.leg_length*sinf(tiltrad));
	_pylon[0].angle = atan2f(_pylon[0].N - _start_pylon.N, _pylon[0].E - _start_pylon.E) + pi/2.0f;
	_pylon[1].E  = roundf(_pylon[0].E + _parameters.leg_length*cosf(tiltrad-2.0f*pi/3.0f));
	_pylon[1].N  = roundf(_pylon[0].N + _parameters.leg_length*sinf(tiltrad-2.0f*pi/3.0f));
	_pylon[1].angle = atan2f(_pylon[1].N - _pylon[0].N, _pylon[1].E - _pylon[0].E) + pi/2.0f;

	// set up start gate
	_start_gate[0].E = _start_pylon.E + _parameters.gate_width/2.0f * cosf(tiltrad - 2.0f*pi/3.0f) - 10.0f*cosf(tiltrad - pi/6.0f); // 10 insignificant, just for visibility
	_start_gate[0].N = _start_pylon.N + _parameters.gate_width/2.0f * sinf(tiltrad - 2.0f*pi/3.0f) - 10.0f*sinf(tiltrad - pi/6.0f);
	_start_gate[1].E = _start_pylon.E + _parameters.gate_width/2.0f * cosf(tiltrad - 2.0f*pi/3.0f);
	_start_gate[1].N = _start_pylon.N + _parameters.gate_width/2.0f * sinf(tiltrad - 2.0f*pi/3.0f);
	_start_gate[2].E = _start_pylon.E + _parameters.gate_width/2.0f * cosf(tiltrad + pi/3.0f);
	_start_gate[2].N = _start_pylon.N + _parameters.gate_width/2.0f * sinf(tiltrad + pi/3.0f);
	_start_gate[3].E = _start_pylon.E + _parameters.gate_width/2.0f * cosf(tiltrad + pi/3.0f) - 10.0f*cosf(tiltrad - pi/6.0f);
	_start_gate[3].N = _start_pylon.N + _parameters.gate_width/2.0f * sinf(tiltrad + pi/3.0f) - 10.0f*sinf(tiltrad - pi/6.0f);
}


//CHECK_FIELD_BOUNDS check if airplane has left boundaries of Lake Lag
void AA241xMission::check_field_bounds()
{
	// Assign struct boundaries

	_land_pos lake_boundaries[9];

	lake_boundaries[0].E = -173.0f; lake_boundaries[0].N =  143.0f;
	lake_boundaries[1].E = -82.0f;  lake_boundaries[1].N =  228.0f;
	lake_boundaries[2].E = 176.0f;  lake_boundaries[2].N =   81.0f;
	lake_boundaries[3].E = 181.0f;  lake_boundaries[3].N = -138.0f;
	lake_boundaries[4].E =  54.0f;  lake_boundaries[4].N = -148.0f;
	lake_boundaries[5].E =  62.0f;  lake_boundaries[5].N = -219.0f;
	lake_boundaries[6].E = -36.0f;  lake_boundaries[6].N = -216.0f;
	lake_boundaries[7].E = -101.0f; lake_boundaries[7].N = -142.0f;
	lake_boundaries[8].E = -181.0f; lake_boundaries[8].N = -112.0f;

	//% Set inbounds to start
	_out_of_bounds = false;

	//% Check if outside convex portions
	uint8_t convex[5] = {1, 2, 3, 6, 9};

	for (int i = 0; i < 5; i++) {
	    // If at the last boundary (wrapping)
		uint8_t nextpt = i+1;
	    if (i == 4) {
	        nextpt = 0;
	    }
	    
	    if (line_side(lake_boundaries[convex[i]], lake_boundaries[convex[nextpt]], _cur_pos) > 0 
	            && (_in_mission == true || _cur_pos.D < -40)) {
	        _mission_failed = true;
	        _in_mission = false;
	        _out_of_bounds = true;
	        // TONE
	        // send msg: aa241x mission failed, boundary violation
	        mavlink_log_critical(_mavlink_fd, "AA241x mission failed, lake boundary violation");
	    }
	}

	// Check if outside concave portions

	uint8_t concave[2] = {4, 7};

	for (int i = 0; i < 2; i++) {
	    if (line_side(lake_boundaries[concave[i]], lake_boundaries[concave[i]+1], _cur_pos) > 0 
	    && line_side(lake_boundaries[concave[i]+1],lake_boundaries[concave[i]+2], _cur_pos) > 0 
	    && (_in_mission == true || _cur_pos.D < -40) ) {
	        _mission_failed = true;
	        _in_mission = false;
	        _out_of_bounds = true;
	        // TONE
	        // send msg: aa241x mission failed, boundary violation
	        mavlink_log_critical(_mavlink_fd, "AA241x mission failed, lake boundary violation");
	    }
	}

	// Check if violating the flight window (5m safety buffer for errors)
	if (_in_mission == true && (_cur_pos.D < -(_parameters.max_alt + 5.0f) || _cur_pos.D > -(_parameters.min_alt - 5.0f))) {
	    _mission_failed = true;
	    _in_mission = false;
	    _out_of_bounds = true;
	    // TONE
	    // send msg: aa241x mission failed, boundary violation
	        mavlink_log_critical(_mavlink_fd, "AA241x mission failed, altitude violation");
	}
}

void AA241xMission::check_start()
{
	//check that previous position was within bounds
	if (line_side(_start_gate[0],_start_gate[1],_prev_pos) > 0 
	    && line_side(_start_gate[1],_start_gate[2],_prev_pos) > 0 
	    && line_side(_start_gate[2],_start_gate[3],_prev_pos) > 0 
	    && _cur_pos.D < -_parameters.min_alt && _cur_pos.D > -_parameters.max_alt) {
	    //check that new position is across start line
	        if (line_side(_start_gate[1],_start_gate[2],_cur_pos) < 0) {
	            _in_mission = true;
	            _turn_num = 0;
		    // MESSAGE, race started
		    mavlink_log_info(_mavlink_fd, "#audio: AA241x race started");
	        }
	}

}

void AA241xMission::check_finished()
{
	//check that previous position was within bounds
	if (line_side(_start_gate[1],_start_gate[2],_cur_pos) < 0) {
		if (line_side(_start_gate[0],_start_gate[1],_prev_pos) > 0 
		    && line_side(_start_gate[1],_start_gate[2],_prev_pos) > 0 
		    && line_side(_start_gate[2],_start_gate[3],_prev_pos) > 0 
		    && _cur_pos.D < -_parameters.min_alt && _cur_pos.D > -_parameters.max_alt) {
		    //check that new position is across start line
	            _in_mission = false;
	            _turn_num = -1;
		    // MESSAGE, race completed
		    mavlink_log_info(_mavlink_fd, "#audio: AA241x race completed");
	        }
	}
}

void AA241xMission::check_turn_start()
{
	// Calculate airplane current and previous angle relative to current pylon to turn around

	float cur_angle = atan2f(_cur_pos.N - _pylon[_turn_num].N, _cur_pos.E - _pylon[_turn_num].E); // float
	float prev_angle = atan2f(_prev_pos.N - _pylon[_turn_num].N, _prev_pos.E - _pylon[_turn_num].E); //float

	// Check if the airplane has passed clockwise past the turn (assumes must
	// have small angle

	float cur_angle_diff = angular_difference(cur_angle,_pylon[_turn_num].angle); //float
	float prev_angle_diff = angular_difference(prev_angle,_pylon[_turn_num].angle); //float

	// If current angle is clockwise of required angle and previous angle is
	// counterclockwise, plus the angular differences are less than pi/4 from
	// the required angle (req'd so that it doesn't trigger when crossing
	// opposite side) then start the turn
	if (cur_angle_diff <= 0.0f && prev_angle_diff >= 0.0f && fabsf(cur_angle_diff) < 2.0f*pi/3.0f) {
	    _in_turn = true;
	    _just_started_turn = true;
	    // MESSAGE, turn started
	    mavlink_log_info(_mavlink_fd, "#audio: AA241x turn started");
	}
}

void AA241xMission::check_turn_end() 
{
	// Less than negative because clockwise rotation
	if (_turn_radians < _req_turn_degrees * deg2rad) {
	    // end turn
	    _in_turn = false;
	    // reset accumulator
	    _turn_radians = 0.0f;
	    // step to next turn
	    _turn_num = _turn_num + 1;
	    // MESSAGE, turn completed
	    mavlink_log_info(_mavlink_fd, "#audio: AA241x turn completed");
	}
}

void AA241xMission::turn_accumulate()
{

	// initialize vars
	float prev_angle;

	// if just entered turn then accumulate from start line
	if (_just_started_turn) {
	    _just_started_turn = false;
	    prev_angle        = _pylon[_turn_num].angle;
	} else { // else just use the previous position's angle relative to the pylon
	    prev_angle        = atan2f(_prev_pos.N - _pylon[_turn_num].N, _prev_pos.E - _pylon[_turn_num].E);
	}

	// Calc current angle relative to pylon
	float cur_angle       = atan2f(_cur_pos.N - _pylon[_turn_num].N, _cur_pos.E - _pylon[_turn_num].E);

	// Calc angle traversed (assumes < 180 deg)
	float accumAngle = angular_difference(cur_angle,prev_angle);



	// Accumulate in the turn angle counter
	_turn_radians = _turn_radians + accumAngle;

	// Make sure you can't go positive (CCW accumulation) in accumulated angle
	if (_turn_radians > 0.0f) {
	    _turn_radians = 0.0f;
	}
}

void AA241xMission::check_violation()
{
	// Calculate r^2 distance from pylon
	float r2 =   (_cur_pos.E - _pylon[_turn_num].E)*(_cur_pos.E - _pylon[_turn_num].E)
	     + (_cur_pos.N - _pylon[_turn_num].N)*(_cur_pos.N - _pylon[_turn_num].N);
	 
	// calculate minimum radius^2 (with 2.5 meter safety buffer)
	float min_r2 = (_parameters.keepout_radius-2.5f)*(_parameters.keepout_radius-2.5f);

	// Check if distance is smaller than min distance and reset accumulator if
	// so

	if (r2 < min_r2) {
	    _turn_radians = 0;
	    _in_violation = true;
	} else if (_in_violation) {
	    _in_violation = false;
	    _num_violations = _num_violations + 1;
		// MESSAGE, violation occured
		mavlink_log_critical(_mavlink_fd, "AA241x turn keep out violation");
	}

}

/*void
AA241xMission::calculate_score()
{
	
	// TODO: WRITE YOUR SCORE FUNCTION HERE

	//_score = 0.0f;
}*/

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
	_aa241x_local_data_sub = orb_subscribe(ORB_ID(aa241x_local_data));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);
	/* rate limit local data (including position) updates to 50Hz */
	orb_set_interval(_aa241x_local_data_sub, 20);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_update();
	global_pos_update();
	local_pos_update();
	vehicle_status_update();
	aa241x_local_data_update();
	battery_status_update();

	/* build the racecourse */
	build_racecourse();

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2] = {};

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _aa241x_local_data_sub;
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
			aa241x_local_data_update(); // _cur_pos gets updated in here
		}

		/* check all other subscriptions */
		global_pos_update();
		vehicle_control_mode_update();
		local_pos_update();
		vehicle_status_update();
		battery_status_update();

		//  run required auto mission code
		if (_vcontrol_mode.flag_control_auto_enabled) {
			_timestamp = hrt_absolute_time();
			

			// Check if there is no GPS lock and warn the user upon
			// starting auto mode if that is the case.
			if (!_vehicle_status.condition_global_position_valid
				&& (_timestamp - _previous_loop_timestamp) > 1000000) {

				mavlink_log_critical(_mavlink_fd, "AA241x. No GPS lock, do not launch airplane");
			}
			
			// If not yet in mission check if mission has started
			if (!_in_mission) {
				check_start();
				// If check start sets mission to true set the start time
	            if (_in_mission) {
	                _start_time = hrt_absolute_time(); // make hrt_absolute_time
	            }
	        }

			if (_in_mission) {
			            
	            // Report current time
	            _current_time = (float)(hrt_absolute_time() - _start_time)/1000000.0f;
	            
	            if (_turn_num < _num_of_turns) {
	                if (!_in_turn) {
	                    check_start();
	                }
	                // Not an elseif so that turn accumulate on first run
	                if (_in_turn) {
	                    turn_accumulate();
	                    check_violation();
	                    check_turn_end();
	                }
	            } else {// turns completed; check for finishing conditions
	                check_finished();
	                if (!_in_mission) {
	                    _final_time = _current_time;
	                }
	            }
	        }

        } else {// in manual mode
	        // if still in mission when activating manual, fail the mission
	        if (_in_mission) {
	            _mission_failed = true;
	            // tone, msg output
	            mavlink_log_critical(_mavlink_fd, "AA241x. Mission failed, manual mode activated");
	        }
	    }
		    
	    // CHECK IF HARD VIOLATIONS
	    // if yea, mission_failed = true, final_time = 0
	    //
		check_field_bounds();
		_previous_loop_timestamp = _timestamp;
		_prev_pos = _cur_pos;
		

		#if 0
		/* check auto start requirements 
		 * not being used for this, _can_start will be checked for each
		 * loop and be invisibile to the user and make sure they cross the start
		 * gate from the appropriate spot.
		*/
		/*
		if (_can_start && !_in_mission) {

			if (!_vehicle_status.gps_failure && -_aa241x_local_data.D_gps >= _parameters.auto_alt && !_vcontrol_mode.flag_control_auto_enabled) {
				// not allowed to start is above auto alt and not in auto mode
				_can_start = false;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission start conditions violated");
			}
		}
		*/
		
		


		/* check mission start requirements */
		if (_can_start && !_in_mission) {

			if (_vcontrol_mode.flag_control_auto_enabled) {
				/* start the mission once have crossed over the minimum altitude */
				initialize_mission();
			}
		}

		/* ensure abiding by mission rules */
		if (_in_mission) {

			/* check to see if we have crossed min alt for first time */
			if (!_cross_min && -_aa241x_local_data.D_gps >= _parameters.min_alt) {
				_cross_min = true;
			}

			// ---- Check soft terminations first ---- //

			/* check auto requirements */
			if (!_vcontrol_mode.flag_control_auto_enabled) {
				// end mission and set score to 0 if switch to manual mode
				_in_mission = false;
				//_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: control mode violation");
			}

			/* check min altitude requirements (with 10m buffer) only if plane has gotten above it already */
			if (_cross_min && -_aa241x_local_data.D_gps <= (_parameters.min_alt - 10.0f)) {
				// end mission, but let fire propagate for rest of time
				_in_mission = false;
				//_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: below min alt");
			}

			/* check battery requirements */
			if ((_batt_stat.discharged_mah - _mission_start_battery) > _parameters.max_discharge) {
				_in_mission = false;
				//_early_termination = true;
				mavlink_log_info(_mavlink_fd, "#audio: AA241x mission termination: max battery discharge reached");
			}

			// propagate through the rest of the fire as needed
			/*if (_early_termination) {

				// TODO: DO ANYTHING YOU WANT FOR A VIOLATION

				calculate_score();
			}*/

			// ---- Check hard failures ---- //

			/* check strict requirements (max alt and radius) (with 10 and 5 m buffers, respectively) */
			float r2 = _aa241x_local_data.N*_aa241x_local_data.N + _aa241x_local_data.E*_aa241x_local_data.E;
			float max_r2 = (_parameters.max_radius + 5.0f)*(_parameters.max_radius + 5.0f); // with additional 5 meter buffer
			if (-_aa241x_local_data.D_gps >= (_parameters.max_alt + 10.0f) || r2 > max_r2) {
				// end mission and set score to 0 if violate max altitude
				_in_mission = false;
				_mission_failed = true;
				//_score = 0.0f;
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
		#endif

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
