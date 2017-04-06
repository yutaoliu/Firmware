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
 * @file AA241xMission.h
 *
 * Class definition for Stanford's Spring 2015 AA241x mission.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */
#pragma once

#ifndef AA241XMISSION_H_
#define AA241XMISSION_H_
#include <drivers/drv_hrt.h>
#include <time.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>

#include <uORB/topics/aa241x_mission_status.h>
#include <uORB/topics/aa241x_local_data.h>

// Important math defines
#define pi 3.141592653589793f
#define deg2rad (pi/180.0f)
#define rad2deg (1.0f / deg2rad)

class AA241xMission
{
public:
	/**
	 * Constructor
	 */
	AA241xMission();

	/**
	 * Destructor
	 */
	~AA241xMission();

	/**
	 * Start the AA241xMission task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool	task_running() { return _task_running; }


private:

	bool	_task_should_exit;		/**< if true, aa241x mission should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for aa241x mission */

	int		_mavlink_fd;			/**< file description for mavlink to be able to send warnings */
	int		_buzzer;				/**< descriptor for the buzzer */

	// handles to subscriptions needed
	int		_vcontrol_mode_sub;		/**< vehicle status (control mode) subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status (navigation mode) subscription */
	int		_params_sub;			/**< parameters update subscription */
	int		_aa241x_local_data_sub;		/**< custom data fields */
	int		_battery_status_sub;	/**< battery information */

	// TODO: ADD ADDITIONAL SUBSCRIBERS HERE

	// structures for subscribed data
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct aa241x_local_data_s			_aa241x_local_data;		/**< custom calc data */
	struct battery_status_s				_batt_stat;			/**< battery status */

	orb_advert_t	_mission_status_pub;

	// TODO: DEFINE PUBLISHERS HERE

	struct {
		float min_alt;
		float max_alt;
		float start_pos_N;
		float start_pos_E;
		float keepout_radius;
		float tilt;
		float ctr_lat;
		float ctr_lon;
		float ctr_alt;
		float leg_length;
		float gate_width;
		int mis_fail;
		int team_num;
		int debug_mode;
		int force_start;
                int mission_restart;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min_alt;
		param_t max_alt;
		param_t start_pos_N;
		param_t start_pos_E;
		param_t keepout_radius;
		param_t tilt;
		param_t ctr_lat;
		param_t ctr_lon;
		param_t ctr_alt;
		param_t leg_length;
		param_t gate_width;
		param_t mis_fail;
		param_t team_num;
		param_t debug_mode;
		param_t force_start;
                param_t mission_restart;
	}		_parameter_handles;		/**< handles for interesting parameters */

	hrt_abstime _mission_start_time;	/**< timestamp of when entered mission */
	//float		_mission_start_battery;	/**< the mAh used when entered the mission */
	bool 		_in_mission;			/**< if true, currently running a mission (fire is spreading) */
	//bool		_can_start;				/**< if false conditions for starting have been violated */
	//bool		_early_termination;		/**< if true terminating mission early, but still need to finish running fire */
	bool		_mission_failed;		/**< if true terminating mission entirely with a score of 0 */
	//float		_score;					/**< the current mission score */
	//bool		_cross_min;				/**< if plane has crossed the minimum altitude (to delay checks between takeoff and first cross) */

	// 

	struct _land_pos {
		float N;
		float E;
	}	;

	struct _airplane_pos {
		float N;
		float E;
		float D;
	} 	;

	struct _pylon_pos {
		float N;
		float E;
		float angle;
	}	;

	_land_pos _start_gate[4];
	_land_pos _lake_boundaries[9];

	_land_pos  _start_pylon;
	
	_pylon_pos _pylon[2];

	_airplane_pos _cur_pos;
	_airplane_pos _prev_pos;


	hrt_abstime _start_time;
	float _current_time;
	float _final_time;

	bool 	_in_turn;
	bool 	_just_started_turn;
	int8_t _turn_num;
	float 	_turn_radians;
	float 	_turn_degrees;
	float 	_req_turn_degrees;
	uint8_t _num_of_turns;
	uint8_t _num_violations;
	bool 	_in_violation;
	bool	_out_of_bounds;

	

	hrt_abstime _timestamp; 				// Current loop timestamp
	hrt_abstime _previous_loop_timestamp; 	// previous loop timestamp

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void	vehicle_control_mode_update();

	/**
	 * Check for global position updates.
	 */
	void	global_pos_update();

	/**
	 * Check for local position updates.
	 */
	void	local_pos_update();

	/**
	 * Check for vehicle status updates.
	 */
	void	vehicle_status_update();

	/**
	 * Check for custom calc updates.
	 */
	void	aa241x_local_data_update();

	/**
	 * Check for battery status updates.
	 */
	void	battery_status_update();

	/**
	 * Publish the current mission status information.
	 */
	void 	publish_mission_status();

	/**
	 * Initialize the mission parameters needed
	 */
	void	initialize_mission();

	/**
	 * Calculate the current score.
	 */
	//void 	calculate_score();


	// TODO: ADD ADDITIONAL MISSION SPECIFIC FUNCTIONS NEEDED


	// Functions for the racetrack (AA241x 2016)
	// implement a quick signum function
	int8_t 	sgnf(const float &val);
	// find angular difference
	float 	angular_difference(const float &theta1, const float &theta0);
	// check which side of line you are on ( left = +1, right = -1 )
	int8_t  line_side(const _land_pos &a, 
					  const _land_pos &b, 
					  const _airplane_pos &c); 
	// build the racecourse from the parameters
	void 	build_racecourse();
	bool	_build_racecourse_run;
	// check for hard field boundary violations
	void 	check_field_bounds();
	bool	_check_field_bounds_run; 
	// check if you have finished the race
	void 	check_finished();  
	bool	_check_finished_run;
	// check if you have started the race
	void 	check_start();		
	bool	_check_start_run;
	// check if a turn has started
	void 	check_turn_start(); 
	bool	_check_turn_start_run;
	// check if a turn has ended
	void 	check_turn_end();  
	bool	_check_turn_end_run;
	// check whether turn keepout has been violated
	void 	check_violation(); 
	bool	_check_violation_run;
	// accumulate degrees on a turn
	void 	turn_accumulate();
	bool	_turn_accumulate_run;
        // reset the mission variables if desired
        void    reset_mission();

	hrt_abstime _debug_timestamp;
	bool	_debug_yell;


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void	task_main();
};

#endif /* AA241XMISSION_H_ */
