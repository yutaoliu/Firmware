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

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <string>
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
#include <systemlib/mavlink_log.h>


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
	_mavlink_log_pub(nullptr),
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
	_start_time(0),
	_phase_start_time(0),
	_mission_time(0.0f),
	_final_time(0.0f),

	_phase_num(-1),
	_num_plumes_found(0),
	_out_of_bounds(false),
	_all_plumes_found(false),

	_timestamp(0),
	_previous_loop_timestamp(0),


	_build_plumes_run(false),
	_check_field_bounds_run(false),
	_check_finished_run(false),
	_check_start_run(false),
	_check_violation_run(false),

	_debug_timestamp(0),
	_debug_yell(false)


{
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_batt_stat = {};
	_cur_pos.N = 0.0f; _cur_pos.E = 0.0f; _cur_pos.D = 0.0f;
	_prev_pos = _cur_pos;
    	for (int i= 0; i<5; i++) {
		_plume_N[i] = 0.0f;
		_plume_E[i] = 0.0f;
		_plume_radius[i] = 0.0f;
	}    


	_parameter_handles.min_alt = param_find("AAMIS_ALT_MIN");
	_parameter_handles.max_alt = param_find("AAMIS_ALT_MAX");
	_parameter_handles.max_phase_time = param_find("AAMIS_PHASE_MAXT");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");
	_parameter_handles.mis_fail = param_find("AAMIS_MIS_FAIL");
	_parameter_handles.debug_mode = param_find("AAMIS_DEBUG");
	_parameter_handles.mission_seed = param_find("AAMIS_MIS_SEED");
	_parameter_handles.lake_lag = param_find("AAMIS_LAKE_LAG");
	_parameter_handles.bounds_enforced = param_find("AAMIS_BOUND_ON");	

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
	param_get(_parameter_handles.max_phase_time, &(_parameters.max_phase_time));
	param_get(_parameter_handles.ctr_lat, &(_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(_parameters.ctr_alt));
	param_get(_parameter_handles.mis_fail, &(_parameters.mis_fail));
	param_get(_parameter_handles.debug_mode, &(_parameters.debug_mode));
	param_get(_parameter_handles.mission_seed, &(_parameters.mission_seed));
	param_get(_parameter_handles.lake_lag, &(_parameters.lake_lag));
	param_get(_parameter_handles.bounds_enforced, &(_parameters.bounds_enforced));

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
	mis_stat.mission_time 	= _mission_time;
	mis_stat.final_time 	= _final_time;
	mis_stat.mission_failed = _mission_failed;
	mis_stat.phase_num	= _phase_num;
	mis_stat.num_plumes_found = _num_plumes_found;
	mis_stat.in_plume	= false;  //TODO: variable not used; remove it
	mis_stat.out_of_bounds	= _out_of_bounds;
	memcpy(&mis_stat.plume_N, _plume_N, sizeof(_plume_N));
	memcpy(&mis_stat.plume_E, _plume_E, sizeof(_plume_E));
	memcpy(&mis_stat.plume_radius, _plume_radius, sizeof(_plume_radius));


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
	mavlink_log_info(&_mavlink_log_pub, "#AA241x mission initialized");

	_in_mission = true;
	//_can_start = false;	// don't allow restarting of a mission
	_start_time = hrt_absolute_time();
    _num_plumes_found = 0;

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


//CHECK_FIELD_BOUNDS check if airplane has left boundaries of Lake Lag
void AA241xMission::check_field_bounds()
{
	if (_parameters.debug_mode == 1 && (_check_field_bounds_run == false || _debug_yell == true)) {
		_check_field_bounds_run = true;
		//mavlink_log_info(&_mavlink_log_pub, "Check field bounds ran")
	}


	//% Set inbounds to start
	_out_of_bounds = false;

	//// Coyote Hill Boundary check ////
	if (_parameters.lake_lag == 0 && _parameters.bounds_enforced == 1) {
		// Assign struct boundaries

		_land_pos lake_boundaries[4];

		lake_boundaries[0].E =   16.9f;  lake_boundaries[0].N = -198.3f;
		lake_boundaries[1].E = -102.7f;  lake_boundaries[1].N = -208.5f;
		lake_boundaries[2].E = -138.3f;  lake_boundaries[2].N =  210.0f;
		lake_boundaries[3].E =  -18.7f;  lake_boundaries[3].N =  220.2f;
	

		//% Check if outside convex portions
		uint8_t convex[4] = {0, 1, 2, 3};

		for (int i = 0; i < 4; i++) {
			// If at the last boundary (wrapping)
			uint8_t nextpt = convex[i]+1;
			if (i == 3) {
				nextpt = convex[0];
			}

			if (line_side(lake_boundaries[convex[i]], lake_boundaries[nextpt], _cur_pos) > 0 ) {
				// If not already out of bounds, send mavlink
			        if (!_out_of_bounds) {
				        mavlink_log_critical(&_mavlink_log_pub, "Out of bounds at %5.1f E, %5.1f N; mission failed",(double)_cur_pos.E,(double)_cur_pos.N);
					_out_of_bounds = true;
				}
			        _mission_failed = true;
				_in_mission = false;
			}
		}
	
		// Check if outside concave portions
		// Note: All concave portions of Coyote Hill bounds were removed
		/*uint8_t concave[1] = {4};
	
		for (int i = 0; i < 1; i++) {
		    if (line_side(lake_boundaries[concave[i]], lake_boundaries[concave[i]+1], _cur_pos) > 0 
		    && line_side(lake_boundaries[concave[i]+1],lake_boundaries[concave[i]+2], _cur_pos) > 0) {
		        _mission_failed = true;
			_in_mission = false;
		        // If not already out of bounds, send mavlink
		        if (!_out_of_bounds) {
			        mavlink_log_critical(_mavlink_fd, "AA241x mission failed, out of bounds");
				_out_of_bounds = true;
			}
		    }
		}
        	*/

	//// Lake Lag Boundary Check ////
	} else if (_parameters.bounds_enforced == 1) {
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

		//% Check if outside convex portions
		uint8_t convex[5] = {0, 1, 2, 5, 8};

		for (int i = 0; i < 5; i++) {
			// If at the last boundary (wrapping)
			uint8_t nextpt = convex[i]+1;
			if (i == 4) {
				nextpt = convex[0];
			}

			if (line_side(lake_boundaries[convex[i]], lake_boundaries[nextpt], _cur_pos) > 0 ) {
				// If not already out of bounds, send mavlink
			        if (!_out_of_bounds) {
				        mavlink_log_critical(&_mavlink_log_pub, "Out of bounds at %5.1f E, %5.1f N; mission failed",(double)_cur_pos.E,(double)_cur_pos.N);
					_out_of_bounds = true;
				}
			        _mission_failed = true;
				_in_mission = false;
			}
		}


		// Check if outside concave portions
		// Note: All concave portions of Coyote Hill bounds were removed
		uint8_t concave[2] = {3, 6};
	
		for (int i = 0; i < 2; i++) {
		    if (line_side(lake_boundaries[concave[i]], lake_boundaries[concave[i]+1], _cur_pos) > 0 
		    && line_side(lake_boundaries[concave[i]+1],lake_boundaries[concave[i]+2], _cur_pos) > 0) {
		        _mission_failed = true;
			_in_mission = false;
		        // If not already out of bounds, send mavlink
		        if (!_out_of_bounds) {
			        mavlink_log_critical(&_mavlink_log_pub, "Out of bounds at %5.1f E, %5.1f N; mission failed",(double)_cur_pos.E,(double)_cur_pos.N);
				_out_of_bounds = true;
			}
		    }
		}

	}

	if (_parameters.bounds_enforced == 1) {
		// Altitude check same for lag and coyote hill
		// Check if violating the flight window (5m safety buffer for errors)
		if (-_cur_pos.D > (_parameters.max_alt + 5.0f) || -_cur_pos.D < (_parameters.min_alt - 5.0f)) {
			_mission_failed = true;
			_in_mission = false;
			if (!_out_of_bounds) {
				mavlink_log_critical(&_mavlink_log_pub, "AA241x mission failed, altitude violation");
				_out_of_bounds = true;
			}
		}
	}
}	
	

// build plume locations and radii here
void AA241xMission::build_plumes() {
    
    float coord_N, coord_E;
    _keys _mission_seeds[30];

    if (_parameters.lake_lag == 0) { // Coyote Hill specific code
    // students choose a number from 0 to size(keys)
    _mission_seeds[0].key_one = 781231111971462951; _mission_seeds[0].key_two = 61591791601922272;
    _mission_seeds[1].key_one = 912111313291001132; _mission_seeds[1].key_two = 401572212031811381;
    _mission_seeds[2].key_one = 731821123732373151; _mission_seeds[2].key_two = 82701461371212671;
    _mission_seeds[3].key_one = 851161421581722061; _mission_seeds[3].key_two = 422521933001081333;
    _mission_seeds[4].key_one = 341622081872433081; _mission_seeds[4].key_two = 241811583133971941;
    _mission_seeds[5].key_one = 783041561121632091; _mission_seeds[5].key_two = 491331172691662431;
    _mission_seeds[6].key_one = 262673941263772011; _mission_seeds[6].key_two = 501133601861771891;
    _mission_seeds[7].key_one = 511751731641821701; _mission_seeds[7].key_two = 271372862641841721;
    _mission_seeds[8].key_one = 213591713111511871; _mission_seeds[8].key_two = 351562351161912181;
    _mission_seeds[9].key_one = 772501172573331873; _mission_seeds[9].key_two = 211451822201062231;
    _mission_seeds[10].key_one = 812321951283723401; _mission_seeds[10].key_two = 521813461073441291;
    _mission_seeds[11].key_one = 932531091813991422; _mission_seeds[11].key_two = 581661482212861791;
    _mission_seeds[12].key_one = 912381641511881163; _mission_seeds[12].key_two = 761723331921481191;
    _mission_seeds[13].key_one = 323801991422981291; _mission_seeds[13].key_two = 113422073641772901;
    _mission_seeds[14].key_one = 432121763132682901; _mission_seeds[14].key_two = 332473701221101741;
    _mission_seeds[15].key_one = 72863741501833163; _mission_seeds[15].key_two = 651783133251522991;
    _mission_seeds[16].key_one = 481771262132383912; _mission_seeds[16].key_two = 941881451122672861;
    _mission_seeds[17].key_one = 133601711571422021; _mission_seeds[17].key_two = 922362151782121441;
    _mission_seeds[18].key_one = 71812463573321061; _mission_seeds[18].key_two = 831241261482041701;
    _mission_seeds[19].key_one = 211241471262833621; _mission_seeds[19].key_two = 551671031162413913;
    _mission_seeds[20].key_one = 923331161863073631; _mission_seeds[20].key_two = 651872181021332461;
    _mission_seeds[21].key_one = 101361341382181613; _mission_seeds[21].key_two = 111811451201641981;
    _mission_seeds[22].key_one = 832901472261471763; _mission_seeds[22].key_two = 961073362762531741;
    _mission_seeds[23].key_one = 812333451223901722; _mission_seeds[23].key_two = 512501113631782301;
    _mission_seeds[24].key_one = 782263572551341923; _mission_seeds[24].key_two = 61862221391131501;
    _mission_seeds[25].key_one = 813123631272801651; _mission_seeds[25].key_two = 691951432763182581;
    _mission_seeds[26].key_one = 751541141841811191; _mission_seeds[26].key_two = 612582911382071931;
    _mission_seeds[27].key_one = 562163813082433862; _mission_seeds[27].key_two = 231181411782091571;
    _mission_seeds[28].key_one = 232821041541741362; _mission_seeds[28].key_two = 923482813201221291;
    _mission_seeds[29].key_one = 422081801923482232; _mission_seeds[29].key_two = 672363913091691181;

    // lower left corner:
    coord_E =  -93.574f;
    coord_N = -197.675f;

    } else  { // Lake Lag mission keys:
    _mission_seeds[0].key_one = 991941151071511981; _mission_seeds[0].key_two = 812751531122882462;
    _mission_seeds[1].key_one = 171581712111061532; _mission_seeds[1].key_two = 921422291901111751;
    _mission_seeds[2].key_one = 112191971212252611; _mission_seeds[2].key_two = 801133762531081481;
    _mission_seeds[3].key_one = 881581253591831263; _mission_seeds[3].key_two = 872253641521841401;
    _mission_seeds[4].key_one = 883332283433071382; _mission_seeds[4].key_two = 1572613241221091;
    _mission_seeds[5].key_one = 813212881872361281; _mission_seeds[5].key_two = 141791132271291611;
    _mission_seeds[6].key_one = 522153991782372423; _mission_seeds[6].key_two = 1582541901041951;
    _mission_seeds[7].key_one = 691191522161863081; _mission_seeds[7].key_two = 11152751431981401;
    _mission_seeds[8].key_one = 861261061272901783; _mission_seeds[8].key_two = 222141673211421601;
    _mission_seeds[9].key_one = 413751261172111612; _mission_seeds[9].key_two = 921251753591401211;
    _mission_seeds[10].key_one = 372842213813131382; _mission_seeds[10].key_two = 351682612841991441;
    _mission_seeds[11].key_one = 541031821613391153; _mission_seeds[11].key_two = 851252883613201221;
    _mission_seeds[12].key_one = 553981491041781761; _mission_seeds[12].key_two = 723733311471041172;
    _mission_seeds[13].key_one = 621552251391791253; _mission_seeds[13].key_two = 411822541153511491;
    _mission_seeds[14].key_one = 291951782852051321; _mission_seeds[14].key_two = 241223712582852261;
    _mission_seeds[15].key_one = 171132853383833222; _mission_seeds[15].key_two = 781351812881111591;
    _mission_seeds[16].key_one = 322391783833401211; _mission_seeds[16].key_two = 981441241071211981;
    _mission_seeds[17].key_one = 862591361872721123; _mission_seeds[17].key_two = 281462171961731881;
    _mission_seeds[18].key_one = 423811991151931511; _mission_seeds[18].key_two = 552051543191691811;
    _mission_seeds[19].key_one = 322831483331491612; _mission_seeds[19].key_two = 961821553061701691;
    _mission_seeds[20].key_one = 351041431601281561; _mission_seeds[20].key_two = 721763121163411431;
    _mission_seeds[21].key_one = 832142562361911161; _mission_seeds[21].key_two = 721871732373222801;
    _mission_seeds[22].key_one = 301661723183541021; _mission_seeds[22].key_two = 891301743121101161;
    _mission_seeds[23].key_one = 251911482221271862; _mission_seeds[23].key_two = 141241172911321491;
    _mission_seeds[24].key_one = 463941061823371401; _mission_seeds[24].key_two = 571491763512151281;
    _mission_seeds[25].key_one = 712051591163782112; _mission_seeds[25].key_two = 801313691851161081;
    _mission_seeds[26].key_one = 862491081071401361; _mission_seeds[26].key_two = 621762243481711991;
    _mission_seeds[27].key_one = 701343921281233701; _mission_seeds[27].key_two = 641181772623041231;
    _mission_seeds[28].key_one = 691523291343851691; _mission_seeds[28].key_two = 911361571541832091;
    _mission_seeds[29].key_one = 821863262262321021; _mission_seeds[29].key_two = 771143531683812951;

    // lower left corner:
    coord_E = -100.0f;
    coord_N = -100.0f;
    }

    int k = _parameters.mission_seed;
    _keys key;
    key = _mission_seeds[k];

    //
    int cell[5] = {0,0,0,0,0};
    int diameter[5] = {-2,-2,-2,-2,-2};
    float North[5] = {0.0,0.0,0.0,0.0,0.0};
    float East[5] = {0.0,0.0,0.0,0.0,0.0};


    // parse key:
    uint64_t key_cur, N, n, cur, old, star;
    if (_phase_num == 1){
        key_cur = key.key_one / pow(10,9);
    }
    else if (_phase_num == 2){
        key_cur = key.key_one/pow(10,9);
        key_cur = key_cur*pow(10,9);
        key_cur = key.key_one - key_cur;
        key_cur = key_cur*pow(10,3) + key.key_two/pow(10,15);
    }
    else if (_phase_num == 3){
        key_cur = key.key_two/pow(10,15);
        key_cur = key_cur*pow(10,15);
        key_cur = key.key_two - key_cur;
    }
    else {key_cur = 0;}

    old = 0;
    star = 3*(_phase_num+1);
    for (int i= 0; i<(_phase_num+2); i++) {
        n = star - i*3;
        cur = key_cur/pow(10,n);
        N   = cur - old; 
        old = cur*1000;
        
        cell[i] = (int)N/10;
        diameter[i] = (int)N - cell[i]*10;
    }

    // convert cells to East, North coordinates:
    for (int i = 0; i<5; i++) {
        if (diameter[i] > 0) {
            int findrow, findcol;
            if (_parameters.lake_lag == 0){
                findrow = cell[i]/5;
                findcol = cell[i]%5; }
            else {
                findrow = cell[i]/10;
                findcol = cell[i]%10; }
            float np, ep;
            np = 10.0f + (float)findrow*20.0f;
            ep = 10.0f + (float)findcol*20.0f;
            
            if (_parameters.lake_lag == 0){
                // rotate coordinates: (useful for Coyote Hill)
                float theta = -atanf(1/11.75); // angle of Coyote Hill fly-area rectangle to vertical
                North[i] = cosf(theta)*np - sinf(theta)*ep;
                East[i]  = sinf(theta)*np + cosf(theta)*ep; 
            } 
            else {
                North[i] = np;
                East[i] = ep; 
            }
        } 
    }

    // assign plume data:
    for (int i = 0; i<5; i++) {
        _plume_N[i] = coord_N + North[i];
        _plume_E[i] = coord_E + East[i];
        _plume_radius[i] = (float)diameter[i]*20/2;
    }

    // MESSAGE, debugging
    //mavlink_log_info(&_mavlink_log_pub, "#AA241x plume1 N: %.1f m, E: %.1f, radius: %.1f", (double)_plume_N[0],(double)_plume_E[0],(double)_plume_radius[0]);
}


// Start mission if in bounds
void AA241xMission::check_start()
{

	if (_parameters.debug_mode == 1 && (_check_start_run == false || _debug_yell == true)) {
		_check_start_run = true;
		mavlink_log_info(&_mavlink_log_pub, "Check start ran");
	}

	if (!_parameters.bounds_enforced == 1) {
		mavlink_log_critical(&_mavlink_log_pub, "Boundaries not enforced; started at %5.1f E, %5.1f N",(double)_cur_pos.E,(double)_cur_pos.N);
		_in_mission = true;
	} else if (!_out_of_bounds) {
		_in_mission = true;
            	// MESSAGE, competition started
            	mavlink_log_info(&_mavlink_log_pub, "Valid starting position at %5.1f E, %5.1f N",(double)_cur_pos.E,(double)_cur_pos.N);

        } else {
        	_mission_failed = true;
		mavlink_log_critical(&_mavlink_log_pub, "Invalid starting position; mission failed");
        }
}

// Advance through mission phases and finish
void AA241xMission::check_finished()
{
	if (_parameters.debug_mode == 1 && (_check_finished_run == false || _debug_yell == true)) {
		_check_finished_run = true;
		mavlink_log_info(&_mavlink_log_pub, "Check finish ran");
	}


	// Time in current phase
	float time_in_phase = (float)(hrt_absolute_time() - _phase_start_time)/1000000.0f;

	// Init at true; set to false if any plumes not visited
	_all_plumes_found = true;

	for (int i= 0; i<5; i++) {
		if (_plume_radius[i] > 0) {_all_plumes_found = false;}
	}

	// Boolean if new phase should be triggered
	bool new_phase = false;

	// Move to next phase if all plumes found
	if (_all_plumes_found) {
		mavlink_log_info(&_mavlink_log_pub, "All plumes in phase %i found.",_phase_num);
		new_phase = true;

	// Move to next phase if time expired
	} else if (time_in_phase > _parameters.max_phase_time) { 
		mavlink_log_info(&_mavlink_log_pub, "Time limit in phase %i reached.",_phase_num);
		new_phase = true;

	}

	if (new_phase) {
		_phase_num += 1;
		mavlink_log_info(&_mavlink_log_pub, "Current number of plumes found: %i",_num_plumes_found);
		if (_phase_num < 4) {
			build_plumes();
			_all_plumes_found = false;
			mavlink_log_info(&_mavlink_log_pub, "Phase %i started",_phase_num);
			_phase_start_time = hrt_absolute_time();
		} else {
			mavlink_log_info(&_mavlink_log_pub, "Mission completed successfully in %4.1f seconds",(double)_mission_time);
			_final_time = _mission_time;
		}
	}
}



void AA241xMission::check_near_plume() 
{	
	if (_parameters.debug_mode == 1 && (_check_violation_run == false || _debug_yell == true)) {
		_check_violation_run = true;
		mavlink_log_info(&_mavlink_log_pub, "Check near plume ran");
	}

	for (int i= 0; i<5; i++) {

		// Calculate distance from plume center
		float rp = sqrtf(powf(_cur_pos.E - _plume_E[i],2) + powf(_cur_pos.N - _plume_N[i],2));

		// Check if inside radius
		if (rp < _plume_radius[i]) {
			_num_plumes_found += 1;	
			mavlink_log_info(&_mavlink_log_pub, "Plume Found; %i total plumes found",_num_plumes_found);
			//mavlink_log_info(&_mavlink_log_pub, "Plume found: N: %.1f m, E: %.1f, radius: %.1f", (double)_plume_N[i],(double)_plume_E[i],(double)_plume_radius[i]);
			_plume_radius[i] = -1.0f; //mark plume as visited
		}
	}
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
	/* rate limit local data (including position) updates to 20Hz */
	orb_set_interval(_aa241x_local_data_sub, 50);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_control_mode_update();
	global_pos_update();
	local_pos_update();
	vehicle_status_update();
	aa241x_local_data_update();
	battery_status_update();

	// Set current position at start
	_cur_pos.N = _aa241x_local_data.N;
	_cur_pos.E = _aa241x_local_data.E;
	_cur_pos.D = _aa241x_local_data.D_gps;

	_prev_pos.N = 0.0f;
	_prev_pos.E = 0.0f;
	_prev_pos.D = 0.0f;

	/* build the first set of plumes */
	//build_plumes();  //wait until mission starts

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

			_prev_pos.N = _cur_pos.N;
			_cur_pos.N = _aa241x_local_data.N;
			_prev_pos.E = _cur_pos.E;
			_cur_pos.E = _aa241x_local_data.E;
			_prev_pos.D = _cur_pos.D;
			_cur_pos.D = _aa241x_local_data.D_gps;
		}

		/* check all other subscriptions */
		global_pos_update();
		vehicle_control_mode_update();
		local_pos_update();
		vehicle_status_update();
		battery_status_update();

		// Yell position every 5 seconds if debugging
		_debug_yell = false;
		if (_parameters.debug_mode == 1 && (hrt_absolute_time() - _debug_timestamp > 5000000)) {
			_debug_timestamp = hrt_absolute_time();
			_debug_yell = true;
			mavlink_log_info(&_mavlink_log_pub, "Current: N %.3f, E %.3f, D %.3f", (double)_cur_pos.N, (double)_cur_pos.E, (double)_cur_pos.D);
			mavlink_log_info(&_mavlink_log_pub, "Origin at (%9.6f, %9.6f)",(double)_parameters.ctr_lat,(double)_parameters.ctr_lon)
			//mavlink_log_info(&_mavlink_log_pub, "Previous: N %.3f, E %.3f D %.3f", (double)_prev_pos.N, (double)_prev_pos.E, (double)_prev_pos.D);
		}


		//  run required auto mission code
		if (_vcontrol_mode.flag_control_auto_enabled) {
			_timestamp = hrt_absolute_time();	

			// Check if there is no GPS lock and warn the user upon
			// starting auto mode if that is the case.
			// TODO: This gps flag is missing now; find where it went and add it back in

			//if (!_vehicle_status.condition_global_position_valid
				//&& (_timestamp - _previous_loop_timestamp) > 1000000) {

				//mavlink_log_critical(&_mavlink_log_pub, "AA241x. No GPS lock, do not launch airplane");
			//}

			// If not yet in mission check if mission has started
			if (_in_mission == false && _mission_failed == false) {
				// Check if in bounds
				check_field_bounds();
				check_start();
				// If check start sets mission to true set the start time
	            		if (_in_mission) {
	                		_start_time = hrt_absolute_time(); 	// initialize mission start time
					_phase_start_time = _start_time;   	// initialize phase start time
					_phase_num = 1;				// initialize phase number
					_num_plumes_found = 0;			// reset plumes found
					_final_time = 0.0f;			// reset final time
					build_plumes();				// initialize plumes
	            		}
	        	}

			if (_in_mission == true && _phase_num < 4 && _mission_failed == false) {
	            		// Report current time
	            		_mission_time = (float)(hrt_absolute_time() - _start_time)/1000000.0f;
				check_field_bounds();
				check_near_plume();
				check_finished();

				// TODO: add mission stuff here

	            	}
	        

        	} else {// in manual mode
	        	// if still in mission when activating manual, fail the mission
			_mission_failed = false;		// reset mission fail when exiting mission mode
	        	if (_in_mission == true) {
				_in_mission = false;
				if (_phase_num < 4) {
					mavlink_log_critical(&_mavlink_log_pub, "AA241x Mission Failed; manual mode activated");
				}
	        	}
		}
		_previous_loop_timestamp = _timestamp;
		
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
