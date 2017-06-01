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

/*
 * @file aa241x_fw_control_params.h
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_PARAMS_H_
#define AA241X_FW_CONTROL_PARAMS_H_


#include <systemlib/param/param.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Struct of all of the custom parameters.
 *
 * Please make sure to add a variable for each of your newly defined
 * parameters here.
 */
struct aah_params {

	float example_high_param;
	float proportional_roll_gain;

	// TODO: add custom parameter variable names here......
        float proportional_pitch_gain;
        float proportional_altitude_gain;
        float proportional_yaw_gain;
        float proportional_throttle_gain;
        float proportional_heading_gain;
        int caseNum;
        float roll_trim;
        float pitch_trim;
        float yaw_trim;
        float input_speed;
        float banking_angle;
        //float input_position_E;
        //float input_position_N;
        float a;
        float b;
        float c;
        float delta_E;
        float unitVector_N_line;
        float unitVector_E_line;
        float waypoint_N;
        float waypoint_E;
        float input_altitude;
        float input_heading_angle_deg;
        //float heading_to_yaw_gain;
        float proportional_dist_gain;

};


/**
 * Struct of handles to all of the custom parameters.
 *
 *  Please make sure to add a variable for each of your newly
 *  defined parameters here.
 *
 *  NOTE: these variable names can be the same as the ones above
 *  (makes life easier if they are)
 */
struct aah_param_handles {

	param_t example_high_param;
	param_t proportional_roll_gain;

	// TODO: add custom parameter variable names here.......
        param_t proportional_pitch_gain;
        param_t proportional_altitude_gain;
        param_t proportional_yaw_gain;
        param_t proportional_throttle_gain;
        param_t proportional_heading_gain;
        param_t caseNum;
        param_t roll_trim;
        param_t pitch_trim;
        param_t yaw_trim;
        param_t input_speed;
        param_t banking_angle;
        //param_t input_position_E;
        //param_t input_position_N;
        param_t a;
        param_t b;
        param_t c;
        param_t delta_E;
        param_t unitVector_N_line;
        param_t unitVector_E_line;
        param_t waypoint_N;
        param_t waypoint_E;
        param_t input_altitude;
        param_t input_heading_angle_deg;
        //param_t heading_to_yaw_gain;
        param_t proportional_dist_gain;
};

/**
 * Initialize all parameter handles and values
 *
 */
int aah_parameters_init(struct aah_param_handles *h);

/**
 * Update all parameters
 *
 */
int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p);

#ifdef __cplusplus
}
#endif




#endif /* AA241X_FW_CONTROL_PARAMS_H_ */
