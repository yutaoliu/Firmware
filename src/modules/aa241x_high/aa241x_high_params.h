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

        // Trims for maintaining level flight
        float trim_pitch;
        float trim_bank;
        float trim_throttle;
        float trim_rudder;
        float trim_elevator;
        float trim_aileron;
        int motor_active;

        // Gains for maintaining altitude
        float alt_gain_pitch;
        float gain_throttle;

        // Gains for maintaining heading
        float hdg_gain_rudd;
        float hdg_gain_bank;
        float hdg_gain_head;
        float ydot_gain;

        // Climb 1 Trims
        float climb1_throttle;
        float climb1_pitch;
        float climb2_pitch;

        // Descent 1 trims
        float descent1_throttle;
        float descent1_pitch;

        // Desired  velocity, heading and altitude
        float desired_heading;
        float desired_altitude;
        float desired_velocity;
        int path_no;

        // Control surface gains
        float gain_aileron;
        float gain_elevator;
        float gain_rudder;

        // Limits
        float max_bank;
        float redzone;

        // Turn stuff
        float waypoint_radius;

	// TODO: add custom parameter variable names here......

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

	// TODO: add custom parameter variable names here.......

        // Trims for maintaining level flight
        param_t trim_pitch;
        param_t trim_bank;
        param_t trim_throttle;
        param_t trim_rudder;
        param_t trim_elevator;
        param_t trim_aileron;
        param_t motor_active;

        // Gains for maintaining altitude
        param_t alt_gain_pitch;
        param_t gain_throttle;

        // Gains for maintaining heading
        param_t hdg_gain_rudd;
        param_t hdg_gain_bank;
        param_t hdg_gain_head;
        param_t ydot_gain;
        param_t path_no;

        // Climb 1 Trims
        param_t climb1_throttle;
        param_t climb1_pitch;
        param_t climb2_pitch;

        // Descent 1 trims
        param_t descent1_throttle;
        param_t descent1_pitch;

        // Desired heading and altitude
        param_t desired_heading;
        param_t desired_altitude;
        param_t desired_velocity;

        // Control surface gains
        param_t gain_aileron;
        param_t gain_elevator;
        param_t gain_rudder;

        // Limits
        param_t max_bank;
        param_t redzone;

        // Turn stuff
        param_t waypoint_radius;
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
