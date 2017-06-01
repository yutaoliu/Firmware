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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_EXAMPLE, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_ROLLGAIN, 1.0f);

// TODO: define custom parameters here
PARAM_DEFINE_FLOAT(AAH_PITCHGAIN, 1.0f);
PARAM_DEFINE_FLOAT(AAH_ALTITUDEGAIN, 1.0f);
PARAM_DEFINE_FLOAT(AAH_YAWGAIN, 1.0f);
PARAM_DEFINE_FLOAT(AAH_THROTTLEGAIN, 1.0f);
PARAM_DEFINE_FLOAT(AAH_HEADINGGAIN, 1.0f);
PARAM_DEFINE_INT32(AAH_CASE, 0);
PARAM_DEFINE_FLOAT(AAH_ROLLTRIM, 0.0f);
PARAM_DEFINE_FLOAT(AAH_PITCHTRIM, 0.0f);
PARAM_DEFINE_FLOAT(AAH_YAWTRIM, 0.0f);
PARAM_DEFINE_FLOAT(AAH_INPUTSPEED, 15.0f);
PARAM_DEFINE_FLOAT(AAH_BANKINGANGLE, 0.0f);
//PARAM_DEFINE_FLOAT(AAH_INPUT_POS_E, 0.0f);
//PARAM_DEFINE_FLOAT(AAH_INPUT_POS_N, 0.0f);
PARAM_DEFINE_FLOAT(AAH_LINE_A, 1.0f);
PARAM_DEFINE_FLOAT(AAH_LINE_B, 0.0f);
PARAM_DEFINE_FLOAT(AAH_LINE_C, 1.0f);
PARAM_DEFINE_FLOAT(AAH_DELTA_E, 100.0f);
PARAM_DEFINE_FLOAT(AAH_LINE_N_LINE, 0.9091019f);
PARAM_DEFINE_FLOAT(AAH_LINE_E_LINE, -0.416573805f);
PARAM_DEFINE_FLOAT(AAH_LINE_N_WAYPT, -88.0f);
PARAM_DEFINE_FLOAT(AAH_LINE_E_WAYPT, -155.6017f);
PARAM_DEFINE_FLOAT(AAH_LINE_ALT, 80.0f);
PARAM_DEFINE_FLOAT(AAH_HEADING, 0.0f);
//PARAM_DEFINE_FLOAT(AAH_YAWHEADGAIN, 1.0f);
PARAM_DEFINE_FLOAT(AAH_DISTGAIN, 1.0f);


int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h->example_high_param		= param_find("AAH_EXAMPLE");
        h->proportional_roll_gain 	= param_find("AAH_ROLLGAIN");

	// TODO: add the above line for each of your custom parameters........
        h->proportional_pitch_gain      = param_find("AAH_PITCHGAIN");
        h->proportional_altitude_gain   = param_find("AAH_ALTITUDEGAIN");
        h->proportional_yaw_gain        = param_find("AAH_YAWGAIN");
        h->proportional_throttle_gain   = param_find("AAH_THROTTLEGAIN");
        h->proportional_heading_gain    = param_find("AAH_HEADINGGAIN");
        h->caseNum                      = param_find("AAH_CASE");
        h->roll_trim                    = param_find("AAH_ROLLTRIM");
        h->pitch_trim                   = param_find("AAH_PITCHTRIM");
        h->yaw_trim                     = param_find("AAH_YAWTRIM");
        h->input_speed                  = param_find("AAH_INPUTSPEED");
        h->banking_angle                = param_find("AAH_BANKINGANGLE");
        //h->input_position_E             = param_find("AAH_INPUT_POS_E");
        //h->input_position_N             = param_find("AAH_INPUT_POS_N");
        h->a                            = param_find("AAH_LINE_A");
        h->b                            = param_find("AAH_LINE_B");
        h->c                            = param_find("AAH_LINE_C");
        h->unitVector_N_line            = param_find("AAH_LINE_N_LINE");
        h->unitVector_E_line            = param_find("AAH_LINE_E_LINE");
        h->waypoint_N                   = param_find("AAH_LINE_N_WAYPT");
        h->waypoint_E                   = param_find("AAH_LINE_E_WAYPT");
        h->input_altitude               = param_find("AAH_LINE_ALT");
        h->input_heading_angle_deg      = param_find("AAH_HEADING");
        //h->heading_to_yaw_gain          = param_find("AAH_YAWHEADGAIN");
        h->delta_E                      = param_find("AAH_DELTA_E");
        h->proportional_dist_gain       = param_find("AAH_DISTGAIN");
	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->example_high_param, &(p->example_high_param));
	param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));

	// TODO: add the above line for each of your custom parameters.....
        param_get(h->proportional_pitch_gain, &(p->proportional_pitch_gain));
        param_get(h->proportional_altitude_gain, &(p->proportional_altitude_gain));
        param_get(h->proportional_yaw_gain, &(p->proportional_yaw_gain));
        param_get(h->proportional_throttle_gain, &(p->proportional_throttle_gain));
        param_get(h->proportional_heading_gain, &(p->proportional_heading_gain));
        param_get(h->caseNum, &(p->caseNum));
        param_get(h->roll_trim, &(p->roll_trim));
        param_get(h->pitch_trim, &(p->pitch_trim));
        param_get(h->yaw_trim, &(p->yaw_trim));
        param_get(h->input_speed, &(p->input_speed));
        param_get(h->banking_angle, &(p->banking_angle));
        //param_get(h->input_position_E, &(p->input_position_E));
        //param_get(h->input_position_N, &(p->input_position_N));
        param_get(h->a, &(p->a));
        param_get(h->b, &(p->b));
        param_get(h->c, &(p->c));
        param_get(h->unitVector_N_line , &(p->unitVector_N_line ));
        param_get(h->unitVector_E_line , &(p->unitVector_E_line ));
        param_get(h->waypoint_N, &(p->waypoint_N));
        param_get(h->waypoint_E, &(p->waypoint_E));
        param_get(h->input_altitude, &(p->input_altitude));
        param_get(h->input_heading_angle_deg, &(p->input_heading_angle_deg));
        //param_get(h->heading_to_yaw_gain , &(p->heading_to_yaw_gain ));
        param_get(h->delta_E , &(p->delta_E ));
        param_get(h->proportional_dist_gain  , &(p->proportional_dist_gain));
	return OK;
}
