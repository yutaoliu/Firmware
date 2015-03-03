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
 * Definition of parameters for fixedwing example
 */

#include "aa241x_fw_control_params.h"



/* controller parameters, use max. 15 characters for param name!
 *
 * DO NOT DELETE AA_WATER_WGHT PARAMETER!!!!!!
 */

/**
 * The amount of weight in [grams] of "water" being carried by this UAV.
 *
 * @unit grams
 * @min 0
 * @group AA241x Student Params
 */
PARAM_DEFINE_FLOAT(AA_WATER_WGHT, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AA_EXAMPLE and will be in the AA dropdown.  Make sure to always
 * start your parameters with AA to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @min 0 							(optional minimum value for displaying in the ground station)
 * @max 100 						(optional max)
 * @group AA241x Student Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AA_EXAMPLE, 10.0f);

// TODO: define custom parameters here


int aa_parameters_init(struct aa_param_handles *h)
{

	h->water_weight 	=	param_find("AA_WATER_WGHT");

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h->example_param	=	param_find("AA_EXAMPLE");

	// TODO: add the above line for each of your custom parameters........



	return OK;
}

int aa_parameters_update(const struct aa_param_handles *h, struct aa_params *p)
{
	param_get(h->water_weight, &(p->water_weight));

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->example_param, &(p->example_param));

	// TODO: add the above line for each of your custom parameters.....

	return OK;
}
