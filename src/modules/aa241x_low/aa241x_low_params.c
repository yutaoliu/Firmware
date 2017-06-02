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
 * @file aa241x_low_params.c
 *
 * Definition of custom parameters for low priority controller.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include "aa241x_low_params.h"



/*
 * controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAL_EXAMPLE and will be in the AAL dropdown.  Make sure to always
 * start your parameters with AAL to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x Low Params			(always include this)
 */
PARAM_DEFINE_FLOAT(AAL_TARGET1_N, -2000.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET1_E, 1900.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET2_N, -2000.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET2_E, 1900.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET3_N, -2000.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET3_E, 1900.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET4_N, -2000.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET4_E, 1900.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET5_N, -2000.0f);
PARAM_DEFINE_FLOAT(AAL_TARGET5_E, 1900.0f);

// TODO: define custom parameters here


int aal_parameters_init(struct aal_param_handles *h)
{


	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
        h->target1_N	=	param_find("AAL_TARGET1_N");
        h->target1_E	=	param_find("AAL_TARGET1_E");
        h->target2_N	=	param_find("AAL_TARGET2_N");
        h->target2_E	=	param_find("AAL_TARGET2_E");
        h->target3_N	=	param_find("AAL_TARGET3_N");
        h->target3_E	=	param_find("AAL_TARGET3_E");
        h->target4_N	=	param_find("AAL_TARGET4_N");
        h->target4_E	=	param_find("AAL_TARGET4_E");
        h->target5_N	=	param_find("AAL_TARGET5_N");
        h->target5_E	=	param_find("AAL_TARGET5_E");
	return OK;
}

int aal_parameters_update(const struct aal_param_handles *h, struct aal_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
        param_get(h->target1_N, &(p->target1_N));
        param_get(h->target1_E, &(p->target1_E));
        param_get(h->target2_N, &(p->target2_N));
        param_get(h->target2_E, &(p->target2_E));
        param_get(h->target3_N, &(p->target3_N));
        param_get(h->target3_E, &(p->target3_E));
        param_get(h->target4_N, &(p->target4_N));
        param_get(h->target4_E, &(p->target4_E));
        param_get(h->target5_N, &(p->target5_N));
        param_get(h->target5_E, &(p->target5_E));
	return OK;
}
