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
 * @min 0
 * @max 100
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_EXAMPLE, 10.0f);

// TODO: define custom parameters here

//
//
// SURFACE TRIM PARAMETERS: Undefined, passthrough from conroller used...

//
//
// AAH_TRIM FLIGHT PARAMETERS:

 /* Level Pitch Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -45
 * @max 45
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIM_PITC, 0.0f);

 /* Level Bank Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -45
 * @max 45
 * @group AA241x High Params		(always include this)
 */

PARAM_DEFINE_FLOAT(AAH_TRIM_BANK, 0.0f);

 /* Level Throttle Parameter
 *
 * @min 0
 * @max 1
 * @group AA241x High Params		(always include this)
 */

PARAM_DEFINE_FLOAT(AAH_TRIM_THRO, 0.8f);

/* Motor active parameter
 *
 * @min  0 						
 * @max  1
 * @group AA241x High Params	(always include this)
 */
PARAM_DEFINE_INT32(AAH_MOT_ACT, 1);


/* Level elevator Parameter
 *
 * @min -1 						
 * @max  1
 * @group AA241x High Params	(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIM_ELEV, 0.023f);

/* Level aileron Parameter
 *
 * @min -1 						
 * @max  1
 * @group AA241x High Params	(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_TRIM_AILE, 0.23f);

 /* Level Rudder Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -45
 * @max 45
 * @group AA241x High Params		(always include this)
 */

PARAM_DEFINE_FLOAT(AAH_TRIM_RUDD, -0.00f);

//
//
// CONTROL SURFACE GAINS:

 /* Aileron Gain
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAIN_AILE, 2.0f);

 /* Elevator Gain
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAIN_ELEV, 2.0f);

 /* Rudder Gain
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAIN_RUDD, 1.0f);

//
//
// AAH_TRIM ALTITUDE HOLD GAINS:

 /* Level Pitch Gain
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_AGAIN_PITC, 3.0f);

 /* Level Throttle Alt Gain
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_GAIN_THRO, 0.2f);

//
//
// AAH_TRIM HEADING HOLD GAINS:

 /* Level Bank Gain
 *
 * @group AA241x High Params		(always include this)
 */


PARAM_DEFINE_FLOAT(AAH_HGAIN_HEAD, -0.1f);

 /* Heading gain, output is in !rad!
 *
 * @group AA241x High Params		(always include this)
 */


PARAM_DEFINE_FLOAT(AAH_HGAIN_BANK, 0.6f);

 /* Level Rudder Gain
 *
 * @group AA241x High Params		(always include this)
 */

PARAM_DEFINE_FLOAT(AAH_HGAIN_RUDD, 0.5f);

/**
  Ydot gain for damping oscillations. Output is deg / (m/s)
 *
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_HGAIN_YDOT, -1.0f);

//
//
// CLIMB 1 TRIMS:

 /* Climb 1 Throttle Parameter
 *
 * @min 0
 * @max 1
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CLMB1_THRO, 0.9f);

 /* Climb 1 Pitch Trim Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -45
 * @max 45
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CLMB1_PITC, 10.0f);

//
//
// DESCENT 1 TRIMS:

 /* Descent 1 Throttle Parameter
 *
 * @min 0
 * @max 1
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DSC1_THRO, 0.2f);

 /* Descent 1 Pitch Trim Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -45
 * @max 45
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DSC1_PITC, -20.0f);

//
//
// DEFINE AAH_DES HEADING DIRECTION AND ALTITUDE

 /* Heading Parameter
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min -180                        
 * @max 180
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DES_HDG, 90.0f);

 /* Altitude parameter
 *
 * @unit m 						(the unit attribute (not required, just helps for sanity))
 * @min 50							
 * @max 150						
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DES_ALT, 117.0f);

 /* path no.
 *
 * @min 1
 * @max 4                           
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_INT32(AAH_PATH_NO, 7);


//
//
// DEFINE AAH_DES VELOCITY

 /* Heading Parameter
 *
 * @unit m/s 						(the unit attribute (not required, just helps for sanity))
 * @min 0
 * @max 14  
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_DES_VEL, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit degrees 						(the unit attribute (not required, just helps for sanity))
 * @min -180
 * @max 180
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_MAX_BANK, 60.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meters 						(the unit attribute (not required, just helps for sanity))
 * @min 0
 * @max 50
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_WAYPNT_RAD, 10.0f);

/**
 *
 * Redzone buffer size
 * @unit meters 						(the unit attribute (not required, just helps for sanity))
 * @min 0
 * @max 50
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_REDZONE, 20.0f);

/**
 *
 *
 * @unit deg 						(the unit attribute (not required, just helps for sanity))
 * @min 0
 * @max 10
 * @group AA241x High Params		(always include this)
 */
PARAM_DEFINE_FLOAT(AAH_CLMB2_PITC, 6.0f);

int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h->example_high_param	=	param_find("AAH_EXAMPLE");

	// TODO: add the above line for each of your custom parameters........

        // Trims for maintaining level flight
        h->trim_pitch        =     param_find("AAH_TRIM_PITC");
        h->trim_bank         =     param_find("AAH_TRIM_BANK");
        h->trim_throttle         =     param_find("AAH_TRIM_THRO");
        h->trim_rudder         =     param_find("AAH_TRIM_RUDD");
        h->trim_elevator        =     param_find("AAH_TRIM_ELEV");
        h->trim_aileron        =     param_find("AAH_TRIM_AILE");
        h->motor_active        =     param_find("AAH_MOT_ACT");

        // Gains for maintaining altitude
        h->alt_gain_pitch         =     param_find("AAH_AGAIN_PITC");

        // Velocity hold gain
        h->gain_throttle        =     param_find("AAH_GAIN_THRO");

        // Gains for maintaining heading
        h->hdg_gain_head         =     param_find("AAH_HGAIN_HEAD");
        h->hdg_gain_rudd         =     param_find("AAH_HGAIN_RUDD");
        h->hdg_gain_bank        =     param_find("AAH_HGAIN_BANK");
        h->ydot_gain         =     param_find("AAH_HGAIN_YDOT");
        h->path_no              =     param_find("AAH_PATH_NO");

        // Climb 1 Trims
        h->climb1_throttle        =     param_find("AAH_CLMB1_THRO");
        h->climb1_pitch         =     param_find("AAH_CLMB1_PITC");
        h->climb2_pitch         =     param_find("AAH_CLMB2_PITC");

        // Descent 1 trims
        h->descent1_throttle        =     param_find("AAH_DSC1_THRO");
        h->descent1_pitch         =     param_find("AAH_DSC1_PITC");

        // Desired heading and altitude
        h->desired_heading        =     param_find("AAH_DES_HDG");
        h->desired_altitude         =     param_find("AAH_DES_ALT");
        h->desired_velocity         =     param_find("AAH_DES_VEL");

        // Control surface gains
        h->gain_aileron         =     param_find("AAH_GAIN_AILE");
        h->gain_elevator        =     param_find("AAH_GAIN_ELEV");
        h->gain_rudder        =     param_find("AAH_GAIN_RUDD");

        h->max_bank         =     param_find("AAH_MAX_BANK");
        h->waypoint_radius         =     param_find("AAH_WAYPNT_RAD");

        h->redzone         =     param_find("AAH_REDZONE");

	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->example_high_param, &(p->example_high_param));

	// TODO: add the above line for each of your custom parameters.....
        
        // Trims for maintaining level flight
        param_get(h->trim_pitch, &(p->trim_pitch));
        param_get(h->trim_bank, &(p->trim_bank));
        param_get(h->trim_throttle, &(p->trim_throttle));
        param_get(h->trim_rudder, &(p->trim_rudder));
        param_get(h->trim_elevator, &(p->trim_elevator)); 
        param_get(h->trim_aileron, &(p->trim_aileron)); 

        // Set the motor to run or not
        param_get(h->motor_active, &(p->motor_active)); 
        
        // Gains for maintaining altitude
        param_get(h->alt_gain_pitch, &(p->alt_gain_pitch));
        param_get(h->gain_throttle, &(p->gain_throttle));

        // Gains for maintaining heading
        param_get(h->hdg_gain_rudd, &(p->hdg_gain_rudd));
        param_get(h->hdg_gain_bank, &(p->hdg_gain_bank));
        param_get(h->hdg_gain_head, &(p->hdg_gain_head));
        param_get(h->ydot_gain, &(p->ydot_gain));

        // Climb 1 Trims
        param_get(h->climb1_throttle, &(p->climb1_throttle));
        param_get(h->climb1_pitch, &(p->climb1_pitch));
        param_get(h->climb2_pitch, &(p->climb2_pitch));

        // Descent 1 trims
        param_get(h->descent1_throttle, &(p->descent1_throttle));
        param_get(h->descent1_pitch, &(p->descent1_pitch));

        // Desired heading and altitude
        param_get(h->desired_heading, &(p->desired_heading));
        param_get(h->desired_altitude, &(p->desired_altitude));
        param_get(h->desired_velocity, &(p->desired_velocity));
        param_get(h->path_no, &(p->path_no));

        // Control surface gains
        param_get(h->gain_aileron, &(p->gain_aileron));
        param_get(h->gain_elevator, &(p->gain_elevator));
        param_get(h->gain_rudder, &(p->gain_rudder));

        param_get(h->max_bank, &(p->max_bank));
        param_get(h->waypoint_radius, &(p->waypoint_radius));

        param_get(h->redzone, &(p->redzone));

	return OK;
}
