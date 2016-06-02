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
 * @file aa241x_mission_params.c
 *
 * Definition of parameters needed for the AA241x Spring 2015 mission.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */
#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * Minimum allowed altitude during mission in [m] above Lake Lag.
 * Also acts at the mission start threshold altitude.
 *
 * @unit meters
 * @group AA241x Student Params
 */
PARAM_DEFINE_FLOAT(AA_ALT_MIN, 40.0f);

/**
 * Maximum allowed altitude during mission  in [m] above Lake Lag.
 *
 * @unit meters
 * @group AA241x Student Params
 */
PARAM_DEFINE_FLOAT(AA_ALT_MAX, 60.0f);

/**
 * North location of starting gate pylon
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_SPOS_N, 150.0f);

/**
 * East location of starting gate pylon
 *
 * @unit minutes
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_SPOS_E, -96.0f);

/**
 * Minimum allowed radius from pylon
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_RAD_KPT, 5.0f);

/**
 * Tilt angle of first leg of course relative to East, positive CCW
 *
 * @unit degrees
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_TILT, -25.0f);

/**
 * Length of straight legs
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_LEG_LEN, 243.84f);

/**
 * Width of start/finish gate
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_GTE_WID, 50.0f);

/**
 * Reset the mission parameters. MUST BE SET TO ZERO FOR MISSION TO RUN!
 *
 * @unit none
 * @min 0
 * @max 1
 * @group AA241x Mission
 */
PARAM_DEFINE_INT32(AA_MIS_RESET, 0);

/**
 * Activate/deactivate mission failure. 1 means mission failure kills the current mission,
 * 0 means it allows the mission to continue
 *
 * @unit none
 * @min 0
 * @max 1
 * @group AA241x Mission
 */
PARAM_DEFINE_INT32(AAMIS_MIS_FAIL, 1);

/**
 * Origin latitude for local position.
 *
 * Latitude in decimal degrees for the origin of the local position estimate.
 * A value outside of the min/max will result in using the home location
 * latitude.
 *
 * @min -90.0
 * @max 90.0
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_CTR_LAT, 37.4224444f);

/**
 * Origin longitude for local position.
 *
 * Longitude in decimal degrees for the origin of the local position estimate.
 * A value outside of the min/max will result in using the home location
 * longitude.
 *
 * @min -180.0
 * @max 180.0
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_CTR_LON, -122.1760917f);

/**
 * Origin altitude for local position.
 *
 * Altitude in meters for the origin of the local position estimate.
 * A negative value will result in using the home location altitude.
 *
 * @min 0.0
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_CTR_ALT, 40.0f);

/**
 * Turn debug mode on and off
 *
 *
 * @group AA241x Mission
 */
PARAM_DEFINE_INT32(AAMIS_DEBUG, 0);

/**
 * Force mission to run (regardless of whether you cross the start line)
 *
 *
 * @group AA241x Mission
 */
PARAM_DEFINE_INT32(AAMIS_FSTART, 0);

/**
 * Team number (1-5).
 *
 * @group AA241x Student Params
 */
PARAM_DEFINE_INT32(AA_TEAM, 0);


// TODO: DEFINE ADDITIONAL GLOBAL MISSION PARAMETERS HERE
