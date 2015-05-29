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
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_ALT_MIN, 30.48f);

/**
 * Maximum allowed altitude during mission  in [m] above Lake Lag.
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_ALT_MAX, 121.92f);

/**
 * Maximum allowed radial distance in [m] from the center of the lake.
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_RAD_MAX, 170.0f);

/**
 * Width of a grid cell in [m].
 *
 * @unit meters
 * @min 0
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_CELL_W, 20.0f);


/**
 * Altitude in [m] by which autopilot needs to be engaged in order
 * to be eligible for a mission start.
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_ALT_AUTO, 15.24f);

/**
 * Total duration in [mins] that the fire will spread for once
 * mission clock starts.
 *
 * @unit minutes
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_DURATION, 10.0f);

/**
 * Timestep in [sec] for each propagation of the fire.
 *
 * @unit seconds
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_TSTEP, 15.0f);

/**
 * Standard deviation for fire propagation.
 *
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_STD, 0.3f);

/**
 * Minimum time in [sec] required between successive pictures.
 *
 * @min 0
 * @unit seconds
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_TPIC, 3.0f);

/**
 * Minimum camera field of view diameter in [m].
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_FOV_MIN, 30.0f);

/**
 * Maximum camera field of view diameter in [m].
 *
 * @unit meters
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_FOV_MAX, 60.0f);

/**
 * Grams of water per drop.
 *
 * @unit grams/drop
 * @min 0
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_WGHT_DRP, 10.0f);

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
 * Maximum battery capacity allowed.
 *
 * Maximum battery discharge allowed during the mission.
 *
 * @unit mAh
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_BATT_MAX, 650.0f);

/**
 * The amount of weight in [grams] of "water" being carried by this UAV.
 *
 * @unit grams
 * @min 0
 * @group AA241x Student Params
 */
PARAM_DEFINE_FLOAT(AA_WATER_WGHT, 10000.0f);

/**
 * Index (0 based) of the mission to be executed.
 *
 * @min 0
 * @group AA241x Student Params
 */
PARAM_DEFINE_INT32(AA_MIS_INDEX, 0);

/**
 * Team number (1-4).
 *
 * @group AA241x Student Params
 */
PARAM_DEFINE_INT32(AA_TEAM, 0);
