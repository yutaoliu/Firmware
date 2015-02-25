/*
 * aa241x_mission_params.c
 *
 *  Created on: Feb 22, 2015
 *      Author: Adrien
 */


#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * Minimum allowed altitude during mission in [ft] above Lake Lag.
 * Also acts at the mission start threshold altitude.
 *
 * @unit feet
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_MIN_ALT, 100.0f);

/**
 * Maximum allowed altitude during mission  in [ft] above Lake Lag.
 *
 * @unit feet
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_MAX_ALT, 400.0f);

/**
 * Maximum allowed radial distance in [ft] from the center of the lake.
 *
 * @unit ft
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_MAX_RAD, 300.0f);

/**
 * Altitude in [ft] by which autopilot needs to be engaged in order
 * to be eligible for a mission start.
 *
 * @unit feet
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_AUTO_ALT, 50.0f);

/**
 * Total duration in [mins] that the fire will spread for once
 * mission clock starts.
 *
 * @unit minutes
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_DURATION, 8.0f);

/**
 * Timestep in [sec] for each propagation of the fire.
 *
 * @unit seconds
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_TSTEP, 10.0f);

/**
 * Standard deviation for fire propagation.
 *
 * @group AA241x Mission
 */
PARAM_DEFINE_FLOAT(AAMIS_STD, 0.35f);

/**
 * Index (0 based) of the mission to be executed.
 *
 * @min 0
 * @group AA241x Mission
 */
PARAM_DEFINE_INT32(AAMIS_INDEX, 0);
