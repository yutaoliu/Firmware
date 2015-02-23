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
 */
PARAM_DEFINE_FLOAT(AAMIS_MIN_ALT, 100.0f);

/**
 * Maximum allowed altitude during mission  in [ft] above Lake Lag.
 */
PARAM_DEFINE_FLOAT(AAMIS_MAX_ALT, 400.0f);

/**
 * Altitude in [ft] by which autopilot needs to be engaged in order
 * to be eligible for a mission start.
 */
PARAM_DEFINE_FLOAT(AAMIS_AUTO_ALT, 50.0f);

/**
 * Total duration in [mins] that the fire will spread for once
 * mission clock starts.
 */
PARAM_DEFINE_FLOAT(AAMIS_DURATION, 8.0f);
