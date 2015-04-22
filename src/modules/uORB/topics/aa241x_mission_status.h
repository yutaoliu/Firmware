/*
 * aa241x_mission_status.h
 *
 *  Created on: Feb 25, 2015
 *      Author: adrienp
 */

#ifndef TOPIC_AA241X_MISSION_STATUS_H_
#define TOPIC_AA241X_MISSION_STATUS_H_


#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

struct aa241x_mission_status_s {
	bool in_mission;		/**< if true, currently running a mission */
	bool can_start;			/**< if false, mission starting conditions violated */
	float mission_time;		/**< time since mission start (in minutes) */
	float score;			/**< current mission score */
	int wind_direction;		/**< the wind direction for this mission */
	int mission_index;		/**< the index of the currently running mission */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_mission_status);

#endif /* TOPIC_AA241X_MISSION_STATUS_H_ */
