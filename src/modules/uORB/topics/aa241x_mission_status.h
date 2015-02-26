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
};



#endif /* TOPIC_AA241X_MISSION_STATUS_H_ */
