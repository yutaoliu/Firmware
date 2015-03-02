/*
 * aa241x_new_fire.h
 *
 *  Created on: Feb 25, 2015
 *      Author: adrienp
 */

#ifndef TOPIC_AA241X_NEW_FIRE_H_
#define TOPIC_AA241X_NEW_FIRE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
// #include <vector>
#include "../uORB.h"

struct aa241x_new_fire_s {
	uint64_t mission_time;		/**< the current mission time in microseconds */
	int num_new;
	int *i_new;
	int *j_new;

	// std::vector<int> i_new;		/**< list of i coords of the new cells */
	// std::vector<int> j_new;		/**< list of the j coords of the new cells */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_new_fire);

#endif /* TOPIC_AA241X_NEW_FIRE_H_ */
