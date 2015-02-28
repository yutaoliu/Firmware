/*
 * aa241x_water_drop.h
 *
 *  Created on: Feb 25, 2015
 *      Author: adrienp
 */

#ifndef TOPIC_AA241X_WATER_DROP_H_
#define TOPIC_AA241X_WATER_DROP_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

struct aa241x_water_drop_s {
	uint64_t time_us;	/**< the current unix time in microseconds */
	bool success;		/**< if true, successfully dropped water */
	int i;				/**< i grid coordinate of the water drop location */
	int j;				/**< j grid coordinate of the water drop location */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_water_drop);


#endif /* TOPIC_AA241X_WATER_DROP_H_ */
