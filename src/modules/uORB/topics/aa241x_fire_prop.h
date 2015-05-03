/*
 * aa241x_fire_prop.h
 *
 *  Created on: Apr 22, 2015
 *      Author: Adrien
 */

#ifndef AA241X_FIRE_PROP_H_
#define AA241X_FIRE_PROP_H_


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../uORB.h"

struct aa241x_fire_prop_s {
	uint64_t time_us;		/**< the current clock time in microseconds */
	int props_remaining;	/**< how many propagations remain */
	int num_new;			/**< the number of new fires there are */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_fire_prop);

#endif /* AA241X_FIRE_PROP_H_ */
