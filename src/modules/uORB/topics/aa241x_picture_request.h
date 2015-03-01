/*
 * aa241x_picture.h
 *
 *  Created on: Feb 28, 2015
 *      Author: Adrien
 */

#ifndef TOPIC_AA241X_PICTURE_REQUEST_H_
#define TOPIC_AA241X_PICTURE_REQUEST_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

struct picture_request_s {
	uint64_t time_us;	/**< the current unix time in microseconds */
	float pos_N;		/**< North position of the requested picture */
	float pos_E;		/**< East position of the requested picture */
	float pos_D;		/**< Down position of the requested picture */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_picture_request);


#endif /* TOPIC_AA241X_PICTURE_REQUEST_H_ */
