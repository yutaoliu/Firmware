/*
 * aa241x_picture.h
 *
 *  Created on: Feb 28, 2015
 *      Author: Adrien
 */

#ifndef TOPIC_AA241X_PICTURE_RESULT_H_
#define TOPIC_AA241X_PICTURE_RESULT_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "../uORB.h"

struct picture_result_s {
	bool pic_taken;			/**< if true, a picture was successfully taken */
	uint64_t time_us;		/**< unix timestamp in microseconds at which the picture was taken */
	float center_n;			/**< North coordinate (North distance from center in [m]) of the center of the picture */
	float center_e;			/**< East coordinate (East distance from center in [m]) of the center of the picture */
	float center_d;			/**< Down coordinate (negative vertical distance from bottom) of the center of the picture */
	float pic_d;			/**< diameter in [m] of the picture taken */
	int num_cells;			/**< number of grid cells in view (length of vectors i, j, and state) */
	int *i;
	int *j;
	int *state;

	// std::vector<int> i;		/**< list of i grid coordinates in view */
	// std::vector<int> j;		/**< list of j grid coordinates in view */
	// std::vector<int> state; /**< state of each grid coordinate in view (-1 = water, 0 = nothing, 1 = on fire) */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_picture_result);


#endif /* TOPIC_AA241X_PICTURE_RESULT_H_ */
