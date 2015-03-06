/*
 * aa241x_local_data.h
 *
 *  Created on: Mar 5, 2015
 *      Author: Adrien
 */

#ifndef TOPIC_AA241X_LOCAL_DATA_H_
#define TOPIC_AA241X_LOCAL_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

struct aa241x_local_data_s {
	float N;			/**< local North coordinate (from center of lake) */
	float E;			/**< local East coordinate (from center of lake) */
	float D_baro;		/**< local Down coordinate (from center of lake) using barometer */
	float D_gps;		/**< local Down coordinate (from center of lake) using gps */
	float body_u;		/**< Body u velocity */
	float body_v;		/**< Body v velocity */
	float body_w;		/**< Body w velocity */
	float ground_speed;	/**< the ground speed in m/s */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_local_data);


#endif /* TOPIC_AA241X_LOCAL_DATA_H_ */
