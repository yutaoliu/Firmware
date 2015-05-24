/*
 * aa241x_condensed_grid.h
 *
 *  Created on: May 16, 2015
 *      Author: adrienp
 */

#ifndef SRC_MODULES_UORB_TOPICS_AA241X_CONDENSED_GRID_H_
#define SRC_MODULES_UORB_TOPICS_AA241X_CONDENSED_GRID_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../uORB.h"

struct aa241x_cgrid_s {
	uint64_t time_us;		/**< the current clock time in microseconds */
	uint32_t cells[12];		/**< the status of each of the cells within bounds */
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_cgrid);




#endif /* SRC_MODULES_UORB_TOPICS_AA241X_CONDENSED_GRID_H_ */
