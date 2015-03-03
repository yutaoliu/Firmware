/*
 * aa241x_high_data.h
 *
 *  Created on: Mar 2, 2015
 *      Author: adrienp
 */

#ifndef TOPIC_AA241X_HIGH_DATA_H_
#define TOPIC_AA241X_HIGH_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

#define HIGH_FIELD1 variable_name1;
#define HIGH_FIELD2 variable_name2;
#define HIGH_FIELD3 variable_name3;
#define HIGH_FIELD4 variable_name4;
#define HIGH_FIELD5 variable_name5;
#define HIGH_FIELD6 variable_name6;
#define HIGH_FIELD7 variable_name7;
#define HIGH_FIELD8 variable_name8;
#define HIGH_FIELD9 variable_name9;
#define HIGH_FIELD10 variable_name10;
#define HIGH_FIELD11 variable_name11;
#define HIGH_FIELD12 variable_name12;

struct aa241x_high_data_s {
	float HIGH_FIELD1;
	float HIGH_FIELD2;
	float HIGH_FIELD3;
	float HIGH_FIELD4;
	float HIGH_FIELD5;
	float HIGH_FIELD6;
	float HIGH_FIELD7;
	float HIGH_FIELD8;
	float HIGH_FIELD9;
	float HIGH_FIELD10;
	float HIGH_FIELD11;
	float HIGH_FIELD12;
};

/* register this as object request broker structure */
ORB_DECLARE(aa241x_high_data);



#endif /* TOPIC_AA241X_HIGH_DATA_H_ */
