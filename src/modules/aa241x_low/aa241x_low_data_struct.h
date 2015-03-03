/*
 * aa241x_low_data_struct.h
 *
 *  Created on: Mar 2, 2015
 *      Author: adrienp
 */

#ifndef AA241X_LOW_DATA_STRUCT_H_
#define AA241X_LOW_DATA_STRUCT_H_

/**
 * This is the structure that will contain the data to be sent
 * from the low priority thread to the high priority thread.
 *
 * NOTE: do not edit the define name (LOW_FIELD#), but please
 * do change variable_name# to be your desired variable names.
 *
 * NOTE: any fields you add to this structure will NOT be logged
 * or sent to the ground station, therefore it is recommended
 * you DO NOT ADD FIELDS TO THIS STRUCT.
 *
 * You may also change the data type of fields to one of the following:
 * boolean, int (of any size and type)
 * Though note that all the data will be logged as a float.
 *
 */

// list of variable names
#define LOW_FIELD1 variable_name1;		/**< change variable_name1 to the desired variable name */
#define LOW_FIELD2 variable_name2;
#define LOW_FIELD3 variable_name3;
#define LOW_FIELD4 variable_name4;
#define LOW_FIELD5 variable_name5;
#define LOW_FIELD6 variable_name6;
#define LOW_FIELD7 variable_name7;
#define LOW_FIELD8 variable_name8;
#define LOW_FIELD9 variable_name9;
#define LOW_FIELD10 variable_name10;
#define LOW_FIELD11 variable_name11;
#define LOW_FIELD12 variable_name12;
#define LOW_FIELD13 variable_name13;
#define LOW_FIELD14 variable_name14;
#define LOW_FIELD15 variable_name15;
#define LOW_FIELD16 variable_name16;

/**
 * This string is a list of labels for each field in the log file.
 *
 * Feel free to edit f## to the desired name, but please note length of each
 * label must be less than 3 char per label!!!!
 *
 * THIS STRING MUST BE < 64 CHARS LONG AND CONTAIN 16 LABELS, IF IT DOES NOT
 * MEET THESE REQUIREMENTS, LOGGING WILL FAIL!!!!!!
 *
 */
#define LOW_DATA_LABELS "f01,f02,f03,f04,f05,f06,f07,f08,f09,f10,f11,f12,f13,f14,f15,f16"

struct aa241x_low_data_s {
	float LOW_FIELD1;
	float LOW_FIELD2;
	float LOW_FIELD3;
	float LOW_FIELD4;
	float LOW_FIELD5;
	float LOW_FIELD6;
	float LOW_FIELD7;
	float LOW_FIELD8;
	float LOW_FIELD9;
	float LOW_FIELD10;
	float LOW_FIELD11;
	float LOW_FIELD12;
	float LOW_FIELD13;
	float LOW_FIELD14;
	float LOW_FIELD15;
	float LOW_FIELD16;
};


#endif /* AA241X_LOW_DATA_STRUCT_H_ */
