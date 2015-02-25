/*
 * fires.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Adrien
 */

#ifndef FIRES_H_
#define FIRES_H_


const int NUM_FIRES = 1;
const int NUM_STARTS = 2;

int8_t fire_wind_dir[NUM_FIRES] = {0};

int8_t i_start[NUM_FIRES][NUM_STARTS] = {{11, -1}};
int8_t j_start[NUM_FIRES][NUM_STARTS] = {{11, -1}};

unsigned int seed_start[NUM_FIRES] = {100};

#endif /* FIRES_H_ */
