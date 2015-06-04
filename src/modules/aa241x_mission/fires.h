/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fires.h
 *
 * Contains list of mission initialization parameters for AA241x.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */
#ifndef FIRES_H_
#define FIRES_H_


const int NUM_FIRES = 40;
const int NUM_STARTS = 2;

int8_t fire_wind_dir[NUM_FIRES] = {0, 2, 0, 5, 3, 3, 0, 6, 1, 1, 4, 4, 2, 2, 4, 6, 1, 7, 4, 3,
								   7, 1, 6, 1, 3, 1, 0, 7, 3, 6, 1, 0, 7, 5, 3, 0, 2, 4, 0, 4};

int8_t i_start[NUM_FIRES][NUM_STARTS] = {{8,-1}, {10,6}, {11,-1}, {5,10},  {7,3},  {6,-1}, {12,9}, {11,8}, {9,10}, {10,-1}, {3,12}, {5,11}, {12,6}, {2,11},  {4,-1},  {9,10}, {4,11},  {14,5}, {4,-1}, {6,12},
										 {7,-1}, {15,6}, {11,-1}, {15,11}, {4,10}, {10,-1}, {14,8}, {10,10}, {7,10}, {7,-1}, {4,10}, {8,8}, {9,8}, {1,10}, {3,-1}, {10,8}, {9,2}, {7,5}, {12,-1}, {7,3}};
int8_t j_start[NUM_FIRES][NUM_STARTS] = {{7,-1},  {4,5},  {5,-1}, {8,12}, {1,12}, {10,-1}, {8,13}, {11,3},  {9,3},  {6,-1},  {8,4},  {6,5},  {2,5}, {4,10}, {13,-1}, {14,10},  {7,9}, {10,10}, {6,-1},  {4,8},
										 {11,-1}, {9,8},  {9,-1}, {11,1}, {5,1},  {2,-1},  {9,10}, {14,8}, {3,12}, {7,-1}, {5,8}, {7,5}, {3,11}, {7,13}, {4,-1}, {13,5}, {5,4}, {15,3}, {11,-1}, {10,10}};

unsigned int seed_start[NUM_FIRES] = {20812, 26844, 32283, 15945, 25873, 9120, 25882, 17994,  6670, 9192, 26242, 29892, 18164, 26925, 1454, 29271, 19315,  8415, 28292, 12492,
									  32594,   573,  6921,  8015, 11699, 1268, 28483,  7354, 18023, 3610,  3732, 27723,   245,  6185, 9448, 14985,  3269, 20880,  5909, 8280};


#endif /* FIRES_H_ */
