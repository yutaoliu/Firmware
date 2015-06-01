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


const int NUM_FIRES = 20;
const int NUM_STARTS = 2;

int8_t fire_wind_dir[NUM_FIRES] = {0, 2, 0, 5, 3, 3, 0, 6, 1, 1, 4, 4, 2, 2, 4, 6, 1, 7, 4, 3};

int8_t i_start[NUM_FIRES][NUM_STARTS] = {{8,-1}, {10,6}, {11,-1}, {5,10}, {6,12},  {6,-1}, {12,9}, {11,8}, {9,10}, {10,-1}, {3,12}, {5,11}, {12,6}, {2,11},  {4,-1},  {9,10}, {4,11},  {14,5}, {4,-1}, {7,3}};
int8_t j_start[NUM_FIRES][NUM_STARTS] = {{7,-1},  {4,5},  {5,-1}, {8,12},  {4,8}, {10,-1}, {8,13}, {11,3},  {9,3},  {6,-1},  {8,4},  {6,5},  {2,5}, {4,10}, {13,-1}, {14,10},  {7,9}, {10,10}, {6,-1}, {1,12}};

unsigned int seed_start[NUM_FIRES] = {20812, 26844, 32283, 15945, 12492, 9120, 25882, 17994, 6670, 9192, 26242, 29892, 18164, 26925, 1454, 29271, 19315, 8415, 28292, 25873};


#endif /* FIRES_H_ */
