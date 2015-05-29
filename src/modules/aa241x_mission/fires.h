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


const int NUM_FIRES = 8;
const int NUM_STARTS = 2;

int8_t fire_wind_dir[NUM_FIRES] = {0, 1, 2, 3, 4, 5, 6, 7};

int8_t i_start[NUM_FIRES][NUM_STARTS] = {{15, -1}, {15, 8}, {8, -1}, {3, 8}, {2, -1}, {2, 6}, {8, 4}, {12, -1}};
int8_t j_start[NUM_FIRES][NUM_STARTS] = {{8, -1}, {8, 3}, {3, -1}, {3, 9}, {8, -1}, {12, 8}, {15, 8}, {15, -1}};

unsigned int seed_start[NUM_FIRES] = {100, 101, 102, 103, 104, 105, 106, 107};


#endif /* FIRES_H_ */
