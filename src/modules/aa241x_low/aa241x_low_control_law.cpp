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

/*
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>

using namespace aa241x_low;

Target currTarget;
std::vector<Target> targetList;
int currTargetIndex = 0;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

// Fill in the targetList
// If already reach target5, call this function again???
void fillTargetList() {
    // Target1
    Target target1(aal_parameters.target1_N, aal_parameters.target1_E);
    targetList.push_back(target1);
    // Target2
    Target target2(aal_parameters.target2_N, aal_parameters.target2_E);
    targetList.push_back(target2);
    // Target3
    Target target3(aal_parameters.target3_N, aal_parameters.target3_E);
    targetList.push_back(target3);
    // Target4
    Target target4(aal_parameters.target4_N, aal_parameters.target4_E);
    targetList.push_back(target4);
    // Target5
    Target target5(aal_parameters.target5_N, aal_parameters.target5_E);
    targetList.push_back(target5);
}

void computeABC() {
    float computed_a = 1.0f;
    float computed_b = 0.0f;
    float computed_c = 1.0f + currTarget.N + currTarget.E;
    low_data.field1 = computed_a;
    low_data.field2 = computed_b;
    low_data.field3 = computed_c;
}

void updateCurrentIndex() {
    if (currTargetIndex < 4) {
        currTargetIndex += 1;
    } else { // currTargetIndex = 4
        fillTargetList(); // new set of 5 targets
        currTargetIndex = 0;
    }
}


void low_loop()
{
    /*if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
        fillTargetList();
        currTarget = targetList[0];
        computeABC();
    }*/

    /*if (high_data.field5) { // if currentTarget has reach --> maybe check from high module
        updateCurrentIndex();
        currTarget = targetList[currTargetIndex];
        computeABC();
    }*/
}
