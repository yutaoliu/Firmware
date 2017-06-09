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
 *  @author Chonnuttida Koracharkornradt<kchonnut@stanford.edu>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>

using namespace aa241x_low;
const float g = 9.8f;
const float rho = 1.225f;
const float rMin = 15.0f;
const float PI =3.14159265358979f;

Target currTarget;
Target prevTarget;
std::vector<Target> targetList;
std::vector<Target> temp_targetList;
int currTargetIndex = 0;
int currPhase = 0;

/*std::vector<float> distances;
Aircraft aircraft = Aircraft(0.87, 0.8, 6.0, 0.103, 10.0, 15.0);*/

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

void fillTargetList() {
    // Target(East, North, Radius)
    targetList.clear();
    temp_targetList.clear();
    if ((int) high_data.field3 == 9 || (int) high_data.field3 == 15 || currPhase == 4 || mission_failed == true) {
        Target target1;
        Target target2;
        Target target3;
        Target target4;
        target1.N = high_data.field4;
        target1.E = high_data.field5 + aal_parameters.distance;
        target1.radius = aal_parameters.targetBoundary;
        target2.N = high_data.field4 + aal_parameters.distance;
        target2.E = high_data.field5 + aal_parameters.distance;
        target2.radius = aal_parameters.targetBoundary;
        target3.N = high_data.field4 + aal_parameters.distance;
        target3.E = high_data.field5;
        target3.radius = aal_parameters.targetBoundary;
        target4.N = high_data.field4;
        target4.E = high_data.field5;
        target4.radius = aal_parameters.targetBoundary;
        targetList.push_back(target1);
        targetList.push_back(target2);
        targetList.push_back(target3);
        targetList.push_back(target4);
    } else if ((int) high_data.field3 == 10 || (int) high_data.field3 == 16) {
        Target target1;
        Target target2;
        Target target3;
        Target target4;
        target1.N = high_data.field4 + aal_parameters.distance;
        target1.E = high_data.field5;
        target1.radius = aal_parameters.targetBoundary;
        target2.N = high_data.field4 + aal_parameters.distance + (aal_parameters.distance*cosf(15.0f*PI/180.0f));
        target2.E = high_data.field5 + (aal_parameters.distance*sinf(15.0f*PI/180.0f));
        target2.radius = aal_parameters.targetBoundary;
        target3.N = high_data.field4 + aal_parameters.distance + (aal_parameters.distance*cosf(22.5f*PI/180.0f)*2*sinf(52.5f*PI/180.0f));
        target3.E = high_data.field5 + (aal_parameters.distance*cosf(22.5f*PI/180.0f)*2*cosf(52.5f*PI/180.0f));
        target3.radius = aal_parameters.targetBoundary;
        target4.N = high_data.field4;
        target4.E = high_data.field5;
        target4.radius = aal_parameters.targetBoundary;
        targetList.push_back(target1);
        targetList.push_back(target2);
        targetList.push_back(target3);
        targetList.push_back(target4);
    } else { // mission
        // fill in temp_targetList
        for (int i = 0; i < 5; i++) {
            if (plume_radius[i] > 0) { //
                Target target;
                target.N = plume_N[i];
                target.E = plume_E[i];
                target.radius = plume_radius[i];
                //targetList.push_back(target);
                temp_targetList.push_back(target);
            }

        }
        // rearrange targetList --> closest target comes first --> store in targetList
        int numTarget = temp_targetList.size();
        Target start;
        start.N = position_N;
        start.E = position_E;
        for (int i = 0; i < numTarget - 1; i++) {
            float minDistance = 1000000.0f;
            int minIndex = 0;
            for (int j = 0; j < temp_targetList.size() - 1; j++) {
                float distance = sqrt(pow(start.E - temp_targetList[j].E, 2) + pow(start.N - temp_targetList[j].N, 2));
                if (distance < minDistance) {
                    minDistance = distance;
                    minIndex = j;
                }
            }
            targetList.push_back(temp_targetList[minIndex]);
            start.N = temp_targetList[minIndex].N;
            start.E = temp_targetList[minIndex].E;
            temp_targetList.erase(temp_targetList.begin() + minIndex);
        }
    }
}

void computeABC() {
    float theta = atan2(currTarget.E - prevTarget.E, currTarget.N - prevTarget.N);
    float computed_a = cos(theta);
    float computed_b = -sin(theta);
    float computed_c = -(cosf(theta) * prevTarget.E) + (sinf(theta) * prevTarget.N);
    low_data.field1 = theta;
    low_data.field2 = computed_a;
    low_data.field3 = computed_b;
    low_data.field4 = computed_c;
}

void updateCurrentIndex() {
    if (currTargetIndex < targetList.size() - 1) {
        currTargetIndex += 1;
    } else { // currTargetIndex = targetList.size() - 1
        currTargetIndex = 0;
    }
}

bool reachTarget() {
    if (pow(position_E - currTarget.E, 2) + pow(position_N - currTarget.N, 2) < pow(currTarget.radius, 2)) {
        return true;
    }
    return false;
}

bool missTarget() {
    // perpendicular line equation: tan(theta)*x + y - currY - (tan(theta)*currX) = 0
    float theta = atan2(currTarget.E - prevTarget.E, currTarget.N - prevTarget.N);
    float prevTargetValue = (tanf(theta) * prevTarget.E) + prevTarget.N - currTarget.N - (tanf(theta) * currTarget.E);
    float currPositionValue = (tanf(theta) * position_E) + position_N - currTarget.N - (tanf(theta) * currTarget.E);
    if ((prevTargetValue > 0 && currPositionValue > 0) || (prevTargetValue < 0 && currPositionValue < 0)) {
        return false;
    }
    return true;
}

void low_loop()
{
    // Set up after switch to auto mode
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
        currPhase = phase_num;
        fillTargetList();
        prevTarget.N = high_data.field4; // latest position_N before switch to auto mode
        prevTarget.E = high_data.field5; // latest position_E before switch to auto mode
        currTargetIndex = 0;
        currTarget = targetList[0];
        computeABC();
    }

    if (currPhase != phase_num) { // phase change --> time up or get all targets
        currPhase = phase_num;
        fillTargetList();
        currTargetIndex = 0;
        currTarget = targetList[0];
        computeABC();
    } else { // currPhase == phase_num --> still in the same phase
        if (reachTarget() == true || missTarget() == true) { // go to next target
            prevTarget.N = currTarget.N;
            prevTarget.E = currTarget.E;
            updateCurrentIndex();
            currTarget = targetList[currTargetIndex];
            computeABC();
        }
    }

    if (mission_failed == true || currPhase == 4) {
        currPhase = 0;
        fillTargetList();
        prevTarget.N = high_data.field4; // latest position_N before switch to auto mode
        prevTarget.E = high_data.field5; // latest position_E before switch to auto mode
        currTargetIndex = 0;
        currTarget = targetList[0];
        computeABC();
    }

    // logging data
    low_data.field5 = prevTarget.N;
    low_data.field6 = prevTarget.E;
    low_data.field7 = prevTarget.radius;
    low_data.field8 = currTarget.N;
    low_data.field9 = currTarget.E;
    low_data.field10 = currTarget.radius;
    low_data.field11 = currTargetIndex;
    low_data.field12 = aal_parameters.distance;
    low_data.field13 = aal_parameters.targetBoundary;
    low_data.field14 = currPhase;
    low_data.field15 = mission_failed;
    low_data.field16 = num_plumes_found;
}
