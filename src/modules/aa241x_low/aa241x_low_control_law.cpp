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
const float g = 9.8f;
const float rho = 1.225f;
const float rMin = 15.0f;
const float PI =3.14159265358979f;

Target currTarget;
Target prevTarget;
std::vector<Target> targetList;
int currTargetIndex = 0;
float startTime = 0;

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

// Fill in the targetList
// If already reach target5, call this function again???
void fillTargetList() {
    // Target(East, North, Radius)
    targetList.clear();
    Target target1;
    Target target2;
    Target target3;
    Target target4;
    if ((int) high_data.field3 == 9 || (int) high_data.field3 == 18 || (int) high_data.field3 == 26 || (int) high_data.field3 == 28) {
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
    } else if ((int) high_data.field3 == 11 || (int) high_data.field3 == 20) {
        target1.N = -2400.0;
        target1.E = 1900.0;
        target1.radius = aal_parameters.targetBoundary;
        target2.N = -2500.0;
        target2.E = 1900.0;
        target2.radius = aal_parameters.targetBoundary;
        target3.N = -2450.0;
        target3.E = 1975.0;
        target3.radius = aal_parameters.targetBoundary;
        target4.N = -2450.0;
        target4.E = 1850.0;
        target4.radius = aal_parameters.targetBoundary;
    } else {
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
    }
    targetList.push_back(target1);
    targetList.push_back(target2);
    targetList.push_back(target3);
    targetList.push_back(target4);
}

void computeABC() {
    float theta = atan2(currTarget.E - prevTarget.E, currTarget.N - prevTarget.N);
    float computed_a = cos(theta);
    float computed_b = sin(theta);
    float computed_c = -(computed_a * prevTarget.E + computed_b * prevTarget.N);
    if ((int) high_data.field3 == 26 || (int) high_data.field3 == 27 || (int) high_data.field3 == 28 || (int) high_data.field3 == 29) {
        computed_b = -sin(theta);
        computed_c = -(cosf(theta) * prevTarget.E) + (sinf(theta) * prevTarget.N);
    }
    low_data.field1 = theta;
    low_data.field2 = computed_a;
    low_data.field3 = computed_b;
    low_data.field4 = computed_c;
}

void updateCurrentIndex() {
    if (currTargetIndex < targetList.size() - 1) {
        currTargetIndex += 1;
    } else { // currTargetIndex = targetList.size() - 1
        fillTargetList(); // new set of targets
        currTargetIndex = 0;
    }
}

bool reachTarget() {
    if (abs(position_E - currTarget.E) <= aal_parameters.targetBoundary && abs(position_N - currTarget.N) <= aal_parameters.targetBoundary) {
        // remove reached target from targetList
        // targetList.erase(currTargetIndex);
        // if targetList has no element left, fill the targetList with new set of 5 targets
        // if (targetList.size() == 0) resetEverything;
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

void findClosestTarget() {
    // Find the closest target to the current plane's position
    float minLength = 1000000; // Just put in some large number
    int minIndex = 0;
    // Coordinate(East, North)
    Coordinate A = Coordinate(position_E - vel_E, position_N - vel_N); // A = position - velocity
    Coordinate B = Coordinate(position_E, position_N); // B = position
    Coordinate D; // D = target
    for (int i = 0; i < targetList.size(); i++) {
        D = Coordinate(targetList[i].E, targetList[i].N);
        float length = exitPath(A, B, D, false);
        // update minLength and minIndex
        if (length < minLength) {
            minLength = length;
            minIndex = i;
        }
    }
    // update currTargetIndex, currTarget = the closest target
    currTargetIndex = minIndex;
    currTarget = targetList[currTargetIndex];
    // update prevTarget = tangent point on the circle to currTarget = C
    exitPath(A, B, Coordinate(currTarget.E, currTarget.N), true);
}

// A = position - velocity
// B = position
// D = target
// C = tangent point
float exitPath(Coordinate A, Coordinate B, Coordinate D, bool isUpdatePrevTarget) {
    // Translation
    Coordinate ATranslate = Coordinate(A.E - B.E, A.N - B.N);
    Coordinate BTranslate = Coordinate(0, 0);
    Coordinate DTranslate = Coordinate(D.E - B.E, D.N - B.N);
    // Rotation
    Coordinate AB = Coordinate (BTranslate.E - ATranslate.E, BTranslate.N - ATranslate.N);
    float normAB = sqrt(pow(AB.E, 2) + pow(AB.N, 2));
    float cost = AB.E / normAB;
    float sint = AB.N / normAB;
    //Coordinate ARotate = Coordinate((sint*ATranslate.E) + (-cost*ATranslate.N), (cost*ATranslate.E) + (sint*ATranslate.N));
    //Coordinate BRotate = Coordinate((sint*BTranslate.E) + (-cost*BTranslate.N), (cost*BTranslate.E) + (sint*BTranslate.N));
    Coordinate DRotate = Coordinate((sint*DTranslate.E) + (-cost*DTranslate.N), (cost*DTranslate.E) + (sint*DTranslate.N));
    // Compute length and C (tangent point)
    float beta = 1.0;
    Coordinate CRotate;
    if ((DRotate.E > 0 && pow(DRotate.E - rMin, 2) + pow(DRotate.N, 2) > pow(rMin, 2))
            || (DRotate.E < 0 && pow(DRotate.E + rMin, 2) + pow(DRotate.N, 2) < pow(rMin, 2))) {
        // right turn
        // system of equations --> get beta
        beta = 1.0;
        CRotate = Coordinate(rMin - rMin*cosf(beta), rMin*sinf(beta));
    } else {
        // left turn
        // system of equations --> get beta
        beta = 1.0;
        CRotate = Coordinate(-rMin + rMin*cosf(beta), rMin*sinf(beta));
    }
    if (isUpdatePrevTarget) {
        Coordinate C;
        // C = Rot.'*CRotate + B
        prevTarget.N = C.N;
        prevTarget.E = C.E;
    }
    float normDC = sqrt(pow(DRotate.E - CRotate.E, 2) + pow(DRotate.N - CRotate.N, 2));
    float length = normDC + rMin*beta;
    return length;
}

void low_loop()
{
    // Set up after switch to auto mode
    if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
        fillTargetList();
        prevTarget.N = high_data.field4; // latest position_N before switch to auto mode
        prevTarget.E = high_data.field5; // latest position_E before switch to auto mode
        currTargetIndex = 0;
        currTarget = targetList[0];
        // update currTargetIndex, currTarget, prevTarget --> replace the above 4 lines with findClosestTarget()
        // findClosestTarget();
        low_data.field5 = prevTarget.N;
        low_data.field6 = prevTarget.E;
        low_data.field7 = currTarget.N;
        low_data.field8 = currTarget.E;
        low_data.field9 = currTargetIndex;
        low_data.field10 = aal_parameters.distance;
        low_data.field11 = aal_parameters.targetBoundary;
        computeABC();
    }

    if ((int) high_data.field3 == 28 || (int) high_data.field3 == 29) {
        if (reachTarget() || missTarget()) {
            prevTarget.N = currTarget.N;
            prevTarget.E = currTarget.E;
            updateCurrentIndex();
            currTarget = targetList[currTargetIndex];
            // update currTargetIndex, currTarget, prevTarget --> replace the above 4 lines with findClosestTarget()
            // findClosestTarget();
            low_data.field5 = prevTarget.N;
            low_data.field6 = prevTarget.E;
            low_data.field7 = currTarget.N;
            low_data.field8 = currTarget.E;
            low_data.field9 = currTargetIndex;
            low_data.field10 = aal_parameters.distance;
            low_data.field11 = aal_parameters.targetBoundary;
            computeABC();
        }
    } else {
        if (reachTarget()) { // if currentTarget has reach
            prevTarget.N = currTarget.N;
            prevTarget.E = currTarget.E;
            updateCurrentIndex();
            currTarget = targetList[currTargetIndex];
            // update currTargetIndex, currTarget, prevTarget --> replace the above 4 lines with findClosestTarget()
            // findClosestTarget();
            low_data.field5 = prevTarget.N;
            low_data.field6 = prevTarget.E;
            low_data.field7 = currTarget.N;
            low_data.field8 = currTarget.E;
            low_data.field9 = currTargetIndex;
            low_data.field10 = aal_parameters.distance;
            low_data.field11 = aal_parameters.targetBoundary;
            computeABC();
        }
    }

    /*if (hrt_absolute_time() - startTime > 31000000.0) { // 31 seconds
        resetEverything;
    }*/

    //distance to target
    float distanceToTarget = sqrt(pow((position_N - currTarget.N), 2) + pow((position_E- currTarget.E), 2));
    low_data.field12 = distanceToTarget;
}
