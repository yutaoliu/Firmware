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
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Chonnuttida Koracharkornradt<kchonnut@stanford.edu>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// high_data
// field1 = distance to the line

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;
float speed_desired = 10.0f;
const float PI =3.14159265358979f;
float input_c = 0.0f;
float lastThrottleSetting = 0.0f;

/*
 * Do bounds checking to keep the roll/pitch/yaw/throttle correction within the -1..1 limits of the servo output
 */
float bound_checking(float correction_value) {
    if (correction_value > 1.0f) {
        correction_value = 1.0f;
    } else if (correction_value < -1.0f) {
        correction_value = -1.0f;
    }
    return correction_value;
}

float wrap_to_pi(float correction_value) {
    while (correction_value > PI) {
        correction_value -= 2*PI;
    }
    while (correction_value < -PI) {
        correction_value += 2*PI;
    }
    return correction_value;
}

float roll_control() {
    float proportionalRollCorrection = aah_parameters.proportional_roll_gain * (roll_desired - roll) + aah_parameters.roll_trim;
    return bound_checking(proportionalRollCorrection);
}

float pitch_control() {
    float proportionalPitchCorrection = aah_parameters.proportional_pitch_gain * (pitch_desired - pitch) + aah_parameters.pitch_trim;
    return bound_checking(proportionalPitchCorrection);
}

float altitude_control() {
    pitch_desired = aah_parameters.proportional_altitude_gain * (altitude_desired - (-position_D_gps));
    return pitch_control();
}

float yaw_control() {
    float proportionalYawCorrection = aah_parameters.proportional_yaw_gain * (yaw_desired - yaw) + aah_parameters.yaw_trim;
    return bound_checking(proportionalYawCorrection);
}

float throttle_control() {
    float proportionalSpeedCorrection = aah_parameters.proportional_throttle_gain * (speed_desired - speed_body_u) + aah_parameters.throttle_trim;
    return bound_checking(proportionalSpeedCorrection);
}


float heading_control_roll_input_desired_heading() {
    float yaw_target = (aah_parameters.input_heading_angle_deg * PI / 180);
    roll_desired = aah_parameters.proportional_heading_gain * wrap_to_pi(yaw_target - yaw);
    return roll_control();
}

// banking angle = 1 --> turn right
// bangking angle = -1 --> turn left
float coordinated_turn() {
    roll_desired = aah_parameters.banking_angle;
    return roll_control();
}

float line_acquisition_straightline() {
    // compute distance to a line defined by (ax + by + c = 0)
    float distance = -(aah_parameters.a * position_E + aah_parameters.b * position_N + input_c);
    float line_heading = atan2(aah_parameters.b,aah_parameters.a);
    float yaw_target = line_heading + (PI/2 * bound_checking((aah_parameters.proportional_dist_gain * distance)/PI*2));
    roll_desired = aah_parameters.proportional_heading_gain * wrap_to_pi(yaw_target - yaw);
    // logging data
    high_data.field1 = distance;
    high_data.field2 = roll_desired;
    high_data.field6 = aah_parameters.delta_E;
    high_data.field7 = aah_parameters.proportional_dist_gain;
    high_data.field8 = aah_parameters.proportional_heading_gain;
    high_data.field9 = aah_parameters.delta_N;
    return roll_control();
}

float line_acquisition_transitions() {
    float theta = low_data.field1;
    float a = low_data.field2;
    float b = low_data.field3;
    float c = low_data.field4;
    float distance = -(a * position_E + b * position_N + c);
    float yaw_target = theta + (PI/2 * bound_checking((aah_parameters.proportional_dist_gain * distance)/PI*2));
    roll_desired = aah_parameters.proportional_heading_gain * wrap_to_pi(yaw_target - yaw);
    // logging data
    high_data.field1 = distance;
    high_data.field2 = roll_desired;
    high_data.field7 = aah_parameters.proportional_dist_gain;
    high_data.field8 = aah_parameters.proportional_heading_gain;

    return roll_control();
}

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {


        // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
        if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                yaw_desired = yaw; 				// yaw_desired already defined in aa241x_high_aux.h
                roll_desired = roll;
                pitch_desired = pitch;
                altitude_desired = -position_D_gps; 		// altitude_desired needs to be declared outside flight_control() function
                speed_desired = aah_parameters.input_speed;
                high_data.field4 = position_N;
                high_data.field5 = position_E;
                lastThrottleSetting = man_throttle_in;
        }

        switch (aah_parameters.caseNum) {
        // auto roll only by using roll_control
        case 0:
            roll_servo_out = roll_control();
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 0;
            break;
        // auto pitch only by using pitch_control
        case 1:
            roll_servo_out = man_roll_in;
            pitch_servo_out = pitch_control();
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 1;
            break;
        // auto pitch only by using altitude_control
        case 2:
            roll_servo_out = man_roll_in;
            pitch_servo_out = altitude_control();
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 2;
            break;
        // auto roll, pitch, yaw by using roll_control, altitude_control, yaw_control, manual throttle
        case 3:
            roll_servo_out = roll_control();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 3;
            break;
        // full auto by using roll_control, altitude_control, yaw_control
        case 4:
            roll_servo_out = roll_control();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 4;
            break;
        // full auto by using roll = coordinated_turn
        case 5:
            roll_servo_out = coordinated_turn();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 5;
            break;
        // full auto by using roll = heading_control_roll_input_desired_heading()
        case 6:
            roll_servo_out = heading_control_roll_input_desired_heading();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 6;
            break;
        // full auto by using roll = line_acquisition_straightline East
        case 7:
            input_c = -(high_data.field5 + aah_parameters.delta_E);
            roll_servo_out = line_acquisition_straightline();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 7;
            break;
        // full auto by using roll = line_acquisition_straightline North
        case 8:
            input_c = -(high_data.field4 + aah_parameters.delta_N);
            roll_servo_out = line_acquisition_straightline();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 8;
            break;
        // full auto by using roll = line_acquisition_transitions rectangle
        case 9:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 9;
            break;
        // full auto by using roll = line_acquisition_transitions (turn 15 and 45 degree)
        case 10:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 10;
            break;
        // BACONNNNN - Manual throttle
        // roll = coordinated_turn, manual throttle
        case 11:
            roll_servo_out = coordinated_turn();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 11;
            break;
        // roll = heading_control_roll_input_desired_heading(), manual throttle
        case 12:
            roll_servo_out = heading_control_roll_input_desired_heading();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 12;
            break;
        // full auto by using roll = line_acquisition_straightline East
        case 13:
            input_c = -(high_data.field5 + aah_parameters.delta_E);
            roll_servo_out = line_acquisition_straightline();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 13;
            break;
        // full auto by using roll = line_acquisition_straightline North
        case 14:
            input_c = -(high_data.field4 + aah_parameters.delta_N);
            roll_servo_out = line_acquisition_straightline();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 14;
            break;
        // full auto by using roll = line_acquisition_transitions rectangle
        case 15:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 15;
            break;
        // full auto by using roll = line_acquisition_transitions (turn 15 and 45 degree)
        case 16:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            high_data.field3 = 16;
            break;
        // MISSION
        // Bixler
        case 17:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            high_data.field3 = 17;
            /*if (mission_failed == true) {
                roll_desired = -0.5;
                roll_servo_out = roll_control();
            }*/
            break;
        // Bacon
        case 18:
            roll_servo_out = line_acquisition_transitions();
            pitch_servo_out = altitude_control();
            yaw_servo_out = yaw_control();
            //throttle_servo_out = man_throttle_in;
            throttle_servo_out = lastThrottleSetting;
            high_data.field3 = 18;
            /*if (mission_failed == true) {
                roll_desired = -0.5;
                roll_servo_out = roll_control();
            }*/
            break;
        // full manual
        default:
            roll_servo_out = man_roll_in;
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
        }

}
