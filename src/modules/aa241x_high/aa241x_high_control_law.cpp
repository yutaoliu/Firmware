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
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define global variables (can be seen by all files in aa241x_high directory unless static keyword used)
float altitude_desired = 0.0f;
float speed_desired = 0.0f;

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

float roll_control() {
    float proportionalRollCorrection = aah_parameters.proportional_roll_gain * (roll_desired - roll);
    return bound_checking(proportionalRollCorrection);
}

float pitch_control() {
    float proportionalPitchCorrection = aah_parameters.proportional_pitch_gain * (pitch_desired - pitch);
    return bound_checking(proportionalPitchCorrection);
}

float altitude_control() {
    pitch_desired = aah_parameters.proportional_altitude_gain * (altitude_desired - (-position_D_baro));
    return pitch_control();
}

float yaw_control() {
    float proportionalYawCorrection = aah_parameters.proportional_yaw_gain * (yaw_desired - yaw);
    return bound_checking(proportionalYawCorrection);
}

float throttle_control() {
    float proportionalSpeedCorrection = aah_parameters.proportional_throttle_gain * (speed_desired - speed_body_u);
    return bound_checking(proportionalSpeedCorrection);
}

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

        float my_float_variable = 0.0f;		/**< example float variable */


        // An example of how to run a one time 'setup' for example to lock one's altitude and heading...
        if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop,
                yaw_desired = yaw; 				// yaw_desired already defined in aa241x_high_aux.h
                roll_desired = roll;
                pitch_desired = pitch;
                altitude_desired = -position_D_baro; 		// altitude_desired needs to be declared outside flight_control() function
                speed_desired = speed_body_u;
        }

        // TODO: write all of your flight control here...


        // getting low data value example
        // float my_low_data = low_data.field1;

        // setting high data value example
        high_data.field1 = my_float_variable;

        // Set servo output
        switch (aah_parameters.caseNum) {
        // auto roll only
        case 0:
            roll_servo_out = roll_control();
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
            break;
        // auto pitch only
        case 1:
            roll_servo_out = man_roll_in;
            pitch_servo_out = pitch_control();
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
            break;
        // auto yaw only
        case 2:
            roll_servo_out = man_roll_in;
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            break;
        // auto speed only
        case 3:
            roll_servo_out = man_roll_in;
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = throttle_control();
            break;
        // auto roll, pitch, yaw
        case 4:
            roll_servo_out = roll_control();
            pitch_servo_out = pitch_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = man_throttle_in;
            break;
        // full auto
        case 5:
            roll_servo_out = roll_control();
            pitch_servo_out = pitch_control();
            yaw_servo_out = yaw_control();
            throttle_servo_out = throttle_control();
            break;
        // full manual
        default:
            roll_servo_out = man_roll_in;
            pitch_servo_out = -man_pitch_in;
            yaw_servo_out = man_yaw_in;
            throttle_servo_out = man_throttle_in;
        }

        /*
        // // Make a really simple proportional roll stabilizer // //
        //

        roll_desired = 0.0f; // roll_desired already exists in aa241x_high_aux so no need to repeat float declaration

        // Now use your parameter gain and multiply by the error from desired
        float proportionalRollCorrection = aah_parameters.proportional_roll_gain * (roll - roll_desired);

        // Note the use of x.0f, this is important to specify that these are single and not double float values!

        // Do bounds checking to keep the roll correction within the -1..1 limits of the servo output
        if (proportionalRollCorrection > 1.0f) {
                proportionalRollCorrection = 1.0f;
        } else if (proportionalRollCorrection < -1.0f ) {
                proportionalRollCorrection = -1.0f;
        }

        // ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
        // Set output of roll servo to the control law output calculated above
        roll_servo_out = proportionalRollCorrection;
        // as an example, just passing through manual control to everything but roll
        pitch_servo_out = man_pitch_in; // had to be flipped for our controller
        yaw_servo_out = man_yaw_in;
        throttle_servo_out = man_throttle_in;*/
}
// Test comment
