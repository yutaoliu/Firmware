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
#include <uORB/topics/aa241x_picture_result.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

int pic_res_sub = 0;
hrt_abstime pic_taken_time = 0;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

	// printf("loop deltaT = %fms\n", (double) (timestamp - previous_loop_timestamp)/1000.0);

	if (new_pic) {

		// printf("New pic received:\n");
		int f_count = 0;
		for (int i = 0; i < pic_result.num_cells; i++) {
			// printf("(%i,%i) -> %i\n", pic_result.i[i], pic_result.j[i], pic_result.state[i]);
			if (pic_result.state[i] == 1) {
				f_count++;
			}
		}

		// just going to want to drop water
		// printf("sending water drop request\n");
		if (f_count > 0) {
			drop_water();
		}

		// TODO: run picture result logic here......

		// set new_pic to false, as just processed this pic result
		new_pic = false;
	}


	// TODO: write all of your flight control here...



	// setting high data values
	high_data.variable_name1 = roll;
	high_data.variable_name2 = pitch;
	high_data.variable_name3 = yaw;

	high_data.variable_name4 = roll_rate;
	high_data.variable_name5 = pitch_rate;
	high_data.variable_name6 = yaw_rate;

	high_data.variable_name7 = speed_body_u;
	high_data.variable_name8 = speed_body_v;
	high_data.variable_name9 = speed_body_w;

	high_data.variable_name10 = low_data.variable_name10;
	high_data.variable_name11 = low_data.variable_name11;
	high_data.variable_name12 = low_data.variable_name12;
	high_data.variable_name13 = low_data.variable_name13;
	high_data.variable_name14 = low_data.variable_name14;
	high_data.variable_name15 = low_data.variable_name15;
	// high_data.variable_name16 = low_data.variable_name16;


	// roll control (stabilize)
	float roll_error = aah_parameters.roll_command - roll;
	float roll_control = aah_parameters.roll_p * roll_error;
	roll_servo_out = math::constrain(man_roll_in + (2.0f/M_PI_F) * roll_control, -1.0f, 1.0f);

	high_data.variable_name16 = roll_control;


	// ENSURE THAT YOU SET THE SERVO OUTPUTS!!!
	// outputs should be set to values between -1..1 (except throttle is 0..1)
	// where zero is no actuation, and -1,1 are full throw in either the + or - directions
	// roll_servo_out = man_roll_in;		// as an example, just passing through manual control
	pitch_servo_out = -man_pitch_in;
	yaw_servo_out = man_yaw_in;
	throttle_servo_out = man_throttle_in;

	/* to take a picture */
	if ((hrt_absolute_time() - last_pic_request_time)/1000000.0f > 3.0f) {
		// printf("sending pic request\n");
		take_picture();

		pic_taken_time = hrt_absolute_time();
		last_pic_request_time = hrt_absolute_time();
		/*
		int fuck = 0;
		while (!new_pic) {
			// just wait

			// check if there is a new picture result
			bool pic_result_updated;
			if (pic_res_sub <= 0) {
				pic_res_sub = orb_subscribe(ORB_ID(aa241x_picture_result));
			}
			orb_check(pic_res_sub, &pic_result_updated);

			if (pic_result_updated) {
				orb_copy(ORB_ID(aa241x_picture_result), pic_res_sub, &pic_result);

				// set the data to be used by students
				new_pic = true;
			}
			fuck++;
			if (fuck >= 10) {
				break;
			}
			printf(".");
		}
		printf("pic lag: %fus\n", (double) (hrt_absolute_time() - pic_taken_time));
		*/

	}
}
