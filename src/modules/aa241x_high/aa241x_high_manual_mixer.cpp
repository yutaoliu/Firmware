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
 * @file aa241x_high_manual_mixer.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author Matthew Berk			<matthew@msberk.net>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_manual_mixer.h"
#include "aa241x_high_aux.h"

using namespace aa241x_high;


/**
 * Mixer function in which you can define the way in which manual control 
 * inputs will be mapped to the output channels (useful for flying wings,
 * flaperons, etc). This is active when in "Manual" mode, not "Mission"
 */

void manual_mixer() {
  // Default channel in to channel out passthrough
  roll_servo_out      = man_roll_in;
  pitch_servo_out     = man_pitch_in;
  yaw_servo_out       = man_yaw_in;
  throttle_servo_out  = man_throttle_in;
  flaps_servo_out     = man_flaps_in; // will only work if flaps calibrated
  aux1_servo_out      = man_aux1_in; // will only work if aux1 calibrated

  /*
   * Example config for flying wing where one elevon is on ch1 (roll)
   * and the other elevon is on ch2 (pitch). Remember to comment out above
   * config if using this one...
   *
   * This config assumes mechanically reversed servos (i.e., one servo points
   * left, the other points right)
  */

   /*

   float pitch_command = 0.6f * man_pitch_in; // if pitch is reversed, put minus sign
   float roll_command  = 0.6f * man_roll_in;  // if roll is reversed, put minus sign

   // if pitch and roll are mixed up, then make both pitch_commands positive
   // and set one roll_command negative
   roll_servo_out     = roll_command + pitch_command;
   pitch_servo_out    = roll_command - pitch_command;

   throttle_servo_out = man_throttle_in;

   */


}