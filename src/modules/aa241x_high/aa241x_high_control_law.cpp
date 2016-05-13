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

using namespace aa241x_high;


/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */

// DEFINE VARIABLES HERE!:

// Ratio of radio to deflection of controls!
// Change these variables:
// Bixler vars right now:
#define max_deflection_elev 45.0f // deg
#define max_deflection_aile 45.0f // deg
#define max_deflection_rudd 45.0f // deg

// Leave these alone:
#define pi 3.141592653589793f
#define deg2rad (pi/180.0f)
#define rad2deg (1.0f / deg2rad)
#define rc_to_rad_elev (1.0f / (max_deflection_elev * deg2rad))
#define rc_to_rad_aile (1.0f / (max_deflection_aile * deg2rad))
#define rc_to_rad_rudd (1.0f / (max_deflection_rudd * deg2rad))
#define rad_to_rc_elev (1.0f/rc_to_rad_elev)
#define rad_to_rc_aile (1.0f/rc_to_rad_aile)
#define rad_to_rc_rudd (1.0f/rc_to_rad_rudd)
#define field_radius 170.0f
#define redzone_buffer 18.0f
#define redzone_radius (field_radius-redzone_buffer)
#define redzone_radius_squared (redzone_radius*redzone_radius)

// make a couple of convenient macros
#define sqr(inp) ((inp)*(inp))
#define cub(inp) ((inp)*(inp)*(inp))



// Vars for altitude and heading control:
float desired_altitude = 0.0f;
float desired_velocity = 0.0f;
// float yaw_desired = 0.0f;
float desired_pitch = 0.0f;
float adjust_bank_gain=0.0f;
float adjust_head_gain=0.0f;

// Define globals for origin and destination points, as well as waypoint
int waypoint=1;
float x_orig=0.0f;
float y_orig=0.0f;
float x_dest=0.0f;
float y_dest=0.0f;

// Distance and rate of change of distance for line following
float ydot = 0.0f;
float y = 0.0f;
float dist2dest;

// Unit normal vectors
float nyE = 0.0f;
float nyN = 0.0f;

// Initial plane points
float E_start = 0.0f;
float N_start = 0.0f;

// Define a function to cycle through waypoints
void follow_waypoints(float dist2dest, const float * xx, const float * yy, int length) {

  // Compare squared distance from point to squared waypoint parameter
  if(dist2dest < aah_parameters.waypoint_radius)
  {
    waypoint=waypoint+1;
    if(waypoint==length){
      waypoint=1;
    }
    ydot = 0.0f;
  }

  x_orig=xx[waypoint-1];
  y_orig=yy[waypoint-1];

  x_dest=xx[waypoint];
  y_dest=yy[waypoint];
}

// Define a function to see whether or not we are in the red zone, make sure this
// is the LAST check before sending to the servo
void redzone(const float &a, const float &b) 
{
  if ((sqr(a) + sqr(b)) > sqr(field_radius - aah_parameters.redzone)) 
  {
    x_orig = a;
    y_orig = b;
    x_dest = 0.0f;
    y_dest = 0.0f;
    high_data.inRedzone = 1.0f;
  } else {
    high_data.inRedzone = 0.0f;
  }
}

// Main function for flight control
void flight_control() {

  // Initialize current position along with resetting deviation from line
  float a=position_E;
  float b=position_N;

  // Check if auto is being enabled:
  // RUNS ONCE!!! AT START OF AUTOPILOT
  if ((timestamp - previous_loop_timestamp) > 1000000.0f) {
    // Reset waypoint every time auto mode is enabled
    waypoint = 1;
    high_data.wpreached_high = 0.0f;
    high_data.wet = 0.0f;
    // Set initial origin to autopilot turn on location
    E_start = a;
    N_start = b;
    x_orig = E_start;
    y_orig = N_start;
  }

  desired_altitude = aah_parameters.desired_altitude;
  desired_velocity = aah_parameters.desired_velocity; // later maybe to be changed to ground velocity

  //
  // Altitude hold code:
  //
  //

  // Check height difference
  float delta_alt = desired_altitude + position_D_gps; // meters, plus sign due to position_D_gps being negative with altitude increase

  // Check Velocity difference
  float delta_vel = 0; // meters/second

  // Compute Throttle input change due to velocity
  float delta_throttle = delta_vel * aah_parameters.gain_throttle; // RC units  

  // Compute Pitch Angle change due to altitude
  float delta_pitch = delta_alt * aah_parameters.alt_gain_pitch; // degrees

  // Check bounds on pitch:
  if ( ( delta_pitch > aah_parameters.climb1_pitch ) && ( -position_D_gps < (0.3048f * 130.0f) )) { // degrees
    delta_pitch = aah_parameters.climb1_pitch;
  } else if ( delta_pitch > aah_parameters.climb2_pitch) {
    delta_pitch = aah_parameters.climb2_pitch; // degrees. This is to improve speed during the first search legs
  } else if ( delta_pitch < aah_parameters.descent1_pitch ) { // degrees
    delta_pitch = aah_parameters.descent1_pitch;
  }

  // Compute desired pitch by combining trim and altitude pitch change
  pitch_desired = aah_parameters.trim_pitch * deg2rad + delta_pitch * deg2rad;
  
  // Set new elevator position using control law 
  float delta_elevator = aah_parameters.gain_elevator * ( pitch_desired - pitch ); //rad

  // Send output to pitch servo
  pitch_servo_out = pitch_trim + rad_to_rc_elev * delta_elevator;

  // Code for turning motor active or inactive
  if (aah_parameters.motor_active == 1){
    throttle_servo_out = aah_parameters.trim_throttle + delta_throttle;
    throttle_desired = throttle_servo_out;
  } else {
    throttle_servo_out = 0.0f;
    throttle_desired = throttle_servo_out;
  }

  //
  // Heading hold code:
  //
  //

  
  //float yold = y;
  //float dist2dest;  // distance to P2 in the x-direction in m

    // compute y and distance to destination
  yaw_desired=atan2(x_dest-x_orig,y_dest-y_orig); // in rad

  y=cosf(yaw_desired)*(a-x_orig) -sinf(yaw_desired)*(b-y_orig);

  dist2dest=sinf(yaw_desired)*(x_dest-a) +cosf(yaw_desired)*(y_dest-b);

  //************************************************************
  // Waypoint paths
  //
  // Define different lines
  // 1: Straight line north
  if(aah_parameters.path_no == 1){
       x_dest=-50.0f;
       y_dest=100.0f;
       x_orig=-50.0f;
       y_orig=0.0f;
  }
  // 2: Straight line east
  if((aah_parameters.path_no) == 2){
       x_dest=0.0f;
       y_dest=100.0f;
       x_orig=-100.0f;
       y_orig=100.0f;
  }
  // 3: Straight line NE
  if((aah_parameters.path_no) == 3){
      x_dest=0.0f;
      y_dest=100.0f;
      x_orig=-100.0f;
      y_orig=0.0f;
  }
  //
  // 4: Pentagon shape
  // Waypoints for path 4
  float xx[6]={50.0f,0.0f,-50.0f,-100.0f,-25.0f,50.0f};
  float yy[6]={0.0f,-50.0f,-50.0f,0.0f,9.87f,0.0f};

  if((aah_parameters.path_no) == 4){
    follow_waypoints(dist2dest, xx, yy, 6);
  }
  // 
  // 5: Square shape
  // Waypoints for path 5
  float xsquare[5] = {-25.0f, 750.0f, 75.0f, -25.0f, -25.0f};
  float ysquare[5] = {50.0f, 50.0f, -50.0f, -50.0f, 50.0f};

  if((aah_parameters.path_no) == 5){
    follow_waypoints(dist2dest, xsquare, ysquare, 5);
  }
  //
  // 6: Triangle shape (60deg turn)
  // Waypoints for path 6
  float xtri[4] = {0.0f, 100.0f, 50.0f, 0.0f};
  float ytri[4] = {86.6f, 86.6f, 0.0f, 86.6f};

  if((aah_parameters.path_no) == 6){
    follow_waypoints(dist2dest, xtri, ytri, 4);
  }
  
  //
  // 7: Racecourse
  // Waypoints for path 7
  float xcourse[92] = {E_start,-87.548f,-65.448f,-43.349f,-21.249f,0.85f,22.949f,45.049f,67.148f,89.248f,111.35f,133.45f,142.49f,144.85f,139.62f,128.81f,116.54f,107.5f,105.14f,110.37f,121.18f,133.45f,142.49f,144.85f,139.62f,128.81f,116.54f,107.5f,105.14f,110.37f,121.18f,133.45f,137.03f,140.09f,142.49f,144.12f,144.92f,144.85f,143.9f,142.14f,139.62f,136.47f,116.49f,96.517f,76.543f,56.569f,36.595f,16.62f,-3.3538f,-23.328f,-43.302f,-63.276f,-75.097f,-86.784f,-93.874f,-93.658f,-86.219f,-74.399f,-62.712f,-55.622f,-55.838f,-63.276f,-75.097f,-86.784f,-93.874f,-93.658f,-86.219f,-74.399f,-62.712f,-55.622f,-55.838f,-63.276f,-66.933f,-70.932f,-75.097f,-79.247f,-83.2f,-86.784f,-89.842f,-92.24f,-93.874f,-94.672f,-96.797f,-98.922f,-101.05f,-103.17f,-105.3f,-107.42f,-109.55f,-111.67f,-113.8f,-115.92f};
  float ycourse[92] = {N_start,168.13f,157.82f,147.52f,137.21f,126.91f,116.6f,106.3f,95.99f,85.685f,75.38f,65.075f,56.645f,44.511f,33.309f,27.316f,28.823f,37.253f,49.386f,60.589f,66.581f,65.075f,56.645f,44.511f,33.309f,27.316f,28.823f,37.253f,49.386f,60.589f,66.581f,65.075f,62.921f,60.07f,56.645f,52.796f,48.692f,44.511f,40.437f,36.648f,33.309f,30.566f,16.58f,2.5935f,-11.393f,-25.379f,-39.365f,-53.351f,-67.337f,-81.323f,-95.309f,-109.3f,-112.91f,-108.88f,-98.76f,-86.401f,-76.529f,-72.915f,-76.939f,-87.065f,-99.423f,-109.3f,-112.91f,-108.88f,-98.76f,-86.401f,-76.529f,-72.915f,-76.939f,-87.065f,-99.423f,-109.3f,-111.32f,-112.54f,-112.91f,-112.4f,-111.04f,-108.88f,-106.03f,-102.61f,-98.76f,-94.655f,-70.364f,-46.073f,-21.782f,2.5096f,26.801f,51.092f,75.383f,99.674f,123.97f,148.26f};
  

  if((aah_parameters.path_no) == 7){
    follow_waypoints(dist2dest, xcourse, ycourse, 92);
  }
  
  //************************************************************
  
  // BACKUP RED ZONE, check if in red zone and reset the line to head back
  // to center if so:
  // redzone(a,b);
  


  // Compute orthogonal unit vectors (positive right) in N and E coords
  nyE = -(y_dest-y_orig)/sqrtf( sqr(y_dest-y_orig) + sqr(x_dest-x_orig) );
  nyN =  (x_dest-x_orig)/sqrtf( sqr(y_dest-y_orig) + sqr(x_dest-x_orig) );

  // Compute ydot
  ydot = vel_E*nyE + vel_N*nyN;
  
  // Save values to the log
  high_data.xOrigin = x_orig;
  high_data.yOrigin = y_orig;

  // check whether the destination was missed (and switch P1 and P2)
  if(dist2dest+20.0f<0.00001f){
    //  yaw_desired=atan2(x_orig-x_dest,y_orig-y_dest); // in rad
    //  y=cosf(yaw_desired)*(a-x_dest) -sinf(yaw_desired)*(b-y_dest);  
    x_orig = a;
    y_orig = b;
  }
  
  // adjust the lateral gains in order to make the control more agressive 
  if(fabsf(y)>2.0f){
   adjust_bank_gain=1.0f;
   adjust_head_gain=1.0f;   
  }
  else{
   adjust_bank_gain=1.0f;
   adjust_head_gain=1.0f;   
  }
  
  
  // compute the deviation from the desired heading (along the line)
  // also compute the change from damping term
  float delta_heading = adjust_head_gain*aah_parameters.hdg_gain_head * y
    + ydot * aah_parameters.ydot_gain * deg2rad;
 // rad

  // clipping, making sure the deviation is not larger than +-90 degrees
  if (delta_heading > pi/2){
    delta_heading=pi/2;}
  else if(delta_heading < -pi/2){
    delta_heading=-pi/2;
  }


  // difference between target and actual heading
  float ddelta_heading = (yaw_desired + delta_heading - yaw); // rad

  // Worry about the specifics of dealing with the discontinuity later
  if (ddelta_heading > pi ) {
    ddelta_heading -= 2*pi;
  } else if ( ddelta_heading < -pi ) {
    ddelta_heading += 2*pi;
  }

  // Determine bank angle and rudder desired for this heading error
  float delta_bank = adjust_bank_gain*aah_parameters.hdg_gain_bank * ddelta_heading; // radians

  // Set up condition for having less bank at low altitude:
  float max_bank = aah_parameters.max_bank * deg2rad;

  // Bounds check for bank to ensure it doesn't try to turn too tight:
  if (delta_bank > max_bank ) { //rad
    delta_bank = max_bank;
  }
  else if (delta_bank < -max_bank) { //rad
    delta_bank = -max_bank;
  }


  // Send output to aileron servos
  roll_servo_out = roll_trim + (aah_parameters.trim_bank+delta_bank-roll) * aah_parameters.gain_aileron* rad_to_rc_aile; // RC units converted from radians

  // Output for rudder is unused...
  yaw_servo_out = yaw_trim;

  // check ch05 and set to manual yaw in
  flaps_servo_out = man_yaw_in;

  // check ch06 and set to negative of the roll
  aux1_servo_out = -roll_servo_out;
  
  high_data.dist2dest = dist2dest;
  high_data.y = y;
  high_data.ydot = ydot;


}
