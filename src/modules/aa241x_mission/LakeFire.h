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
 * @file LakeFire.h
 *
 * Class definition for Stanford's Spring 2015 AA241x mission.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#ifndef LAKEFIRE_H_
#define LAKEFIRE_H_
#include <drivers/drv_hrt.h>
#include <time.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/aa241x_mission_status.h>
#include <uORB/topics/aa241x_new_fire.h>
#include <uORB/topics/aa241x_water_drop_result.h>
#include <uORB/topics/aa241x_water_drop_request.h>
#include <uORB/topics/aa241x_picture_result.h>
#include <uORB/topics/aa241x_picture_request.h>
#include <uORB/topics/aa241x_local_data.h>

#define GRID_WIDTH 17		/**< the number of cells wide and tall the grid is */
#define GRID_CENTER 8		/**< the index of the center row and column */

class LakeFire
{
public:
	/**
	 * Constructor
	 */
	LakeFire();

	/**
	 * Destructor
	 */
	~LakeFire();

	/**
	 * Start the LakeFire task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool	task_running() { return _task_running; }

	/**
	 * Trigger the virtual camera to take a picture and report the information.
	 *
	 * @return struct containing the success and information from the camera.
	 */
	picture_result_s	take_picture();

	/**
	 * Trigger the virtual water dropping.
	 *
	 * @return struct containing the success and details of the water drop.
	 */
	water_drop_result_s	drop_water();

private:

	bool	_task_should_exit;		/**< if true, aa241x mission should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for aa241x mission */

	int		_mavlink_fd;			/**< file description for mavlink to be able to send warnings */

	// handles to subscriptions needed
	int		_vcontrol_mode_sub;		/**< vehicle status (control mode) subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status (navigation mode) subscription */
	int		_params_sub;			/**< parameters update subscription */
	int		_pic_request_sub;		/**< requests for taking a picture */
	int		_water_drop_request_sub; /**< requests for triggering water drop */
	int		_local_data_sub;		/**< custom data fields */

	// structures for subscribed data
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct picture_request_s			_pic_request;		/**< picture taking request */
	struct water_drop_request_s			_water_drop_request; /**< water drop request */
	struct aa241x_local_data_s			_local_data;		/**< custom calc data */

	orb_advert_t	_mission_status_pub;
	orb_advert_t	_new_fire_pub;
	orb_advert_t	_pic_result_pub;
	orb_advert_t	_water_drop_result_pub;

	struct {
		float min_alt;
		float max_alt;
		float auto_alt;
		float cell_width;
		float duration;
		float max_radius;
		float timestep;
		float std;
		float t_pic;
		float min_fov;
		float max_fov;
		int index;
		float water_weight;
		float weight_per_drop;
		float ctr_lat;
		float ctr_lon;
		float ctr_alt;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min_alt;
		param_t max_alt;
		param_t auto_alt;
		param_t cell_width;
		param_t duration;
		param_t max_radius;
		param_t timestep;
		param_t std;
		param_t t_pic;
		param_t min_fov;
		param_t max_fov;
		param_t index;
		param_t water_weight;
		param_t weight_per_drop;
		param_t ctr_lat;
		param_t ctr_lon;
		param_t ctr_alt;
	}		_parameter_handles;		/**< handles for interesting parameters */

	hrt_abstime _mission_start_time;	/**< timestamp of when entered mission */
	hrt_abstime	_last_propagation_time;	/**< timestamp of when the last fire propagation was done */
	bool 		_in_mission;			/**< if true, currently running a mission (fire is spreading) */
	bool		_can_start;				/**< if false conditions for starting have been violated */
	bool		_early_termination;		/**< if true terminating mission early, but still need to finish running fire */
	bool		_mission_failed;		/**< if true terminating mission entirely with a score of 0 */
	float		_score;					/**< the current mission score */

	hrt_abstime _last_picture;			/**< timestamp of when the last picture was taken */

	int			_water_drops_remaining;	/**< the number of water shots remaining */

	enum WIND_DIRECTION {
		WIND_OTHER = -1,
		NORTH,
		NORTH_EAST,
		EAST,
		SOUTH_EAST,
		SOUTH,
		SOUTH_WEST,
		WEST,
		NORTH_WEST
	} _wind_direction;					/**< the direction of the wind */

	enum FIRE_STATE {
		OPEN_LAND = 0,
		ON_FIRE,
		WATER
	};


	int8_t	_grid[GRID_WIDTH][GRID_WIDTH];			/**< the grid that represents the lake */
	bool	_grid_mask[GRID_WIDTH][GRID_WIDTH];		/**< mask to determine in bounds in grid */

	/**
	 * Put together the mask that determines what is and isn't in bouds
	 */
	void	build_grid_mask();

	/**
	 * Get the diameter of the fov for a give altitude.
	 */
	float	get_fov_d(const float &alt);

	/**
	 * Convert from North position to i coordinate in grid.
	 */
	int		n2i(const float &n);

	/**
	 * Convert from East position to j coordinate in grid.
	 */
	int		e2j(const float &e);

	/**
	 * Convert from (i,j) grid coordinate to N,E coordinate of the center of the grid.
	 */
	math::Vector<2>		ij2ne(const float &i, const float &j);

	/**
	 * Determine which grid cells are in view and populate the information
	 * into the picture_result struct.
	 */
	void	get_fire_info(picture_result_s *pic_result);

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void	vehicle_control_mode_update();

	/**
	 * Check for global position updates.
	 */
	void	global_pos_update();

	/**
	 * Check for local position updates.
	 */
	void	local_pos_update();

	/**
	 * Check for vehicle status updates.
	 */
	void	vehicle_status_update();

	/**
	 * Check for custom calc updates.
	 */
	void	local_data_update();

	/**
	 * Handle the picture request, taking picture if necessary and publishing response.
	 */
	void	handle_picture_request();

	/**
	 * Handle the water dropping request, dropping water as needed and publishing response.
	 */
	void	handle_water_drop_request();

	/**
	 * Publish the current mission status information.
	 */
	void 	publish_mission_status();

	/**
	 * Publish a list of (i,j) coords of the new fire locations.
	 */
	void	publish_new_fire(const std::vector<int> &i_new, const std::vector<int> &j_new);

	/**
	 * Publish the picture result data when a picture is taken.
	 */
	void	publish_picture_result(const picture_result_s &pic_result);

	/**
	 * Publish the water drop data when a drop is requested.
	 */
	void	publish_water_drop(const water_drop_result_s &water_drop);

	/**
	 * Publish the condensed version of the grid for sending to the ground.
	 */
	void	publish_condensed_grid();

	/**
	 * Gaussian random number generator.
	 *
	 * Uses the Box-Muller transform
	 */
	float	generate_normal_random(const float &mean);

	/**
	 * Determine the coordinates of the cell to which the
	 * fire has propagated.
	 */
	void	get_prop_coords(int *i_prop, int *j_prop, const int &prop_dir);

	/**
	 * Initialize the mission parameters needed
	 */
	void	initialize_mission();

	/**
	 * Spread the fire at a given time step.
	 */
	void	propagate_fire();

	/**
	 * Calculate the current score.
	 */
	void 	calculate_score();



	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	void testing();

	void prop_testing();

	void sim_testing();

	void print_grid();

	/**
	 * Main task.
	 */
	void	task_main();
};

#endif /* LAKEFIRE_H_ */
