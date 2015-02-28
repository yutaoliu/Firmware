/*
 * LakeFire.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Adrien
 */

#ifndef LAKEFIRE_H_
#define LAKEFIRE_H_
#include <drivers/drv_hrt.h>
#include <time.h>
#include <mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/aa241x_mission_status.h>
#include <uORB/topics/aa241x_new_fire.h>
#include <uORB/topics/aa241x_water_drop.h>
#include <uORB/topics/aa241x_picture_result.h>

#define GRID_WIDTH 21
#define GRID_CENTER 11

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

	picture_result_s	take_picture();

private:

	bool	_task_should_exit;		/**< if true, aa241x mission should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for aa241x mission */

	// handles to subscriptions needed
	int		_vcontrol_mode_sub;		/**< vehicle status (control mode) subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status (navigation mode) subscription */
	int		_params_sub;			/**< parameters update subscription */

	// structures for subscribed data
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	orb_advert_t	_mission_status_pub;
	orb_advert_t	_new_fire_pub;
	orb_advert_t	_pic_result_pub;

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
		WATER = -1,
		OPEN_LAND,
		ON_FIRE
	};


	int8_t	_grid[GRID_WIDTH][GRID_WIDTH];			/**< the grid that represents the lake */


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

	void print_grid();

	/**
	 * Main task.
	 */
	void	task_main();
};

#endif /* LAKEFIRE_H_ */
