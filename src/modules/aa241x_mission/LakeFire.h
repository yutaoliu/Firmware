/*
 * LakeFire.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Adrien
 */

#ifndef LAKEFIRE_H_
#define LAKEFIRE_H_

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

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

private:

	bool	_task_should_exit;		/**< if true, aa241x mission should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle for aa241x mission */

	// handles to subscriptions needed
	int		_vcontrol_mode_sub;		/**< vehicle status (control mode) subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status (navigation mode) subscription */

	// structures for subscribed data
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */

	struct {
		float min_alt;
		float max_alt;
		float start_alt;
		float duration;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min_alt;
		param_t max_alt;
		param_t start_alt;
		param_t duration;
	}		_parameter_handles;		/**< handles for interesting parameters */

	hrt_abstime _mission_start_time;	/**< timestamp of when entered mission */
	bool 		_in_mission;			/**< if true, currently running a mission (fire is spreading) */

	int8_t	_grid[21][21];			/**< the grid that represents the lake */

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void	vehicle_control_mode_poll();

	/**
	 * Check for global position updates.
	 */
	void	global_pos_poll();

	/**
	 * Check for local position updates.
	 */
	void	local_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void	vehicle_status_poll();

	/**
	 * Spread the fire at a given time step.
	 */
	void	propagate_fire();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main task.
	 */
	void	task_main();
};

#endif /* LAKEFIRE_H_ */
