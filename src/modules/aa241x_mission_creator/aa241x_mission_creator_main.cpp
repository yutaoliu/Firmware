/*
 * aa241x_mission_creator_main.c
 *
 *  Created on: May 31, 2015
 *      Author: Adrien
 */


#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <algorithm>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <uORB/uORB.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>
#include <drivers/drv_tone_alarm.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

#define GRID_WIDTH 17
#define STD_DEV 0.3f
#define CELL_WIDTH 20.0f
#define MAX_RAD 170.0f
#define GRID_CENTER 8

enum FIRE_STATE {
		OPEN_LAND = 0,
		ON_FIRE,
		WATER
};

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


bool	_grid_mask[GRID_WIDTH][GRID_WIDTH] = {{0}};		/**< mask to determine in bounds in grid */
int8_t	_grid[GRID_WIDTH][GRID_WIDTH] = {{0}};

extern "C" __EXPORT int aa241x_mission_creator_main(int argc, char *argv[]);

void print_grid();
void propagate_fire();
float generate_normal_random(const float &mean);
void get_prop_coords(int *i_prop, int *j_prop, const int &prop_dir);

void build_grid_mask();
math::Vector<2> ij2ne(const float &i, const float &j);


math::Vector<2>
ij2ne(const float &i, const float &j)
{
	math::Vector<2> ne;

	ne(0) = (GRID_CENTER - i)*CELL_WIDTH;
	ne(1) = (j - GRID_CENTER)*CELL_WIDTH;

	return ne;

}


void
build_grid_mask()
{
	float hw = CELL_WIDTH/2.0f;
	math::Vector<2> center;

	math::Vector<2> left = math::Vector<2>(-hw, -hw);
	math::Vector<2> right = math::Vector<2>(hw, hw);
	math::Vector<2> top = math::Vector<2>(-hw, hw);
	math::Vector<2> bottom = math::Vector<2>(hw, -hw);

	math::Vector<2> sides[4] = {left, right, top, bottom};

	float r2 = MAX_RAD * MAX_RAD;
	bool valid = true;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {
			valid = true;

			// get center coord
			center = ij2ne((float) i, (float) j);

			// if center isn't valid, definite not a valid cell
			if (center.length_squared() >= r2) {
				continue;
			}

			// check each of the sides
			for (int k = 0; k < 4; k++) {
				if ((center + sides[k]).length_squared() > r2) {
					valid = false;
					break;
				}
			}

			// set mask to true if an inbound cell
			if (valid) {
				_grid_mask[i][j] = true;
			}
		}
	}
}

float
generate_normal_random(const float &mean)
{
	// TODO: it is never actually using the spare, would need to make params global
	bool have_spare = false;

	/* get uniform random values on interval (0,1) */
	float rand1 = (float) rand() / ((float) MAX_RAND);
	float rand2 = (float) rand() / ((float) MAX_RAND);

	float R;
	float theta;

	/* generate 2 random numbers at a time, but only use 1, check for leftover from previous request */
	if (have_spare) {
		have_spare = false;
		return (STD_DEV * sqrtf(R) * sinf(theta)) + mean;
	}

	have_spare = true;
	R = -2*logf(rand1);
	theta = rand2 * 2.0f * (float) M_PI;

	/* return Box-Muller transform for normal random */
	return (STD_DEV * sqrtf(R) * cosf(theta)) + mean;
}

void
get_prop_coords(int *i_prop, int *j_prop, const int &prop_dir)
{

	/* adjust new fire cell coords based on propagation direction */
	switch (WIND_DIRECTION(prop_dir)) {
	case NORTH:
		(*i_prop)--;
		break;
	case NORTH_EAST:
		(*i_prop)--;
		(*j_prop)++;
		break;
	case EAST:
		(*j_prop)++;
		break;
	case SOUTH_EAST:
		(*i_prop)++;
		(*j_prop)++;
		break;
	case SOUTH:
		(*i_prop)++;
		break;
	case SOUTH_WEST:
		(*i_prop)++;
		(*j_prop)--;
		break;
	case WEST:
		(*j_prop)--;
		break;
	case NORTH_WEST:
		(*i_prop)--;
		(*j_prop)--;
		break;
	default:
		break;
	}
}

void
propagate_fire()
{
	float prop_dir;
	int8_t cell_val;
	int i_prop;
	int j_prop;

	std::vector<int> i_new;
	std::vector<int> j_new;

	int count = 0;

	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0; j < GRID_WIDTH; j++) {

			/* make sure this cell isn't a new fire cell */
			/*
			int ai = std::find(i_new.begin(), i_new.end(), i);
			int aj = std::find(j_new.begin(), j_new.end(), i);
			if ( ai != i_new.end() &&  aj != j_new.end()) {
				printf("new prop found: (%d, %d)\n", ai, aj);
				continue;
			}
			*/
			if ( std::find(i_new.begin(), i_new.end(), i) != i_new.end() &&  std::find(j_new.begin(), j_new.end(), j) != j_new.end()) {
				continue;
			}

			/* check for fire in cell, continue if no fire in cell */
			cell_val = _grid[i][j];
			if (cell_val != ON_FIRE) continue;
			count++;

			prop_dir = roundf(generate_normal_random(_wind_direction));

			/* wrap the propagation direction to be within (0,8) */
			if (prop_dir < 0) prop_dir += 8;
			if (prop_dir > 7) prop_dir -= 8;

			i_prop = i;
			j_prop = j;

			get_prop_coords(&i_prop, &j_prop, (int) prop_dir);

			/* check to make sure new fire cell is a valid location */
			if (i_prop >= GRID_WIDTH || i_prop < 0 || j_prop >= GRID_WIDTH || j_prop < 0) continue;

			/* check that the new cell is in bounds */
			if (!_grid_mask[i_prop][j_prop]) continue;

			/* check for new fire cell value */
			cell_val = _grid[i_prop][j_prop];
			if (cell_val == OPEN_LAND) {
				/* add fire to this cell */
				_grid[i_prop][j_prop] = ON_FIRE;
				//printf("(%d, %d) prop to: (%d, %d)\n", i, j, i_prop, j_prop);
				i_new.push_back(i_prop);
				j_new.push_back(j_prop);
			}
		}
	}

	//printf("new fire count: %d\n", count);

	/* clear vectors after having published the info */
	i_new.clear();
	j_new.clear();
}


void
print_grid() {
	printf("\n");
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0;j < GRID_WIDTH; j++) {
			printf("%d ", _grid[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

int aa241x_mission_creator_main(int argc, char *argv[])
{

	int numStart = atoi(argv[1]);

	if (atoi(argv[2]) == -1) {
		// create random wind direction
		_wind_direction = WIND_DIRECTION((int) round(7.0f*((float) rand() / ((float) MAX_RAND))));
	} else {
		printf("using a wind direction of: %d\n", atoi(argv[1]));
		_wind_direction = WIND_DIRECTION(atoi(argv[1]));
	}

	build_grid_mask();

	memset(&_grid, 0, sizeof(_grid));

	float duration = 10.0f; // in minutes
	float timeStep = 15.0f; // seconds

	// reset the random seed
	// srand(112358);

	int iStart[2] = {-1};
	int jStart[2] = {-1};

	// create starting points
	for (int k = 0; k < numStart; k++) {
		iStart[k] = (int) round(16.0f*((float) rand() / ((float) MAX_RAND)));
		jStart[k] = (int) round(16.0f*((float) rand() / ((float) MAX_RAND)));

		while (_grid_mask[iStart[k]][jStart[k]] == 0) {
			iStart[k] = (int) round(16.0f*((float) rand() / ((float) MAX_RAND)));
			jStart[k] = (int) round(16.0f*((float) rand() / ((float) MAX_RAND)));
		}

	}

	int seed = rand();
	srand(seed);

	printf("Wind direction: %d\n", _wind_direction);
	printf("Starting coords: (%d,%d) (%d,%d)\n",iStart[0], jStart[0], iStart[1], jStart[1]);
	printf("Random seed: %d\n", seed);


	// initialize the fire
	for (int k = 0; k < 2; k++) {
		_grid[iStart[k]][jStart[k]] = ON_FIRE;
	}

	// add some water
	/*
	for (int j = 0; j < GRID_WIDTH; j++) {
		_grid[6][j] = WATER;
	}
	*/

	printf("Starting Grid:\n");
	print_grid();

	// now propagate the fire
	int props = (int) duration*60.0f/timeStep;
	printf("props: %d\n", props);

	while (props > 0) {
		propagate_fire();
		//print_grid();
		//usleep(2000000);
		props--;
	}

	// check to make sure enough of it is on fire by the end
	printf("Final Grid:\n");
	int count = 0;
	printf("\n");
	for (int i = 0; i < GRID_WIDTH; i++) {
		for (int j = 0;j < GRID_WIDTH; j++) {
			printf("%d ", _grid[i][j]);
			if (_grid[i][j] == ON_FIRE) {
				count++;
			}
		}
		printf("\n");
	}
	printf("\n");

	printf("Total coverage: %d\n", count);

	return 0;
}
