/*
 * aa241x_mission_main.cpp
 *
 *  Created on: Feb 22, 2015
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
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include "LakeFire.h"


/**
 * aa241x_mission app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_mission_main(int argc, char *argv[]);


namespace aa241x_mission
{

LakeFire	*g_aa241x_mission;
}

LakeFire::LakeFire() {
	// TODO Auto-generated constructor stub

}

LakeFire::~LakeFire() {
	// TODO Auto-generated destructor stub
}




int aa241x_mission_main(int argc, char *argv[])
{

	return 0;
}
