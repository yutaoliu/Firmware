/*
 * aa241x_fw_control_main.h
 *
 *  Created on: Feb 9, 2015
 *      Author: adrienp
 */
#pragma once

#ifndef AA241X_FW_CONTROL_MAIN_H_
#define AA241X_FW_CONTROL_MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>


hrt_abstime pic_gotten_time = 0;
hrt_abstime last_pic_request_time = 0;



#endif /* AA241X_FW_CONTROL_MAIN_H_ */
