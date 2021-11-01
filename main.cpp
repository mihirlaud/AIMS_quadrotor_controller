/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 * Flight control program designed to follow a given trajectory by sending a
 * quadrotor yaw rate, pitch rate, roll rate, and thrust commands.
 *
 * @author Mihir Laud <mihirlaud@gmail.com>
 */

#include <string.h>
#include <sched.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_local_position.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/tasks.h>

#include "control.h"

static int daemon_task;

void usage();

extern "C" __EXPORT int flight_control_main(int argc, char *argv[]);
int flight_control_main(int argc, char *argv[]) {
	if(argc < 2) {
		usage();
		return 1;
	} else {
		if(!strcmp(argv[1], "start")) {
			PX4_INFO("Starting flight control...\n");

			daemon_task = px4_task_spawn_cmd("flight_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 (px4_main_t)flight_control_app_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		} else if(!strcmp(argv[1], "stop")) {
			PX4_INFO("Stopping flight control...\n");
			ControlApp::state.requestExit();
			return 0;
		} else if(!strcmp(argv[1], "status")) {
			PX4_INFO("Flight control status: %s\n", ControlApp::state.isRunning() ? "running" : "not running");
		} else {
			usage();
			return 1;
		}

		return OK;
	}
}

void usage() {
	PX4_WARN("usage: flight_control {start|stop|status}\n");
}
