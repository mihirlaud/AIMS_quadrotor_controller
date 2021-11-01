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
 * @file control.cpp
 * Flight control object containing system task and all logic for quadrotor control
 *
 * @author Mihir Laud <mihirlaud@gmail.com>
 */

#include "control.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <limits>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

int flight_control_app_main(int argc, char **argv)
{
	px4::init(argc, argv, "flight_control");

	printf("flight_control\n");

	ControlApp app;
	app.main();

	return 0;
}

px4::AppState ControlApp::state;

ControlApp::ControlApp() {}

int ControlApp::main() {
	state.setRunning(true);

	struct vehicle_rates_setpoint_s sp;
	memset(&sp, 0, sizeof(sp));
	orb_advert_t sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &sp);

	struct vehicle_local_position_s pos;
	memset(&pos, 0, sizeof(pos));

	int pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	sp.yaw = 0;

	double z_total_error = 0;
	double z_prev_error = 0;

	while(!state.exitRequested()) {

		bool pos_updated;
		orb_check(pos_sub, &pos_updated);
		if(pos_updated) {
			orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);
		}

		double z_sp = -1;

		double z_kP = 0.2;
		double z_kD = 100.0;
		double z_kI = 0.0001;

		double z_error = z_sp - (double)pos.z;
		double z_error_deriv = z_error - z_prev_error;

		if(abs(z_error) < 0.1) {
			z_total_error += z_error;
		} else {
			z_total_error = 0;
		}

		double command = z_kP * z_error + z_kD * z_error_deriv + z_kI * z_total_error;
		if(command > -0.71) {
			command = -0.71;
		} else if(command < -1) {
			command = -1;
		}

		sp.thrust_body[2] = command;

		orb_publish(ORB_ID(vehicle_rates_setpoint), sp_pub, &sp);

		z_prev_error = z_error;

	}

	bool pos_updated;
	orb_check(pos_sub, &pos_updated);
	if(pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), pos_sub, &pos);
	}

	PX4_INFO("Final Position:\tx: %.4f\ty: %.4f\tz: %.4f", (double)pos.x, (double)pos.y, (double)pos.z);
	PX4_INFO("Final Velocity:\tvx: %.4f\tvy: %.4f\tvz: %.4f", (double)pos.vx, (double)pos.vy, (double)pos.vz);


	return 0;
}
