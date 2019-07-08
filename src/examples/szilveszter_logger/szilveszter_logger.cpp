/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file szilveszter_logger.c
 * Minimal application example for PX4 autopilot
 *
 * @author Szilveszter Zsigmond <zsigmondszilveszter@yahoo.com>
 */

#include <px4_log.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <systemlib/err.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <iostream>
#include <fstream>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

extern "C" __EXPORT int szilveszter_logger_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Mainloop of daemon.
 */
int szilveszter_logger_thread_main(int argc, char *argv[]);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: szilveszter_logger {start|stop|status}\n\n");
}

void gen_random(char *s, const int len) {
	srand ( time(NULL) ); // 
	static const char alphanum[] =
		"0123456789"
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
		"abcdefghijklmnopqrstuvwxyz";

	for (int i = 0; i < len; ++i) {
		s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
	}

	s[len] = 0;
}

int szilveszter_logger_thread_main(int argc, char *argv[]) {
	char rand_str[11] = {0};
	gen_random(rand_str, 10);
	char output_filename[65] = {0};
	sprintf(output_filename, "/home/quadcopter/szilveszter_logger/px4_logs_%s.csv", rand_str);
	std::ofstream logfile;
        logfile.open(output_filename);
	logfile << "Sequence,measurement_type,Timestamp,Acc_x,Acc_y,Acc_z,Gyro_x,Gyro_y,Gyro_z,Att_x,Att_y,Att_z\n";
	long long seq = 0;

    PX4_INFO("Szilveszter logger started logging!");

    /* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to ? Hz */
	orb_set_interval(sensor_sub_fd, 10);

	int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to ? Hz */
	orb_set_interval(attitude_sub_fd, 10);

	int barometer_sub_fd = orb_subscribe(ORB_ID(sensor_baro));
	/* limit the update rate to ? Hz */
	orb_set_interval(barometer_sub_fd, 10);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = attitude_sub_fd,   .events = POLLIN },
		{ .fd = barometer_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	while(1) {
		if(thread_should_exit){
			break;
		}
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 3, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

				seq++;
		                logfile << seq << "," << 1 << "," << raw.timestamp << ",";
				logfile << raw.accelerometer_m_s2[0] << "," << raw.accelerometer_m_s2[1] << "," << raw.accelerometer_m_s2[2] << ",";
				logfile << raw.gyro_rad[0] << "," << raw.gyro_rad[1] << "," << raw.gyro_rad[2] << "\n";
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_attitude_s att;
	                        memset(&att, 0, sizeof(att));

				orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &att);

				seq++;
		                logfile << seq << "," << 2 << "," << att.timestamp << ",";;
				logfile << att.q[0] << "," << att.q[1] << "," << att.q[2] << "\n";
			}

			if(fds[2].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_baro_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_baro), barometer_sub_fd, &raw);

				seq++;
		                logfile << seq << "," << 3 << "," << raw.timestamp << ",";;
				logfile << raw.pressure << "\n";
			}
		}
	}

	logfile.close();

	thread_running = false;
    	return OK;
}

int szilveszter_logger_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("szilveszter_logger already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("szilveszter_logger",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 szilveszter_logger_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tszilveszter_logger is running\n");

		} else {
			printf("\tszilveszter_logger not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}