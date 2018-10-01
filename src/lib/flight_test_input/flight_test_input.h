/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 *  - provides attitude controller frequency injection for the purpose of system identification
 */

#ifndef FLIGHTTESTINPUT_H
#define FLIGHTTESTINPUT_H

#include <mathlib/mathlib.h>
#include <stdint.h>

#include <controllib/block/BlockParam.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/flight_test_input.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_status.h>

class __EXPORT FlightTestInput : public control::SuperBlock
{
public:
	FlightTestInput();
	~FlightTestInput();

	// Update test input computation
	void update(float dt);

	// Inject current test input
	float inject(const uint8_t injection_point, const float inject_input);

private:

	enum FlightTestInputState {
		TEST_INPUT_OFF = 0,
		TEST_INPUT_INIT,
		TEST_INPUT_RUNNING,
		TEST_INPUT_COMPLETE
	} _state;

	void computeSweep(float dt);
	void computeDoublet(float dt);

	orb_advert_t	_mavlink_log_pub;

	float _time_running;
	float _raw_output;

	// frequency sweep sine sum
	float _sweep_sine_input;


	/** parameters **/
	control::BlockParamInt _mode;
	control::BlockParamInt _enable;
	control::BlockParamInt _injection_point;

	/** frequency sweep parameters **/
	control::BlockParamFloat _sweep_duration;
	control::BlockParamFloat _sweep_freq_begin;
	control::BlockParamFloat _sweep_freq_end;
	control::BlockParamFloat _sweep_freq_ramp;
	control::BlockParamFloat _sweep_amplitude_begin;
	control::BlockParamFloat _sweep_amplitude_end;

	/** doublet parameters **/
	control::BlockParamFloat _doublet_pulse_length;
	control::BlockParamFloat _doublet_pulse_amplitude;

	orb_advert_t _flight_test_input_pub;
	struct flight_test_input_s _flight_test_input;

	// system main_state and nav_state captured during test input init
	uint8_t _main_state;
	uint8_t _nav_state;

	// check for vehicle status updates.
	int _vehicle_status_sub;
	int _commander_state_sub;

	struct vehicle_status_s _vehicle_status;
	struct commander_state_s _commander_state;

	void vehicle_status_poll();
	void commander_state_poll();

};

#endif // FLIGHTTESTINPUT_H
