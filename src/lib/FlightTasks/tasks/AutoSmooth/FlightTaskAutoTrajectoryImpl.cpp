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

/**
 * @file FlightTaskAutoTrajectoryImpl.cpp
 */

#include "FlightTaskAutoTrajectoryImpl.hpp"
#include <mathlib/mathlib.h>

using namespace matrix;

static constexpr float SIGMA_SINGLE_OP = 0.000001f;
static constexpr float BEZ_LINE_THRESHOLD = 1.0f;

FlightTaskAutoTrajectoryImpl::FlightTaskAutoTrajectoryImpl() :
	_line(nullptr, _deltatime, _position)
{}

void FlightTaskAutoTrajectoryImpl::_generateSetpoints()
{

	_update_control_points();

	_bezier_threshold = NAV_ACC_RAD.get() + BEZ_LINE_THRESHOLD;

	TrajectoryState traj_state_update;

	if (_type == WaypointType::loiter) {
		// just want to go to target and stop there
		traj_state_update = TrajectoryState::Hover;

	} else if ((_position_setpoint - _target).length() < _bezier_threshold) {
		traj_state_update = TrajectoryState::Bezier;

	} else {
		traj_state_update = TrajectoryState::Line;
	}

	if (_traj_state == TrajectoryState::Bezier) {
		// we don't do any updates until we reached the end
		Vector3f acceleration;
		float time = 0.0f;
		_bezier.getStatesClosest(_position_setpoint, _velocity_setpoint, acceleration, time, _position);

		if (fabsf(_bezier.getDuration() - time) < 0.1f) {
			// need to update bezier
			if (traj_state_update == TrajectoryState::Bezier) {
				_update_bezier();

			} else {
				_traj_state = traj_state_update;
			}
		}
	}

	if (_traj_state == TrajectoryState::Hover) {

		bool update = false;

		if (traj_state_update != TrajectoryState::Hover) {
			update = true;
		}

		if ((_line.getTarget() - _target).length() > SIGMA_SINGLE_OP) {
			update = true;
		}

		if (update) {
			_traj_state = traj_state_update;
			_update_bezier();
			_update_line();

		} else {
			_line.generateSetpoints(_position_setpoint, _velocity_setpoint);

		}
	}

	if (_traj_state == TrajectoryState::Line) {
		bool update = false;

		if (traj_state_update != TrajectoryState::Line) {
			update = true;
		}

		if ((_line.getTarget() - _pt_0).length() > SIGMA_SINGLE_OP) {
			update = true;
		}

		if (update) {
			_traj_state = traj_state_update;
			_update_bezier();
			_update_line();

		} else {
			_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
		}
	}


	if (!PX4_ISFINITE(_yaw_setpoint)) {
		_compute_heading_from_2D_vector(_yaw_setpoint, Vector2f(&_velocity_setpoint(0)));
	}
}

void FlightTaskAutoTrajectoryImpl::_update_control_points()
{
	Vector3f u_prev_to_target = (_target - _prev_wp).unit_or_zero();
	Vector3f u_target_to_next = (_next_wp - _target).unit_or_zero();

	_pt_0 = _target - (u_prev_to_target * (NAV_ACC_RAD.get() + BEZ_LINE_THRESHOLD));
	_pt_1 = _target + (u_target_to_next * (NAV_ACC_RAD.get() +  BEZ_LINE_THRESHOLD));

	Vector3f pt_0_next = _next_wp - (u_target_to_next * (NAV_ACC_RAD.get() + BEZ_LINE_THRESHOLD));

	if ((pt_0_next - _pt_1) * u_target_to_next < 0.0f) {
		// pt_0_next is closer to target than _pt_1. set _pt_1 to pt_0_next
		_pt_1 = pt_0_next;
	}

	if ((_pt_1 - _prev_wp) * u_prev_to_target < 0.0f) {
		_pt_1 = _prev_wp + u_prev_to_target * (_target - _prev_wp).length() / 2.0f;
	}
}

void FlightTaskAutoTrajectoryImpl::_update_bezier()
{
	float angle = 2.0f;

	// set velocity depending on the angle between the 3 waypoints
	if (Vector2f(&(_target - _prev_wp)(0)).unit_or_zero().length() > SIGMA_SINGLE_OP &&
	    Vector2f(&(_target - _next_wp)(0)).unit_or_zero().length() > SIGMA_SINGLE_OP) {

		angle = Vector2f(&(_target - _prev_wp)(0)).unit_or_zero()
			* Vector2f(&(_target - _next_wp)(0)).unit_or_zero()
			+ 1.0f;
	}

	// for now just set it to half the acceleration because no parameter exists.
	const float deceleration = MPC_ACC_HOR.get() / 2.0f;

	// the desired velocity at target
	float desired_vel = math::getVelocityFromAngle(angle, 0.5f, MPC_CRUISE_90.get(), _mc_cruise_speed);

	// compute speed at transition line - bezier based on deceleration and desired velocity at target
	bezier::BezierQuad_f bez_tmp(_pt_0, _target, _pt_1); // only care about bezier position with default dureation = 1s
	float distance_half = 0.5f * bez_tmp.getArcLength(0.1f); // half the the bezier distance
	float duration = 2.0f * (sqrtf(2.0f * deceleration * distance_half + desired_vel * desired_vel) - desired_vel) /
			 deceleration; // total time to travel bezier
	_bezier.setBezier(_pt_0, _target, _pt_1, duration);

	// if speed at intersection is larger than cruise speed, then compute duration based
	// on maximums speed at intersection.
	if (_bezier.getVelocity(0).length() > _mc_cruise_speed) {
		duration = 2.0f * (_target - _pt_0).length() / _mc_cruise_speed;
		_bezier.setBezier(_pt_0, _target, _pt_1, duration);
	}

}

void FlightTaskAutoTrajectoryImpl::_update_line()
{
	// straight line
	if (_traj_state == TrajectoryState::Hover) {
		_line.setLineFromTo(_prev_wp, _target);
		_line.setDeceleration(0.5f);
		_line.setSpeedAtTarget(0.0f);

	} else {
		_line.setLineFromTo(_prev_wp, _pt_0);
		_line.setDeceleration(MPC_ACC_HOR.get() / 2.0f);
		_line.setSpeedAtTarget(_bezier.getVelocity(0).length());

	}

	_line.setSpeed(_mc_cruise_speed);
	_line.setAcceleration(MPC_ACC_HOR.get());

}
