/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocation.cpp
 *
 * Interface for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocation.hpp"

ControlAllocation::ControlAllocation()
{
	_control_allocation_scale.setAll(1.f);
	_actuator_min.setAll(0.f);  // _actuator_min 값을 float 0.0 으로 설정
	_actuator_max.setAll(1.f);  // _actuator_max 값을 float 1.0 으로 설정
}

void
ControlAllocation::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	_effectiveness = effectiveness;
	ActuatorVector linearization_point_clipped = linearization_point;
	clipActuatorSetpoint(linearization_point_clipped);
	_actuator_trim = actuator_trim + linearization_point_clipped;
	clipActuatorSetpoint(_actuator_trim);
	_num_actuators = num_actuators;
	_control_trim = _effectiveness * linearization_point_clipped;
}

void
ControlAllocation::setActuatorSetpoint(
	const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator_sp)
{
	// Set actuator setpoint
	_actuator_sp = actuator_sp;
	// Clip
	clipActuatorSetpoint(_actuator_sp);
}

void
ControlAllocation::clipActuatorSetpoint(matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator) const
{
	// std::cout << "[ControlAllocation] 444 _actuator_sp = " << actuator << std::endl;
	/*
	[ControlAllocation] 444 _actuator_sp =
	0.29236072	0.28662348	0.29184109	0.29080185	0.28818235	0.28714311	0.28766271	0.29132149	       0	       0	       0	       0	       0       0	       0	       0
	*/
	// std::cout << "[ControlAllocation] 444 _actuator_sp = " << actuator(0) << std::endl;

	for (int i = 0; i < _num_actuators; i++) {
		/*
		_actuator_min = 0
		_actuator_max = 1
		_actuator_trim = 0
		actuator 에는 0.28 ~ 0.29
		*/
		// std::cout << "[ControlAllocation] _actuator_min = " << _actuator_min(i) << std::endl;
		// std::cout << "[ControlAllocation] _actuator_max = " << _actuator_max(i) << std::endl;
		// std::cout << "[ControlAllocation] _actuator_trim = " << _actuator_trim(i) << std::endl;
		// std::cout << "[ControlAllocation] actuator[" << i << "] = " << actuator(i) << "  a = "<< a << std::endl;

		if (_actuator_max(i) < _actuator_min(i)) {
			// std::cout << "[ControlAllocation] 111111111111111111111111111111111 = " << std::endl;
			actuator(i) = _actuator_trim(i);

		} else if (actuator(i) < _actuator_min(i)) {
			// std::cout << "[ControlAllocation] 222222222222222222222222222222222 = " << std::endl;
			actuator(i) = _actuator_min(i);

		} else if (actuator(i) > _actuator_max(i)) {
			// std::cout << "[ControlAllocation] 33333333333333333333333333333333 = " << std::endl;
			actuator(i) = _actuator_max(i);
		}
		// std::cout << "[ControlAllocation] actuator after if [" << i << "] = " << actuator(i) << "  a = "<< a << std::endl;
	}
	// a++;
}

matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>
ControlAllocation::normalizeActuatorSetpoint(const matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> &actuator)
const
{
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_normalized;

	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_min(i) < _actuator_max(i)) {
			actuator_normalized(i) = (actuator(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));

		} else {
			actuator_normalized(i) = (_actuator_trim(i) - _actuator_min(i)) / (_actuator_max(i) - _actuator_min(i));
		}
	}

	return actuator_normalized;
}

void ControlAllocation::applySlewRateLimit(float dt)
{
	for (int i = 0; i < _num_actuators; i++) {
		if (_actuator_slew_rate_limit(i) > FLT_EPSILON) {
			float delta_sp_max = dt * (_actuator_max(i) - _actuator_min(i)) / _actuator_slew_rate_limit(i);
			float delta_sp = _actuator_sp(i) - _prev_actuator_sp(i);

			if (delta_sp > delta_sp_max) {
				_actuator_sp(i) = _prev_actuator_sp(i) + delta_sp_max;

			} else if (delta_sp < -delta_sp_max) {
				_actuator_sp(i) = _prev_actuator_sp(i) - delta_sp_max;
			}
		}
	}
}
