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
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocationPseudoInverse.hpp"

void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;
	_normalization_needs_update = update_normalization_scale;
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	// _mix_update_needed 는 CA_METHOD 가 바뀔때만 1로 되고 바뀌고 나면 다시 0으로 된다.
	// 그러니깐 안봐도 되는게 아니다 처음 실행할때 한번은 들어온다
	if (_mix_update_needed) {
		// std::cout << "[ControlAllocationPseudoInverse] _mix_update_needed = " << _mix_update_needed << std::endl;
		/**
		 * geninv 함수는 내가 가지고 있는 control allocatioin matrix 를 유사역행렬을 이용하여 matrix를 구하는거다
		 * 계산할 원본 행렬 MbyN인 G(= _effectiveness) 이다 / 계산된 유사행렬이 저장될 행렬은 NbyM인 res(= _mix) 이다
		 */

		// _effectiveness(2,0) = 0.5;
		// _effectiveness(2,1) = -0.5;
		// _effectiveness(2,2) = 0.5;
		// _effectiveness(2,3) = -0.5;
		// _effectiveness(2,4) = -0.5;
		// _effectiveness(2,5) = 0.5;
		// _effectiveness(2,6) = 0.5;
		// _effectiveness(2,7) = -0.5;
		// _effectiveness(0,0) = 0.f;
		// _effectiveness(1,0) = 0.f;
		// _effectiveness(2,0) = 0.f;
		// _effectiveness(3,0) = 0.f;
		// _effectiveness(4,0) = 0.f;
		// _effectiveness(5,0) = 0.f;


		matrix::geninv(_effectiveness, _mix);
		std::cout << "[ControlAllocationPseudoInverse] _effectiveness = " << _effectiveness << std::endl;
		std::cout << "[ControlAllocationPseudoInverse] _mix = " << _mix << std::endl;

		if (_normalization_needs_update && !_had_actuator_failure) {
			// std::cout << "[ControlAllocationPseudoInverse] hihihihihihhi" << std::endl;
			updateControlAllocationMatrixScale();
			_normalization_needs_update = false;
		}

		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
}

void
ControlAllocationPseudoInverse::updateControlAllocationMatrixScale()
{
	// Same scale on roll and pitch
	/**
	 * ControlAllocator.cpp 파일에서 _control_allocation[i]->setNormalizeRPY(normalize_rpy[i]); 를 통해
	 * _normalize_rpy에 true값을 넣었다
	 */
	// std::cout << "[ControlAllocationPseudoInverse] _mix(1, 0) = " << _mix(1, 0) << std::endl;

	if (_normalize_rpy) {
		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int i = 0; i < _num_actuators; i++) {

			/**
			 * _num_actuators : 옥토면 8 쿼드면 4
			 * abs : int 절댓값을 반환
			 * fabs : double형 실수 절댓값을 반환
			 * fabsf : float형 실수 절댓값을 반환
			 */
			if (fabsf(_mix(i, 0)) > 1e-3f) {
				++num_non_zero_roll_torque;
				// std::cout << "[ControlAllocationPseudoInverse] num_non_zero_roll_torque = " << num_non_zero_roll_torque << std::endl;
			}

			if (fabsf(_mix(i, 1)) > 1e-3f) {
				++num_non_zero_pitch_torque;
				// std::cout << "[ControlAllocationPseudoInverse] num_non_zero_pitch_torque = " << num_non_zero_pitch_torque << std::endl;
			}
		}

		float roll_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
			// std::cout << "[ControlAllocationPseudoInverse] roll_norm_scale = " << roll_norm_scale << std::endl;
			// roll_norm_scale = 0.325556
		}

		float pitch_norm_scale = 1.f;

		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(_mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
			// std::cout << "[ControlAllocationPseudoInverse] pitch_norm_scale = " << pitch_norm_scale << std::endl;
			// pitch_norm_scale = 0.219603
		}

		// fmaxf : float 형태로 max값 반환
		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);
		// _control_allocation_scale(0), (1)에는 0.325556 값이 들어간다

		// Scale yaw separately
		_control_allocation_scale(2) = _mix.col(2).max();
		// _control_allocation_scale(2) 에는 0 값이 들어간다

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale thrust by the sum of the individual thrust axes, and use the scaling for the Z axis if there's no actuators
	// (for tilted actuators)
	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
				// std::cout << "[ControlAllocationPseudoInverse] num_non_zero_thrust = " << num_non_zero_thrust << std::endl;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationPseudoInverse::normalizeControlAllocationMatrix()
{
	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
}

void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
