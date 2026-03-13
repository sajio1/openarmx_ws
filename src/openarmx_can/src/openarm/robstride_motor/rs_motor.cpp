// Copyright 2025 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
// Website: https://openarmx.com/
// Contact: Mr Wang
// Phone & WeChat: +86-17746530375
// Email: openarmrobot@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author Information:
// Company: 成都长数机器人有限公司 (Chengdu Changsu Robot Co., Ltd.)
// Website: https://openarmx.com/
// Contact Person: Mr Wang
// Phone & WeChat: +86-17746530375
// Email: openarmrobot@gmail.com

#include "openarm/robstride_motor/rs_motor.hpp"

#include <stdexcept>

namespace openarm::robstride_motor {

Motor::Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id)
    : send_can_id_(send_can_id),
      recv_can_id_(recv_can_id),
      motor_type_(motor_type),
      enabled_(false),
      control_mode_(ControlMode::MOTION_CONTROL),
      state_angle_(0.0f),
      state_speed_(0.0f),
      state_torque_(0.0f),
      state_temp_(0.0f),
      state_pattern_(0),
      error_code_(0) {}

void Motor::update_state(float angle, float speed, float torque, float temp, int pattern) {
    state_angle_ = angle;
    state_speed_ = speed;
    state_torque_ = torque;
    state_temp_ = temp;
    state_pattern_ = pattern;
}

void Motor::set_enabled(bool enabled) {
    enabled_ = enabled;
}

void Motor::set_control_mode(ControlMode mode) {
    control_mode_ = mode;
}

void Motor::set_param(uint16_t index, float value) {
    param_dict_[index] = value;
}

void Motor::set_error_code(uint8_t error_code) {
    error_code_ = error_code;
}

float Motor::get_param(uint16_t index) const {
    auto it = param_dict_.find(index);
    if (it != param_dict_.end()) {
        return it->second;
    }
    return 0.0f;  // 默认值
}

LimitParam Motor::get_limit_param(MotorType motor_type) {
    if (static_cast<std::size_t>(motor_type) >= MOTOR_LIMIT_PARAMS.size()) {
        throw std::invalid_argument("Invalid motor type");
    }
    return MOTOR_LIMIT_PARAMS[static_cast<std::size_t>(motor_type)];
}

}  // namespace openarm::robstride_motor