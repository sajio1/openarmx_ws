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

#pragma once

#include <cstdint>
#include <cstring>
#include <map>

#include "rs_motor_constants.hpp"

namespace openarm::robstride_motor {

class Motor {
    friend class RSCANDevice;
    friend class RSControl;
    friend class RSMotorDeviceCollection;

public:
    // Constructor
    Motor(MotorType motor_type, uint32_t send_can_id, uint32_t recv_can_id);

    // State getters
    double get_position() const { return state_angle_; }
    double get_velocity() const { return state_speed_; }
    double get_torque() const { return state_torque_; }
    float get_temperature() const { return state_temp_; }
    int get_pattern() const { return state_pattern_; }  // 电机模式（0复位1标定2运行）

    // Motor property getters
    uint32_t get_send_can_id() const { return send_can_id_; }
    uint32_t get_recv_can_id() const { return recv_can_id_; }
    MotorType get_motor_type() const { return motor_type_; }

    // Enable status getters
    bool is_enabled() const { return enabled_; }
    ControlMode get_control_mode() const { return control_mode_; }

    // Parameter methods
    float get_param(uint16_t index) const;
    float get_param(int index) const { return get_param(static_cast<uint16_t>(index)); }  // For API compatibility
    bool has_error() const { return error_code_ != 0; }
    uint8_t get_error_code() const { return error_code_; }

    // Static methods for motor properties
    static LimitParam get_limit_param(MotorType motor_type);

protected:
    // State update methods
    void update_state(float angle, float speed, float torque, float temp, int pattern);
    void set_enabled(bool enabled);
    void set_control_mode(ControlMode mode);
    void set_param(uint16_t index, float value);
    void set_error_code(uint8_t error_code);

    // Motor identifiers
    uint32_t send_can_id_;
    uint32_t recv_can_id_;
    MotorType motor_type_;

    // Control state
    bool enabled_;
    ControlMode control_mode_;

    // Current state (基于Robstride反馈格式)
    float state_angle_;    // 角度 (rad)
    float state_speed_;    // 速度 (rad/s)
    float state_torque_;   // 扭矩 (Nm)
    float state_temp_;     // 温度 (°C)
    int state_pattern_;    // 电机模式（0复位1标定2运行）

    // Error handling
    uint8_t error_code_;

    // Parameter storage
    std::map<uint16_t, float> param_dict_;
};

}  // namespace openarm::robstride_motor