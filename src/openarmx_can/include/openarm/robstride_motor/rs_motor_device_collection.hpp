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

#include <memory>
#include <unordered_map>
#include <vector>

#include "../canbus/can_device_collection.hpp"
#include "rs_motor.hpp"
#include "rs_motor_device.hpp"

namespace openarm::robstride_motor {

class RSMotorDeviceCollection {
public:
    RSMotorDeviceCollection();
    RSMotorDeviceCollection(canbus::CANSocket* can_socket);
    ~RSMotorDeviceCollection() = default;

    // 添加电机设备
    void add_motor(std::unique_ptr<Motor> motor, bool use_fd = false);

    // 获取电机设备
    Motor* get_motor(uint32_t motor_id) const;
    RSCANDevice* get_device(uint32_t motor_id) const;

    // Motor access (matching damiao API style)
    // NOTE: get_motors() returns POINTERS to actual motors (not copies) so state changes are visible
    std::vector<Motor*> get_motors() const { return get_all_motors(); }
    Motor* get_motor(int i) const;
    std::vector<Motor*> get_all_motors() const;
    std::vector<RSCANDevice*> get_all_devices() const;

    // CAN device collection access
    std::vector<canbus::CANDevice*> get_can_devices() const;
    canbus::CANDeviceCollection& get_device_collection();

    // Motor operations (matching damiao API style)
    void enable_all();
    void disable_all();
    void set_zero_all();
    void refresh_all();
    void refresh_one(int i);
    void set_callback_mode_all(CallbackMode callback_mode);
    void query_param_all(int param_index);
    void query_param_one(int i, int param_index);

    // Motion control operations
    void send_motion_control_commands(const std::vector<MotionControlParam>& commands);

    // State retrieval
    std::vector<StateResult> get_all_motor_states() const;

private:
    // 使用vector保持顺序，像DaMiao原版一样
    std::vector<std::unique_ptr<Motor>> motors_;
    std::vector<std::unique_ptr<RSCANDevice>> devices_;
    // 保留unordered_map用于按ID快速查找
    std::unordered_map<uint32_t, Motor*> motor_id_map_;
    std::unordered_map<uint32_t, RSCANDevice*> device_id_map_;
    canbus::CANSocket* can_socket_;
    std::unique_ptr<canbus::CANDeviceCollection> device_collection_;
};

}  // namespace openarm::robstride_motor