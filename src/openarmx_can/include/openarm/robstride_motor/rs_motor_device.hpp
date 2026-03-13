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

#include "../canbus/can_device.hpp"
#include "../canbus/can_socket.hpp"
#include "rs_motor.hpp"
#include "rs_motor_control.hpp"

namespace openarm::robstride_motor {

enum CallbackMode {
    STATE,
    PARAM,
    ERROR,
    IGNORE
};

class RSCANDevice : public canbus::CANDevice {
public:
    explicit RSCANDevice(Motor& motor, canid_t recv_can_mask, bool use_fd);
    void callback(const can_frame& frame) override;
    void callback(const canfd_frame& frame) override;

    // 创建CAN帧
    can_frame create_can_frame(canid_t send_can_id, std::vector<uint8_t> data);
    canfd_frame create_canfd_frame(canid_t send_can_id, std::vector<uint8_t> data);

    // 获取电机实例
    Motor& get_motor() { return motor_; }

    // 设置回调模式
    void set_callback_mode(CallbackMode callback_mode) { callback_mode_ = callback_mode; }

private:
    // 从CAN帧中提取数据
    std::vector<uint8_t> get_data_from_frame(const can_frame& frame);
    std::vector<uint8_t> get_data_from_frame(const canfd_frame& frame);

    // 处理接收到的数据
    void handle_received_data(const std::vector<uint8_t>& data, uint32_t can_id);

    // 处理不同类型的反馈
    void handle_state_feedback(const std::vector<uint8_t>& data, uint32_t can_id);
    void handle_param_feedback(const std::vector<uint8_t>& data);
    void handle_error_feedback(const std::vector<uint8_t>& data);

    Motor& motor_;
    CallbackMode callback_mode_;
    bool use_fd_;
};

}  // namespace openarm::robstride_motor