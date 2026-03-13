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

#include "openarm/robstride_motor/rs_motor_device.hpp"

#include <iostream>

namespace openarm::robstride_motor {

RSCANDevice::RSCANDevice(Motor& motor, canid_t recv_can_mask, bool use_fd)
    : CANDevice(motor.get_send_can_id(), motor.get_recv_can_id(), recv_can_mask, use_fd),
      motor_(motor),
      callback_mode_(CallbackMode::STATE),
      use_fd_(use_fd) {}

void RSCANDevice::callback(const can_frame& frame) {
    // std::cout << "[DEBUG] 接收到CAN帧: ID=0x" << std::hex << frame.can_id << std::dec
    //           << ", DLC=" << (int)frame.can_dlc << std::endl;
    auto data = get_data_from_frame(frame);
    handle_received_data(data, frame.can_id);
}

void RSCANDevice::callback(const canfd_frame& frame) {
    auto data = get_data_from_frame(frame);
    handle_received_data(data, frame.can_id);
}

can_frame RSCANDevice::create_can_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    can_frame frame;
    frame.can_id = send_can_id | CAN_EFF_FLAG;  // 扩展帧
    frame.can_dlc = std::min(static_cast<size_t>(8), data.size());

    std::fill(frame.data, frame.data + 8, 0);
    std::copy(data.begin(), data.begin() + frame.can_dlc, frame.data);

    return frame;
}

canfd_frame RSCANDevice::create_canfd_frame(canid_t send_can_id, std::vector<uint8_t> data) {
    canfd_frame frame;
    frame.can_id = send_can_id | CAN_EFF_FLAG;  // 扩展帧
    frame.len = std::min(static_cast<size_t>(64), data.size());
    frame.flags = 0;

    std::fill(frame.data, frame.data + 64, 0);
    std::copy(data.begin(), data.begin() + frame.len, frame.data);

    return frame;
}

std::vector<uint8_t> RSCANDevice::get_data_from_frame(const can_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc);
}

std::vector<uint8_t> RSCANDevice::get_data_from_frame(const canfd_frame& frame) {
    return std::vector<uint8_t>(frame.data, frame.data + frame.len);
}

void RSCANDevice::handle_received_data(const std::vector<uint8_t>& data, uint32_t can_id) {
    // std::cout << "收到CAN数据: ID=0x" << std::hex << can_id << std::dec << std::endl;

    // 根据回调模式和CAN ID判断数据类型
    if (callback_mode_ == CallbackMode::IGNORE) {
        return;
    }

    // 检查CAN ID是否匹配这个电机
    // 根据实际观察，电机ID在bit15~8位置 (CAN ID格式: 0x828005fd，电机ID=05)
    uint8_t motor_id = (can_id >> 8) & 0xFF;
    uint8_t expected_motor_id = motor_.get_send_can_id() & 0xFF;
    // std::cout << "电机ID检查: 收到=" << (int)motor_id << ", 期望=" << (int)expected_motor_id << std::endl;

    if (motor_id != expected_motor_id) {
        std::cout << "电机ID不匹配，忽略此数据" << std::endl;
        return;  // 不是给这个电机的数据
    }

    // 根据Robstride协议判断数据类型
    // Python代码中通信类型在高位字节: int((ID_ExtId&0x3F000000)>>24)
    uint8_t comm_type = (can_id >> 24) & 0x3F;
    // std::cout << "通信类型: " << (int)comm_type << std::endl;

    switch (comm_type) {
        case 2:  // 运动控制反馈 (Python中的通信类型2)
            // std::cout << "运动控制反馈: " << (int)comm_type << std::endl;
            if (callback_mode_ == CallbackMode::STATE || callback_mode_ == CallbackMode::IGNORE) {
                handle_state_feedback(data, can_id);
            }
            break;

        case 17:  // 参数反馈 (Python中的通信类型17)
            if (callback_mode_ == CallbackMode::PARAM || callback_mode_ == CallbackMode::IGNORE) {
                handle_param_feedback(data);
            }
            break;

        default:
            std::cout << "未知通信类型: " << (int)comm_type << std::endl;
            // 尝试按状态反馈处理
            if (callback_mode_ == CallbackMode::STATE || callback_mode_ == CallbackMode::IGNORE) {
                handle_state_feedback(data, can_id);
            }
            break;
    }
}

void RSCANDevice::handle_state_feedback(const std::vector<uint8_t>& data, uint32_t can_id) {
    // std::cout << "处理状态反馈数据" << std::endl;
    auto result = CanPacketDecoder::parse_motor_state_data(motor_, data, can_id);
    if (result.valid) {
        // std::cout << "状态数据有效 - 位置:" << result.angle << ", 速度:" << result.speed
        //           << ", 扭矩:" << result.torque << ", 温度:" << result.temp << std::endl;
        motor_.update_state(result.angle, result.speed, result.torque, result.temp, result.pattern);
    } else {
        // std::cout << "状态数据无效" << std::endl;
    }
}

void RSCANDevice::handle_param_feedback(const std::vector<uint8_t>& data) {
    auto result = CanPacketDecoder::parse_param_data(data);
    if (result.valid) {
        motor_.set_param(result.index, result.value);
    }
}

void RSCANDevice::handle_error_feedback(const std::vector<uint8_t>& data) {
    uint8_t error_code = CanPacketDecoder::parse_error_data(data);
    motor_.set_error_code(error_code);
}

}  // namespace openarm::robstride_motor