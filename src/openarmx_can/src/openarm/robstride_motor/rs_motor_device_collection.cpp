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

#include "openarm/robstride_motor/rs_motor_device_collection.hpp"

#include <iostream>
#include <stdexcept>

namespace openarm::robstride_motor {

RSMotorDeviceCollection::RSMotorDeviceCollection() : can_socket_(nullptr), device_collection_(nullptr) {
}

RSMotorDeviceCollection::RSMotorDeviceCollection(canbus::CANSocket* can_socket) : can_socket_(can_socket), device_collection_(nullptr) {
}

void RSMotorDeviceCollection::add_motor(std::unique_ptr<Motor> motor, bool use_fd) {
    uint32_t motor_id = motor->get_send_can_id();

    // 创建CAN设备 - 使用扩展帧掩码以接收Robstride反馈
    // 基于robstride桥接器：只需要匹配bit15~8的电机ID
    uint32_t extended_mask = 0xFF00;  // 掩码：只检查bit15~8的电机ID
    auto device = std::make_unique<RSCANDevice>(*motor, extended_mask, use_fd);

    // 存储指针用于快速查找
    motor_id_map_[motor_id] = motor.get();
    device_id_map_[motor_id] = device.get();

    // 按顺序存储电机和设备（像DaMiao原版一样）
    motors_.push_back(std::move(motor));
    devices_.push_back(std::move(device));
}

Motor* RSMotorDeviceCollection::get_motor(uint32_t motor_id) const {
    auto it = motor_id_map_.find(motor_id);
    return (it != motor_id_map_.end()) ? it->second : nullptr;
}

RSCANDevice* RSMotorDeviceCollection::get_device(uint32_t motor_id) const {
    auto it = device_id_map_.find(motor_id);
    return (it != device_id_map_.end()) ? it->second : nullptr;
}

std::vector<Motor*> RSMotorDeviceCollection::get_all_motors() const {
    std::vector<Motor*> result;
    for (const auto& motor : motors_) {
        result.push_back(motor.get());
    }
    return result;
}

std::vector<RSCANDevice*> RSMotorDeviceCollection::get_all_devices() const {
    std::vector<RSCANDevice*> result;
    for (const auto& device : devices_) {
        result.push_back(device.get());
    }
    return result;
}

std::vector<canbus::CANDevice*> RSMotorDeviceCollection::get_can_devices() const {
    std::vector<canbus::CANDevice*> result;
    for (const auto& device : devices_) {
        result.push_back(static_cast<canbus::CANDevice*>(device.get()));
    }
    return result;
}


void RSMotorDeviceCollection::send_motion_control_commands(const std::vector<MotionControlParam>& commands) {
    if (commands.size() != motors_.size()) {
        std::cerr << "命令数量与电机数量不匹配: commands=" << commands.size()
                  << ", motors=" << motors_.size() << std::endl;
        return;
    }

    // 使用索引访问，保持顺序
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_motion_control_command(*motor, commands[i]);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        // std::cout << "[DEBUG] 发送运动控制命令: Motor ID=" << motor->get_send_can_id()
        //           << ", CAN ID=0x" << std::hex << packet.send_can_id << std::dec
        //           << ", Position=" << commands[i].position
        //           << ", KP=" << commands[i].kp
        //           << ", KD=" << commands[i].kd << std::endl;

        // 发送CAN数据
        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        } else {
            std::cout << "[ERROR] CAN socket 为空，无法发送命令" << std::endl;
        }
    }
}

std::vector<StateResult> RSMotorDeviceCollection::get_all_motor_states() const {
    std::vector<StateResult> states;

    for (const auto& motor : motors_) {
        StateResult state;
        state.angle = motor->get_position();
        state.speed = motor->get_velocity();
        state.torque = motor->get_torque();
        state.temp = motor->get_temperature();
        state.pattern = motor->get_pattern();
        state.valid = true;

        states.push_back(state);
    }

    return states;
}

// New API methods to match damiao style
// NOTE: Removed get_motors() - now inline in header, returns pointers not copies

Motor* RSMotorDeviceCollection::get_motor(int i) const {
    if (i >= 0 && i < static_cast<int>(motors_.size())) {
        return motors_[i].get();
    }
    // Return nullptr if index is out of bounds
    return nullptr;
}

canbus::CANDeviceCollection& RSMotorDeviceCollection::get_device_collection() {
    if (!device_collection_) {
        throw std::runtime_error("Device collection not initialized");
    }
    return *device_collection_;
}

void RSMotorDeviceCollection::enable_all() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_enable_command(*motor);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        // 发送CAN数据
        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }

        motor->set_enabled(true);
        // std::cout << "启用电机 ID: " << motor->get_send_can_id() << std::endl;
    }
}

void RSMotorDeviceCollection::disable_all() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_disable_command(*motor);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        // 发送CAN数据
        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }

        motor->set_enabled(false);
        // std::cout << "禁用电机 ID: " << motor->get_send_can_id() << std::endl;
    }
}

void RSMotorDeviceCollection::set_zero_all() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_set_zero_command(*motor);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        // 发送CAN数据
        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }

        // std::cout << "设置电机零位 ID: " << motor->get_send_can_id() << std::endl;
    }
}

void RSMotorDeviceCollection::refresh_all() {
    // Send state request to all motors
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_state_request_command(*motor);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }
    }
}

void RSMotorDeviceCollection::refresh_one(int i) {
    if (i >= 0 && i < static_cast<int>(motors_.size())) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_state_request_command(*motor);
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }
    }
}

void RSMotorDeviceCollection::set_callback_mode_all(CallbackMode callback_mode) {
    for (auto& device : devices_) {
        device->set_callback_mode(callback_mode);
    }
}

void RSMotorDeviceCollection::query_param_all(int param_index) {
    for (size_t i = 0; i < motors_.size(); ++i) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_query_param_command(*motor, static_cast<uint16_t>(param_index));
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }
    }
}

void RSMotorDeviceCollection::query_param_one(int i, int param_index) {
    if (i >= 0 && i < static_cast<int>(motors_.size())) {
        auto* motor = motors_[i].get();
        auto* device = devices_[i].get();

        auto packet = CanPacketEncoder::create_query_param_command(*motor, static_cast<uint16_t>(param_index));
        auto frame = device->create_can_frame(packet.send_can_id, packet.data);

        if (can_socket_) {
            can_socket_->write_can_frame(frame);
        }
    }
}

}  // namespace openarm::robstride_motor