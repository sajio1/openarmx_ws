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

#include "openarm/robstride_motor/rs_motor_control.hpp"

#include <array>
#include <cstring>
#include <map>
#include <iostream>
#include <iomanip>

namespace openarm::robstride_motor {

// ================= CanPacketEncoder Implementation =================

CANPacket CanPacketEncoder::create_enable_command(const Motor& motor) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::MOTOR_ENABLE, motor.get_send_can_id());
    // 按照Python版本实现：发送全0数据来使能电机
    std::vector<uint8_t> data(8, 0);
    packet.data = data;
    return packet;
}

CANPacket CanPacketEncoder::create_disable_command(const Motor& motor) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::MOTOR_STOP, motor.get_send_can_id());
    packet.data = pack_command_data(0x01);
    return packet;
}

CANPacket CanPacketEncoder::create_set_zero_command(const Motor& motor) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::SET_POS_ZERO, motor.get_send_can_id());
    // 按照Python版本实现：data_s[0] = 0x01，其余为0
    std::vector<uint8_t> data(8, 0);
    data[0] = 0x01;
    packet.data = data;
    return packet;
}

CANPacket CanPacketEncoder::create_set_control_mode_command(const Motor& motor, ControlMode mode) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::SET_SINGLE_PARAM, motor.get_send_can_id());

    // 按照Python版本实现：全部数据为0代表设置为运控模式
    // Python注释：数据位第[4]位 0：运控模式，1：位置模式，2：速度模式，3：电流模式
    // 但实际Python代码发送全0数据，即默认运控模式
    std::vector<uint8_t> data(8, 0);
    if (mode != ControlMode::MOTION_CONTROL) {
        data[4] = static_cast<uint8_t>(mode);  // 非运控模式时才设置
    }
    packet.data = data;
    return packet;
}

CANPacket CanPacketEncoder::create_motion_control_command(const Motor& motor,
                                                          const MotionControlParam& param) {
    CANPacket packet;

    // 扭矩参数编码到CAN ID中
    packet.send_can_id = generate_motion_control_id(motor.get_send_can_id(), param.torque);
    packet.data = pack_motion_control_data(param, motor.get_motor_type());
    return packet;
}

CANPacket CanPacketEncoder::create_state_request_command(const Motor& motor) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::MOTOR_REQUEST, motor.get_send_can_id());
    packet.data = pack_command_data(0x01);
    return packet;
}

CANPacket CanPacketEncoder::create_query_param_command(const Motor& motor, uint16_t param_index) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::GET_SINGLE_PARAM, motor.get_send_can_id());

    std::vector<uint8_t> data(8, 0);
    data[0] = (param_index >> 8) & 0xFF;
    data[1] = param_index & 0xFF;
    packet.data = data;
    return packet;
}

CANPacket CanPacketEncoder::create_set_param_command(const Motor& motor, uint16_t param_index, float value) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::SET_SINGLE_PARAM, motor.get_send_can_id());
    packet.data = pack_param_data(param_index, value);
    return packet;
}

CANPacket CanPacketEncoder::create_write_param_command(const Motor& motor, uint16_t param_index,
                                                       float value, ParamValueType value_type) {
    CANPacket packet;
    packet.send_can_id = generate_can_id(FunctionCode::SET_SINGLE_PARAM, motor.get_send_can_id());

    std::vector<uint8_t> data(8, 0);

    // 参数索引：低字节在data[0]，高字节在data[1]（按照Python实现）
    data[0] = param_index & 0xFF;
    data[1] = (param_index >> 8) & 0xFF;

    if (value_type == ParamValueType::FLOAT) {
        // 按小端写入到data[4:8]
        union {
            float f;
            uint8_t bytes[4];
        } value_union;
        value_union.f = value;

        data[4] = value_union.bytes[0];
        data[5] = value_union.bytes[1];
        data[6] = value_union.bytes[2];
        data[7] = value_union.bytes[3];
    } else if (value_type == ParamValueType::UINT8) {
        data[4] = static_cast<uint8_t>(value) & 0xFF;
    }

    packet.data = data;
    return packet;
}

// Private helper methods
std::vector<uint8_t> CanPacketEncoder::pack_motion_control_data(const MotionControlParam& param, const MotorType& motor_type_) {
    std::vector<uint8_t> data(8, 0);
    // std::cout << "[DEBUG pack] 66666666666666666666" << "\n" << std::endl;
    // 获取该电机类型的运控限制参数
    const MotionControlLimits& limits = MOTION_CONTROL_LIMITS[static_cast<std::size_t>(motor_type_)];

    // 调试输出: 打印电机类型和对应的运控限制参数
    // std::cout << "[DEBUG pack] =============== 运控限制参数 ===============" << "\n"
    //           << "电机类型: " << static_cast<int>(motor_type_) << "\n"
    //           << "控制角度: " << param.position << "\n"
    //           << "最小角度: " << limits.pMin << ", 最大角度: " << limits.pMax << "\n"
    //           << "最小速度: " << limits.vMin << ", 最大速度: " << limits.vMax << "\n"
    //           << "最小扭矩: " << limits.tMin << ", 最大扭矩: " << limits.tMax << "\n"
    //           << "最小KP: " << limits.kpMin << ", 最大KP: " << limits.kpMax << "\n"
    //           << "最小KD: " << limits.kdMin << ", 最大KD: " << limits.kdMax << std::endl;

    // 角度 (0-1字节)
    uint16_t pos_uint16 = float_to_uint16(param.position, limits.pMin, limits.pMax);
    data[0] = (pos_uint16 >> 8) & 0xFF;
    data[1] = pos_uint16 & 0xFF;

    // 速度 (2-3字节)
    uint16_t vel_uint16 = float_to_uint16(param.velocity, limits.vMin, limits.vMax);
    data[2] = (vel_uint16 >> 8) & 0xFF;
    data[3] = vel_uint16 & 0xFF;

    // KP (4-5字节)
    uint16_t kp_uint16 = float_to_uint16(param.kp, limits.kpMin, limits.kpMax);
    data[4] = (kp_uint16 >> 8) & 0xFF;
    data[5] = kp_uint16 & 0xFF;

    // KD (6-7字节)
    uint16_t kd_uint16 = float_to_uint16(param.kd, limits.kdMin, limits.kdMax);
    data[6] = (kd_uint16 >> 8) & 0xFF;
    data[7] = kd_uint16 & 0xFF;

    return data;
}

std::vector<uint8_t> CanPacketEncoder::pack_param_data(uint16_t param_index, float value) {
    std::vector<uint8_t> data(8, 0);

    // 参数索引 (0-1字节)
    data[0] = (param_index >> 8) & 0xFF;
    data[1] = param_index & 0xFF;

    // 参数值 (2-5字节，IEEE 754 float)
    union {
        float f;
        uint8_t bytes[4];
    } value_union;
    value_union.f = value;

    data[2] = value_union.bytes[0];
    data[3] = value_union.bytes[1];
    data[4] = value_union.bytes[2];
    data[5] = value_union.bytes[3];

    return data;
}

std::vector<uint8_t> CanPacketEncoder::pack_command_data(uint8_t cmd) {
    std::vector<uint8_t> data(8, 0);
    data[0] = cmd;
    return data;
}

double CanPacketEncoder::limit_value(double x, double min, double max) {
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

uint16_t CanPacketEncoder::float_to_uint16(double value, double min, double max) {
    double limited = limit_value(value, min, max);
    return static_cast<uint16_t>((limited - min) / (max - min) * 65535.0);
}

uint32_t CanPacketEncoder::generate_can_id(uint16_t function_code, uint8_t motor_id) {
    // 根据Python代码: 0x[功能码]fd00 + motorID
    // 例如: 0x0600fd00 + 5 = 0x0600fd05
    uint32_t base_id = (static_cast<uint32_t>(function_code) << 16) |
                       (static_cast<uint32_t>(HOST_ID) << 8);
    return base_id + motor_id;
}

uint32_t CanPacketEncoder::generate_motion_control_id(uint8_t motor_id, double torque) {
    // 根据电机ID获取电机类型
    MotorType motor_type = get_motor_type_by_id(motor_id);

    // 获取该电机类型的运控限制参数
    const MotionControlLimits& limits = MOTION_CONTROL_LIMITS[static_cast<std::size_t>(motor_type)];

    // 调试输出: 打印电机ID和对应的扭矩限制参数
    // std::cout << "电机ID: " << static_cast<int>(motor_id) << "\n"
    //           << "电机类型: " << static_cast<int>(motor_type) << "\n"
    //           << "最小位置: " << limits.pMin << "\n"
    //           << "最大位置: " << limits.pMax << "\n"
    //           << "torque: " << torque << "\n" << std::endl;

    // 根据Python代码: 0x01000000 | (torque_data << 8) | motorID
    // 注意: Python中torque_data是16位数据，左移8位后与motorID或运算
    // 使用该电机类型特定的扭矩限制
    uint16_t torque_uint16 = float_to_uint16(torque, limits.tMin, limits.tMax);
    return 0x01000000 | (static_cast<uint32_t>(torque_uint16) << 8) | static_cast<uint32_t>(motor_id);
}

// ================= CanPacketDecoder Implementation =================

StateResult CanPacketDecoder::parse_motor_state_data(const Motor& motor,
                                                      const std::vector<uint8_t>& data,
                                                      uint32_t /* can_id */) {
    StateResult result;
    result.valid = false;

    if (data.size() < 8) {
        return result;
    }

    const MotionControlLimits& limits = MOTION_CONTROL_LIMITS[static_cast<std::size_t>(motor.get_motor_type())];

    // 调试输出: 打印电机类型和对应的解析限制参数
    // std::cout << "[DEBUG parse] ================= 运控限制参数解析数据 =================\n"
    //           << "电机类型: " << static_cast<int>(motor.get_motor_type()) << "\n"
    //           << "最小扭矩: " << limits.tMin << ", 最大扭矩: " << limits.tMax << "\n"
    //           << "最小速度: " << limits.vMin << ", 最大速度: " << limits.vMax << "\n"
    //           << "最小位置: " << limits.pMin << ", 最大位置: " << limits.pMax << "\n" << std::endl;

    // 根据Robstride协议解析状态数据
    // 这里需要根据实际的Robstride反馈格式来实现
    // 目前基于Python代码中的反馈格式推测

    try {
        // 角度解析 (假设在前两个字节)
        uint16_t angle_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        result.angle = uint16_to_float(angle_raw, limits.pMin, limits.pMax);

        // 速度解析 (假设在3-4字节)
        uint16_t speed_raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
        result.speed = uint16_to_float(speed_raw, limits.vMin, limits.vMax);

        // 扭矩解析 (假设在5-6字节)
        uint16_t torque_raw = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        result.torque = uint16_to_float(torque_raw, limits.tMin, limits.tMax);

        // 温度解析 (Byte6~7) - 根据Python版本修正
        uint16_t temp_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];
        result.temp = static_cast<float>(temp_raw) / 10.0f;  // 温度×10
        result.pattern = 0;  // 暂时设为0，实际模式信息可能在其他地方

        result.valid = true;

        // DEBUG: 打印所有电机的前几次数据
        // {
        //     static std::map<uint8_t, int> debug_counts;
        //     uint8_t motor_id = motor.get_send_can_id() & 0xFF;
        //     if (debug_counts[motor_id]++ < 3) {
        //         std::cout << "[Motor" << (int)motor_id << " DEBUG] "
        //                   << "raw_bytes=[" << std::hex
        //                   << (int)data[0] << "," << (int)data[1] << "], "
        //                   << std::dec
        //                   << "angle_raw=" << angle_raw
        //                   << ", angle=" << result.angle
        //                   << ", limits=[" << limits.pMin << "," << limits.pMax << "]" << std::endl;
        //     }
        // }
    } catch (...) {
        result.valid = false;
    }

    return result;
}

ParamResult CanPacketDecoder::parse_param_data(const std::vector<uint8_t>& data) {
    ParamResult result;
    result.valid = false;

    if (data.size() < 6) {
        return result;
    }

    try {
        // 参数索引
        result.index = (static_cast<uint16_t>(data[0]) << 8) | data[1];

        // 参数值 (IEEE 754 float)
        union {
            float f;
            uint8_t bytes[4];
        } value_union;

        value_union.bytes[0] = data[2];
        value_union.bytes[1] = data[3];
        value_union.bytes[2] = data[4];
        value_union.bytes[3] = data[5];

        result.value = value_union.f;
        result.valid = true;
    } catch (...) {
        result.valid = false;
    }

    return result;
}

uint8_t CanPacketDecoder::parse_error_data(const std::vector<uint8_t>& data) {
    if (data.size() > 0) {
        return data[0];  // 错误码通常在第一个字节
    }
    return 0;
}

// Private helper methods
float CanPacketDecoder::uint16_to_float(uint16_t value, double min, double max) {
    return static_cast<float>(min + (static_cast<double>(value) / 65535.0) * (max - min));
}

float CanPacketDecoder::bytes_to_float(const std::array<uint8_t, 4>& bytes) {
    union {
        float f;
        uint8_t b[4];
    } value_union;

    std::memcpy(value_union.b, bytes.data(), 4);
    return value_union.f;
}

uint32_t CanPacketDecoder::bytes_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    return (static_cast<uint32_t>(byte1) << 24) |
           (static_cast<uint32_t>(byte2) << 16) |
           (static_cast<uint32_t>(byte3) << 8) |
           static_cast<uint32_t>(byte4);
}

bool CanPacketDecoder::is_state_feedback(uint32_t can_id) {
    uint16_t function_code = (can_id >> 16) & 0xFFFF;
    return function_code == FunctionCode::MOTOR_REQUEST;
}

bool CanPacketDecoder::is_param_feedback(uint32_t can_id) {
    uint16_t function_code = (can_id >> 16) & 0xFFFF;
    return function_code == FunctionCode::GET_SINGLE_PARAM ||
           function_code == FunctionCode::SET_SINGLE_PARAM;
}

bool CanPacketDecoder::is_error_feedback(uint32_t can_id) {
    uint16_t function_code = (can_id >> 16) & 0xFFFF;
    return function_code == FunctionCode::ERROR_FEEDBACK;
}

}  // namespace openarm::robstride_motor