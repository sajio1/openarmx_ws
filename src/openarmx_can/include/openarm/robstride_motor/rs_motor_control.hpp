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

#include <linux/can.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <vector>

#include "rs_motor.hpp"
#include "rs_motor_constants.hpp"

namespace openarm::robstride_motor {

// Forward declarations
class Motor;

struct ParamResult {
    uint16_t index;
    float value;
    bool valid;
};

struct StateResult {
    float angle;
    float speed;
    float torque;
    float temp;
    int pattern;
    bool valid;
};

struct CANPacket {
    uint32_t send_can_id;
    std::vector<uint8_t> data;
};

// 运控模式参数结构
struct MotionControlParam {
    double kp;
    double kd;
    double position;   // 目标位置 (rad)
    double velocity;   // 目标速度 (rad/s)
    double torque;     // 目标扭矩 (Nm)
};

// CSP模式参数结构
struct CSPControlParam {
    double position;        // 目标位置 (rad)
    double speed_limit;     // 速度限制 (rad/s)
    double current_limit;   // 电流限制 (N·m)
};

// 参数值类型枚举
enum class ParamValueType {
    FLOAT,
    UINT8
};

class CanPacketEncoder {
public:
    // 基本控制命令
    static CANPacket create_enable_command(const Motor& motor);
    static CANPacket create_disable_command(const Motor& motor);
    static CANPacket create_set_zero_command(const Motor& motor);
    static CANPacket create_set_control_mode_command(const Motor& motor, ControlMode mode);

    // 运控模式控制命令
    static CANPacket create_motion_control_command(const Motor& motor,
                                                   const MotionControlParam& param);

    // 参数操作命令
    static CANPacket create_query_param_command(const Motor& motor, uint16_t param_index);
    static CANPacket create_set_param_command(const Motor& motor, uint16_t param_index, float value);

    // CSP模式相关命令
    static CANPacket create_write_param_command(const Motor& motor, uint16_t param_index,
                                                float value, ParamValueType value_type);

    // 状态查询命令
    static CANPacket create_state_request_command(const Motor& motor);

private:
    // 数据打包辅助函数
    static std::vector<uint8_t> pack_motion_control_data(const MotionControlParam& param, const MotorType& motor_type_);
    static std::vector<uint8_t> pack_param_data(uint16_t param_index, float value);
    static std::vector<uint8_t> pack_command_data(uint8_t cmd);

    // 数值转换函数 (基于Robstride协议)
    static double limit_value(double x, double min, double max);
    static uint16_t float_to_uint16(double value, double min, double max);

    // CAN ID生成
    static uint32_t generate_can_id(uint16_t function_code, uint8_t motor_id);
    static uint32_t generate_motion_control_id(uint8_t motor_id, double torque);
};

class CanPacketDecoder {
public:
    // 解析电机状态反馈
    static StateResult parse_motor_state_data(const Motor& motor,
                                              const std::vector<uint8_t>& data,
                                              uint32_t can_id);

    // 解析参数反馈
    static ParamResult parse_param_data(const std::vector<uint8_t>& data);

    // 解析错误反馈
    static uint8_t parse_error_data(const std::vector<uint8_t>& data);

private:
    // 数值转换函数
    static float uint16_to_float(uint16_t value, double min, double max);
    static float bytes_to_float(const std::array<uint8_t, 4>& bytes);
    static uint32_t bytes_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

    // 状态解析辅助函数
    static bool is_state_feedback(uint32_t can_id);
    static bool is_param_feedback(uint32_t can_id);
    static bool is_error_feedback(uint32_t can_id);
};

// ================= CSP模式控制函数 =================
// 注意: 这些函数需要在实际的CAN设备类中实现，这里只提供生成CAN数据包的功能
// 实际使用时需要配合CAN发送和接收函数
//
// CSP模式使用流程（对应Python中的csp_move_to_flowwork）:
// 1. 禁用电机: create_disable_command()
// 2. 切换到CSP模式: csp_set_mode_packet()
// 3. 设置速度限制: csp_set_speed_limit_packet()
// 4. 设置电流限制: csp_set_current_limit_packet()
// 5. 使能电机: create_enable_command()
// 6. 设置目标位置: csp_set_target_position_packet()
//
// 示例代码:
// ```cpp
// // 完整的CSP控制流程
// Motor motor(1, MotorType::RS03);
//
// // 1. 禁用电机
// auto disable_packet = CanPacketEncoder::create_disable_command(motor);
// send_and_wait(disable_packet);
//
// // 2. 切换到CSP模式
// auto mode_packet = csp_set_mode_packet(motor);
// send_and_wait(mode_packet);
//
// // 3. 设置速度和电流限制
// auto speed_packet = csp_set_speed_limit_packet(motor, 5.0f);  // 5 rad/s
// send_and_wait(speed_packet);
//
// auto current_packet = csp_set_current_limit_packet(motor, 8.0f);  // 8 N·m
// send_and_wait(current_packet);
//
// // 4. 使能电机
// auto enable_packet = CanPacketEncoder::create_enable_command(motor);
// send_and_wait(enable_packet);
//
// // 5. 设置目标位置
// auto position_packet = csp_set_target_position_packet(motor, 1.2f);  // 1.2 rad
// send_and_wait(position_packet);
//
// // 对于循环控制（类似Python的csp_move_to）:
// for (float position : trajectory) {
//     auto packet = csp_set_target_position_packet(motor, position);
//     send(packet);  // 不等待响应，快速发送
//     std::this_thread::sleep_for(std::chrono::milliseconds(2));  // 2ms周期
// }
// ```

/**
 * @brief CSP模式设置 - 切换到位置模式(CSP)
 *
 * @param motor 电机对象
 * @return CANPacket 要发送的CAN数据包
 * @note 切换模式需在失能状态下，请先调用create_disable_command
 */
inline CANPacket csp_set_mode_packet(const Motor& motor) {
    // RUN_MODE = 0x7005, 5 = CSP
    return CanPacketEncoder::create_write_param_command(motor, 0x7005, 5.0f, ParamValueType::UINT8);
}

/**
 * @brief CSP模式速度限制设置
 *
 * @param motor 电机对象
 * @param speed_limit 速度限制 (rad/s)
 * @return CANPacket 要发送的CAN数据包
 */
inline CANPacket csp_set_speed_limit_packet(const Motor& motor, float speed_limit) {
    // LIMIT_SPD = 0x7017
    return CanPacketEncoder::create_write_param_command(motor, 0x7017, speed_limit, ParamValueType::FLOAT);
}

/**
 * @brief CSP模式电流限制设置
 *
 * @param motor 电机对象
 * @param current_limit 电流限制 (N·m)
 * @return CANPacket 要发送的CAN数据包
 */
inline CANPacket csp_set_current_limit_packet(const Motor& motor, float current_limit) {
    // LIMIT_CUR = 0x7018
    return CanPacketEncoder::create_write_param_command(motor, 0x7018, current_limit, ParamValueType::FLOAT);
}

/**
 * @brief CSP模式目标位置设置
 *
 * @param motor 电机对象
 * @param position 目标位置 (rad)
 * @return CANPacket 要发送的CAN数据包
 */
inline CANPacket csp_set_target_position_packet(const Motor& motor, float position) {
    // LOC_REF = 0x7016
    return CanPacketEncoder::create_write_param_command(motor, 0x7016, position, ParamValueType::FLOAT);
}

}  // namespace openarm::robstride_motor