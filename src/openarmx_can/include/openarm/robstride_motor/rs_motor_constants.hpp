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

#include <array>
#include <cstddef>
#include <cstdint>

namespace openarm::robstride_motor {

enum class MotorType : uint8_t {
    RS00 = 0,  // 对应原来的DM4310 - 关节5-7, 夹爪
    RS03 = 1,  // 对应原来的DM4340 - 关节3-4
    RS04 = 2,  // 对应原来的DM8009 - 关节1-2
    COUNT = 3
};

enum class ControlMode : uint8_t {
    MOTION_CONTROL = 0,  // 运控模式
    POSITION = 1,        // 位置模式
    VELOCITY = 2,        // 速度模式
    CURRENT = 3,         // 电流模式
    ZERO_POSITION = 4    // 零点模式
};

// 通信功能码
namespace FunctionCode {
    constexpr uint16_t GET_ID = 0x0000;               // 获取设备ID
    constexpr uint16_t MOTION_CONTROL = 0x0100;      // 运控模式控制
    constexpr uint16_t MOTOR_REQUEST = 0x0200;       // 电机状态请求
    constexpr uint16_t MOTOR_ENABLE = 0x0300;        // 电机使能
    constexpr uint16_t MOTOR_STOP = 0x0400;          // 电机停止
    constexpr uint16_t SET_POS_ZERO = 0x0600;        // 设置机械零位
    constexpr uint16_t CAN_ID = 0x0700;              // 更改CAN ID
    constexpr uint16_t GET_SINGLE_PARAM = 0x1100;    // 读取单个参数
    constexpr uint16_t SET_SINGLE_PARAM = 0x1200;    // 设定单个参数/控制模式
    constexpr uint16_t ERROR_FEEDBACK = 0x1500;      // 故障反馈
}

// 主机ID
constexpr uint8_t HOST_ID = 0xFD;

// 运控模式限制参数
// 运控模式限制参数结构体
struct MotionControlLimits {
    double pMin;   // 扭矩下限 (Nm)
    double pMax;   // 扭矩上限 (Nm)
    double tMin;   // 角度下限 (rad)
    double tMax;   // 角度上限 (rad)
    double vMin;   // 速度下限 (rad/s)
    double vMax;   // 速度上限 (rad/s)
    double kpMin;  // KP下限
    double kpMax;  // KP上限
    double kdMin;  // KD下限
    double kdMax;  // KD上限
};

// 根据电机型号设定的运控模式限制参数，
inline constexpr std::array<MotionControlLimits, static_cast<std::size_t>(MotorType::COUNT)>
    MOTION_CONTROL_LIMITS = {{
        // RS00 (替代DM4310) - 关节5-7, 夹爪
        {-12.57, 12.57, -14.0, 14.0, -33.0, 33.0, 0.0, 500.0, 0.0, 5.0},
        // RS03 (替代DM4340) - 关节3-4
        {-12.57, 12.57, -60.0, 60.0, -20.0, 20.0, 0.0, 5000.0, 0.0, 100.0},
        // RS04 (替代DM8009) - 关节1-2
        {-12.57, 12.57, -120.0, 120.0, -15.0, 15.0, 0.0, 5000.0, 0.0, 100.0},
    }};

// 兼容旧代码的全局常量 (使用RS00的参数作为默认值)
constexpr double P_MIN = -12.5;    // 扭矩下限 (Nm)
constexpr double P_MAX = 12.5;     // 扭矩上限 (Nm)
constexpr double T_MIN = -12.0;    // 角度下限 (rad)
constexpr double T_MAX = 12.0;     // 角度上限 (rad)
constexpr double V_MIN = -30.0;    // 速度下限 (rad/s)
constexpr double V_MAX = 30.0;     // 速度上限 (rad/s)
constexpr double KP_MIN = 0.0;     // KP下限
constexpr double KP_MAX = 500.0;   // KP上限
constexpr double KD_MIN = 0.0;     // KD下限
constexpr double KD_MAX = 5.0;     // KD上限

// 每种电机型号的限制参数
struct LimitParam {
    double pMax;  // 位置限制 (rad)
    double vMax;  // 速度限制 (rad/s)
    double tMax;  // 扭矩限制 (Nm)
    double kpMax;  // KP上限
    double kdMax;  // KD上限
};

// 根据Robstride电机规格设定的限制参数，P_MAX,V_MAX,T_MAX,KP_MAX,KD_MAX
inline constexpr std::array<LimitParam, static_cast<std::size_t>(MotorType::COUNT)>
    MOTOR_LIMIT_PARAMS = {{
        {12.57, 33, 14, 500.0, 5.0},      // RS00 (替代DM4310) - 关节5-7, 夹爪
        {12.57, 20, 60, 5000.0, 100.0},   // RS03 (替代DM4340) - 关节3-4
        {12.57, 15, 120, 5000.0, 100.0},  // RS04 (替代DM8009) - 关节1-2
    }};

// 电机ID到电机类型的映射结构
struct MotorIDMapping {
    uint8_t motor_id;
    MotorType motor_type;
};

// 电机ID映射表 (最新版配置: ID 1-2=RS04, 3-4=RS03, 5-8=RS00)
inline constexpr std::array<MotorIDMapping, 8> MOTOR_ID_MAP = {{
    {1, MotorType::RS04},  // Joint 1 (新版: RS04替代DM8009)
    {2, MotorType::RS04},  // Joint 2 (新版: RS04替代DM8009)
    {3, MotorType::RS03},  // Joint 3 (新版: RS03替代DM4340)
    {4, MotorType::RS03},  // Joint 4 (新版: RS03替代DM4340)
    {5, MotorType::RS00},  // Joint 5
    {6, MotorType::RS00},  // Joint 6
    {7, MotorType::RS00},  // Joint 7
    {8, MotorType::RS00},  // Gripper
}};

// 根据电机ID获取电机类型的辅助函数
inline MotorType get_motor_type_by_id(uint8_t motor_id) {
    for (const auto& mapping : MOTOR_ID_MAP) {
        if (mapping.motor_id == motor_id) {
            return mapping.motor_type;
        }
    }
    // 默认返回RS00 (如果ID不在映射表中)
    return MotorType::RS00;
}

// 参数索引
namespace ParamIndex {
    constexpr uint16_t RUN_MODE = 0x7005;       // 运行模式
    constexpr uint16_t IQ_REF = 0x7006;         // 电流模式Iq指令
    constexpr uint16_t SPD_REF = 0x700A;        // 转速模式转速指令
    constexpr uint16_t LIMIT_TORQUE = 0x700B;   // 转矩限制
    constexpr uint16_t CUR_KP = 0x7010;         // 电流Kp
    constexpr uint16_t CUR_KI = 0x7011;         // 电流Ki
    constexpr uint16_t CUR_FILT_GAIN = 0x7014;  // 电流滤波系数
    constexpr uint16_t LOC_REF = 0x7016;        // 位置模式角度指令
    constexpr uint16_t LIMIT_SPD = 0x7017;      // 位置模式速度设置
    constexpr uint16_t LIMIT_CUR = 0x7018;      // 速度位置模式电流设置
    constexpr uint16_t MECH_POS = 0x7019;       // 负载端计圈机械角度 (只读)
    constexpr uint16_t IQF = 0x701A;            // iq滤波值 (只读)
    constexpr uint16_t MECH_VEL = 0x701B;       // 负载端转速 (只读)
    constexpr uint16_t VBUS = 0x701C;           // 母线电压 (只读)
    constexpr uint16_t ROTATION = 0x701D;       // 圈数 (只读)
}

}  // namespace openarm::robstride_motor