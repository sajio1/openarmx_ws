#!/bin/bash
# Shadow Mode 快速启动脚本
# 使用: 
#   ./shadow_mode/start.sh              # 正常启动 (真实硬件)
#   ./shadow_mode/start.sh --zero (-z)  # 启动前校准零点
#   ./shadow_mode/start.sh --vr         # 启动 ROS2 + rosbridge (VR遥操作)
#   ./shadow_mode/start.sh -z --vr      # 校准 + ROS2 + rosbridge
#   ./shadow_mode/start.sh --sim        # 模拟模式 (无需硬件)
#   ./shadow_mode/start.sh --sim --vr   # 模拟 + VR (测试遥操作)

cd "$(dirname "$0")/.."
python3 shadow_mode/start_shadow.py "$@"
