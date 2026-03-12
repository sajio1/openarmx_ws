# 1. 启动 rosbridge（Quest 3 WebSocket 桥）
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# 2. 启动 teleop_node（数据监控 + rich CLI 显示）
source /opt/ros/humble/setup.bash && source ~/Desktop/openarmx_ws/install/setup.bash
python3 ~/Desktop/openarmx_ws/teleop_node.py &

# 3. 启动可视化器（实时预览 + 关窗自动导出视频和CSV）
DISPLAY=:1 python3 ~/Desktop/openarmx_ws/teleop_visualizer.py &