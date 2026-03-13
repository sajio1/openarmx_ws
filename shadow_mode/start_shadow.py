#!/usr/bin/env python3
"""
OpenArmX 开机脚本
=================
一键启动：CAN → ROS2 → forward_position_controller

使用方法:
    python3 shadow_mode/start_shadow.py              # 正常启动 (真实硬件)
    python3 shadow_mode/start_shadow.py --zero       # 启动前设置零点
    python3 shadow_mode/start_shadow.py --vr         # 启动 ROS2 + rosbridge (VR 遥操作)
    python3 shadow_mode/start_shadow.py -z --vr      # 校准 + ROS2 + rosbridge
    python3 shadow_mode/start_shadow.py --sim        # 模拟模式 (无需硬件，用于开发测试)
    python3 shadow_mode/start_shadow.py --sim --vr   # 模拟 + VR (测试遥操作)

按 Ctrl+C 安全退出（自动清理 CAN）
"""

import os
import sys
import time
import subprocess
import signal
import argparse
from pathlib import Path

# ═══════════════════════════════════════════════════════════════════════════════
# 配置
# ═══════════════════════════════════════════════════════════════════════════════

WORKSPACE = Path(__file__).resolve().parent.parent
RIGHT_CAN = "can2"
LEFT_CAN = "can3"
CAN_BITRATE = 1000000
SUDO_PASSWORD = "123456"

ROS_DISTRO = "humble"
ROS_SETUP = f"/opt/ros/{ROS_DISTRO}/setup.bash"
WS_SETUP = str(WORKSPACE / "install" / "setup.bash")

# ═══════════════════════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════════════════════

def run_sudo(cmd: str) -> subprocess.CompletedProcess:
    return subprocess.run(f"echo {SUDO_PASSWORD} | sudo -S {cmd}", 
                          shell=True, capture_output=True, text=True)

def source_ros_cmd(cmd: str) -> str:
    return f"source {ROS_SETUP} && source {WS_SETUP} && {cmd}"

def print_step(step: int, total: int, msg: str):
    print(f"\n{'='*60}")
    print(f"[{step}/{total}] {msg}")
    print('='*60)

def print_ok(msg: str):
    print(f"  ✓ {msg}")

def print_err(msg: str):
    print(f"  ✗ {msg}")

def print_info(msg: str):
    print(f"  → {msg}")

def get_local_ip() -> str:
    """获取本机局域网 IP 地址"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"

# ═══════════════════════════════════════════════════════════════════════════════
# 步骤
# ═══════════════════════════════════════════════════════════════════════════════

def step_0_calibrate_zero() -> bool:
    """设置所有电机零点"""
    print_step(0, 4, "校准零点")
    
    try:
        import sys as _sys
        _sys.path.insert(0, str(WORKSPACE / "src" / "openarmx_arm_driver"))
        from openarmx_arm_driver import Robot
        
        print_info("正在连接电机...")
        robot = Robot(right_can_channel=RIGHT_CAN, left_can_channel=LEFT_CAN)
        
        print_info("设置右臂零点...")
        for i in range(1, 9):
            robot.right_arm.set_zero(i)
            time.sleep(0.05)
        print_ok("右臂零点已设置")
        
        print_info("设置左臂零点...")
        for i in range(1, 9):
            robot.left_arm.set_zero(i)
            time.sleep(0.05)
        print_ok("左臂零点已设置")
        
        robot.shutdown()
        print_ok("校准完成，当前位置已记录为零点")
        return True
        
    except Exception as e:
        print_err(f"校准失败: {e}")
        return False


def step_1_setup_can() -> bool:
    """启动 CAN 接口"""
    print_step(1, 3, "启动 CAN 接口")
    
    for can in [RIGHT_CAN, LEFT_CAN]:
        run_sudo(f"ip link set {can} down 2>/dev/null")
        time.sleep(0.1)
        run_sudo(f"ip link set {can} type can bitrate {CAN_BITRATE}")
        result = run_sudo(f"ip link set {can} up")
        if result.returncode != 0 and "busy" not in result.stderr.lower():
            print_err(f"{can} 启动失败")
            return False
        print_ok(f"{can} 已启动")
    
    time.sleep(0.3)
    print_ok("CAN 接口已就绪")
    return True


def step_2_launch_ros2(sim_mode: bool = False) -> subprocess.Popen | None:
    """启动 ROS2 控制"""
    mode_name = "模拟模式" if sim_mode else "真实硬件"
    print_step(2, 3, f"启动 ROS2 控制系统 ({mode_name})")
    
    # robot_controller:=forward_position_controller 会跳过 gripper_controller 启动
    if sim_mode:
        cmd = source_ros_cmd(
            f"ros2 launch openarmx_bringup openarm.bimanual.launch.py "
            f"use_fake_hardware:=true "
            f"robot_controller:=forward_position_controller"
        )
    else:
        cmd = source_ros_cmd(
            f"ros2 launch openarmx_bringup openarm.bimanual.launch.py "
            f"right_can_interface:={RIGHT_CAN} left_can_interface:={LEFT_CAN} "
            f"robot_controller:=forward_position_controller"
        )
    
    print_info("正在启动 ros2 launch...")
    
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )
    
    print_info("等待 ROS2 初始化 (6秒)...")
    time.sleep(6)
    
    if proc.poll() is not None:
        print_err("ROS2 启动失败")
        return None
    
    print_ok(f"ROS2 控制系统已启动 ({mode_name})")
    return proc


def step_3_verify_controllers() -> bool:
    """验证 forward_position_controller 已激活"""
    print_step(3, 3, "验证控制器状态")
    
    # 检查 topic 是否存在
    cmd = source_ros_cmd("ros2 topic list | grep forward_position_controller")
    result = subprocess.run(cmd, shell=True, executable="/bin/bash",
                            capture_output=True, text=True, timeout=15)
    
    if "/right_forward_position_controller/commands" in result.stdout:
        print_ok("right_forward_position_controller 已激活")
    else:
        print_err("right_forward_position_controller 未找到")
        return False
    
    if "/left_forward_position_controller/commands" in result.stdout:
        print_ok("left_forward_position_controller 已激活")
    else:
        print_err("left_forward_position_controller 未找到")
        return False
    
    # 检查 gripper_controller 没有启动
    cmd = source_ros_cmd("ros2 topic list | grep gripper_controller")
    result = subprocess.run(cmd, shell=True, executable="/bin/bash",
                            capture_output=True, text=True, timeout=15)
    if "gripper_controller" not in result.stdout:
        print_ok("gripper_controller 未启动 (正常)")
    else:
        print_info("gripper_controller 已启动 (不影响使用)")
    
    return True


def step_4_start_rosbridge() -> subprocess.Popen | None:
    """启动 rosbridge WebSocket 服务 (用于 Quest 3 VR 遥操作)"""
    print_step(4, 5, "启动 rosbridge WebSocket")
    
    cmd = source_ros_cmd("ros2 launch rosbridge_server rosbridge_websocket_launch.xml")
    
    print_info("正在启动 rosbridge...")
    
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )
    
    print_info("等待 rosbridge 初始化 (3秒)...")
    time.sleep(3)
    
    if proc.poll() is not None:
        print_err("rosbridge 启动失败")
        return None
    
    print_ok("rosbridge WebSocket 已启动 (端口 9090)")
    return proc


def step_5_start_vr_controller(
    enable_viz: bool = False,
    no_clutch: bool = False,
    input_unity: bool = False,
    rot_offset: bool = False,
    debug_topics: bool = False,
) -> subprocess.Popen | None:
    """启动 VR IK 控制节点 (vr_controller)"""
    print_step(5, 5, "启动 VR IK 控制节点")

    # 清除 Python 字节码缓存，确保源码改动立即生效
    import shutil
    for cache_dir in (WORKSPACE / "shadow_mode").rglob("__pycache__"):
        shutil.rmtree(cache_dir, ignore_errors=True)
    print_info("已清除 __pycache__ 缓存")
    
    # 构建命令
    flags = ""
    if enable_viz:
        flags += " --viz"
    if no_clutch:
        flags += " --no-clutch"
        print_info("[调试模式] Clutch 检查已禁用，收到 VR 数据即开始控制!")
    if input_unity:
        print_info("[默认] 输入按 Unity 左手坐标系处理")
    if rot_offset:
        flags += " --rot-offset"
        print_info("[调试模式] TCP 180°旋转补偿已启用")
    if debug_topics:
        flags += " --debug-topics"
        print_info("[调试模式] /debug/* 诊断 topic 已启用")
    cmd = source_ros_cmd(f"python3 -m shadow_mode.vr_controller{flags}")
    
    print_info("正在启动 vr_controller...")
    
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )
    
    print_info("等待 IK 初始化 (3秒)...")
    time.sleep(3)
    
    if proc.poll() is not None:
        print_err("vr_controller 启动失败")
        # 尝试读取错误输出
        try:
            out, _ = proc.communicate(timeout=1)
            if out:
                print_err(f"错误信息: {out.decode()[-500:]}")
        except:
            pass
        return None
    
    print_ok("vr_controller 已启动 (VR IK 控制节点)")
    return proc


def step_6_start_replay(jsonl_file: str, rate: float = 1.0) -> subprocess.Popen | None:
    """启动 Quest 消息回放 (jsonl)."""
    print_step(6, 6, "启动 Quest 消息回放")
    replay_path = Path(jsonl_file).expanduser().resolve()
    if not replay_path.exists():
        print_err(f"回放文件不存在: {replay_path}")
        return None

    cmd = source_ros_cmd(
        f"python3 shadow_mode/replay_quest_stream.py replay --file \"{replay_path}\" --rate {rate}"
    )
    print_info(f"正在回放: {replay_path}")
    print_info(f"回放倍率: x{rate}")
    proc = subprocess.Popen(
        cmd,
        shell=True,
        executable="/bin/bash",
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=str(WORKSPACE),
        preexec_fn=os.setsid,
    )
    time.sleep(1)
    if proc.poll() is not None:
        print_err("回放进程启动失败")
        return None
    print_ok("回放进程已启动")
    return proc


def disable_motors():
    """禁用所有电机"""
    try:
        # 添加路径
        import sys
        sys.path.insert(0, str(WORKSPACE / "src" / "openarmx_arm_driver"))
        from openarmx_arm_driver import Robot
        
        print_info("正在禁用电机...")
        robot = Robot(right_can_channel=RIGHT_CAN, left_can_channel=LEFT_CAN)
        robot.right_arm.disable_all()
        robot.left_arm.disable_all()
        robot.shutdown()
        print_ok("电机已禁用")
        return True
    except Exception as e:
        print_err(f"禁用电机失败: {e}")
        return False


def cleanup(ros2_proc, rosbridge_proc=None, vr_ctrl_proc=None, replay_proc=None, sim_mode: bool = False):
    """清理"""
    print("\n\n正在安全关闭...")
    
    # 关闭 vr_controller
    if vr_ctrl_proc and vr_ctrl_proc.poll() is None:
        try:
            os.killpg(os.getpgid(vr_ctrl_proc.pid), signal.SIGTERM)
            vr_ctrl_proc.wait(timeout=3)
            print_ok("vr_controller 已关闭")
        except:
            os.killpg(os.getpgid(vr_ctrl_proc.pid), signal.SIGKILL)

    # 关闭 replay
    if replay_proc and replay_proc.poll() is None:
        try:
            os.killpg(os.getpgid(replay_proc.pid), signal.SIGTERM)
            replay_proc.wait(timeout=3)
            print_ok("replay 已关闭")
        except:
            os.killpg(os.getpgid(replay_proc.pid), signal.SIGKILL)
    
    # 关闭 rosbridge
    if rosbridge_proc and rosbridge_proc.poll() is None:
        try:
            os.killpg(os.getpgid(rosbridge_proc.pid), signal.SIGTERM)
            rosbridge_proc.wait(timeout=3)
            print_ok("rosbridge 已关闭")
        except:
            os.killpg(os.getpgid(rosbridge_proc.pid), signal.SIGKILL)
    
    # 关闭 ROS2（释放 CAN 总线）
    if ros2_proc and ros2_proc.poll() is None:
        try:
            os.killpg(os.getpgid(ros2_proc.pid), signal.SIGTERM)
            ros2_proc.wait(timeout=5)
            print_ok("ROS2 已关闭")
        except:
            os.killpg(os.getpgid(ros2_proc.pid), signal.SIGKILL)
    
    # 模拟模式不需要清理硬件
    if not sim_mode:
        time.sleep(0.5)
        disable_motors()
        time.sleep(0.3)
        run_sudo(f"ip link set {RIGHT_CAN} down")
        run_sudo(f"ip link set {LEFT_CAN} down")
        print_ok("CAN 接口已关闭")
    
    print("\n✓ 已安全退出\n")


# ═══════════════════════════════════════════════════════════════════════════════
# 主程序
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="OpenArmX 开机脚本")
    parser.add_argument("--zero", "-z", action="store_true", 
                        help="启动前设置当前位置为零点")
    parser.add_argument("--vr", action="store_true",
                        help="启动 VR 遥操作 (rosbridge + vr_controller)")
    parser.add_argument("--sim", action="store_true",
                        help="模拟模式 (无需硬件，用于 VLA 开发/VR 测试)")
    parser.add_argument("--viz", action="store_true",
                        help="启用 meshcat 可视化 (仅 VR 模式)")
    parser.add_argument("--no-clutch", action="store_true",
                        help="[调试] 绕过 clutch 检查，收到 VR 数据即开始控制")
    parser.add_argument("--input-unity", action="store_true",
                        help="[兼容] 输入按 Unity 坐标处理 (启用 Unity->ROS 转换)")
    parser.add_argument("--rot-offset", action="store_true",
                        help="[调试] 启用 TCP 180°旋转补偿")
    parser.add_argument("--debug-topics", action="store_true",
                        help="[调试] 发布 /debug/* 诊断 topics")
    parser.add_argument("--replay-jsonl", type=str, default="",
                        help="[测试] 启动后自动回放 Quest JSONL 数据")
    parser.add_argument("--replay-rate", type=float, default=1.0,
                        help="[测试] 回放速度倍率 (默认 1.0)")
    args = parser.parse_args()
    
    # 构建模式描述
    if args.sim:
        mode_str = "ROS2 (sim) -> fwd_pos_ctrl"
    else:
        mode_str = "CAN -> ROS2 -> fwd_pos_ctrl"
    if args.vr:
        mode_str += " -> rosbridge -> vr_ik"
    
    # sim 模式警告
    sim_warning = ""
    if args.sim:
        sim_warning = "║  [SIM] No real hardware, for dev/test only               ║\n"
        if args.zero:
            sim_warning += "║  [SIM] --zero ignored in simulation mode                 ║\n"
    
    print(f"""
╔══════════════════════════════════════════════════════════════╗
║                   OpenArmX Startup                           ║
║                                                              ║
║  Mode: {mode_str:<53}║
{sim_warning}║  Press Ctrl+C to exit safely                                 ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    ros2_proc = None
    rosbridge_proc = None
    vr_ctrl_proc = None
    replay_proc = None
    do_calibrate = args.zero
    do_vr = args.vr
    do_sim = args.sim
    do_viz = args.viz
    do_no_clutch = args.no_clutch
    do_input_unity = args.input_unity
    do_rot_offset = args.rot_offset
    do_debug_topics = args.debug_topics
    replay_jsonl = args.replay_jsonl
    replay_rate = args.replay_rate
    
    try:
        # 模拟模式不需要 CAN 和校准
        if not do_sim:
            if not step_1_setup_can():
                return 1
            
            # 可选：校准零点
            if do_calibrate:
                if not step_0_calibrate_zero():
                    print_err("校准失败，继续启动...")
                # 校准后重置 CAN，确保 socket 完全释放，避免 ROS2 连接失败
                print_info("等待 CAN 释放 (2秒)...")
                time.sleep(2)
                if not step_1_setup_can():
                    print_err("CAN 重置失败")
                    return 1
        
        ros2_proc = step_2_launch_ros2(sim_mode=do_sim)
        if ros2_proc is None:
            cleanup(ros2_proc, rosbridge_proc, vr_ctrl_proc, replay_proc, sim_mode=do_sim)
            return 1
        
        if not step_3_verify_controllers():
            cleanup(ros2_proc, rosbridge_proc, vr_ctrl_proc, replay_proc, sim_mode=do_sim)
            return 1
        
        # 可选：启动 VR 遥操作 (rosbridge + vr_controller)
        if do_vr:
            rosbridge_proc = step_4_start_rosbridge()
            if rosbridge_proc is None:
                print_err("rosbridge 启动失败，继续运行...")
            
            vr_ctrl_proc = step_5_start_vr_controller(
                enable_viz=do_viz,
                no_clutch=do_no_clutch,
                input_unity=do_input_unity,
                rot_offset=do_rot_offset,
                debug_topics=do_debug_topics,
            )
            if vr_ctrl_proc is None:
                print_err("vr_controller 启动失败，继续运行...")
            elif replay_jsonl:
                replay_proc = step_6_start_replay(replay_jsonl, replay_rate)
                if replay_proc is None:
                    print_err("replay 启动失败，继续运行...")
        
        # 显示就绪信息
        extra_info = ""
        
        # 模拟模式提示
        if do_sim:
            extra_info += """║                                                              ║
║  ⚠️  模拟模式 - 无真实硬件动作，适用于:                      ║
║      • VR 遥操作测试                                         ║
║      • VLA/策略模型开发                                      ║
║      • RViz 可视化调试                                       ║
"""
        
        # VR 连接地址和控制器状态
        if do_vr:
            local_ip = get_local_ip()
            ws_url = f"ws://{local_ip}:9090"
            vr_status = "运行中 ✓" if vr_ctrl_proc else "启动失败 ✗"
            extra_info += f"""║                                                              ║
║  VR 遥操作 (Meta Quest 连接地址):                            ║
║    {ws_url:<56}║
║                                                              ║
║  VR IK 控制器: {vr_status:<44}║
║    Clutch 按下 → 开始控制 (Snapshot + Gain Ramp)            ║
║    Clutch 松开 → 自动归零                                   ║
"""
        
        print(f"""
╔══════════════════════════════════════════════════════════════╗
║                      系统已就绪                              ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  ROS2 Topics:                                                ║
║    /right_forward_position_controller/commands               ║
║    /left_forward_position_controller/commands                ║
║    /joint_states                                             ║
{extra_info}║                                                              ║
║  GUI 启动命令 (新终端):                                       ║
║    cd ~/Desktop/openarmx_ws                                  ║
║    source /opt/ros/humble/setup.bash                         ║
║    source install/setup.bash                                 ║
║    python3 shadow_mode/gui.py                                ║
║                                                              ║
║  按 Ctrl+C 安全退出                                          ║
╚══════════════════════════════════════════════════════════════╝
""")
        
        # 保持运行
        while True:
            if ros2_proc.poll() is not None:
                print_err("ROS2 进程意外退出")
                break
            if rosbridge_proc and rosbridge_proc.poll() is not None:
                print_err("rosbridge 进程意外退出")
                rosbridge_proc = None
            if vr_ctrl_proc and vr_ctrl_proc.poll() is not None:
                print_err("vr_controller 进程意外退出")
                vr_ctrl_proc = None
            if replay_proc and replay_proc.poll() is not None:
                print_info("replay 进程已退出")
                replay_proc = None
            time.sleep(1)
    
    except KeyboardInterrupt:
        pass
    finally:
        cleanup(ros2_proc, rosbridge_proc, vr_ctrl_proc, replay_proc, sim_mode=do_sim)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
