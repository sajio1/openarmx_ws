#!/usr/bin/env python3
"""Quest 数据录制/回放工具.

支持 topic:
  - /quest3/right_hand_pose (geometry_msgs/PoseStamped)
  - /quest3/left_hand_pose  (geometry_msgs/PoseStamped)
  - /quest3/right_gripper   (std_msgs/Float32)
  - /quest3/left_gripper    (std_msgs/Float32)
  - /quest3/clutch          (std_msgs/Bool)
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32


TRACKED_TOPICS = [
    "/quest3/right_hand_pose",
    "/quest3/left_hand_pose",
    "/quest3/right_gripper",
    "/quest3/left_gripper",
    "/quest3/clutch",
]


class QuestRecorder(Node):
    def __init__(self, output_file: Path):
        super().__init__("quest_stream_recorder")
        self.output_file = output_file
        self.fp = output_file.open("w", encoding="utf-8")

        self.create_subscription(PoseStamped, "/quest3/right_hand_pose", self._on_right_pose, 20)
        self.create_subscription(PoseStamped, "/quest3/left_hand_pose", self._on_left_pose, 20)
        self.create_subscription(Float32, "/quest3/right_gripper", self._on_right_gripper, 20)
        self.create_subscription(Float32, "/quest3/left_gripper", self._on_left_gripper, 20)
        self.create_subscription(Bool, "/quest3/clutch", self._on_clutch, 20)
        self.get_logger().info(f"开始录制到: {self.output_file}")

    def _write(self, topic: str, payload: dict):
        row = {"t": time.time(), "topic": topic, "msg": payload}
        self.fp.write(json.dumps(row, ensure_ascii=True) + "\n")
        self.fp.flush()

    def _on_right_pose(self, msg: PoseStamped):
        self._write(
            "/quest3/right_hand_pose",
            {
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            },
        )

    def _on_left_pose(self, msg: PoseStamped):
        self._write(
            "/quest3/left_hand_pose",
            {
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w,
                },
            },
        )

    def _on_right_gripper(self, msg: Float32):
        self._write("/quest3/right_gripper", {"data": float(msg.data)})

    def _on_left_gripper(self, msg: Float32):
        self._write("/quest3/left_gripper", {"data": float(msg.data)})

    def _on_clutch(self, msg: Bool):
        self._write("/quest3/clutch", {"data": bool(msg.data)})

    def close(self):
        self.fp.close()


class QuestReplayer(Node):
    def __init__(self, input_file: Path, rate: float):
        super().__init__("quest_stream_replayer")
        self.input_file = input_file
        self.rate = max(rate, 1e-3)
        # 与 vr_teleop_ik_node 的订阅 QoS 对齐，避免 DURABILITY 不兼容
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.right_pose_pub = self.create_publisher(PoseStamped, "/quest3/right_hand_pose", qos)
        self.left_pose_pub = self.create_publisher(PoseStamped, "/quest3/left_hand_pose", qos)
        self.right_gripper_pub = self.create_publisher(Float32, "/quest3/right_gripper", qos)
        self.left_gripper_pub = self.create_publisher(Float32, "/quest3/left_gripper", qos)
        self.clutch_pub = self.create_publisher(Bool, "/quest3/clutch", qos)

    def replay(self):
        rows = []
        with self.input_file.open("r", encoding="utf-8") as fp:
            for line in fp:
                line = line.strip()
                if not line:
                    continue
                rows.append(json.loads(line))

        if not rows:
            self.get_logger().warning("回放文件为空，退出")
            return

        t0 = rows[0]["t"]
        start = time.time()
        self.get_logger().info(f"开始回放: {self.input_file} (x{self.rate})")

        for row in rows:
            target_dt = (row["t"] - t0) / self.rate
            while True:
                now_dt = time.time() - start
                if now_dt >= target_dt:
                    break
                time.sleep(min(0.001, target_dt - now_dt))
            self._publish_row(row)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.get_logger().info("回放完成")

    def _publish_row(self, row: dict):
        topic = row["topic"]
        msg = row["msg"]
        if topic == "/quest3/right_hand_pose":
            self.right_pose_pub.publish(self._pose_from_dict(msg))
        elif topic == "/quest3/left_hand_pose":
            self.left_pose_pub.publish(self._pose_from_dict(msg))
        elif topic == "/quest3/right_gripper":
            self.right_gripper_pub.publish(Float32(data=float(msg["data"])))
        elif topic == "/quest3/left_gripper":
            self.left_gripper_pub.publish(Float32(data=float(msg["data"])))
        elif topic == "/quest3/clutch":
            self.clutch_pub.publish(Bool(data=bool(msg["data"])))

    @staticmethod
    def _pose_from_dict(data: dict) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.frame_id = "quest_control_frame"
        msg.pose.position.x = float(data["position"]["x"])
        msg.pose.position.y = float(data["position"]["y"])
        msg.pose.position.z = float(data["position"]["z"])
        msg.pose.orientation.x = float(data["orientation"]["x"])
        msg.pose.orientation.y = float(data["orientation"]["y"])
        msg.pose.orientation.z = float(data["orientation"]["z"])
        msg.pose.orientation.w = float(data["orientation"]["w"])
        return msg


def parse_args():
    parser = argparse.ArgumentParser(description="Quest topic 录制/回放")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_rec = sub.add_parser("record", help="录制")
    p_rec.add_argument("--file", required=True, help="输出 jsonl 路径")

    p_rep = sub.add_parser("replay", help="回放")
    p_rep.add_argument("--file", required=True, help="输入 jsonl 路径")
    p_rep.add_argument("--rate", type=float, default=1.0, help="回放倍率")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    if args.mode == "record":
        recorder = QuestRecorder(Path(args.file).expanduser().resolve())
        try:
            rclpy.spin(recorder)
        except KeyboardInterrupt:
            pass
        finally:
            recorder.close()
            recorder.destroy_node()
            rclpy.shutdown()
    else:
        replayer = QuestReplayer(Path(args.file).expanduser().resolve(), args.rate)
        try:
            replayer.replay()
        finally:
            replayer.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
