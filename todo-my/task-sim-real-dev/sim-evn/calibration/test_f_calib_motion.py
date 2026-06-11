#!/usr/bin/env python3
"""
Test-F 校准运动脚本
动作序列：前进 1 m → 原地旋转 360° → 前进 2 m → 原地旋转 180° → 前进 3 m

仿真和真机使用完全相同的动作序列。真机默认通过 AUTO/MANUAL 仲裁链运行：
脚本发布 /nav2/cmd_vel，A 切 AUTO 后运行，B 切 MANUAL 后暂停，再切 AUTO 后续跑。

用法：
  source ~/robot_ws/install/setup.bash

  # 仿真 (DOMAIN_ID=20)：
  export ROS_DOMAIN_ID=20
  python3 ~/robot_ws/tools/test_f_calib_motion.py --label sim --direct-control

  # 真机 (DOMAIN_ID=13)：
  export ROS_DOMAIN_ID=13
  python3 ~/robot_ws/tools/test_f_calib_motion.py --label real

  # 只运动，不录包：
  python3 ~/robot_ws/tools/test_f_calib_motion.py --label sim --direct-control --no-bag

Bag 保存路径：~/robot_ws/calib_bags/calib_<profile>_<label>_<trial>_<timestamp>/
"""

import argparse
import math
import os
import signal
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_messages.msg import DriveMotor, SteerMotor
from std_msgs.msg import String

# ── 运动参数 ──────────────────────────────────────────────────────────────
LINEAR_VEL  = 0.15   # m/s  — 适合仿真和真机的保守速度
ANGULAR_VEL = 0.50   # rad/s

PUBLISH_HZ  = 10     # 发布频率 — 满足真机 motor timeout watchdog (0.3 s)
STEER_READY_TOL = 0.08  # rad, about 4.6 degrees
WHEEL_DIST = (0.32330, 0.32330, 0.38500)
WHEEL_PHI = (-0.883978, 0.883978, math.pi)

# ── 动作序列 ─────────────────────────────────────────────────────────────
# (标签, vx, vy, wz, 持续时间_s)
BASELINE_SEGMENTS = [
    ("start_pause", 0.0,        0.0, 0.0,         1.0),
    ("forward_1m",  LINEAR_VEL, 0.0, 0.0,         1.0 / LINEAR_VEL),       # 6.67 s
    ("settle_1",    0.0,        0.0, 0.0,         1.5),
    ("spin_360",    0.0,        0.0, ANGULAR_VEL, 2 * math.pi / ANGULAR_VEL),  # 12.57 s
    ("settle_2",    0.0,        0.0, 0.0,         1.5),
    ("forward_2m",  LINEAR_VEL, 0.0, 0.0,         2.0 / LINEAR_VEL),       # 13.33 s
    ("settle_3",    0.0,        0.0, 0.0,         1.5),
    ("spin_180",    0.0,        0.0, ANGULAR_VEL, math.pi / ANGULAR_VEL),   # 6.28 s
    ("settle_4",    0.0,        0.0, 0.0,         1.5),
    ("forward_3m",  LINEAR_VEL, 0.0, 0.0,         3.0 / LINEAR_VEL),       # 20.0 s
    ("end_pause",   0.0,        0.0, 0.0,         2.0),
]
# TOTAL ≈ 67.8 s


def build_segments(profile):
    profiles = {
        "baseline": BASELINE_SEGMENTS,
        "straight-forward": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("forward_3m", LINEAR_VEL, 0.0, 0.0, 3.0 / LINEAR_VEL),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
        "straight-reverse": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("reverse_3m", -LINEAR_VEL, 0.0, 0.0, 3.0 / LINEAR_VEL),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
        "lateral-left": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("left_2m", 0.0, 0.10, 0.0, 20.0),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
        "lateral-right": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("right_2m", 0.0, -0.10, 0.0, 20.0),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
        "spin-ccw": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("spin_ccw_720", 0.0, 0.0, ANGULAR_VEL, 4 * math.pi / ANGULAR_VEL),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
        "spin-cw": [
            ("start_pause", 0.0, 0.0, 0.0, 3.0),
            ("spin_cw_720", 0.0, 0.0, -ANGULAR_VEL, 4 * math.pi / ANGULAR_VEL),
            ("end_pause", 0.0, 0.0, 0.0, 3.0),
        ],
    }
    return profiles[profile]

# ── 录包 topic 列表 ──────────────────────────────────────────────────────
# 仿真和真机通用；不存在的 topic 会被 ros2 bag record 忽略
BAG_TOPICS = [
    "/cmd_vel",
    "/cmd_vel_raw",
    "/nav2/cmd_vel",
    "/teleop/cmd_vel",
    "/robot_mode",
    "/safety_status",
    "/calibration/event",
    "/joy",
    "/wheel_odom",               # 仿真: Isaac Sim Bridge; 真机: robot_odom_node
    "/tf",
    "/tf_static",
    "/joint_states",             # 仿真: Isaac Sim; 真机: 通常无此 topic
    "/sim/target_joint_states",  # 仿真专用
    "/drive_vel",                # 真机驱动速度命令
    "/drive_odom",               # 真机专用 (BLV-R feedback)
    "/om_query0",                # 真机 BLV-R Modbus query/debug
    "/om_response0",             # 真机 BLV-R Modbus response/debug
    "/om_state0",                # 真机 BLV-R Modbus communication state
    "/steer_angle",              # 真机转向角命令
    "/steer_odom",               # 真机转向角反馈
    "/clock",
]


class MotionPublisher(Node):
    def __init__(self, command_topic, direct_control, profile, segments):
        super().__init__("test_f_calib_motion")
        self._pub = self.create_publisher(Twist, command_topic, 10)
        self._direct_control = direct_control
        self._mode = "DIRECT" if direct_control else "UNKNOWN"
        self._last_drive_odom_time = None
        self._last_steer_odom_time = None
        self._steer_odom = None
        self._profile = profile
        self._segments = segments
        self._event_pub = self.create_publisher(String, "/calibration/event", 10)
        if not direct_control:
            self.create_subscription(String, "/robot_mode", self._mode_cb, 10)
            self.create_subscription(
                DriveMotor, "/drive_odom", self._drive_odom_cb, 10
            )
            self.create_subscription(
                SteerMotor, "/steer_odom", self._steer_odom_cb, 10
            )
        self.get_logger().info(
            f"command_topic={command_topic}, control_mode={self._mode}"
        )

    def _mode_cb(self, msg):
        if msg.data != self._mode:
            self.get_logger().info(f"robot mode: {self._mode} -> {msg.data}")
            self._mode = msg.data

    def _auto_enabled(self):
        return self._direct_control or self._mode == "AUTO"

    def _drive_feedback_fresh(self):
        return self._direct_control or (
            self._last_drive_odom_time is not None
            and time.monotonic() - self._last_drive_odom_time <= 0.5
        )

    def _drive_odom_cb(self, msg):
        self._last_drive_odom_time = time.monotonic()

    def _steer_odom_cb(self, msg):
        self._last_steer_odom_time = time.monotonic()
        self._steer_odom = (msg.phi1, msg.phi2, msg.phi3)

    @staticmethod
    def _angle_error(actual, target):
        return abs(math.atan2(math.sin(actual - target), math.cos(actual - target)))

    @staticmethod
    def _target_steer(vx, vy, wz):
        targets = []
        for distance, position_angle in zip(WHEEL_DIST, WHEEL_PHI):
            rx = distance * math.cos(position_angle)
            ry = distance * math.sin(position_angle)
            angle = math.atan2(vy + rx * wz, vx - ry * wz)
            if angle < -0.71 * math.pi:
                angle += math.pi
            elif angle > 0.71 * math.pi:
                angle -= math.pi
            targets.append(angle)
        return targets

    def _steer_ready(self, vx, vy, wz):
        if self._direct_control or (vx == 0.0 and vy == 0.0 and wz == 0.0):
            return True
        if (
            self._steer_odom is None
            or self._last_steer_odom_time is None
            or time.monotonic() - self._last_steer_odom_time > 0.5
        ):
            return False
        targets = self._target_steer(vx, vy, wz)
        return max(
            self._angle_error(actual, target)
            for actual, target in zip(self._steer_odom, targets)
        ) <= STEER_READY_TOL

    def _can_run(self):
        return self._auto_enabled() and self._drive_feedback_fresh()

    def _event(self, text):
        msg = String()
        msg.data = f"{self._profile}:{text}"
        self._event_pub.publish(msg)
        self.get_logger().info(f"event: {msg.data}")

    def _wait_for_auto(self):
        announced = False
        while rclpy.ok() and not self._can_run():
            if not announced:
                self.get_logger().info(
                    "等待 AUTO 与新鲜 /drive_odom：按 JoyCon A 开始/继续；"
                    "按 B 切 MANUAL 并暂停。"
                )
                announced = True
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        total = sum(s[4] for s in self._segments)
        self.get_logger().info(
            f"Test-F 动作序列开始 — 总时长 ≈ {total:.1f} s"
        )
        _print_plan(self._segments)

        interval = 1.0 / PUBLISH_HZ
        twist = Twist()
        self._wait_for_auto()

        self._event("run_start")
        for label, vx, vy, wz, duration in self._segments:
            self.get_logger().info(
                f"  [{label}]  vx={vx:.2f}  vy={vy:.2f}  wz={wz:.2f}  {duration:.2f} s"
            )
            twist.linear.x  = vx
            twist.linear.y  = vy
            twist.angular.z = wz
            self._event(f"segment_start:{label}")

            active_elapsed = 0.0
            active_started = False
            paused = False
            while rclpy.ok() and active_elapsed < duration:
                rclpy.spin_once(self, timeout_sec=0.0)
                if not self._can_run():
                    if not paused:
                        reason = (
                            "MANUAL" if not self._auto_enabled()
                            else "drive_odom 超过 0.5 s 未更新"
                        )
                        self.get_logger().info(
                            f"  [{label}] {reason}：暂停于 "
                            f"{active_elapsed:.2f}/{duration:.2f} s"
                        )
                        paused = True
                    time.sleep(0.05)
                    continue
                started = time.monotonic()
                self._pub.publish(twist)
                time.sleep(interval)
                if not self._steer_ready(vx, vy, wz):
                    if not paused:
                        self.get_logger().info(
                            f"  [{label}] 等待转向轮到位，不计入动作时间"
                        )
                        paused = True
                    continue
                if not active_started:
                    self._event(f"active_start:{label}")
                    active_started = True
                if paused:
                    self.get_logger().info(
                        f"  [{label}] AUTO：从 {active_elapsed:.2f}/{duration:.2f} s 继续"
                    )
                    paused = False
                active_elapsed += time.monotonic() - started
            if active_started:
                self._event(f"active_end:{label}")
            self._event(f"segment_end:{label}")

        # 停止指令（多发几次保证到达）
        twist.linear.x  = 0.0
        twist.linear.y  = 0.0
        twist.angular.z = 0.0
        for _ in range(PUBLISH_HZ):
            self._pub.publish(twist)
            time.sleep(interval)

        self.get_logger().info("动作序列完成。")
        self._event("run_complete")


def _print_plan(segments):
    print("\n  标签               vx      wz     时长")
    print("  " + "-" * 50)
    for label, vx, vy, wz, dur in segments:
        print(f"  {label:18s}  {vx:+.2f}   {wz:+.2f}   {dur:6.2f} s")
    total = sum(s[4] for s in segments)
    print(f"  {'TOTAL':18s}                  {total:6.1f} s\n")


def main():
    parser = argparse.ArgumentParser(
        description="Test-F 校准运动：前进 1m → 转 360° → 前进 2m → 转 180° → 前进 3m"
    )
    parser.add_argument(
        "--label", required=True, choices=["sim", "real"],
        help="环境标识，用于 bag 文件夹命名"
    )
    parser.add_argument(
        "--no-bag", action="store_true",
        help="只执行运动，不录 bag"
    )
    parser.add_argument(
        "--direct-control", action="store_true",
        help="直接发布 /cmd_vel 且不等待 /robot_mode；仅用于无仲裁链的仿真"
    )
    parser.add_argument(
        "--command-topic",
        help="覆盖命令 topic（默认：真机 /nav2/cmd_vel，direct-control /cmd_vel）"
    )
    parser.add_argument(
        "--profile",
        default="baseline",
        choices=[
            "baseline", "straight-forward", "straight-reverse",
            "lateral-left", "lateral-right", "spin-ccw", "spin-cw",
        ],
        help="独立校准实验；长距离/双圈用于降低启动和转向对准误差"
    )
    parser.add_argument(
        "--trial", default="01",
        help="测量编号，用于 bag 文件夹命名，例如 01/02/03"
    )
    args = parser.parse_args()

    def _interrupt_on_termination(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _interrupt_on_termination)
    signal.signal(signal.SIGHUP, _interrupt_on_termination)

    if args.label == "real" and args.direct_control:
        parser.error("真机禁止 --direct-control；必须经过 AUTO/MANUAL 仲裁与 safety layer")
    command_topic = args.command_topic or (
        "/cmd_vel" if args.direct_control else "/nav2/cmd_vel"
    )
    segments = build_segments(args.profile)
    if args.label == "real" and command_topic != "/nav2/cmd_vel":
        parser.error("真机命令 topic 必须是 /nav2/cmd_vel，禁止绕过仲裁与 safety layer")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.expanduser(
        f"~/robot_ws/calib_bags/calib_{args.profile}_{args.label}_{args.trial}_{ts}"
    )

    bag_proc = None
    if not args.no_bag:
        os.makedirs(os.path.expanduser("~/robot_ws/calib_bags"), exist_ok=True)
        bag_cmd = ["ros2", "bag", "record", *BAG_TOPICS, "-o", bag_dir]
        print(f"[bag] 开始录制: {bag_dir}")
        bag_proc = subprocess.Popen(bag_cmd, start_new_session=True)
        time.sleep(1.5)  # 等待 bag recorder 就绪
        if bag_proc.poll() is not None:
            raise RuntimeError("ros2 bag record 启动失败")

    node = None
    try:
        rclpy.init()
        node = MotionPublisher(command_topic, args.direct_control, args.profile, segments)
        node.run()
    except KeyboardInterrupt:
        print("\n[中断] 用户中止，发送停止指令...")
        if node is not None:
            node._pub.publish(Twist())
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if bag_proc:
            os.killpg(bag_proc.pid, signal.SIGINT)
            try:
                bag_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(bag_proc.pid, signal.SIGKILL)
            print(f"[bag] 已保存: {bag_dir}")


if __name__ == "__main__":
    main()
