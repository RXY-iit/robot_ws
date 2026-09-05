#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       BLV-R
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目,3軸目]
#       通信ID: [1, 2, 3]
#       Baudrate: [230400, 230400, 230400]
#
# ●launchファイル設定:
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=100 firstGen:="" secondGen:="1,2,3" globalID:="10" axisNum:="3"
#
# ●処理内容:
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

import os
import sys
import threading
import json
from collections import deque

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from rclpy.node import Node
import time
import datetime
import rclpy
import math
from rclpy.duration import Duration
from rclpy.time import Time
from utils import const
from utils.clientasync import ClientAsync
from om_msgs.msg import Query
from om_msgs.msg import Response
from om_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
# from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from my_messages.msg import DriveMotor
from std_msgs.msg import String

# グローバル変数
_state_driver = 0  # 0:通信可能 1:通信中
_state_mes = 0  # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
_state_error = 0  # 0:エラーなし 1:無応答 2:例外応答
_is_timer_active = False
# Completion counter shared by the serialized runtime publisher.  The ROS
# driver state topic can lag the query callback briefly, so state alone is not
# a sufficient guard against submitting the next Modbus transaction too soon.
_query_completion_counter = 0
_runtime_query_count = 0
_query_timeout_count = 0
_query_timeout_times = deque(maxlen=256)
_last_query_timeout_time = None
_last_query_timeout_kind = None
_last_query_timeout_elapsed_sec = None
# Direct error events reported by the C++ Modbus driver through /om_state0.
# Keep these separate from an upper-layer completion callback deadline miss:
# the latter may be caused by ROS/Python scheduling even when the driver did
# not report a serial error.
_driver_error_event_count = 0
_driver_no_response_event_count = 0
_driver_exception_event_count = 0
_last_driver_error_time = None
_runtime_query_kind = None
left = 0
right = 0
back = 0
v=0.0
omega=0.0
x=0.0
y=0.0
th=0.0
dt1=0
dt2=0
dt3=0
v_b=0
lpr=1
# 直近の指令受信時刻（モノトニック）
last_cmd_time = None
# 直近の非ゼロ指令時刻と値。通信はできるが励磁していない状態の診断に使う。
last_nonzero_cmd_time = None
last_commanded_rpm = (0.0, 0.0, 0.0)
last_servo_diagnostic_time = 0.0
zero_feedback_count_after_cmd = 0
# 指令タイムアウト[s]
CMD_TIMEOUT_SEC = 0.08
# スレッドセーフなロック機構
motor_speed_lock = threading.Lock()
# 定数
const.QUEUE_SIZE = 1
WHEEL_EFFECTIVE_RADIUS_M = 0.107
DRIVE_AXIS_LAYOUT = (
    (1, "back"),
    (2, "front_left"),
    (3, "front_right"),
)
DRIVE_HEALTH_PUBLISH_RATE_HZ = 2.0
# The C++ serial layer uses a 500 ms transaction timeout. Real-robot unicast
# alarm reads complete in about 280 ms, so the previous 250 ms callback
# deadline produced false misses while /om_state0 still reported no error.
COMPLETION_CALLBACK_DEADLINE_SEC = 0.5


def decode_drive_diagnostics(data):
    """Decode NET-ID 106/155/163/164 values grouped by physical axis."""
    if len(data) < 12:
        raise ValueError(f"Expected 12 drive diagnostic values, received {len(data)}")
    direct_io = []
    electrical = []
    for axis, (slave_id, wheel_position) in enumerate(DRIVE_AXIS_LAYOUT):
        base = axis * 4
        direct_raw = int(data[base])
        direct_io.append(
            {
                "slave_id": slave_id,
                "wheel_position": wheel_position,
                "raw": direct_raw,
                "din0": bool(direct_raw & 0x01),
                "din1": bool(direct_raw & 0x02),
                "din2": bool(direct_raw & 0x04),
                "din3": bool(direct_raw & 0x08),
            }
        )
        electrical.append(
            {
                "slave_id": slave_id,
                "wheel_position": wheel_position,
                "main_power_supply_current_raw": int(data[base + 1]),
                "main_power_supply_current_a": int(data[base + 1]) * 0.001,
                "inverter_voltage_raw": int(data[base + 2]),
                "inverter_voltage_v": int(data[base + 2]) * 0.1,
                "main_power_supply_voltage_raw": int(data[base + 3]),
                "main_power_supply_voltage_v": int(data[base + 3]) * 0.1,
            }
        )
    return direct_io, electrical


def decode_drive_alarm_status(data, slave_id):
    """Decode direct BLVD Present alarm and Alarm history 1 registers."""
    if len(data) < 2:
        raise ValueError(f"Expected 2 alarm values, received {len(data)}")
    positions = dict(DRIVE_AXIS_LAYOUT)
    if slave_id not in positions:
        raise ValueError(f"Unexpected BLVD slave ID {slave_id}")
    return {
        "slave_id": int(slave_id),
        "wheel_position": positions[slave_id],
        "present_alarm": int(data[0]),
        "alarm_history_1": int(data[1]),
    }

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class Mysubscription2(Node):
    def __init__(self):
        super().__init__("my_sub_2")
        self.sub = self.create_subscription(
            DriveMotor, "drive_vel", self.drive_callback, const.QUEUE_SIZE
        )
        self.ca = ClientAsync("sub_2")
    
    def destroy_client(self):
        try:
            if self.ca is not None:
                self.ca.destroy_node()
        except Exception:
            pass

    def drive_callback(self, res):
        global lpr
        global right
        global left
        global back
        global last_cmd_time
        global last_commanded_rpm
        global last_nonzero_cmd_time
        # Do not refresh/replay motion while the Modbus layer is reporting a
        # communication error.  A fresh command will be accepted after the
        # state returns to healthy.
        if _state_error != 0:
            with motor_speed_lock:
                right = 0
                left = 0
                back = 0
                last_cmd_time = None
            return
        wheeles_size = WHEEL_EFFECTIVE_RADIUS_M
        # axle_length = 0.5

        # omega = res.angular.z
        v_r = res.vel1
        v_l = res.vel2
        v_b = res.vel3

        # v_r = 2*v
        # v_l = 2*v
        # v_b = 2*v

        v_r = -v_r/(wheeles_size * 2 * 3.14) 
        v_l = v_l/(wheeles_size * 2 * 3.14)
        v_b = v_b/(wheeles_size * 2 * 3.14)
        # try: lpr = (60 * v_l  *30 - left) / (60 * v_r *30 - right)
        # except: lpr = 1
        # self.get_logger().info("lpr %f" % (lpr))
        with motor_speed_lock:
            right = 60 * v_r *30
            left = 60 * v_l  *30
            back = 60 * v_b *30
            last_commanded_rpm = (right, left, back)
            if abs(right) > 1.0 or abs(left) > 1.0 or abs(back) > 1.0:
                last_nonzero_cmd_time = time.monotonic()
        last_cmd_time = time.monotonic()

class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        self.sub1 = self.create_subscription(
            Response, "om_response0", self.response_callback, const.QUEUE_SIZE
        )
        self.sub2 = self.create_subscription(
            State, "om_state0", self.state_callback, const.QUEUE_SIZE
        )
        self.pub = self.create_publisher(TwistWithCovarianceStamped, "/localization/twist_estimator/twist_with_covariance", const.QUEUE_SIZE)
        self.pub2 = self.create_publisher(Odometry, "odom", const.QUEUE_SIZE)
        self.pub_drive_odom = self.create_publisher(DriveMotor, "drive_odom", const.QUEUE_SIZE)
        # Low-rate, factual summary for health consumers.  It is generated in
        # this process from the actual response/state callbacks so monitors do
        # not need to subscribe to the high-rate /om_state0 snapshot stream.
        self.pub_health = self.create_publisher(String, "/drive/health_summary", 10)
        self.health_timer = self.create_timer(
            1.0 / DRIVE_HEALTH_PUBLISH_RATE_HZ,
            self.publish_health_summary,
        )
        self.response_times = deque(maxlen=256)
        self.read_response_times = deque(maxlen=256)
        self.last_response_time = None
        self.last_read_response_time = None
        self.last_response_func_code = None
        self.last_direct_io_time = None
        self.direct_io_status = None
        self.last_electrical_time = None
        self.electrical_status = None
        self.alarm_status = {}
        self.alarm_history_baseline = {}
        self.alarm_event_count = 0
        self.last_alarm_event_time = None
        self.session_started_at = datetime.datetime.now(datetime.timezone.utc).isoformat()
        self.state_update_count = 0
        self.last_state_time = None
        self.last_state = None
        self.cached_axis_num = None
        self.ca = ClientAsync("sub")

    def destroy_client(self):
        try:
            if self.ca is not None:
                self.ca.destroy_node()
        except Exception:
            pass

    def response_callback(self, res):
        global _query_completion_counter
        global _runtime_query_kind
        global v
        global omega
        global x
        global y
        global th
        global dt1
        global dt2
        global last_servo_diagnostic_time
        global zero_feedback_count_after_cmd
        _query_completion_counter += 1
        now_monotonic = time.monotonic()
        self.response_times.append(now_monotonic)
        self.last_response_time = now_monotonic
        self.last_response_func_code = int(res.func_code)
        
        pose = Pose()
        twist_stamp = TwistWithCovarianceStamped()
        twist_stamp.header.stamp = self.get_clock().now().to_msg()
        twist_stamp.header.frame_id = "base_link"
        twist_stamp.twist.twist.linear.x = v
        twist_stamp.twist.twist.angular.z = omega
        twist_stamp.twist.covariance[0] = 1.0
        twist_stamp.twist.covariance[7] = 1.0
        twist_stamp.twist.covariance[14] = 1.0
        twist_stamp.twist.covariance[21] = 1.0
        twist_stamp.twist.covariance[28] = 1.0
        twist_stamp.twist.covariance[35] = 1.0
        self.pub.publish(twist_stamp)
        dt1 = self.get_clock().now().nanoseconds
        # 例外応答のとき
        if _state_error == 2:
            print("Exception")
            return
        # ID Shareモードのとき
        # Read応答のファンクションコードは環境により 0 or 0x03 として返ることがあるため両方許容
        if (res.func_code == 0) or (res.func_code == 0x03):
            self.read_response_times.append(now_monotonic)
            self.last_read_response_time = now_monotonic
            if _runtime_query_kind in {"direct_io", "drive_diagnostics"}:
                # HP-5142 ID-share responses are grouped per physical axis.
                # Nameplates confirm ID 1=back, ID 2=front-left and
                # ID 3=front-right.  The current combined query returns four
                # values per axis: NET-ID 106, 155, 163 and 164.
                values_per_axis = 4 if _runtime_query_kind == "drive_diagnostics" else 1
                if len(res.data) >= 3 * values_per_axis:
                    self.last_direct_io_time = now_monotonic
                    if values_per_axis == 4:
                        self.direct_io_status, self.electrical_status = (
                            decode_drive_diagnostics(res.data)
                        )
                        self.last_electrical_time = now_monotonic
                    else:
                        self.direct_io_status = [
                            {
                                "slave_id": axis + 1,
                                "wheel_position": ("back", "front_left", "front_right")[axis],
                                "raw": int(res.data[axis]),
                                "din0": bool(int(res.data[axis]) & 0x01),
                                "din1": bool(int(res.data[axis]) & 0x02),
                                "din2": bool(int(res.data[axis]) & 0x04),
                                "din3": bool(int(res.data[axis]) & 0x08),
                            }
                            for axis in range(3)
                        ]
                return
            if isinstance(_runtime_query_kind, str) and _runtime_query_kind.startswith("drive_alarm:"):
                try:
                    slave_id = int(_runtime_query_kind.split(":", 1)[1])
                    alarm = decode_drive_alarm_status(res.data, slave_id)
                except (TypeError, ValueError, IndexError) as exc:
                    self.get_logger().error(f"Invalid BLVD alarm response: {exc}")
                    return
                previous = self.alarm_status.get(slave_id)
                baseline = self.alarm_history_baseline.get(slave_id)
                history = alarm["alarm_history_1"]
                if baseline is None:
                    self.alarm_history_baseline[slave_id] = history
                history_changed = baseline is not None and history != baseline
                history_change_started = (
                    history_changed
                    and (previous is None or previous.get("alarm_history_1") != history)
                )
                present_alarm_started = (
                    alarm["present_alarm"] != 0
                    and (previous is None or previous.get("present_alarm") == 0)
                )
                if history_change_started or present_alarm_started:
                    self.alarm_event_count += 1
                    self.last_alarm_event_time = now_monotonic
                alarm["alarm_history_baseline"] = self.alarm_history_baseline[slave_id]
                alarm["alarm_history_changed_in_session"] = history_changed
                alarm["observed_monotonic"] = now_monotonic
                self.alarm_status[slave_id] = alarm
                return
            if self.cached_axis_num is None:
                self.cached_axis_num = self.ca.get_parameters_from_another_node(
                    "om_node", ["axis_num"]
                )
            axis_num = self.cached_axis_num
            if dt2 != 0:
                delta = dt1-dt2
            else:
                delta = 0
            dt2 = dt1
            delta = delta / 1e9
            
            # DriveMotorメッセージを配信
            drive_odom_msg = DriveMotor()
            drive_odom_msg.header.stamp = self.get_clock().now().to_msg()
            drive_odom_msg.header.frame_id = "base_link"
            
            for axis in range(axis_num[0]):
                vR = math.pi*2*WHEEL_EFFECTIVE_RADIUS_M*(-res.data[1])/(60*30) #m/s
                vL = math.pi*2*WHEEL_EFFECTIVE_RADIUS_M*res.data[0]/(60*30) #m/s
                robot_v = (vR+vL)/2
                robot_w = (vR-vL)/0.5
                th = th + robot_w*delta
                th = th % (2 * math.pi)
                x = x + (robot_v*math.cos(th)*delta)
                y = y + (robot_v*math.sin(th)*delta)
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                pose.orientation = yaw_to_quaternion(th)

                # twist_stamp = TwistWithCovarianceStamped()
                # twist_stamp.header.stamp = self.get_clock().now().to_msg()
                # twist_stamp.header.frame_id = "base_link"
                # twist_stamp.twist.twist.linear.x = robot_v
                # twist_stamp.twist.twist.angular.z = robot_w
                # twist_stamp.twist.covariance[0] = 1.0
                # twist_stamp.twist.covariance[7] = 1.0
                # twist_stamp.twist.covariance[14] = 1.0
                # twist_stamp.twist.covariance[21] = 1.0
                # twist_stamp.twist.covariance[28] = 1.0
                # twist_stamp.twist.covariance[35] = 1.0
                # self.pub.publish(twist_stamp)

            
            # 3軸分の検出速度を変換してDriveMotorメッセージに格納
            # Convert physical motor signs to the logical wheel signs used by
            # PickingRobotMatrix. A positive /cmd_vel.x must produce positive
            # right, left, and back wheel feedback when steer angles are zero.
            # ID Share response follows physical axis order:
            # data[0]=back, data[1]=left, data[2]=right [r/min].
            wheeles_size = WHEEL_EFFECTIVE_RADIUS_M
            if len(res.data) >= 3:
                # r/min から m/s に変換
                drive_odom_msg.vel1 = math.pi * 2 * wheeles_size * (-res.data[2]) / (60 * 30) # right
                drive_odom_msg.vel2 = math.pi * 2 * wheeles_size * res.data[1] / (60 * 30)    # left
                drive_odom_msg.vel3 = math.pi * 2 * wheeles_size * res.data[0] / (60 * 30)    # back
                self.pub_drive_odom.publish(drive_odom_msg)
                feedback_zero = (
                    abs(drive_odom_msg.vel1) < 1e-4
                    and abs(drive_odom_msg.vel2) < 1e-4
                    and abs(drive_odom_msg.vel3) < 1e-4
                )
                recently_commanded = (
                    last_nonzero_cmd_time is not None
                    and (time.monotonic() - last_nonzero_cmd_time) < 1.0
                    and any(abs(value) > 1.0 for value in last_commanded_rpm)
                )
                if recently_commanded and feedback_zero:
                    zero_feedback_count_after_cmd += 1
                    now = time.monotonic()
                    if zero_feedback_count_after_cmd >= 3 and (now - last_servo_diagnostic_time) > 2.0:
                        last_servo_diagnostic_time = now
                        self.get_logger().warn(
                            "BLVD-KRD command is non-zero but detected velocity stays zero. "
                            "Communication is alive, so check emergency stop button and driver inhibit states: "
                            "FREE input, SON-MON, S-ON, ALM, STOP/QSTOP/ETO, motor power. "
                            f"last_commanded_rpm=[{last_commanded_rpm[0]:.1f}, "
                            f"{last_commanded_rpm[1]:.1f}, {last_commanded_rpm[2]:.1f}]"
                        )
                elif not feedback_zero:
                    zero_feedback_count_after_cmd = 0
            
            # odom = Odometry()
            # odom.header.stamp = twist_stamp.header.stamp
            # odom.header.frame_id = "odom"
            # odom.child_frame_id = "base_link"
            # odom.pose.pose = pose
            # odom.twist.twist.linear.x = v
            # odom.twist.twist.angular.z =omega
            # odom.twist.covariance[0] = 1.0
            # odom.twist.covariance[7] = 1.0
            # odom.twist.covariance[14] = 1.0
            # odom.twist.covariance[21] = 1.0
            # odom.twist.covariance[28] = 1.0
            # odom.twist.covariance[35] = 1.0
            # self.pub2.publish(odom)
            
            
    def state_callback(self, res):
        global _state_driver
        global _state_mes
        global _state_error
        global _driver_error_event_count
        global _driver_no_response_event_count
        global _driver_exception_event_count
        global _last_driver_error_time
        previous_error = _state_error
        _state_driver = res.state_driver
        _state_mes = res.state_mes
        _state_error = res.state_error
        if _state_error != 0 and _state_error != previous_error:
            _driver_error_event_count += 1
            if _state_error == 1:
                _driver_no_response_event_count += 1
            elif _state_error == 2:
                _driver_exception_event_count += 1
            _last_driver_error_time = time.monotonic()
        self.state_update_count += 1
        self.last_state_time = time.monotonic()
        self.last_state = {
            "state_driver": int(res.state_driver),
            "state_message": int(res.state_mes),
            "state_error": int(res.state_error),
        }
        # Drop any cached non-zero command as soon as the Modbus layer reports
        # an error.  If communication recovers, the next fresh /drive_vel
        # callback may command motion again; stale motion is never replayed.
        if _state_error != 0:
            global left, right, back, last_cmd_time
            with motor_speed_lock:
                left = 0
                right = 0
                back = 0
                last_cmd_time = None

    @staticmethod
    def _recent_rate(times, now, window_sec=5.0):
        recent = [stamp for stamp in times if now - stamp <= window_sec]
        if len(recent) < 2 or recent[-1] <= recent[0]:
            return None
        return (len(recent) - 1) / (recent[-1] - recent[0])

    @staticmethod
    def _age(stamp, now):
        return None if stamp is None else max(0.0, now - stamp)

    def publish_health_summary(self):
        global _query_completion_counter
        global _runtime_query_count
        global _query_timeout_count
        global _last_query_timeout_time
        global _last_query_timeout_kind
        global _last_query_timeout_elapsed_sec
        global _driver_error_event_count
        global _driver_no_response_event_count
        global _driver_exception_event_count
        global _last_driver_error_time
        now = time.monotonic()
        recent_timeouts = [stamp for stamp in _query_timeout_times if now - stamp <= 5.0]
        session_timeout_age = self._age(_last_query_timeout_time, now)
        state_age = self._age(self.last_state_time, now)
        summary = {
            "schema_version": 1,
            "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "source": "drive_motor.py callbacks for /om_response0 and /om_state0",
            "publish_rate_hz": DRIVE_HEALTH_PUBLISH_RATE_HZ,
            "session_started_at": self.session_started_at,
            "state": self.last_state,
            "state_update_count": self.state_update_count,
            "state_snapshot_age_sec": state_age,
            "response_count": _query_completion_counter,
            "runtime_query_count": _runtime_query_count,
            "response_rate_hz_5s": self._recent_rate(self.response_times, now),
            "read_response_rate_hz_5s": self._recent_rate(self.read_response_times, now),
            "last_response_age_sec": self._age(self.last_response_time, now),
            "last_read_response_age_sec": self._age(self.last_read_response_time, now),
            "direct_io": self.direct_io_status,
            "last_direct_io_age_sec": self._age(self.last_direct_io_time, now),
            "electrical_status": self.electrical_status,
            "last_electrical_age_sec": self._age(self.last_electrical_time, now),
            "alarm_status": [
                {
                    key: value
                    for key, value in self.alarm_status[slave_id].items()
                    if key != "observed_monotonic"
                }
                | {
                    "sample_age_sec": self._age(
                        self.alarm_status[slave_id].get("observed_monotonic"), now
                    )
                }
                for slave_id in sorted(self.alarm_status)
            ],
            "alarm_event_count": self.alarm_event_count,
            "last_alarm_event_age_sec": self._age(self.last_alarm_event_time, now),
            "last_response_func_code": self.last_response_func_code,
            "query_timeout_count": _query_timeout_count,
            "query_timeouts_5s": len(recent_timeouts),
            "last_query_timeout_age_sec": session_timeout_age,
            # Preferred truthful names.  Keep the three query_timeout fields
            # above for V1 schema compatibility with existing consumers.
            "completion_deadline_miss_count": _query_timeout_count,
            "completion_deadline_misses_5s": len(recent_timeouts),
            "last_completion_deadline_miss_age_sec": session_timeout_age,
            "last_completion_deadline_miss_query_kind": _last_query_timeout_kind,
            "last_completion_deadline_miss_elapsed_sec": _last_query_timeout_elapsed_sec,
            "driver_error_event_count": _driver_error_event_count,
            "driver_no_response_event_count": _driver_no_response_event_count,
            "driver_exception_event_count": _driver_exception_event_count,
            "last_driver_error_age_sec": self._age(_last_driver_error_time, now),
        }
        self.pub_health.publish(String(data=json.dumps(summary, ensure_ascii=False)))

    # パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
    def is_idshare(self, res):
        global_id = self.ca.get_parameters_from_another_node("om_node", ["global_id"])
        return global_id[0] == res.slave_id
    
class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.operation_cycle = 0
        self.feedback_read_cycle = 0
        self.diagnostic_cycle = 0
        self.query_pending = False
        self.query_completion_seen = 0
        self.query_started_time = 0.0
        self.last_timeout_log_time = 0.0
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        # Serialize writes and feedback reads on one timer. Concurrent publishers
        # starve read requests because the Modbus driver accepts one query at a time.
        # The C++ Modbus driver accepts only one serial transaction at a time.
        # Use one request every 40 ms and alternate command/feedback (12.5 Hz
        # each). This leaves completion margin between requests and avoids the
        # stale state race that previously produced "Driver is busy" bursts.
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.ca = ClientAsync("pub")

    def destroy_client(self):
        try:
            if self.timer is not None:
                self.destroy_timer(self.timer)
        except Exception:
            pass
        try:
            if self.ca is not None:
                self.ca.destroy_node()
        except Exception:
            pass

    def publish_setup_query(self, msg, label, timeout_sec=0.75, max_attempts=3):
        """Publish one setup request and wait for direct driver evidence.

        The C++ driver accepts only one transaction at a time and its serial
        timeout is 500 ms. Setup previously emitted requests every 10 ms,
        which produced a directly observed busy rejection and no-response.
        Response and state callbacks run on the other executor nodes, so this
        publisher callback can wait without blocking their evidence path.
        """
        global _query_completion_counter
        global _driver_error_event_count
        for attempt in range(1, max_attempts + 1):
            completion_seen = _query_completion_counter
            driver_errors_seen = _driver_error_event_count
            self.pub.publish(msg)
            deadline = time.monotonic() + timeout_sec
            failure = "no completion callback"
            while time.monotonic() < deadline:
                if _query_completion_counter != completion_seen:
                    return True
                if _driver_error_event_count != driver_errors_seen:
                    failure = "driver reported an error"
                    break
                time.sleep(0.002)
            if attempt < max_attempts:
                self.get_logger().warn(
                    f"Modbus setup retry {attempt}/{max_attempts - 1} at "
                    f"{label}: {failure}")
                # A timed-out serial transaction can take up to 500 ms in the
                # C++ layer. Leave an explicit quiet interval before the next
                # idempotent setup write.
                time.sleep(0.15)
        self.get_logger().error(
            f"Modbus setup failed at {label} after {max_attempts} attempts")
        return False

    def timer_callback(self):
        # self.get_logger().info("query is published")
        global _is_timer_active
        global _query_completion_counter
        global _query_timeout_count
        global _last_query_timeout_time
        global _last_query_timeout_kind
        global _last_query_timeout_elapsed_sec
        if self.query_pending:
            if _query_completion_counter != self.query_completion_seen:
                self.query_pending = False
            elif time.monotonic() - self.query_started_time < COMPLETION_CALLBACK_DEADLINE_SEC:
                return
            else:
                now = time.monotonic()
                elapsed_sec = now - self.query_started_time
                _query_timeout_count += 1
                _query_timeout_times.append(now)
                _last_query_timeout_time = now
                _last_query_timeout_kind = _runtime_query_kind
                _last_query_timeout_elapsed_sec = elapsed_sec
                if now - self.last_timeout_log_time >= 5.0:
                    self.last_timeout_log_time = now
                    self.get_logger().warn(
                        "Modbus completion callback deadline miss (500 ms); "
                        "this does not by itself prove a serial timeout; "
                        "further logs are suppressed for 5 s; see "
                        "/drive/health_summary for counters")
                self.query_pending = False
        if _state_driver == 1:
            # print("1\n")
            return

        if self.seq == 0:
            self.seq = 1
        elif self.seq == 1:
            self.seq = 2 if self.set_excitation_on() else -1
        elif self.seq == 2:
            self.seq = 3 if self.set_share_data() else -1
        elif self.seq == 3:
            # Alternate command and feedback so neither can starve the other.
            if self.operation_cycle % 2 == 1:
                # Replace every fourth velocity read with one low-rate health
                # transaction. The four-step cycle reads the combined
                # electrical/DIN3 sample, then Present alarm + Alarm history 1
                # from IDs 1, 2 and 3. Total Modbus transaction rate remains
                # unchanged while every per-driver alarm is sampled ~1.3 Hz.
                if self.feedback_read_cycle % 4 == 3:
                    diagnostic = self.diagnostic_cycle % 4
                    if diagnostic == 0:
                        self.read_drive_diagnostics()
                    else:
                        self.read_drive_alarm(diagnostic)
                    self.diagnostic_cycle += 1
                else:
                    self.read_drive_feedback()
                self.feedback_read_cycle += 1
            else:
                self.set_drive_operation()
            self.operation_cycle += 1
            #print("1\n")
            self.seq = 3
        elif self.seq == 4:
            _is_timer_active = False
            self.wait(0.03)
            self.set_excitation_off()
            self.seq = 5
        else:
            pass

    def set_excitation_off(self):
        msg = Query()
        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 運転指令(S-ONをOFFする)
        msg.slave_id = 1  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        if not self.publish_setup_query(msg, "excitation off ID 1"):
            return False

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        if not self.publish_setup_query(msg, "excitation off ID 2"):
            return False

        msg.slave_id = 3  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        if not self.publish_setup_query(msg, "excitation off ID 3"):
            return False
        return True

    def set_excitation_on(self):
        msg = Query()
        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 運転指令(S-ONをONする)
        msg.slave_id = 1  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        if not self.publish_setup_query(msg, "excitation on ID 1"):
            return False

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 1:Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        if not self.publish_setup_query(msg, "excitation on ID 2"):
            return False

        msg.slave_id = 3  # スレーブID
        msg.func_code = 1  # ファンクションコード: 1:Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        if not self.publish_setup_query(msg, "excitation on ID 3"):
            return False
        return True

    # 各軸のID Shareモードの設定を行う
    def set_share_data(self):
        msg = Query()
        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 1軸目の設定
        msg.slave_id = 1  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 3  # Share control number
        msg.data[2] = 1  # Share control local ID
        if not self.publish_setup_query(msg, "ID-share control ID 1"):
            return False

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数*軸数=36
        msg.data[0] = 45  # Share Read data[0] → DDO運転方式
        msg.data[1] = 46  # Share Read data[1] → DDO位置
        msg.data[2] = 47  # Share Read data[2] → DDO速度
        msg.data[3] = 48  # Share Read data[3] → DDO加速レート
        msg.data[4] = 49  # Share Read data[4] → DDO減速レート
        msg.data[5] = 50  # Share Read data[5] → DDOトルク制限値
        msg.data[6] = 102  # Share Read data[6] → 検出位置[step]
        msg.data[7] = 103  # Share Read data[7] → 検出速度[r/min]
        msg.data[8] = 106  # Share Read data[8] → Direct I/O (DIN0..DIN3)
        msg.data[9] = 155  # Share Read data[9] → Main current [0.001 A]
        msg.data[10] = 163  # Share Read data[10] → Inverter voltage [0.1 V]
        msg.data[11] = 164  # Share Read data[11] → Main supply voltage [0.1 V]

        msg.data[12] = 45  # Share Write data[0] → DDO運転方式
        msg.data[13] = 46  # Share Write data[1] → DDO位置
        msg.data[14] = 47  # Share Write data[2] → DDO速度
        msg.data[15] = 48  # Share Write data[3] → DDO加速レート
        msg.data[16] = 49  # Share Write data[4] → DDO減速レート
        msg.data[17] = 51  # Share Write data[5] → DDO反映トリガ
        msg.data[18] = 0  # Share Write data[6] →
        msg.data[19] = 0  # Share Write data[7] →
        msg.data[20] = 0  # Share Write data[8] →
        msg.data[21] = 0  # Share Write data[9] →
        msg.data[22] = 0  # Share Write data[10] →
        msg.data[23] = 0  # Share Write data[11] →
        if not self.publish_setup_query(msg, "ID-share data ID 1"):
            return False

        # 2軸目の設定
        msg.slave_id = 2  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 3  # Share control number
        msg.data[2] = 2  # Share control local ID
        if not self.publish_setup_query(msg, "ID-share control ID 2"):
            return False

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数*軸数=36
        msg.data[0] = 45  # Share Read data[0] → DDO運転方式
        msg.data[1] = 46  # Share Read data[1] → DDO位置
        msg.data[2] = 47  # Share Read data[2] → DDO速度
        msg.data[3] = 48  # Share Read data[3] → DDO加速レート
        msg.data[4] = 49  # Share Read data[4] → DDO減速レート
        msg.data[5] = 50  # Share Read data[5] → DDOトルク制限値
        msg.data[6] = 102  # Share Read data[6] → DDO検出位置[step]
        msg.data[7] = 103  # Share Read data[7] → DDO検出速度[r/min]
        msg.data[8] = 106  # Share Read data[8] → Direct I/O (DIN0..DIN3)
        msg.data[9] = 155  # Share Read data[9] → Main current [0.001 A]
        msg.data[10] = 163  # Share Read data[10] → Inverter voltage [0.1 V]
        msg.data[11] = 164  # Share Read data[11] → Main supply voltage [0.1 V]

        msg.data[12] = 45  # Share Write data[0] → DDO運転方式
        msg.data[13] = 46  # Share Write data[1] → DDO位置
        msg.data[14] = 47  # Share Write data[2] → DDO速度
        msg.data[15] = 48  # Share Write data[3] → DDO加速レート
        msg.data[16] = 49  # Share Write data[4] → DDO減速レート
        msg.data[17] = 51  # Share Write data[5] → DDO反映トリガ
        msg.data[18] = 0  # Share Write data[6] →
        msg.data[19] = 0  # Share Write data[7] →
        msg.data[20] = 0  # Share Write data[8] →
        msg.data[21] = 0  # Share Write data[9] →
        msg.data[22] = 0  # Share Write data[10] →
        msg.data[23] = 0  # Share Write data[11] →
        if not self.publish_setup_query(msg, "ID-share data ID 2"):
            return False

        # 3軸目の設定
        msg.slave_id = 3  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 3  # Share control number
        msg.data[2] = 3  # Share control local ID
        if not self.publish_setup_query(msg, "ID-share control ID 3"):
            return False

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数*軸数=36
        msg.data[0] = 45  # Share Read data[0] → DDO運転方式
        msg.data[1] = 46  # Share Read data[1] → DDO位置
        msg.data[2] = 47  # Share Read data[2] → DDO速度
        msg.data[3] = 48  # Share Read data[3] → DDO加速レート
        msg.data[4] = 49  # Share Read data[4] → DDO減速レート
        msg.data[5] = 50  # Share Read data[5] → DDOトルク制限値
        msg.data[6] = 102  # Share Read data[6] → DDO検出位置[step]
        msg.data[7] = 103  # Share Read data[7] → DDO検出速度[r/min]
        msg.data[8] = 106  # Share Read data[8] → Direct I/O (DIN0..DIN3)
        msg.data[9] = 155  # Share Read data[9] → Main current [0.001 A]
        msg.data[10] = 163  # Share Read data[10] → Inverter voltage [0.1 V]
        msg.data[11] = 164  # Share Read data[11] → Main supply voltage [0.1 V]

        msg.data[12] = 45  # Share Write data[0] → DDO運転方式
        msg.data[13] = 46  # Share Write data[1] → DDO位置
        msg.data[14] = 47  # Share Write data[2] → DDO速度
        msg.data[15] = 48  # Share Write data[3] → DDO加速レート
        msg.data[16] = 49  # Share Write data[4] → DDO減速レート
        msg.data[17] = 51  # Share Write data[5] → DDO反映トリガ
        msg.data[18] = 0  # Share Write data[6] →
        msg.data[19] = 0  # Share Write data[7] →
        msg.data[20] = 0  # Share Write data[8] →
        msg.data[21] = 0  # Share Write data[9] →
        msg.data[22] = 0  # Share Write data[10] →
        msg.data[23] = 0  # Share Write data[11] →
        if not self.publish_setup_query(msg, "ID-share data ID 3"):
            return False
        self.ca.set_parameters_from_another_node("om_node", "global_id", 10)
        return True
        #print("2\n")

    def set_drive_operation(self):
        # self.get_logger().info("-----------------------------------")
        # self.get_logger().info("left %f" % (left))
        # self.get_logger().info("right %f" % (right))
        global _is_timer_active
        global left, right, back
        msg = Query()
        a = 300
        d = 100
        # ID Shareモードで通信するため、global_id=10に設定
        #self.ca.set_parameters_from_another_node("om_node", "global_id", 10)
        #print("3\n")
        # ID Shareモードで各モーターを運転する
        # 120[r/min]で運転させる
        _is_timer_active = False  # タイマー処理停止
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1  # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点
        msg.write_num = 18  # 全軸合わせたデータ項目数を代入する
        # スレッドセーフに速度値を読み込む
        with motor_speed_lock:
            left_val = left
            right_val = right
            back_val = back
        # 直近の指令が古い場合は停止（指令残留対策）
        if last_cmd_time is None or (time.monotonic() - last_cmd_time) > CMD_TIMEOUT_SEC:
            left_val = 0
            right_val = 0
            back_val = 0
        # 1軸目のデータ
        msg.data[0] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = back_val  # DDO運転速度(初期単位：r/min)
        msg.data[3] = a  # DDO加速レート(初期単位：ms)
        msg.data[4] = d  # DDO減速レート(初期単位：ms)
        msg.data[5] = 1  # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = left_val # DDO運転速度(初期単位：r/min)
        msg.data[9] = a  # DDO加速レート(初期単位：ms)
        msg.data[10] = d  # DDO減速レート(初期単位：ms)
        msg.data[11] = 1  # DDO運転トリガ設定
        #3軸目のデータ
        msg.data[12] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[13] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[14] = right_val  # DDO運転速度(初期単位：r/min)
        msg.data[15] = a  # DDO加速レート(初期単位：ms)
        msg.data[16] = d  # DDO減速レート(初期単位：ms)
        msg.data[17] = 1  # DDO運転トリガ設定
        # 配信
        #self.wait(0.03)
        self.publish_runtime_query(msg)
        #self.wait(0.03)
        _is_timer_active = True
        #self.wait(5)
        #self.get_logger().info("broadcast! %f" % (time.time()))

    def read_drive_feedback(self):
        msg = Query()
        msg.slave_id = 10
        msg.func_code = 0
        msg.read_addr = 0x000E
        msg.read_num = 3
        self.publish_runtime_query(msg, kind="velocity")

    def read_drive_direct_io(self):
        """Compatibility alias; the runtime now reads one combined sample."""
        self.read_drive_diagnostics()

    def read_drive_diagnostics(self):
        msg = Query()
        msg.slave_id = 10
        msg.func_code = 0
        # Share Read data[8] starts at 8 * 2 = 0x0010. Read four configured
        # 32-bit values for each of three axes. The C++ ID-share decoder
        # removes per-axis checks and preserves per-axis grouping.
        msg.read_addr = 0x0010
        msg.read_num = 12
        self.publish_runtime_query(msg, kind="drive_diagnostics")

    def read_drive_alarm(self, slave_id):
        msg = Query()
        msg.slave_id = int(slave_id)
        msg.func_code = 0
        # HP-5142-4E register 128/129 is Present alarm (NET-ID 64),
        # followed by register 130/131 Alarm history 1 (NET-ID 65).
        msg.read_addr = 128
        msg.read_num = 2
        self.publish_runtime_query(msg, kind=f"drive_alarm:{int(slave_id)}")

    def publish_runtime_query(self, msg, kind="write"):
        global _query_completion_counter
        global _runtime_query_count
        global _runtime_query_kind
        # Arm the guard before publish; the MultiThreadedExecutor may deliver a
        # very fast response callback immediately after publish returns.
        self.query_completion_seen = _query_completion_counter
        self.query_started_time = time.monotonic()
        self.query_pending = True
        _runtime_query_kind = kind
        _runtime_query_count += 1
        self.pub.publish(msg)

    # t[s]待機する
    def wait(self, t):
        time.sleep(t)
        while _state_driver == 1:
            time.sleep(0.001)


class MyPublisherPolling(Node):
    def __init__(self):
        super().__init__("my_pub_polling")
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.08, self.timer_callback)  # 100ms → 80ms に短縮（控えめ）

    # 一定周期で実行する処理
    def timer_callback(self):
        if _state_driver == 1:
            return
        msg = Query()
        # Always poll for detected velocities regardless of _is_timer_active
        # 到底して常にセンサーデータをポーリング
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 0  # 0:Read
        msg.read_addr = 0x000E  # 読み出すアドレスの起点
        msg.read_num = 3  # 各軸1個ずつ
        self.pub.publish(msg)  # 配信する


def main(args=None):
    rclpy.init(args=args)
    # rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    i = True  # Initialize before try block to avoid UnboundLocalError in finally
    executor = None
    try:
        pub1 = MyPublisher()
        sub_2 = Mysubscription2()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub1)
        executor.add_node(sub_2)
        executor.add_node(sub)
        executor.spin()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     pub1.destroy_node()
    #     # pub2.destroy_node()
    #     sub.destroy_node()
    #     sub_2.destroy_node()
    #     rclpy.shutdown()
    except KeyboardInterrupt:
        # pub1.get_logger().info("Keyboard Interrupt")
        # pub1.seq = 4
        # time.sleep(0.5)
        # pub1.destroy_node()
        # # pub2.destroy_node()
        # sub.destroy_node()
        # sub_2.destroy_node()
        # rclpy.shutdown()
        if 'pub1' in locals():
            pub1.get_logger().info("python fin.")
        i = False
    finally:
        if i:
            try:
                pub1.seq = 4
            except Exception:
                pass
            time.sleep(1.0)
            try:
                pub1.destroy_client()
            except Exception:
                pass
            try:
                sub.destroy_client()
                sub.destroy_node()
            except Exception:
                pass
            try:
                sub_2.destroy_client()
                sub_2.destroy_node()
            except Exception:
                pass
            try:
                if executor is not None:
                    executor.shutdown()
            except Exception:
                pass
            time.sleep(0.5)
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
