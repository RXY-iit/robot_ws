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
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2,3" globalID:="10" axisNum:="3"
#
# ●処理内容:
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

import os
import sys
import threading

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

# グローバル変数
_state_driver = 0  # 0:通信可能 1:通信中
_state_mes = 0  # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
_state_error = 0  # 0:エラーなし 1:無応答 2:例外応答
_is_timer_active = False
msg = Query()
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
CMD_TIMEOUT_SEC = 0.3
# スレッドセーフなロック機構
motor_speed_lock = threading.Lock()
# 定数
const.QUEUE_SIZE = 1

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
        global last_commanded_rpm
        global last_nonzero_cmd_time
        wheeles_size = 0.1125
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
        global last_cmd_time
        last_cmd_time = time.monotonic()
        self.get_logger().info("after calucurate right left %f" % (time.time()))

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
        self.ca = ClientAsync("sub")

    def destroy_client(self):
        try:
            if self.ca is not None:
                self.ca.destroy_node()
        except Exception:
            pass

    def response_callback(self, res):
        global v
        global omega
        global x
        global y
        global th
        global dt1
        global dt2
        global last_servo_diagnostic_time
        global zero_feedback_count_after_cmd
        
        # Debug: func_code and res.data to diagnose why drive_odom is zero
        self.get_logger().info(
            f"response_callback: func_code={res.func_code}, data_len={len(res.data)}, data={res.data[:3] if len(res.data) >= 3 else res.data}"
        )
        
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
            axis_num = self.ca.get_parameters_from_another_node("om_node", ["axis_num"])
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
                vR = math.pi*2*0.1125*(-res.data[1])/(60*30) #m/s
                vL = math.pi*2*0.1125*res.data[0]/(60*30) #m/s
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

                print(
                    "{0}: {1}[m], {2}[m]".format(
                        delta,x,y#res.data[0], res.data[1]
                    )
                )  # [0]:1軸目の検出速度、[1]:2軸目の検出速度
            
            # 3軸分の検出速度を変換してDriveMotorメッセージに格納
            # res.data[0]: 2軸目(left), res.data[1]: 3軸目(right), res.data[2]: 1軸目(back)の検出速度 [r/min]
            wheeles_size = 0.1125
            if len(res.data) >= 3:
                # r/min から m/s に変換
                drive_odom_msg.vel1 = math.pi * 2 * wheeles_size * (-res.data[1]) / (60 * 30)  # right
                drive_odom_msg.vel2 = math.pi * 2 * wheeles_size * res.data[0] / (60 * 30)    # left
                drive_odom_msg.vel3 = math.pi * 2 * wheeles_size * res.data[2] / (60 * 30)    # back
                # パブリッシュ前に可視化用のログを出す
                self.get_logger().info(
                    f"publish drive_odom vel=[{drive_odom_msg.vel1:.4f}, {drive_odom_msg.vel2:.4f}, {drive_odom_msg.vel3:.4f}]"
                )
                self.pub_drive_odom.publish(drive_odom_msg)
                feedback_zero = (
                    abs(drive_odom_msg.vel1) < 1e-4
                    and abs(drive_odom_msg.vel2) < 1e-4
                    and abs(drive_odom_msg.vel3) < 1e-4
                )
                recently_commanded = (
                    last_nonzero_cmd_time is not None
                    and (time.monotonic() - last_nonzero_cmd_time) < 1.0
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
        _state_driver = res.state_driver
        _state_mes = res.state_mes
        _state_error = res.state_error

    # パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
    def is_idshare(self, res):
        global_id = self.ca.get_parameters_from_another_node("om_node", ["global_id"])
        return global_id[0] == res.slave_id
    
class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 20ms → 10ms に短縮
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

    def timer_callback(self):
        # self.get_logger().info("query is published")
        global _is_timer_active
        if _state_driver == 1:
            # print("1\n")
            return

        if self.seq == 0:
            self.seq = 1
        elif self.seq == 1:
            self.set_excitation_on()
            self.seq = 2
        elif self.seq == 2:
            self.set_share_data()
            self.seq = 3
        elif self.seq == 3:
            self.set_drive_operation()
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
        global msg
        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 運転指令(S-ONをOFFする)
        msg.slave_id = 1  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

        msg.slave_id = 3  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

    def set_excitation_on(self):
        global msg
        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 運転指令(S-ONをONする)
        msg.slave_id = 1  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 1:Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

        msg.slave_id = 3  # スレーブID
        msg.func_code = 1  # ファンクションコード: 1:Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        self.pub.publish(msg)  # 配信
        self.wait(0.01)

    # 各軸のID Shareモードの設定を行う
    def set_share_data(self):
        global msg
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
        self.pub.publish(msg)  # 配信する
        self.wait(0.01)

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
        msg.data[8] = 0  # Share Read data[8] →
        msg.data[9] = 0  # Share Read data[9] →
        msg.data[10] = 0  # Share Read data[10] →
        msg.data[11] = 0  # Share Read data[11] →

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
        self.pub.publish(msg)
        self.wait(0.01)

        # 2軸目の設定
        msg.slave_id = 2  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 3  # Share control number
        msg.data[2] = 2  # Share control local ID
        self.pub.publish(msg)
        self.wait(0.01)

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
        msg.data[8] = 0  # Share Read data[8] →
        msg.data[9] = 0  # Share Read data[9] →
        msg.data[10] = 0  # Share Read data[10] →
        msg.data[11] = 0  # Share Read data[11] →

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
        self.pub.publish(msg)
        self.wait(0.01)

        # 3軸目の設定
        msg.slave_id = 3  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 3  # Share control number
        msg.data[2] = 3  # Share control local ID
        self.pub.publish(msg)
        self.wait(0.01)

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
        msg.data[8] = 0  # Share Read data[8] →
        msg.data[9] = 0  # Share Read data[9] →
        msg.data[10] = 0  # Share Read data[10] →
        msg.data[11] = 0  # Share Read data[11] →

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
        self.pub.publish(msg)
        self.wait(0.01)
        self.ca.set_parameters_from_another_node("om_node", "global_id", 10)
        #print("2\n")

    def set_drive_operation(self):
        # self.get_logger().info("-----------------------------------")
        # self.get_logger().info("left %f" % (left))
        # self.get_logger().info("right %f" % (right))
        global _is_timer_active
        global msg
        global left, right, back
        a = 300
        d = 400
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
        self.pub.publish(msg)
        #self.wait(0.03)
        _is_timer_active = True
        #self.wait(5)
        #self.get_logger().info("broadcast! %f" % (time.time()))

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
        global msg

        if _state_driver == 1:
            return
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
        pub2 = MyPublisherPolling()
        sub_2 = Mysubscription2()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub1)
        executor.add_node(pub2)
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
                pub2.destroy_node()
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
