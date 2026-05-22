#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from aruco_interfaces.msg._aruco_markers import ArucoMarkers
import csv
import os
from rclpy.time import Time
import datetime
import math # ★ 距離計算のためにmathをインポート

class CSVLoggerNode(Node):
    def __init__(self):
        super().__init__('angle_arucopose_csv')

        # ... (ファイルパス関連のコードは同じ) ...
        dir_path = os.path.expanduser("~/pickup_ws/angle_arucopose_csv/")
        os.makedirs(dir_path, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%m%d_%H%M%S")
        filename = f"aruco_motor_log_{timestamp}.csv"
        self.filepath = os.path.join(dir_path, filename)
        self.init_csv()

        # ★★★ しきい値（この値以上の変化があった場合に記録する）★★★
        self.POSITION_THRESHOLD = 0.01  # 1cm (0.01m)
        self.ANGLE_THRESHOLD = 1.0     # 1.0度

        # 最新データ
        self.latest_angles = None
        self.latest_chokudo_angles = None

        # ★★★ 最後に記録した状態をマーカーIDごとに保存する辞書 ★★★
        # { marker_id: {'position': [x,y,z], 'angles': [m1,..,m9], 'chokudo': c1}, ... }
        self.last_logged_state = {}


        # Subscribe
        self.sub_angle = self.create_subscription(
            Float32MultiArray, '/motor_current_angles', self.angle_callback, 10
        )
        self.sub_chokudo_angle = self.create_subscription(
            Float32, '/chokudomotor/angle', self.chokudo_angle_callback, 10
        )
        self.sub_marker = self.create_subscription(
            ArucoMarkers, '/aruco/markers', self.marker_callback, 10
        )

    def init_csv(self):
        # ... (変更なし) ...
        os.makedirs(os.path.dirname(self.filepath), exist_ok=True)
        if not os.path.exists(self.filepath):
            with open(self.filepath, mode='w', newline='') as f:
                writer = csv.writer(f)
                header = ['timestamp'] + [f'motor{i+1}' for i in range(10)] + [
                    'marker_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
                ]
                writer.writerow(header)

    def angle_callback(self, msg):
        if len(msg.data) == 9:
            self.latest_angles = list(msg.data)
        else:
            self.get_logger().warn('Invalid motor angle length.')

    def chokudo_angle_callback(self, msg):
        self.latest_chokudo_angles = msg.data

    
    def marker_callback(self, msg):
        # 最新のモーター角度がなければ何もしない
        if self.latest_angles is None or self.latest_chokudo_angles is None:
            self.get_logger().warn('Motor angles not yet received.')
            return
        
        timestamp = self.get_clock().now().to_msg()
        unix_time = Time.from_msg(timestamp).nanoseconds * 1e-9

        # 検出された各マーカーについてチェック
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            
            if marker_id not in [0, 1, 2]:
                continue # forループの次の繰り返し処理へ進む
                        
            # ★★★ 記録すべきかどうかのフラグ ★★★
            should_log = False
            
            # このマーカーIDが初めて記録される場合
            if marker_id not in self.last_logged_state:
                should_log = True
                self.get_logger().info(f"First detection of marker {marker_id}. Logging.")
            else:
                # 前回の状態を取得
                last_state = self.last_logged_state[marker_id]
                last_pos = last_state['position']
                last_angles = last_state['angles']
                last_chokudo = last_state['chokudo']
                
                current_pos = [pose.position.x, pose.position.y, pose.position.z]
                
                # ★ 位置の変化量（ユークリッド距離）を計算
                pos_diff = math.dist(current_pos, last_pos)
                
                # ★ 角度の変化量（各モーターの差の合計）を計算
                angle_diffs = [abs(curr - last) for curr, last in zip(self.latest_angles, last_angles)]
                angle_sum_diff = sum(angle_diffs) + abs(self.latest_chokudo_angles - last_chokudo)

                # しきい値を超えていたら記録フラグを立てる
                if pos_diff > self.POSITION_THRESHOLD:
                    should_log = True
                    self.get_logger().info(f"Marker {marker_id} logged due to position change: {pos_diff:.4f} m")
                elif angle_sum_diff > self.ANGLE_THRESHOLD:
                    should_log = True
                    self.get_logger().info(f"Marker {marker_id} logged due to angle change: {angle_sum_diff:.2f} deg")

            # ★★★ 記録フラグが立っている場合のみファイルに書き込む ★★★
            if should_log:
                row = [unix_time] + self.latest_angles + [self.latest_chokudo_angles] + [
                    marker_id,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                ]
                with open(self.filepath, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(row)

                # ★★★ 書き込んだら、今回の状態を「前回の状態」として更新 ★★★
                self.last_logged_state[marker_id] = {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'angles': self.latest_angles,
                    'chokudo': self.latest_chokudo_angles
                }


def main(args=None):
    # ... (変更なし) ...
    rclpy.init(args=args)
    node = CSVLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()