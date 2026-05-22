#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ
# 処理内容1:運転データNo.0の位置を書き込み

# モジュールのインポート
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import time
import rclpy
from utils import const
from rclpy.node import Node
from om_msgs.msg import Query
from om_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor

# グローバル変数
_state_driver = 0  # 通信可能フラグ変数(0:通信可能,1:通信中)
msg = Query()

# 定数
const.QUEUE_SIZE = 1


class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        self.sub = self.create_subscription(
            State, "om_state0", self.state_callback, const.QUEUE_SIZE
        )

    def state_callback(self, res):
        """ステータスコールバック関数

        購読したステータスデータをグローバル変数に反映する

        """
        global _state_driver
        _state_driver = res.state_driver


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        if _state_driver == 1:
            return

        if self.seq == 0:
            print("START")
            self.seq = 1
        elif self.seq == 1:
            self.write_data()
            self.seq = 2
        elif self.seq == 2:
            print("END")
            self.seq = 3
        else:
            return

    def write_data(self):
        global msg
        # 書き込み(運転データNo.0の位置)
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 6146  # 先頭アドレス選択(Dec): 運転データNo.0の位置
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 1000  # 位置[step]
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def wait(self):
        """処理待ちサービス関数

        規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

        """
        time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
        # 通信が終了するまでループ
        while _state_driver == 1:
            pass


def main(args=None):
    """メイン関数

    処理内容1:運転データNo.0の位置を書き込み


    """
    rclpy.init(args=args)
    try:
        pub = MyPublisher()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
