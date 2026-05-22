#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       AZ
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [230400, 230400]
#
# ●launchファイル設定:
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2," globalID:="10" axisNum:="2"
#
# ●処理内容：
#       ID Shareモードで2軸を5回2000[step]動かしたあと、1回-10000[step]動かす。
#       その間、0.3[s]周期で検出位置を表示させる。
#

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from rclpy.node import Node
import time
import datetime
import rclpy
from utils import const
from utils.clientasync import ClientAsync
from om_msgs.msg import Query
from om_msgs.msg import Response
from om_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor


# グローバル変数
_state_driver = 0  # 0:通信可能 1:通信中
_state_error = 0  # 0:エラーなし 1:無応答 2:例外応答
_is_timer_active = False  # タイマー処理を実行するか
msg = Query()

# 定数
const.QUEUE_SIZE = 1


class MySubscription(Node):
    def __init__(self):
        super().__init__("my_sub")
        self.sub1 = self.create_subscription(
            Response, "om_response0", self.response_callback, const.QUEUE_SIZE
        )
        self.sub2 = self.create_subscription(
            State, "om_state0", self.state_callback, const.QUEUE_SIZE
        )

        self.ca = ClientAsync("sub")

    def __del__(self):
        self.ca.destroy_node()

    # ドライバ状態のコールバック関数
    def state_callback(self, res):
        global _state_driver, _state_error
        _state_driver = res.state_driver
        _state_error = res.state_error

    # レスポンスのコールバック関数
    def response_callback(self, res):
        # 例外応答のとき
        if _state_error == 2:
            print("Exception")
            return
        # ID Shareモードで読み込みを行ったとき
        if self.is_idshare(res) and (res.func_code == 0x03):
            print(
                "{0}: {1}[step], {2}[step]".format(
                    datetime.datetime.now(), res.data[0], res.data[1]
                )
            )  # [0]:1軸目の検出位置、[1]:2軸目の検出位置

    # パラメータサーバとresponseのslave_idから、現在ID Shareモードか調べる
    def is_idshare(self, res):
        global_id = self.ca.get_parameters_from_another_node("om_node", ["global_id"])
        return global_id[0] == res.slave_id


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.ca = ClientAsync("pub")

    def __del__(self):
        self.ca.destroy_node()

    def timer_callback(self):
        global _is_timer_active
        if _state_driver == 1:
            return

        if self.seq == 0:
            self.set_share_data()
            self.seq = 1
        elif self.seq == 1:
            self.set_drive_operation()
            self.seq = 2
        elif self.seq == 2:
            _is_timer_active = False
            self.seq = 3
        else:
            pass

    # t[s]待機する
    def wait(self, t):
        time.sleep(t)

        while _state_driver == 1:
            pass

    # 各軸のID Shareモードの設定を行う
    def set_share_data(self):
        global msg

        # ユニキャストモードで通信するため、global_id=-1に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", -1)

        # 1軸目の設定
        msg.slave_id = 1  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込むデータ数=3
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 2  # Share control number
        msg.data[2] = 1  # Share control local ID
        self.pub.publish(msg)  # 配信
        self.wait(0.03)  # 配信後の待機

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数=24
        msg.data[0] = 45  # Share Read data[0] → DDO運転方式
        msg.data[1] = 46  # Share Read data[1] → DDO位置
        msg.data[2] = 47  # Share Read data[2] → DDO速度
        msg.data[3] = 48  # Share Read data[3] → DDO起動・変速レート
        msg.data[4] = 49  # Share Read data[4] → DDO停止レート
        msg.data[5] = 50  # Share Read data[5] → DDO運転電流
        msg.data[6] = 102  # Share Read data[6] → 検出位置
        msg.data[7] = 103  # Share Read data[7] → 検出速度(r/min)
        msg.data[8] = 0  # Share Read data[8] →
        msg.data[9] = 0  # Share Read data[9] →
        msg.data[10] = 0  # Share Read data[10] →
        msg.data[11] = 0  # Share Read data[11] →

        msg.data[12] = 45  # Share Write data[0] → DDO運転方式
        msg.data[13] = 46  # Share Write data[1] → DDO位置
        msg.data[14] = 47  # Share Write data[2] → DDO速度
        msg.data[15] = 51  # Share Write data[3] → DDO反映トリガ
        msg.data[16] = 0  # Share Write data[4] →
        msg.data[17] = 0  # Share Write data[5] →
        msg.data[18] = 0  # Share Write data[6] →
        msg.data[19] = 0  # Share Write data[7] →
        msg.data[20] = 0  # Share Write data[8] →
        msg.data[21] = 0  # Share Write data[9] →
        msg.data[22] = 0  # Share Write data[10] →
        msg.data[23] = 0  # Share Write data[11] →
        self.pub.publish(msg)  # 配信
        self.wait(0.03)  # 配信後の待機

        # 2軸目の設定
        msg.slave_id = 2  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込むデータ数=3
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 2  # Share control number
        msg.data[2] = 2  # Share control local ID
        self.pub.publish(msg)  # 配信
        self.wait(0.03)  # 配信後の待機

        msg.write_addr = 0x0990  # 書き込むアドレスの起点: Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数=24
        msg.data[0] = 45  # Share Read data[0] → DDO運転方式
        msg.data[1] = 46  # Share Read data[1] → DDO位置
        msg.data[2] = 47  # Share Read data[2] → DDO速度
        msg.data[3] = 48  # Share Read data[3] → DDO起動・変速レート
        msg.data[4] = 49  # Share Read data[4] → DDO停止レート
        msg.data[5] = 50  # Share Read data[5] → DDO運転電流
        msg.data[6] = 102  # Share Read data[6] → 検出位置
        msg.data[7] = 103  # Share Read data[7] → 検出速度(r/min)
        msg.data[8] = 0  # Share Read data[8] →
        msg.data[9] = 0  # Share Read data[9] →
        msg.data[10] = 0  # Share Read data[10] →
        msg.data[11] = 0  # Share Read data[11] →

        msg.data[12] = 45  # Share Write data[0] → DDO運転方式
        msg.data[13] = 46  # Share Write data[1] → DDO位置
        msg.data[14] = 47  # Share Write data[2] → DDO速度
        msg.data[15] = 51  # Share Write data[3] → DDO反映トリガ
        msg.data[16] = 0  # Share Write data[4] →
        msg.data[17] = 0  # Share Write data[5] →
        msg.data[18] = 0  # Share Write data[6] →
        msg.data[19] = 0  # Share Write data[7] →
        msg.data[20] = 0  # Share Write data[8] →
        msg.data[21] = 0  # Share Write data[9] →
        msg.data[22] = 0  # Share Write data[10] →
        msg.data[23] = 0  # Share Write data[11] →
        self.pub.publish(msg)  # 配信
        self.wait(0.03)  # 配信後の待機

    def set_drive_operation(self):
        global msg
        global _is_timer_active

        # ID Shareモードで通信するため、global_id=10に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", 10)

        for i in range(5):
            _is_timer_active = False  # タイマー処理停止
            # ID Shareモードで各軸2000[step]の運転を5回行う
            msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
            msg.func_code = 1  # 0:read 1:write 2:read/write
            msg.write_addr = 0x0000  # DDO運転方式から書き込む(Modbus-Share設定でそのように設定している)
            msg.write_num = 8  # 全軸合わせたデータ項目数を代入する 4個*2軸分
            # 1軸目のデータ
            msg.data[0] = 2  # DDO運転方式 2:相対位置決め(指令位置基準)
            msg.data[1] = 2000  # DDO運転位置
            msg.data[2] = 2000  # DDO運転速度
            msg.data[3] = 1  # DDO運転反映トリガ
            # 2軸目のデータ
            msg.data[4] = 2  # DDO運転方式 2:相対位置決め(指令位置基準)
            msg.data[5] = 2000  # DDO運転位置
            msg.data[6] = 2000  # DDO運転速度
            msg.data[7] = 1  # DDO運転反映トリガ
            # 配信
            self.wait(0.03)  # タイマー処理中のpublishとぶつからないための待機
            self.pub.publish(msg)  # msgの配信
            self.wait(0.03)  # 配信後の待機
            _is_timer_active = True  # タイマー処理再開
            self.wait(2)  # 運転終了まで待機

        # ID Shareモードで各軸ずつ-10000[step]運転させる
        msg.slave_id = 10  # スレーブID指定
        msg.func_code = 1  # 1:write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点:DDO運転方式
        msg.write_num = 8  # 書き込むデータ数
        # 1軸目のデータ
        msg.data[0] = 2  # DDO運転方式 2:相対位置決め(指令位置基準)
        msg.data[1] = -10000  # DDO運転位置
        msg.data[2] = 5000  # DDO運転速度
        msg.data[3] = 1  # DDO反映トリガ
        # 2軸目のデータ
        msg.data[4] = 2  # DDO運転方式 2:相対位置決め(指令位置基準)
        msg.data[5] = -10000  # DDO運転位置
        msg.data[6] = 5000  # DDO運転速度
        msg.data[7] = 1  # DDO反映トリガ
        _is_timer_active = False  # タイマー処理停止
        self.wait(0.03)  # タイマー処理中のpublishとぶつからないための待機
        self.pub.publish(msg)  # msgの配信
        self.wait(0.03)  # 配信後の待機
        _is_timer_active = True  # タイマー処理再開
        self.wait(3)  # 運転終了まで待機

        _is_timer_active = False


class MyPublisherPolling(Node):
    def __init__(self):
        super().__init__("my_pub_polling")
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.3, self.timer_callback)

    # 一定周期で実行する処理
    def timer_callback(self):
        global msg
        if _state_driver == 1:
            return

        if _is_timer_active:
            msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
            msg.func_code = 0  # 0:Read
            msg.read_addr = 0x000C  # 読み出すアドレスの起点(検出位置)
            msg.read_num = 2  # 各軸1個ずつで計2個
            self.pub.publish(msg)  # 配信する


def main(args=None):
    rclpy.init(args=args)
    try:
        pub1 = MyPublisher()
        pub2 = MyPublisherPolling()
        sub = MySubscription()
        executor = MultiThreadedExecutor()
        executor.add_node(pub1)
        executor.add_node(pub2)
        executor.add_node(sub)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pub1.destroy_node()
        pub2.destroy_node()
        sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
