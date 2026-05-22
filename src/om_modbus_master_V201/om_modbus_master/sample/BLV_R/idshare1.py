#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

#
# ●対象機種:
#       BLV-R
#
# ●ドライバ設定:
#       パラメータ名称: [1軸目, 2軸目]
#       通信ID: [1, 2]
#       Baudrate: [230400, 230400]
#
# ●launchファイル設定:
#       com:="/dev/ttyUSB0" topicID:=1 baudrate:=230400 updateRate:=1000 firstGen:="" secondGen:="1,2," globalID:="10" axisNum:="2"
#
# ●処理内容:
#       Writeにより連続運転(速度制御)でモーターを運転させる。
#       Readにより一定周期でモーターの検出速度を取得し、表示させる。

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
_state_mes = 0  # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
_state_error = 0  # 0:エラーなし 1:無応答 2:例外応答
_is_timer_active = False
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

    def response_callback(self, res):
        # 例外応答のとき
        if _state_error == 2:
            print("Exception")
            return
        # ID Shareモードのとき
        if self.is_idshare(res) and (res.func_code == 0x03):
            axis_num = self.ca.get_parameters_from_another_node("om_node", ["axis_num"])
            for axis in range(axis_num[0]):
                print(
                    "{0}: {1}[r/min], {2}[r/min]".format(
                        datetime.datetime.now(), res.data[0], res.data[1]
                    )
                )  # [0]:1軸目の検出速度、[1]:2軸目の検出速度

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
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.ca = ClientAsync("pub")

        def __del__(self):
            self.ca.destroy_node()

    def timer_callback(self):
        global _is_timer_active
        if _state_driver == 1:
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
            self.seq = 4
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
        self.wait(0.03)

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 0:Read 1:Write 2:Read/Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 0  # S-ONを立ち下げる
        self.pub.publish(msg)  # 配信
        self.wait(0.03)

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
        self.wait(0.03)

        msg.slave_id = 2  # スレーブID
        msg.func_code = 1  # ファンクションコード: 1:Write
        msg.write_addr = 124  # アドレス指定： ドライバ入力指令
        msg.write_num = 1  # 書き込みデータ数: 1
        msg.data[0] = 1  # S-ONを立ち上げる
        self.pub.publish(msg)  # 配信
        self.wait(0.03)

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
        msg.data[1] = 2  # Share control number
        msg.data[2] = 1  # Share control local ID
        self.pub.publish(msg)  # 配信する
        self.wait(0.03)

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数*軸数=24
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
        self.wait(0.03)

        # 2軸目の設定
        msg.slave_id = 2  # 書き込むドライバのスレーブID
        msg.func_code = 1  # 1:Write
        msg.write_addr = 0x0980  # 書き込みの起点：Share Control Global IDのアドレス
        msg.write_num = 3  # 書き込む数
        msg.data[0] = 10  # Share control global ID
        msg.data[1] = 2  # Share control number
        msg.data[2] = 2  # Share control local ID
        self.pub.publish(msg)
        self.wait(0.03)

        msg.write_addr = 0x0990  # 書き込みの起点：Share Read data[0]
        msg.write_num = 24  # 書き込むデータ数*軸数=24
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
        self.wait(0.03)

    def set_drive_operation(self):
        global _is_timer_active
        global msg
        # ID Shareモードで通信するため、global_id=10に設定
        self.ca.set_parameters_from_another_node("om_node", "global_id", 10)

        # ID Shareモードで各モーターを運転する
        # 120[r/min]で運転させる
        _is_timer_active = False  # タイマー処理停止
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1  # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点
        msg.write_num = 12  # 全軸合わせたデータ項目数を代入する
        # 1軸目のデータ
        msg.data[0] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = 120  # DDO運転速度(初期単位：r/min)
        msg.data[3] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[4] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[5] = 1  # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = 120  # DDO運転速度(初期単位：r/min)
        msg.data[9] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[10] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[11] = 1  # DDO運転トリガ設定
        # 配信
        self.wait(0.03)
        self.pub.publish(msg)
        self.wait(0.03)
        _is_timer_active = True
        self.wait(5)

        # 240[r/min]で運転させる
        _is_timer_active = False  # タイマー処理停止
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1  # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点
        msg.write_num = 12  # 書き込むデータ数*軸数=12
        # 1軸目のデータ
        msg.data[0] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = 240  # DDO運転速度(初期単位：r/min)
        msg.data[3] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[4] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[5] = 1  # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = 240  # DDO運転速度(初期単位：r/min)
        msg.data[9] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[10] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[11] = 1  # DDO運転トリガ設定
        # 配信
        self.wait(0.03)
        self.pub.publish(msg)
        self.wait(0.03)
        _is_timer_active = True  # タイマー処理再開
        self.wait(5)

        # 360[r/min]で運転させる
        _is_timer_active = False  # タイマー処理停止
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1  # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点
        msg.write_num = 12  # 書き込むデータ数*軸数=12
        # 1軸目のデータ
        msg.data[0] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = 360  # DDO運転速度(初期単位：r/min)
        msg.data[3] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[4] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[5] = 1  # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = 360  # DDO運転速度(初期単位：r/min)
        msg.data[9] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[10] = 1000  # DDO減速レート(初期単位：ms)
        msg.data[11] = 1  # DDO運転トリガ設定
        # 配信
        self.wait(0.03)
        self.pub.publish(msg)
        self.wait(0.03)
        _is_timer_active = True
        self.wait(5)

        # 停止するまで減速させる
        _is_timer_active = False  # タイマー処理停止
        msg.slave_id = 10  # スレーブID指定(ID Shareモードのときはglobal_idとみなされる)
        msg.func_code = 1  # 0:read 1:write 2:read/write
        msg.write_addr = 0x0000  # 書き込むアドレスの起点
        msg.write_num = 12  # 書き込むデータ数*軸数=12
        # 1軸目のデータ
        msg.data[0] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[1] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[2] = 0  # DDO運転速度(初期単位：r/min)
        msg.data[3] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[4] = 3000  # DDO減速レート(初期単位：ms)
        msg.data[5] = 1  # DDO運転トリガ設定
        # 2軸目のデータ
        msg.data[6] = 16  # DDO運転方式 16:連続運転(速度制御)
        msg.data[7] = 0  # DDO運転位置(初期単位：1step = 0.01deg)連続運転(速度制御)なので無関係
        msg.data[8] = 0  # DDO運転速度(初期単位：r/min)
        msg.data[9] = 1000  # DDO加速レート(初期単位：ms)
        msg.data[10] = 3000  # DDO減速レート(初期単位：ms)
        msg.data[11] = 1  # DDO運転トリガ設定
        # 配信
        self.wait(0.03)
        self.pub.publish(msg)
        self.wait(0.03)
        _is_timer_active = True
        self.wait(4)

    # t[s]待機する
    def wait(self, t):
        time.sleep(t)
        while _state_driver == 1:
            pass


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
            msg.read_addr = 0x000E  # 読み出すアドレスの起点
            msg.read_num = 2  # 各軸1個ずつ
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
