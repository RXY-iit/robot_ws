#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:BLV
# 処理内容1:運転入力方式を3ワイヤ方式に変更
# 処理内容2:運転データNo.2の回転速度を書き込み
# 処理内容3:運転指令(FWD方向)
# 処理内容4:停止指令(減速停止)
#

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
_state_mes = 0  # メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー)
_state_error = 0  # エラー(0:エラーなし,1:無応答,2:例外応答)
msg = Query()

# 定数
const.QUEUE_SIZE = 1
const.MESSAGE_ERROR = 2
const.EXCEPTION_RESPONSE = 2


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
        global _state_mes
        global _state_error
        _state_driver = res.state_driver
        _state_mes = res.state_mes
        _state_error = res.state_error


class MyPublisher(Node):
    def __init__(self):
        super().__init__("my_pub")
        self.seq = 0
        self.pub = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        if _state_driver == 1:
            return

        # メッセージエラーの発生
        if _state_mes == const.MESSAGE_ERROR:
            self.stop()  # 運転停止
            exit()  # 処理の強制終了

        # 例外応答の発生
        if _state_error == const.EXCEPTION_RESPONSE:
            self.stop()  # 運転停止
            exit()  # 処理の強制終了

        if self.seq == 0:
            print("START")
            self.init()
            self.seq = 1
        elif self.seq == 1:
            self.set_data()
            self.seq = 2
        elif self.seq == 2:
            self.start()
            self.seq = 3
        elif self.seq == 3:
            time.sleep(5)  # 5秒待機
            self.stop()  # 運転停止
            print("END")  # 終了表示
            self.seq = 4
        else:
            return

    def wait(self):
        """処理待ちサービス関数

        規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

        """
        time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
        # 通信が終了するまでループ
        while _state_driver == 1:
            pass

    def stop(self):
        """停止サービス関数

        運転入力指令をOFFにする（停止指令を行う）サービス

        """
        global msg
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 18  # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def init(self):
        """初期化関数

        処理内容1:運転入力方式を3ワイヤ方式に変更
        処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
        処理内容3:Configrationの実行

        """
        global msg
        # 処理1 3ワイヤに変更
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 4160  # 先頭アドレス選択(Dec): 運転入力方式パラメータ
        msg.write_num = 1  # 書き込みデータサイズ: 1(32bit)
        msg.data[0] = 1  # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # 処理2 運転データNo.2の回転速度の初期化(0[r/min])
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156  # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 0  # 書き込みデータ: 0[r/min]
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # 処理3 Configrationの実行
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 1  # 書き込みデータ: 1(実行)
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def set_data(self):
        global msg
        # 速度設定               # 運転データNo.2の回転速度を書き込み
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156  # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 1000  # 書き込みデータ: 1000[r/min]
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def start(self):
        global msg
        # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 26  # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち


def main(args=None):
    """メイン関数

    処理内容1:運転データNo.2の回転速度を書き込み
    処理内容2:運転指令(FWD方向)
    処理内容3:停止指令(減速停止)

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
