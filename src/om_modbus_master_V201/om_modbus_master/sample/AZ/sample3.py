#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:AZ(2軸)
# 処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)(LOOP:5回)
# 処理内容2:検出位置の確認(LOOP:5回)
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
from om_msgs.msg import Response
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
        self.sub1 = self.create_subscription(
            Response, "om_response0", self.response_callback, const.QUEUE_SIZE
        )
        self.sub2 = self.create_subscription(
            State, "om_state0", self.state_callback, const.QUEUE_SIZE
        )

    def response_callback(self, res):
        """レスポンスコールバック関数

        購読したレスポンスデータをグローバル変数に反映する

        """
        if res.slave_id == 1 and res.func_code == 3:
            # 号機番号が1かつ読み込みのときに値を更新
            motor_pos1 = res.data[0]
            print("FeedbackPosition1 = {0:}[step]".format(motor_pos1))  # 軸1の検出位置の表示
        elif res.slave_id == 2 and res.func_code == 3:
            # 号機番号が2かつ読み込みのときに値を更新
            motor_pos2 = res.data[0]
            print("FeedbackPosition2 = {0:}[step]".format(motor_pos2))  # 軸2の検出位置の表示

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
        self.publisher_ = self.create_publisher(Query, "om_query0", const.QUEUE_SIZE)
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
            self.init()  # 初期化関数のコール
            self.seq = 1
        elif self.seq == 1:
            self.set_drive_operation()
            self.seq = 2
        elif self.seq == 2:
            self.stop()  # 運転停止
            print("END")  # 終了表示
            self.seq = 3
        else:
            return

    def set_drive_operation(self):
        global msg
        for i in range(5):  # LOOP処理(5回)
            # 指令位置の書き込み
            msg.slave_id = 0x01  # 号機選択(Hex): 1
            msg.func_code = 1  # ファンクションコード選択: 1(Write)
            msg.write_addr = 88  # 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h)
            msg.write_num = 8  # 書き込みデータサイズ: 8(8x32bit)
            msg.data[0] = 0  # 運転データNo.: 0
            msg.data[1] = 2  # 方式: 2:相対位置決め(指令位置基準)
            msg.data[2] = 100  # 位置: 100[step]
            msg.data[3] = 2000  # 速度: 2000[Hz]
            msg.data[4] = 2000  # 起動・変速レート: 2.0[kHz/s]
            msg.data[5] = 2000  # 停止レート: 2.0[kHz/s]
            msg.data[6] = 1000  # 運転電流: 100[%]
            msg.data[7] = 1  # 反映トリガ: 1(全データ反映)
            self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()

            msg.slave_id = 0x02  # 号機選択(Hex): 2
            msg.func_code = 1  # ファンクションコード選択: 1(Write)
            msg.write_addr = 88  # 先頭アドレス選択(Dec): ダイレクト運転データNo.(0058h)
            msg.write_num = 8  # 書き込みデータサイズ: 8(8x32bit)
            msg.data[0] = 0  # 運転データNo.: 0
            msg.data[1] = 2  # 方式: 2:相対位置決め(指令位置基準)
            msg.data[2] = 200  # 位置: 200[step]
            msg.data[3] = 2000  # 速度: 2000[Hz]
            msg.data[4] = 2000  # 起動・変速レート: 2.0[kHz/s]
            msg.data[5] = 2000  # 停止レート: 2.0[kHz/s]
            msg.data[6] = 1000  # 運転電流: 100[%]
            msg.data[7] = 1  # 反映トリガ: 1(全データ反映)
            self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()

            time.sleep(0.5)

            msg.slave_id = 0x01  # 号機選択(Hex): 1
            msg.func_code = 0  # ファンクションコード選択: 0(Read)
            msg.read_addr = 204  # 先頭アドレス選択(Dec): 検出位置(step)
            msg.read_num = 1  # 読み込みデータサイズ: 1 (32bit)
            self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()

            msg.slave_id = 0x02  # 号機選択(Hex): 2
            msg.func_code = 0  # ファンクションコード選択: 0(Read)
            msg.read_addr = 204  # 先頭アドレス選択(Dec): 検出位置(step)
            msg.read_num = 1  # 読み込みデータサイズ: 1 (32bit)
            self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()
            time.sleep(1)

    def init(self):
        """初期化関数

        処理内容:信号初期化

        """
        global msg
        # 信号初期化
        msg.slave_id = 0x00  # 号機選択(Hex): ブロードキャスト
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): ドライバ入力指令
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 0  # 書き込みデータ: 全ビットOFF
        self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def stop(self):
        """停止サービス関数

        運転入力指令をOFFにする（停止指令を行う）サービス

        """
        global msg
        msg.slave_id = 0x00  # 号機選択(Hex): ブロードキャスト
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): ドライバ入力指令
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 32  # 書き込みデータ: (0000 0000 0010 0000) = 32(STOP信号ON)
        self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        msg.slave_id = 0x00  # 号機選択(Hex): ブロードキャスト
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): ドライバ入力指令
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 0  # 書き込みデータ: 全ビットOFF
        self.publisher_.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def wait(self):
        """処理待ちサービス関数

        規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

        """
        time.sleep(0.03)
        # ドライバの通信が終了するまでループ
        while _state_driver == 1:
            pass


def main(args=None):
    """メイン関数

    処理内容1:軸1、軸2にダイレクトデータ運転を書き込み(相対位置決め運転)
    処理内容2:検出位置の読み込み(LOOP:5回)

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
