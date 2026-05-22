#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:BLV(2軸)
# 処理内容1:運転データNo.2の回転速度を書き込み
# 処理内容2:運転指令(軸1 FWD方向, 軸2 REV方向)
# 処理内容3:運転データNo.2の回転速度の変更(LOOP:5回)
# 処理内容4:検出速度の確認(LOOP:5回)
# 処理内容5:停止指令(軸1 減速停止、軸2 瞬時停止)
#
#

# モジュールのインポート
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from rclpy.node import Node
import time
import rclpy
from utils import const
from om_msgs.msg import Query
from om_msgs.msg import Response
from om_msgs.msg import State
from rclpy.executors import MultiThreadedExecutor


# グローバル変数
_state_driver = 0  # 0:通信可能 1:通信中
_state_mes = 0  # 0:メッセージなし 1:メッセージ到達 2:メッセージエラー
_state_error = 0  # 0:エラーなし 1:無応答 2:例外応答
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
            motor_spd1 = res.data[0]
            print("FeedbackSpeed1 = {0:}[r/min]".format(motor_spd1))  # 取得した速度の表示[r/min]
        elif res.slave_id == 2 and res.func_code == 3:
            # 号機番号が2かつ読み込みのときに値を更新
            motor_spd2 = res.data[0]
            print("FeedbackSpeed2 = {0:}[r/min]".format(motor_spd2))  # 取得した速度の表示[r/min]

    def state_callback(self, res):
        """ステータスコールバック関数

        購読したレスポンスデータをグローバル変数に反映する

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
            self.stop()  # 運転停止
            print("END")  # 終了表示
            self.seq = 4
        else:
            return

    def wait(self):
        """処理待ちサービス関数

        規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

        """
        time.sleep(0.03)
        # ドライバの通信が終了するまでループ
        while _state_driver == 1:
            pass

    def init(self):
        """初期化関数

        処理内容1:運転入力方式を3ワイヤ方式に変更
        処理内容2:運転データNo.2の回転速度の初期化(0[r/min])
        処理内容3:Configrationの実行

        """
        global msg
        # 運転入力方式の変更(3ワイヤ)
        msg.slave_id = 0x00  # 号機選択(Hex): 0(ブロードキャスト)
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 4160  # 先頭アドレス選択(Dec): 運転入力方式パラメータ
        msg.write_num = 1  # 書き込みデータサイズ: 1(32bit)
        msg.data[0] = 1  # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # 運転データ 回展速度No.2を0[r/min]に初期化
        msg.slave_id = 0x00  # 号機選択(Hex): 0(ブロードキャスト)
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156  # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 0  # 書き込みデータ: 0[r/min]
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # Configrationの実行
        msg.slave_id = 0x00  # 号機選択(Hex): 0(ブロードキャスト)
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 1  # 書き込みデータ: 1(実行)
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def set_data(self):
        global msg
        # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 26  # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # 運転指令(REV方向)(M1,START/STOP,RUN/BRAKE,FWD/REVをON)
        msg.slave_id = 0x02  # 号機選択(Hex): 2号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 58  # 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

    def start(self):
        global msg
        for i in range(5):  # LOOP処理(5回)
            # 回転速度の設定
            msg.slave_id = 0x01  # 号機選択(Hex): 1号機
            msg.func_code = 1  # ファンクションコード選択: 1(Write)
            msg.write_addr = 1156  # 先頭アドレス選択(Dec): データNo.2 回転速度
            msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
            msg.data[0] = 500 + i * 200  # 書き込みデータ: 500/700/900/1100/1300[r/min]
            self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()  # 処理待ち
            # 回転速度の設定
            msg.slave_id = 0x02  # 号機選択(Hex): 2号機
            msg.func_code = 1  # ファンクションコード選択: 1(Write)
            msg.write_addr = 1156  # 先頭アドレス選択(Dec): データNo.2 回転速度
            msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
            msg.data[0] = 500 + i * 200  # 書き込みデータ: 500/700/900/1100/1300[r/min]
            self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()  # 処理待ち

            time.sleep(2)

            # 回転速度読み込み
            msg.slave_id = 0x01  # 号機選択(Hex): 1号機
            msg.func_code = 0  # ファンクションコード選択: 0(Read)
            msg.read_addr = 206  # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
            msg.read_num = 1  # 読み込みデータサイズ: 1 (32bit)
            self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()
            msg.slave_id = 0x02  # 号機選択(Hex): 2号機
            msg.func_code = 0  # ファンクションコード選択: 0(Read)
            msg.read_addr = 206  # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
            msg.read_num = 1  # 読み込みデータサイズ: 1 (32bit)
            self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
            self.wait()

    def stop(self):
        global msg
        # 減速停止
        msg.slave_id = 0x01  # 号機選択(Hex): 1号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 18  # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち

        # 即時停止
        msg.slave_id = 0x02  # 号機選択(Hex): 2号機
        msg.func_code = 1  # ファンクションコード選択: 1(Write)
        msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
        msg.write_num = 1  # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = 10  # 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
        self.pub.publish(msg)  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        self.wait()  # 処理待ち


def main(args=None):
    """メイン関数

    処理内容1:運転指令(軸1 FWD方向、軸2 REV方向)
    処理内容2:運転データNo.2の回転速度の変更
    処理内容3:検出速度の確認
    処理内容4:停止指令(軸1 減速停止、軸2 瞬時停止)

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
