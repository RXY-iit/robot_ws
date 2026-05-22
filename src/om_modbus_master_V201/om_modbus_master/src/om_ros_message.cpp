/*
 * Copyright (c) 2019, ORIENTAL MOTOR CO.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the ORIENTAL MOTOR CO.,LTD. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
* @file	om_ros_message.cpp
* @brief 上位と配信、購読を行う
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.2.00 Oct.18.2022 T.Takahashi
    - ROS2対応

* @version  Ver.1.05 April.5.2022 K.Yamaguchi
       - ID Shareモード対応

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
                         - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

#include "om_modbus_master/om_ros_message.hpp"

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "om_modbus_master/om_idshare_mode.hpp"
#include "om_modbus_master/om_second_gen.hpp"
#include "om_modbus_master/om_util.hpp"
#include "om_msgs/msg/query.hpp"
#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"

using std::cout;
using std::endl;
using std::string;
using std::stringstream;
using namespace std::chrono_literals;

namespace om_modbusRTU_node {

RosMessage::RosMessage() : stop_thread_(false){ 
  isCommEnabled_ = false;
}

RosMessage::~RosMessage() { timer_.reset(); 
  stop_thread_ = true;
  if(monitoring_thread_.joinable()){//MRI6/13
    monitoring_thread_.join();
  }
}

/*---------------------------------------------------------------------------*/
/** アドレスの範囲確認

* @param[in]		int addr
* @param[in]		int id
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkAddress(int addr, int id) {
  ICheckData *pObj = nullptr;
  if (check_data_[id] != nullptr) {
    pObj = check_data_[id].get();
  } else {
    throw SLAVE_ID_ERROR;
  }

  if (false == pObj->chkAddress(addr)) {
    throw ADDRESS_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** 書き込み、読み込み数の範囲確認

* @param[in]		int num
* @param[in]		int id
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkDataNum(int num, int id) {
  ICheckData *pObj = nullptr;
  if (check_data_[id] != nullptr) {
    pObj = check_data_[id].get();
  } else {
    throw SLAVE_ID_ERROR;
  }

  if (false == pObj->chkDataNum(num)) {
    throw DATA_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** 数値の範囲確認

* @param[in]		入力値
* @param[in]		最小値
* @param[in]		最大値
* @return			  int 0:範囲内 1:範囲外
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool RosMessage::chkRange(int val, int min, int max) {
  bool ret = true;

  if (val > max || val < min) {
    ret = false;
  }

  return ret;
}

/*---------------------------------------------------------------------------*/
/** 数値の範囲確認

* @param[in]		const om_msgs::msg::Query::SharedPtr msg
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkRangeOfData(const om_msgs::msg::Query::SharedPtr msg) {
  switch (msg->func_code) {
    case FUNCTION_CODE_READ:
      chkReadSlaveID(msg->slave_id);
      chkAddress(msg->read_addr, msg->slave_id);
      chkDataNum(msg->read_num, msg->slave_id);
      break;
    case FUNCTION_CODE_WRITE:
      chkWriteSlaveID(msg->slave_id);
      chkAddress(msg->write_addr, msg->slave_id);
      chkDataNum(msg->write_num, msg->slave_id);
      break;
    case FUNCTION_CODE_READ_WRITE:
      chkReadSlaveID(msg->slave_id);
      chkAddress(msg->read_addr, msg->slave_id);
      chkAddress(msg->write_addr, msg->slave_id);
      chkDataNum(msg->read_num, msg->slave_id);
      chkDataNum(msg->write_num, msg->slave_id);
      break;
    default:
      throw FUNCTION_CODE_ERROR;
      break;
  }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応　
* 数値の範囲確認
* @param[in]		const om_msgs::msg::Query::SharedPtr msg メッセージ
* @return			  なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkRangeOfDataIdShare(const om_msgs::msg::Query::SharedPtr msg) {
  //パラメータサーバからID Shareで使う軸数データとglobal idをとってくる
  auto axis_num = node_->get_parameter(AXIS_NUMBER).as_int();

  ICheckIdShareMode *pObj = check_idshare_mode_;
  try {
    switch (msg->func_code) {
      case FUNCTION_CODE_READ:
        pObj->chkRangeGlobalId(msg->slave_id);                         // global_idの範囲チェック
        pObj->chkRangeAxis(axis_num);                                  // 軸数チェック
        pObj->chkRangeReadData(std::floor(msg->read_num / axis_num));  // 1軸あたりのデータ数チェック(max12個)
        pObj->chkRangeShareReadAddress(msg->read_addr);  // ID Shareの読み出しアドレスチェック
        // Readの生成クエリ数は8Byte固定のため、判定を行わない
        break;
      case FUNCTION_CODE_WRITE:
        pObj->chkRangeGlobalId(msg->slave_id);
        pObj->chkRangeAxis(axis_num);                                    // 軸数チェック
        pObj->chkRangeWriteData(std::floor(msg->write_num / axis_num));  // 1軸あたりのデータ数チェック(max12個)
        pObj->chkRangeShareWriteAddress(msg->write_addr);       // ID Shareの書き込みアドレスチェック
        pObj->chkDataNumWriteQueryIdShareMode(msg->write_num);  // クエリのByte数が256Byte以内かチェック
        break;
      case FUNCTION_CODE_READ_WRITE:
        pObj->chkRangeGlobalId(msg->slave_id);                         // global_idの範囲チェック
        pObj->chkRangeAxis(axis_num);                                  // 軸数チェック
        pObj->chkRangeReadData(std::floor(msg->read_num / axis_num));  // 1軸あたりのデータ数チェック(max12個)
        pObj->chkRangeWriteData(std::floor(msg->write_num / axis_num));  // 1軸あたりのデータ数チェック(max12個)
        pObj->chkRangeShareReadAddress(msg->read_addr);             // ID Shareの読み出しアドレスチェック
        pObj->chkRangeShareWriteAddress(msg->write_addr);           // ID Shareの書き込みアドレスチェック
        pObj->chkDataNumReadWriteQueryIdShareMode(msg->write_num);  // クエリのByte数が256Byte以内かチェック
        break;
      default:
        throw FUNCTION_CODE_ERROR;
        break;
    }
  } catch (int e) {
    throw e;
  }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* ID Shareモードか調べる
* @param[in]		const om_msgs::msg::Query::SharedPtr msg
* @return			  bool
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool RosMessage::chkIdShareMode(const om_msgs::msg::Query::SharedPtr msg) {
  //パラメータサーバのglobalidを持ってくる
  auto global_id = node_->get_parameter(GLOBAL_ID).as_int();

  //設定されているglobal_idが仕様外だったらfalseを返す
  if (!chkGlobalIdRange(global_id)) {
    return false;
  }

  //パラメータサーバのglobal_idとmsgのglobal_idが同じか調べる
  if (global_id != msg->slave_id) {
    return false;
  }

  return true;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* パラメータサーバのglobal_idが仕様内か調べる
* @param[in]		int id: パラメータサーバから取得したglobal_id
* @return			  true, false
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool RosMessage::chkGlobalIdRange(int id) {
  if (id < MIN_GLOBAL_ID || id > MAX_GLOBAL_ID) {
    return false;
  }
  return true;
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* ID Shareモードかどうかで処理の振り分け
* @param[in]		const om_msgs::msg::Query::SharedPtr msg
* @return			  なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::flowCtrlIdShare(const om_msgs::msg::Query::SharedPtr msg) {
  try {
    if (chkIdShareMode(msg)) {
      chkRangeOfDataIdShare(msg);
    } else {
      chkRangeOfData(msg);
    }
  } catch (int e) {
    throw e;
  }
}

/*---------------------------------------------------------------------------*/
/** 読み込み時のスレーブIDの範囲確認

* @param[in]		int スレーブid
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkReadSlaveID(int id) {
  if (false == chkRange(id, MIN_SLAVE_ID + 1, MAX_SLAVE_ID)) {
    throw SLAVE_ID_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** 書き込み時のスレーブIDの範囲確認

* @param[in]		int スレーブid
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::chkWriteSlaveID(int id) {
  if (false == chkRange(id, MIN_SLAVE_ID, MAX_SLAVE_ID)) {
    throw SLAVE_ID_ERROR;
  }
}

/*---------------------------------------------------------------------------*/
/** エラーメッセージ表示

* @param[in]		int error
* @return			  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::dispErrorMessage(int error) {
  switch (error) {
    case SLAVE_ID_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid slave ID");
      break;
    case FUNCTION_CODE_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid function code");
      break;
    case ADDRESS_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid address");
      break;
    case DATA_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid data num");
      break;
    case GLOBAL_ID_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error Invalid global ID");
      break;
    case AXIS_NUM_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid axis num");
      break;
    case MODBUS_MESSAGE_LENGTH_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: The message length exceeded 256 bytes");
      break;
    default:
      break;
  }
}

/*---------------------------------------------------------------------------*/
/** 初期化(publisherとsubscriberの登録など)

* @param[in]	Base *pObj
* @param[in]	std::shared_ptr<ICheckData> check_obj[]
* @param[in]	ICheckIdShareMode *check_idshare_obj
* @param[in]	rclcpp::Node::SharedPtr om_node
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::init(Base *pObj, std::shared_ptr<ICheckData> check_obj[], ICheckIdShareMode *check_idshare_obj,
                      rclcpp::Node::SharedPtr om_node) {
  string update_rate;
  node_ = om_node;
  base_obj_ = pObj;  //シリアル通信を行うためのオブジェクト
  check_data_ = check_obj;
  check_idshare_mode_ = check_idshare_obj;

  monitoring_thread_ = std::thread(&RosMessage::monitoringFunction,this);//MRI/6/13

  string topic_name;
  om_msgs::msg::State::SharedPtr comm_msg;

  topic_id_ = node_->get_parameter(INIT_TOPIC_ID).as_int();

  if (false == chkRange(topic_id_, MIN_TOPIC_ID, MAX_TOPIC_ID)) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Specified topic ID out of range");
    throw;
  }

  topic_update_rate_ = node_->get_parameter("init_update_rate").as_int();
  if (false == chkRange(topic_update_rate_, MIN_UPDATE_RATE, MAX_UPDATE_RATE)) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Specified updateRate out of range");
    throw;
  }

  topic_name = makeTopicName(topic_id_, "om_query");
  cout << "" << endl;
  cout << "Modbus RTU Node PARAMETERS" << endl;
  cout << " * /Subscriber: " << topic_name << endl;
  query_sub_ = node_->create_subscription<om_msgs::msg::Query>(
      topic_name, rclcpp::QoS(BUF_SIZE), std::bind(&RosMessage::queryCallback, this, std::placeholders::_1));

  topic_name = makeTopicName(topic_id_, "om_response");
  cout << " * /Publisher: " << topic_name << endl;
  response_pub_ = node_->create_publisher<om_msgs::msg::Response>(topic_name, rclcpp::QoS(BUF_SIZE));
  topic_name = makeTopicName(topic_id_, "om_state");
  cout << " * /Publisher: " << topic_name << endl;
  state_pub_ = node_->create_publisher<om_msgs::msg::State>(topic_name, rclcpp::QoS(BUF_SIZE));

  cout << " * /com: " << node_->get_parameter(INIT_COM).as_string() << endl;
  cout << " * /baudrate: " << node_->get_parameter(INIT_BAUDRATE).as_int() << endl;
  cout << " * /globalID: " << node_->get_parameter(GLOBAL_ID).as_int() << endl;
  cout << " * /axisNum: " << node_->get_parameter(AXIS_NUMBER).as_int() << endl;
  cout << " * /update_rate: " << topic_update_rate_ << endl;
  cout << "" << endl;

  if (0 != topic_update_rate_) {
    auto msec = std::chrono::milliseconds(1000 / topic_update_rate_);
    timer_ = node_->create_wall_timer(msec, std::bind(&RosMessage::timerCallback, this));
  }
}

/*---------------------------------------------------------------------------*/
/** queryCallback

* @param[in]	const om_msgs::msg::Query::SharedPtr msg
* @return
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::queryCallback(const om_msgs::msg::Query::SharedPtr msg) {
  //こいつがとまる
  // RCLCPP_INFO(node_->get_logger(), "queryCallback is executed");
  last_execution_time = std::chrono::high_resolution_clock::now();//実行時刻を保持MRI6/16
  if (false == isCommEnabled_) {
    // RCLCPP_INFO(node_->get_logger(), "Callback");
    isCommEnabled_ = true;
    state_msg_.state_driver = STATE_DRIVER_COMM;
    state_msg_.state_mes = STATE_MES_REACH;
    state_msg_.state_error = STATE_ERROR_NONE;
    ISetMessage *pObj = base_obj_;  // Base型

    try {
      // id share modeかどうかで別の範囲チェック判定を行う
      flowCtrlIdShare(msg);

      pObj->setData(msg);//こいつをtimemoniter関数で動かす
    } catch (int error) {
      dispErrorMessage(error);
      isCommEnabled_ = false;
      state_msg_.state_driver = STATE_DRIVER_NONE;
      state_msg_.state_mes = STATE_MES_ERROR;
    }
    // RCLCPP_INFO(node_->get_logger(), "check end\n");
  } else if (true == isCommEnabled_) {
    RCLCPP_INFO(node_->get_logger(), "Driver is busy");
  }
}

void RosMessage::monitoringFunction(){
  while (!stop_thread_)//メインが終了するまで実行するサブスレッドのフラグ6/16MRI
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_execution_time);
    if (duration.count() > 300)//もし0.5秒間実行されなかったら...
    {
      RCLCPP_INFO(node_->get_logger(),"Not callback execution");
      ISetMessage *pObj = base_obj_;  // Base型
      pObj->setQueryData(2,0);
      pObj->setQueryData(8,0);
      // RCLCPP_INFO(node_->get_logger(), "set");
      // base_obj_->setQueryData(2,0);
      // base_obj_->setQueryData(8,0);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));//10msごとにチェック
  }
}

/*-------------------------- -------------------------------------------------*/
/** トピック名の作成

* @param[in]	int num		数値
* @param[in]	string name	文字列
* @return		　string 文字列
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
string RosMessage::makeTopicName(int num, string name) {
  string ret = name;
  stringstream ss;
  ss << num;
  ret += ss.str();
  return ret;
}

/*---------------------------------------------------------------------------*/
/** 定期周期で呼び出されるコールバック関数

* @param[in]	なし
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::timerCallback() { state_pub_->publish(state_msg_); }

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]	上位に配信するレスポンスデータ
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setResponse(const om_msgs::msg::Response res) { response_pub_->publish(res); }

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]	bool val
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setCommEnabled(bool val) { isCommEnabled_ = val; }

/*---------------------------------------------------------------------------*/
/** インターフェイス

* @param[in]	上位に配信するステータスデータ
* @return		　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void RosMessage::setState(const om_msgs::msg::State state) { state_msg_ = state; }

}  // namespace om_modbusRTU_node