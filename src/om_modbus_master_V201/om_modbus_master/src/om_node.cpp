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
* @file	om_node.cpp
* @brief オブジェクト生成
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.2.00 Oct.18.2022 T.Takahashi
    - ROS2対応

* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - ID Share対応

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
                         - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_node.hpp"

#include "om_modbus_master/om_base.hpp"
#include "om_modbus_master/om_broadcast.hpp"
#include "om_modbus_master/om_first_gen.hpp"
#include "om_modbus_master/om_ros_message.hpp"
#include "om_modbus_master/om_second_gen.hpp"
#include "om_modbus_master/om_util.hpp"

#include "rclcpp/rclcpp.hpp"
#include "thread"

namespace ns = om_modbusRTU_node;

namespace om_modbusRTU_node {
rclcpp::Node::SharedPtr node_ = nullptr;
/*---------------------------------------------------------------------------*/
/** 初期化関数
* 		    オブジェクトの生成
* @param[in]  vector<int>& first
* @param[in]  vector<int>& second
* @param[in]  Base *base_obj
* @param[in]  ICheckIdShareMode *check_idshare_obj
* @param[in]  RosMessage *rosmes_obj
* @return     なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
//第一世代や第二世代のクエリを生成する
void init(std::vector<int> &first, std::vector<int> &second, Base *base_obj, ICheckIdShareMode *check_idshare_obj,
          RosMessage *rosmes_obj, IServerParamCommunicator *idshare_obj) {
  //入力変数の妥当性チェック
  if (0 == first.size() && 0 == second.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Error: No slave ID specified");
    throw;
  }
  if (MAX_SLAVE_NUM < (first.size() + second.size())) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Maximum number of connectable devices exceeded");
    throw;
  }

  cout << "" << endl;
  cout << "Connectable devices" << endl;
  check_obj[0] = std::make_unique<ns::BroadcastModbusRTU>();
  convert_obj[0] = std::make_unique<ns::BroadcastModbusRTU>();

  //第一世代のクエリを作成
  cout << " * /First Generation Number: " << first.size() << endl;
  for (unsigned int i = 0; i < first.size(); i++) {
    if (MIN_SLAVE_ID > first[i] || MAX_SLAVE_ID < first[i]) {
      RCLCPP_ERROR(node_->get_logger(), "Error: Specified slave ID out of range");
      throw;
    }
    cout << " * /First Generation(ID):" << first[i] << endl;
    check_obj[first[i]] = std::make_unique<ns::FirstGenModbusRTU>();
    convert_obj[first[i]] = std::make_unique<ns::FirstGenModbusRTU>();

    ICheckData *pobj = check_obj[first[i]].get();
    check_obj[0]->setMaxAddress(pobj->getMaxAddress());
    check_obj[0]->setMaxDataNum(pobj->getMaxDataNum());
  }

  //第二世代のクエリを作成（第一世代を継承している）
  cout << " * /Second Generation Number: " << second.size() << endl;
  for (unsigned int i = 0; i < second.size(); i++) {
    if (MIN_SLAVE_ID > second[i] || MAX_SLAVE_ID < second[i]) {
      RCLCPP_ERROR(node_->get_logger(), "Error: Specified slave ID out of range");
      throw;
    }
    cout << " * /Second Generation(ID):" << second[i] << endl;
    check_obj[second[i]] = std::make_unique<ns::SecondGenModbusRTU>(node_);
    convert_obj[second[i]] = std::make_unique<ns::SecondGenModbusRTU>(node_);

    ICheckData *pobj = check_obj[second[i]].get();
    check_obj[0]->setMaxAddress(pobj->getMaxAddress());
    check_obj[0]->setMaxDataNum(pobj->getMaxDataNum());
  }

  rosmes_obj->init(base_obj, check_obj, check_idshare_obj, node_);  // subscriberやpublisherの登録
  base_obj->init(rosmes_obj, check_obj, std::move(convert_obj), check_idshare_obj, idshare_obj, node_);  // ポート初期化
}

/*---------------------------------------------------------------------------*/
/** 文字列→数値変換
 文字列を数値に変換する
* @param[in]　　　string& src 変換文字列
* @param[in]　　　char delim  区切り文字
* @return　　　　　vector<int>
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::vector<int> split(string &src, char delim = ',') {
  std::vector<int> vec;
  std::istringstream iss(src);
  string tmp;

  if ("\0" == src) {
    return vec;
  }

  chkComma(src);

  while (getline(iss, tmp, delim)) {
    vec.push_back(stoi(tmp));
  }
  return vec;
}

/*---------------------------------------------------------------------------*/
/** 重複チェック

* @param[in]　　　第１世代(配列)
* @param[in]　　　第2世代(配列)
* @return　　　　  void
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkDuplication(std::vector<int> &first, std::vector<int> &second) {
  int size;
  std::vector<int> inter;

  std::sort(first.begin(), first.end());
  std::sort(second.begin(), second.end());

  size = first.size();
  for (int i = 0; i < (size - 1); i++) {
    if (first[i] == first[i + 1]) {
      RCLCPP_ERROR(node_->get_logger(), "Error: Duplicate firstGen");
      throw;
    }
  }

  size = second.size();
  for (int i = 0; i < (size - 1); i++) {
    if (second[i] == second[i + 1]) {
      RCLCPP_ERROR(node_->get_logger(), "Error: Duplicate secondGen");
      throw;
    }
  }

  std::set_intersection(first.begin(), first.end(), second.begin(), second.end(), std::back_inserter(inter));
  if (0 < inter.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Duplicate firstGen and secondGen");
    throw;
  }
}

/*---------------------------------------------------------------------------*/
/** 文字列からスペースを削除

* @param[in]　第１世代(文字列)
* @param[in]　第2世代(文字列)
* @return　　　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void deleteSpace(string &first, string &second) {
  first.erase(remove(first.begin(), first.end(), ' '), first.end());
  first.erase(remove(first.begin(), first.end(), '\t'), first.end());

  second.erase(remove(second.begin(), second.end(), ' '), second.end());
  second.erase(remove(second.begin(), second.end(), '\t'), second.end());
}

/*---------------------------------------------------------------------------*/
/** カンマ有無チェック

* @param[in]　チェックする文字列
* @return　　　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkComma(const string &str) {
  if (string::npos == str.find(',')) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Comma does not exist");
    throw;
  }
}

/*---------------------------------------------------------------------------*/
/**
* ID Share対応
* launchで設定したID Shareモード関係のパラメータが仕様内か調べる。例外を返す。
* @param[in]  int global_id
* @param[in]  int axis_num
* @return     なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void chkIdShareParameter(int global_id, int axis_num) {
  // global_idが仕様外かつ-1でないときに例外を返す。
  if (global_id < MIN_GLOBAL_ID || global_id > MAX_GLOBAL_ID) {
    if (global_id != UNUSE_GLOBAL_ID) {
      RCLCPP_ERROR(node_->get_logger(), "Error: Invalid number of globalId");
      throw;
    }
  }
  if (axis_num < MIN_AXIS_NUM || axis_num > MAX_AXIS_NUM) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Invalid number of axisNum");
    throw;
  }
}

}  // namespace om_modbusRTU_node

//querycallbackが実行されなかったときにtcd_dataを0にする処理
void timemoniter_thread()
{
    auto logger = rclcpp::get_logger("om_modbusRTU_node");  
    RCLCPP_INFO(logger,"This is a log message from timemoniter_thread");
}

/*---------------------------------------------------------------------------*/
/** main関数

* @param[in]  int argc
* @param[in]  char **argv
* @return　　　int
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int main(int argc, char **argv) {
  std::vector<int> first_gen, second_gen;

  rclcpp::init(argc, argv);
  ns::node_ = rclcpp::Node::make_shared("om_node");

  ns::node_->declare_parameter(ns::FIRST_GEN, "");
  ns::node_->declare_parameter(ns::SECOND_GEN, "");
  ns::node_->declare_parameter(ns::INIT_COM, "");
  ns::node_->declare_parameter(ns::INIT_BAUDRATE, 0);
  ns::node_->declare_parameter(ns::INIT_TOPIC_ID, 0);
  ns::node_->declare_parameter(ns::INIT_UPDATE_RATE, 0);
  ns::node_->declare_parameter(ns::GLOBAL_ID, 0);
  ns::node_->declare_parameter(ns::AXIS_NUMBER, 0);

  auto first_str = ns::node_->get_parameter(ns::FIRST_GEN).as_string();
  auto second_str = ns::node_->get_parameter(ns::SECOND_GEN).as_string();
  ns::deleteSpace(first_str, second_str);

  // ID Share関係のパラメータ取得
  auto global_id = ns::node_->get_parameter(ns::GLOBAL_ID).as_int();
  auto axis_num = ns::node_->get_parameter(ns::AXIS_NUMBER).as_int();

  std::thread th_time(timemoniter_thread);

  try {
    first_gen = ns::split(first_str);
    second_gen = ns::split(second_str);

    ns::chkDuplication(first_gen, second_gen);
    ns::chkIdShareParameter(global_id, axis_num);

    // RosMessageクラス(上位と配信/購読する)のインスタンス作成
    std::unique_ptr<ns::RosMessage> ros_mes = std::make_unique<ns::RosMessage>();
    std::unique_ptr<ns::Base> base = std::make_unique<ns::Base>();  //シリアル通信するクラスのインスタンス生成
    std::unique_ptr<ns::ICheckIdShareMode> pcheck_idshare_mode = std::make_unique<ns::SecondGenModbusRTU>(ns::node_);
    std::unique_ptr<ns::IServerParamCommunicator> idshare_obj = std::make_unique<ns::IdShareMode>(ns::node_);

    ns::init(first_gen, second_gen, base.get(), pcheck_idshare_mode.get(), ros_mes.get(), idshare_obj.get());
    rclcpp::spin(ns::node_);
    RCLCPP_INFO(ns::node_->get_logger(), "cppnode fin.");
    th_time.join();
    rclcpp::shutdown();
  } catch (int err) {
    std::cout << "Error:" << err << std::endl;
  }

  return 0;
}
