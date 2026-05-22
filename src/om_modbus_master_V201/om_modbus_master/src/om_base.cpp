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
* @file	om_base.cpp
* @brief シリアル通信の処理を行う
* @details
* @attention
* @note

履歴 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
* @version  Ver.2.00 Oct.18.2022 T.Takahashi
    - ROS2対応

* @version  Ver.1.05 April.5.2022 K.Yamaguchi
    - ID Shareモード対応

* @version	Ver.1.04 Sep.30.2020 T.Takahashi
    - transDelay関数に時間がラウンドしたときの処理を追加
    - ファンクションコードもレスポンス内容に含めるように処理を追加

* @version	Ver.1.01 Jul.22.2019 T.Takahashi
    - bugfix

* @version	Ver.1.00 Mar.11.2019 T.Takahashi
        - 新規作成
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include "om_modbus_master/om_base.hpp"

#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <iterator>
#include <linux/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <typeinfo>
#include <unistd.h>
#include <vector>
#include <atomic>

#include "om_modbus_master/om_util.hpp"

using std::string;
using std::thread;
using namespace std::chrono_literals;
/**
 * @namespace om_modbusRTU_node
 */
namespace om_modbusRTU_node {
/*---------------------------------------------------------------------------*/
/** コンストラクタ
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
Base::Base() {
  is_enabled_ = false;
  ros_mes_ = nullptr;
}

/*---------------------------------------------------------------------------*/
/** デストラクタ
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
Base::~Base() { closeComm(); }

/*---------------------------------------------------------------------------*/
/** レスポンス長のエラーチェック
* レスポンス長から無応答、例外応答の確認を行う
* @param[in]	ドライバからのレスポンス
* @param[in]	レスポンス長
* @return
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::chkResponseLength(char *pFrm, int len) {
  ISetResponse *pMesObj = ros_mes_;
  if (0 == len) {
    setState(STATE_ERROR, STATE_ERROR_NORESPONSE);
    throw NO_RESPONSE;
  } else if (EXCEPTION_RESPONSE_NUM == len) {
    setState(STATE_ERROR, STATE_ERROR_EXCEPTION_RESPONSE);
    for (int i = 0; i < EXCEPTION_RESPONSE_NUM; i++) {
      response_msg_.data[i] = pFrm[i];
    }
    response_msg_.slave_id = pFrm[RESPONSE_SLAVE_ID];
    response_msg_.func_code = pFrm[RESPONSE_FUNCTION_CODE];
    pMesObj->setResponse(response_msg_);
    throw EXCEPTION_RESPONSE_ERROR;
  } else if (8 == len) {
    setState(STATE_ERROR, STATE_ERROR_NONE);
  } else {
    setState(STATE_ERROR, STATE_ERROR_NONE);
  }
}

/*---------------------------------------------------------------------------*/
/**  COMポートクローズ
* @param[in]	なし
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::closeComm(void) {
  close(socket_fd_);
  RCLCPP_INFO(node_->get_logger(), "----------- close: ------------- ");
}

/*---------------------------------------------------------------------------*/
/** 読み込み
* @param[in]	int fd			ファイルディスクリプタ
* @param[out]	char *rdData	読み込みデータ
* @param[in]	int rdLen		データ長
* @return		読み込まれたバイト数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::commRead(int fd, char *rdData, int rdLen) {
  int ret = 0;
  int available_size;

  ioctl(fd, FIONREAD, &available_size);

  if (available_size != 0) {
    ret = read(fd, rdData, rdLen);
  }
  if (0 > ret) {
    throw READ_ERROR;
  }

  return ret;
}

/*---------------------------------------------------------------------------*/
/** 書き込み
* @param[in]	int fd		ファイルディスクリプタ
* @param[out]	vector<char>& wrData	書き込みデータ
* @return		int 書き込まれたバイト数
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::commWrite(int fd, std::vector<char> &wrData) {
  int ret = 0;

  ret = write(fd, wrData.data(), wrData.size());

  if (0 > ret) {
    throw WRITE_ERROR;
  }

  return ret;
}

/*---------------------------------------------------------------------------*/
/** エラーメッセージ表示
* @param[in]	int error
* @return		なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::dispErrorMessage(int error) {
  switch (error) {
    case EXCEPTION_RESPONSE_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Exception response");
      break;
    case NO_RESPONSE:
      RCLCPP_ERROR(node_->get_logger(), "Error: No response");
      break;
    case WRITE_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: write() function");
      break;
    case READ_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: read() function");
      break;
    case TIMEOUT_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Time out");
      break;
    case SELECT_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: select() function");
      break;
    case CRC_ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error: Error checks don't match");
      break;
    default:
      break;
  }
}

/*---------------------------------------------------------------------------*/
/**
通信実行
上位から配信されたときに実行。別スレッドで処理をループさせている。
* @param[in]	なし
* @return		なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::exeComm(void) {
  ISetResponse *pObj = ros_mes_;
  rclcpp::WallRate loop_rate(1ms);
  while (1) {
    if (is_enabled_) {
      for (int i = 0; i < MAX_DATA_NUM; i++) {
        response_msg_.data[i] = 0;
      }
      switch (query_data_.func_code) {
        case FUNCTION_CODE_READ:
          transRead();
          break;
        case FUNCTION_CODE_WRITE:
          transWrite();
          break;
        case FUNCTION_CODE_READ_WRITE:
          transReadAndWrite();
          break;
        default:
          break;
      }
      setState(STATE_DRIVER, STATE_DRIVER_NONE);
      setState(STATE_MES, STATE_MES_NONE);
      is_enabled_ = false;
      pObj->setCommEnabled(false);
    }
    loop_rate.sleep();
  }
}

/*---------------------------------------------------------------------------*/
/**
ID Share対応
レスポンスのスレーブIDがinit_globalIDと同一か調べる
* @param[in]	int global_id
* @return		bool
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool Base::chkGlobalID(int global_id) { return query_data_.slave_id == global_id; }

/*---------------------------------------------------------------------------*/
/**
ID Share対応
ID Shareモードか判定する
* @param[in]
* @return		bool
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool Base::isIdShareMode(void) {
  auto global_id = node_->get_parameter(GLOBAL_ID).as_int();

  if (global_id >= MIN_GLOBAL_ID && global_id <= MAX_GLOBAL_ID) {
    // 設定されたglobal_idが仕様値内のとき
    return true;
  }

  return false;
}

/*---------------------------------------------------------------------------*/
/**
ID Share対応
使用するslave_idのvectorを返す
* @param[in]	string &str: 取得するパラメータ名称
* @return		　std::vector<int> 使用するslave_id一覧
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::vector<int> Base::getSlaveId(const string &str) {
  std::string id_str = node_->get_parameter(str).as_string();
  std::vector<int> v_id = split(id_str, ',');
  return v_id;
}

/*---------------------------------------------------------------------------*/
/**
TrueのときにID Share Modeという文字列を出力する
* @param[in]	bool is_share_mode
* @return
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::informCommunicationMode(bool is_share_mode) {
  if (is_share_mode) {
    RCLCPP_INFO(node_->get_logger(), "ID SHARE MODE!");
  } else {
    RCLCPP_INFO(node_->get_logger(), "NORMAL MODE!");
  }
}

/*---------------------------------------------------------------------------*/
/** ボーレートを返す
* @param[in]	int baudrate ボーレート
* @return		int termios構造体で定義されているボーレート
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::getBaudRate(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      return -1;
  }
}

/*---------------------------------------------------------------------------*/
/** 現在時刻を返す
* @param		なし
* @return		long 時間[ns]
* @details
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
long Base::getCurrentTimeLinux(void) {
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);

  return tv.tv_nsec;
}

/*---------------------------------------------------------------------------*/
/** 初期化
* @param[in]	RosMessage *ros_mes_obj
* @param[in]	std::shared_ptr<ICheckData> check_obj[]
* @param[in]	std::unique_ptr<IConvertQueryAndResponse> convert_obj[]
* @param[in]	ICheckIdShareMode *check_idshare_obj,
* @param[in]	rclcpp::Node::SharedPtr om_node
* @param[in]  rclcpp::Node::SharedPtr om_node
* @return		なし
* @details
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::init(RosMessage *ros_mes_obj, std::shared_ptr<ICheckData> check_obj[],
                std::unique_ptr<IConvertQueryAndResponse> convert_obj[], ICheckIdShareMode *check_idshare_obj,
                IServerParamCommunicator *server_param_communicator, rclcpp::Node::SharedPtr om_node) {
  ros_mes_ = ros_mes_obj;
  check_data_ = check_obj;
  convert_query_response_ = std::move(convert_obj);
  check_idshare_mode_ = check_idshare_obj;
  server_param_communicator_ = server_param_communicator;
  node_ = om_node;

  topic_com_ = node_->get_parameter(INIT_COM).as_string();
  topic_baudrate_ = node_->get_parameter(INIT_BAUDRATE).as_int();

  if (false == openComm(getBaudRate(topic_baudrate_), topic_com_)) {
    throw;
  }

  startThread();
}

/*---------------------------------------------------------------------------*/
/** COMポートオープン
* @param[in]	int baudrate ボーレート
* @param[in]	string &port COMポート名
* @return		  true:成功  false:失敗
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
bool Base::openComm(int baudrate, const string &port) {
  socket_fd_ = open(port.c_str(), O_RDWR);
  if (0 > socket_fd_) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Can't opening serial port");
    return false;
  }
  if (-1 == baudrate) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Specified Baudrate out of range");
    return false;
  }

  struct serial_struct serial_setting;
  ioctl(socket_fd_, TIOCGSERIAL, &serial_setting);
  serial_setting.flags |= ASYNC_LOW_LATENCY;
  ioctl(socket_fd_, TIOCSSERIAL, &serial_setting);

  struct termios newtio;

  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD | PARENB;
  newtio.c_iflag = 0;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 5;
  newtio.c_cc[VMIN] = 0;

  cfsetispeed(&newtio, baudrate);
  cfsetospeed(&newtio, baudrate);

  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  RCLCPP_INFO(node_->get_logger(), "----------- connect: ------------- ");

  return true;
}

/*---------------------------------------------------------------------------*/
/** ステータス更新

* @param[in]	int type (0:state_mes 1:state_driver 2:state_error)
* @param[in]	int val 書き込み値
* @return		　なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::setState(int type, int val) {
  ISetResponse *pObj = ros_mes_;

  switch (type) {
    case STATE_MES:
      state_msg_.state_mes = val;
      break;
    case STATE_DRIVER:
      state_msg_.state_driver = val;
      break;
    case STATE_ERROR:
      state_msg_.state_error = val;
      break;
    default:
      break;
  }

  pObj->setState(state_msg_);
}

/*---------------------------------------------------------------------------*/
/** スレッド関数の開始

* @param[in]	なし
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::startThread(void) {
  thread th(&om_modbusRTU_node::Base::exeComm, this);
  th.detach();
}

/*---------------------------------------------------------------------------*/
/** 送信関数
* @param[in]		int fd		ファイルディスクリプタ
* @param[in]		vector<char>& pCmd	送信データ
* @param[out]		char *pRes	受信データ
* @param[out]		int resLen	受信データ長
* @return			  int 受信サイズ[Byte]
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::transComm(int fd, std::vector<char> &pCmd, char *pRes, int resLen) {
  return transCommEx(fd, pCmd, pRes, resLen);
}

/*---------------------------------------------------------------------------*/
/**
* 送信関数拡張
* @param[in]		int fd			ファイルディスクリプタ
* @param[in]		vector<char>& *pCmd		送信データ
* @param[out]		char *pRes		受信データ
* @param[in]		int resLen		受信データ長
* @param[in]		int delayNs[2]	送信遅延時間
* @return			  int 受信サイズ[Byte]
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
int Base::transCommEx(int fd, std::vector<char> &pCmd, char *pRes, int resLen) {
  int rcvCnt = 0;
  int rcvSize = 0;
  long diffNs = 0;
  long startTime = 0;
  long endTime = 0;
  unsigned long inf = 1000000000;

  commWrite(fd, pCmd);
  if (0 < resLen) {
    startTime = getCurrentTimeLinux();
    do {
      rcvSize = commRead(fd, pRes + rcvCnt, resLen);
      rcvCnt += rcvSize;
      endTime = getCurrentTimeLinux();
      diffNs = endTime - startTime;
      if (diffNs < 0) {
        diffNs = (inf - startTime) + endTime;
      }

      if (diffNs > TIMEOUT) {
        break;
      }
    } while ((resLen - 1) >= rcvCnt);
  }

  return rcvCnt;
}

/*---------------------------------------------------------------------------*/
/** 遅延処理

* @param[in]	遅延秒数[ns]
* @return			遅延秒数[ns]
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
long Base::transDelay(long delayNs) {
  long diffNs = 0;
  long startTime = 0;
  long endTime = 0;
  unsigned long inf = 1000000000;

  if (0 < delayNs) {
    startTime = getCurrentTimeLinux();
    while (diffNs < delayNs) {
      endTime = getCurrentTimeLinux();
      diffNs = endTime - startTime;
      if (diffNs < 0) {
        diffNs = (inf - startTime) + endTime;
      }
    }
  }
  return diffNs;
}

/*---------------------------------------------------------------------------*/
/**
* レスポンスデータ中の軸間エラーチェックを削除
* @param[in] std::array<char, 255> rxd_data レスポンスデータ
* @param[in] int axis_num ID Shareモードの軸数
* @return     std::array<char, 255> 軸間エラーチェックが削除されたレスポンスデータ
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::array<char, 255> Base::deleteAxisErc(std::array<char, 255> rxd_data, int axis_num) {
  //軸間エラーチェックを削除してresponse_msg_にいれる場合
  std::array<char, 255> del_error_check_rxd_data{0};
  int one_axis_read_num_byte = this->query_data_.read_num / axis_num * 4;  // 1軸分のデータのbyte

  std::vector<int> skip_index;
  int byte_index;
  for (int axis = axis_num; axis > 0; axis--) {
    byte_index = 3 + one_axis_read_num_byte * axis + (axis - 1) * 2;  //(axis-1)*2: 軸間チェック分 降順になっている
    skip_index.push_back(byte_index);  //降順になっているため 軸間エラーチェックは2byte/軸のため
    skip_index.push_back(byte_index + 1);
  }

  // rxd_data軸間エラーチェックを削除する
  int skip_count = 0;
  for (int i = 0; i < 255; i++) {
    auto result = std::find(skip_index.begin(), skip_index.end(), i);
    //現在のindexが軸間エラーチェックの要素番号のとき
    if (result != skip_index.end()) {
      skip_count++;
      continue;
    } else {
      del_error_check_rxd_data[i - skip_count] = rxd_data[i];
    }
  }

  return del_error_check_rxd_data;
}

/*---------------------------------------------------------------------------*/
/** 読み込み

* @param[in]  なし
* @return     なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transRead(void) {
  int len;
  int expected_response_size = 0;
  std::vector<std::vector<char>> txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};
  int axis_num;
  int global_id;
  std::vector<int> second_gen_ids;

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pconvert_obj = nullptr;
  ICheckData *pcheck_obj = nullptr;

  if (isIdShareMode()) {
    // ID Shareモードのとき
    // IdShareModeクラスにパラメータサーバのデータを渡す
    axis_num = node_->get_parameter(AXIS_NUMBER).as_int();
    global_id = node_->get_parameter(GLOBAL_ID).as_int();

    second_gen_ids = getSlaveId(SECOND_GEN);
    int head_of_id = second_gen_ids[0];
    // IDShareモードのときはquery_data_.slave_idを使えない。second_genの先頭のidを使用
    pconvert_obj = convert_query_response_[head_of_id].get();
    pcheck_obj = check_data_[head_of_id].get();

    //インターフェイス経由でパラメータを渡す
    server_param_communicator_->setAxisNum(axis_num);
    server_param_communicator_->setGlobalId(global_id);

    int axis_error_check_total = axis_num * 2;
    expected_response_size = 5 + query_data_.read_num * 4 + axis_error_check_total;
  } else {
    // Normalモードのとき
    pconvert_obj = convert_query_response_[(int)query_data_.slave_id].get();
    pcheck_obj = check_data_[(int)query_data_.slave_id].get();

    expected_response_size = 5 + query_data_.read_num * 4;
  }
  pconvert_obj->setRead(query_data_.slave_id, query_data_.read_addr, query_data_.read_num, txd_data);

  //ここでドライバと送受信
  len = transComm(socket_fd_, txd_data[0], rxd_data.data(), expected_response_size);
  try {
    //受信データをmsgに代入している
    chkResponseLength(rxd_data.data(), len);

    //レスポンスのエラーチェック
    pcheck_obj->chkCrc(CRC_INIT_VALUE, (unsigned char *)rxd_data.data(), len);

    //送受信後に得たレスポンスデータを要素ごとに分解し、代入する
    response_msg_.slave_id = rxd_data[RESPONSE_SLAVE_ID];
    response_msg_.func_code = rxd_data[RESPONSE_FUNCTION_CODE];

    for (int i = 0; i < query_data_.read_num; i++) {
      response_msg_.data[i] = pconvert_obj->convertResponse(rxd_data.data(), i);  // 4byteをintへ
    }

    //受信データで生成したmsgをpublishしている
    pMesObj->setResponse(response_msg_);
    // driver stateを更新する(割り込み関数で定期的にpublishされる)
    setState(STATE_MES, STATE_MES_NONE);
  } catch (int error) {
    dispErrorMessage(error);
    setState(STATE_MES, STATE_MES_ERROR);
  }
}

/*---------------------------------------------------------------------------*/
/**
* 読み込み&書き込み
* @param[in]
* @return
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transReadAndWrite(void) {
  int len;
  int expected_response_size = 0;
  int query_num;
  int resLen[MAX_QUERY_NUM] = {0};
  std::vector<std::vector<char>> txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};
  std::vector<int> second_gen_ids;  //スレーブIDのリスト
  int axis_num;
  int global_id;

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pconvert_obj = nullptr;
  ICheckData *pcheck_obj = nullptr;

  if (isIdShareMode()) {
    // ID Shareモードのとき
    // IdShareModeクラスにパラメータサーバのデータを渡す
    axis_num = node_->get_parameter(AXIS_NUMBER).as_int();
    global_id = node_->get_parameter(GLOBAL_ID).as_int();
    second_gen_ids = getSlaveId(SECOND_GEN);

    int head_of_id = second_gen_ids[0];
    pconvert_obj = convert_query_response_[head_of_id].get();
    pcheck_obj = check_data_[head_of_id].get();

    //インターフェイス経由でパラメータを渡す
    server_param_communicator_->setAxisNum(axis_num);
    server_param_communicator_->setGlobalId(global_id);

    int axis_error_check_total = axis_num * 2;
    expected_response_size = 5 + query_data_.read_num * 4 + axis_error_check_total;
  } else {
    // Normalモードのとき
    pconvert_obj = convert_query_response_[(int)query_data_.slave_id].get();
    pcheck_obj = check_data_[(int)query_data_.slave_id].get();

    expected_response_size = 5 + query_data_.read_num * 4;
  }

  query_num =
      pconvert_obj->setReadAndWrite(query_data_.slave_id, query_data_.read_addr, query_data_.write_addr,
                                    query_data_.read_num, query_data_.write_num, query_data_.data, txd_data, resLen);

  for (int i = 0; i < query_num; i++) {
    //送受信
    len = transComm(socket_fd_, txd_data[i], rxd_data.data(), expected_response_size);

    try {
      chkResponseLength(rxd_data.data(), len);

      //レスポンスのエラーチェック
      pcheck_obj->chkCrc(CRC_INIT_VALUE, (unsigned char *)rxd_data.data(), len);

      if (8 == len) {
      } else {
        //メンバ変数に受信データを要素ごとに代入
        response_msg_.slave_id = rxd_data[RESPONSE_SLAVE_ID];
        response_msg_.func_code = rxd_data[RESPONSE_FUNCTION_CODE];

        for (int j = 0; j < query_data_.read_num; j++) {
          response_msg_.data[j] = pconvert_obj->convertResponse(rxd_data.data(), j);  // 4byteをintへ
        }
        pMesObj->setResponse(response_msg_);  // response_msg_の配信
      }
    } catch (int error) {
      dispErrorMessage(error);
      setState(STATE_MES, STATE_MES_ERROR);
      return;
    }
  }
  // driver stateの更新
  setState(STATE_MES, STATE_MES_NONE);
}

/*---------------------------------------------------------------------------*/
/** 書き込み

* @param[in]	なし
* @return		  なし
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::transWrite(void) {
  int len;
  std::vector<std::vector<char>> txd_data;
  txd_data.resize(MAX_QUERY_NUM);
  std::array<char, 255> rxd_data{0};

  ISetResponse *pMesObj = ros_mes_;
  IConvertQueryAndResponse *pconvert_obj = nullptr;
  ICheckData *pcheck_obj = nullptr;

  if (isIdShareMode()) {
    // ID Shareモードのとき
    std::vector<int> second_gen_ids = getSlaveId(SECOND_GEN);

    int head_of_id = second_gen_ids[0];

    // IDShareモードのときはquery_data_.slave_idを使えない。second_genの先頭のidを使用
    pconvert_obj = convert_query_response_[head_of_id].get();
    pcheck_obj = check_data_[head_of_id].get();
  } else {
    // Normalモードのとき
    pconvert_obj = convert_query_response_[(int)query_data_.slave_id].get();
    pcheck_obj = check_data_[(int)query_data_.slave_id].get();
  }

  pconvert_obj->setWrite(query_data_.slave_id, query_data_.write_addr, query_data_.write_num, query_data_.data,
                         txd_data);

  if (0 == query_data_.slave_id) {
    //ブロードキャスト送信
    len = transComm(socket_fd_, txd_data[0], rxd_data.data(), 0);
    setState(STATE_ERROR, STATE_ERROR_NONE);
  } else {
    //ユニキャスト送信
    len = transComm(socket_fd_, txd_data[0], rxd_data.data(), 8);
    try {
      chkResponseLength(rxd_data.data(), len);

      //レスポンスのエラーチェック
      pcheck_obj->chkCrc(CRC_INIT_VALUE, (unsigned char *)rxd_data.data(), len);

      response_msg_.slave_id = rxd_data[RESPONSE_SLAVE_ID];
      response_msg_.func_code = rxd_data[RESPONSE_FUNCTION_CODE];
      for (int i = 0; i < len; i++) {
        response_msg_.data[i] = rxd_data[i];
      }
      pMesObj->setResponse(response_msg_);
      setState(STATE_MES, STATE_MES_NONE);
    } catch (int error) {
      dispErrorMessage(error);
      setState(STATE_MES, STATE_MES_ERROR);
    }
  }
}

/*---------------------------------------------------------------------------*/
/** 購読したメッセージを保存(インターフェイス)

* @param[in]	const om_msgs::msg::Query::SharedPtr msg 購読メッセージ
* @return		　
* @details
* @attention
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::setData(const om_msgs::msg::Query::SharedPtr msg) {
  query_data_.slave_id = msg->slave_id;
  query_data_.func_code = msg->func_code;
  query_data_.write_addr = msg->write_addr;
  query_data_.read_addr = msg->read_addr;
  query_data_.write_num = msg->write_num;
  query_data_.read_num = msg->read_num;

  for (int i = 0; i < MAX_DATA_NUM; i++) {
    query_data_.data[i] = msg->data[i];
  }

  is_enabled_ = true;
}

void Base::setQueryData(int index, int value){//queryCallback
  query_data_.slave_id = 10;
  query_data_.func_code = 1;
  query_data_.write_addr = 0;
  query_data_.read_addr = 0;
  query_data_.write_num = 12;
  query_data_.read_num = 0;

  query_data_.data[0] = 0;
  query_data_.data[3] = 1000;
  query_data_.data[4] = 600;
  query_data_.data[5] = 1;
  query_data_.data[6] = 16;
  query_data_.data[9] = 1000;
  query_data_.data[10] = 600;
  query_data_.data[11] = 1;
  query_data_.data[12] = 45;
  query_data_.data[13] = 46;
  query_data_.data[14] = 47;
  query_data_.data[15] = 48;
  query_data_.data[16] = 49;
  query_data_.data[17] = 51;

  if(index >= 0 && index < MAX_DATA_NUM){
    // RCLCPP_ERROR(node_->get_logger(), "Error: motor input becomes zero");
    query_data_.data[index] = value;
  }
  is_enabled_ = true;
}

/*---------------------------------------------------------------------------*/
/**
* 文字列→数値変換
* 文字列を数値に変換する
* @param[in]　　　string& src 変換文字列
* @param[in]　　　char delim  区切り文字
* @return　　　　　vector<int>
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
std::vector<int> Base::split(string &src, char delim) {
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
/*
* カンマ有無チェック
* @param[in]　チェックする文字列
* @return　　　なし
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
void Base::chkComma(const string &str) {
  if (string::npos == str.find(',')) {
    RCLCPP_ERROR(node_->get_logger(), "Error: Comma does not exist");
    throw;
  }
}

}  // namespace om_modbusRTU_node
