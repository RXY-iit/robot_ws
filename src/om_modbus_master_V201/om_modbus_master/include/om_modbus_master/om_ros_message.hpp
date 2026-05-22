#ifndef OM_ROS_MESSAGE_H
#define OM_ROS_MESSAGE_H

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "om_modbus_master/ICheckData.hpp"
#include "om_modbus_master/ICheckIdShareMode.hpp"
#include "om_modbus_master/ISetResponse.hpp"
#include "om_modbus_master/om_base.hpp"
#include "om_msgs/msg/query.hpp"
#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"

using std::string;

namespace om_modbusRTU_node {
class Base;
class FirstGenModbusRTU;

/*---------------------------------------------------------------------------*/
/**
* @brief メッセージデータのクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class RosMessage : public ISetResponse {
 private:
  Base *base_obj_;
  std::shared_ptr<ICheckData> *check_data_;
  ICheckIdShareMode *check_idshare_mode_;
  rclcpp::Publisher<om_msgs::msg::Response>::SharedPtr response_pub_ = nullptr;
  rclcpp::Publisher<om_msgs::msg::State>::SharedPtr state_pub_ = nullptr;
  rclcpp::Subscription<om_msgs::msg::Query>::SharedPtr query_sub_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;
  om_msgs::msg::State state_msg_;
  bool isCommEnabled_;
  int topic_id_;
  int topic_update_rate_;
  rclcpp::Node::SharedPtr node_;

  //スレッド用の変数MRI
  std::thread monitoring_thread_;  
  bool stop_thread_;

  const static int BUF_SIZE = 1;
  const static int MIN_UPDATE_RATE = 0;
  const static int MAX_UPDATE_RATE = 1000;
  const static int MAX_TOPIC_ID = 15;
  const static int MIN_TOPIC_ID = 0;
  const static int MIN_SLAVE_ID = 0;
  const static int MAX_SLAVE_ID = 31;

  const static int FUNCTION_CODE_READ = 0;
  const static int FUNCTION_CODE_WRITE = 1;
  const static int FUNCTION_CODE_READ_WRITE = 2;

  const static int STATE_DRIVER_NONE = 0;
  const static int STATE_DRIVER_COMM = 1;
  const static int STATE_MES_REACH = 1;
  const static int STATE_MES_ERROR = 2;
  const static int STATE_ERROR_NONE = 0;

  const static int SLAVE_ID_ERROR = -1;
  const static int FUNCTION_CODE_ERROR = -2;
  const static int ADDRESS_ERROR = -3;
  const static int DATA_ERROR = -4;
  const static int GLOBAL_ID_ERROR = -5;
  const static int AXIS_NUM_ERROR = -6;
  const static int CRC_ERROR = -7;
  const static int MODBUS_MESSAGE_LENGTH_ERROR = -8;

  const static int MIN_GLOBAL_ID = 1;    // ID Share対応
  const static int MAX_GLOBAL_ID = 127;  // ID Share対応

  void chkAddress(int addr, int id);
  void chkDataNum(int num, int id);
  bool chkRange(int val, int min, int max);
  void chkRangeOfData(const om_msgs::msg::Query::SharedPtr msg);
  void chkReadSlaveID(int id);
  void chkWriteSlaveID(int id);
  void dispErrorMessage(int error);
  string makeTopicName(int num, string name);
  void timerCallback();
  void queryCallback(const om_msgs::msg::Query::SharedPtr msg);

  // ID Share対応で追加
  void chkRangeOfDataIdShare(const om_msgs::msg::Query::SharedPtr msg);
  bool chkIdShareMode(const om_msgs::msg::Query::SharedPtr msg);
  void flowCtrlIdShare(const om_msgs::msg::Query::SharedPtr msg);
  bool chkGlobalIdRange(int id);
  //監視スレッドの処理
  void monitoringFunction();
  std::chrono::time_point<std::chrono::high_resolution_clock> last_execution_time;

 public:
  RosMessage();
  ~RosMessage();
  void init(Base *pObj, std::shared_ptr<ICheckData> check_obj[], ICheckIdShareMode *check_idshare_obj,
            rclcpp::Node::SharedPtr om_node);
  void setResponse(const om_msgs::msg::Response res) override;
  void setState(const om_msgs::msg::State state) override;
  void setCommEnabled(bool val) override;
};

}  // namespace om_modbusRTU_node

#endif
