/**
* @file om_base.h
* @brief
* @author
* @date

* @details
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/

#ifndef OM_BASE_H
#define OM_BASE_H

#include <string>

#include "om_modbus_master/ICheckData.hpp"
#include "om_modbus_master/ICheckIdShareMode.hpp"
#include "om_modbus_master/IConvertQueryAndResponse.hpp"
#include "om_modbus_master/IServerParamCommunicator.hpp"
#include "om_modbus_master/ISetMessage.hpp"
#include "om_modbus_master/om_first_gen.hpp"
#include "om_modbus_master/om_ros_message.hpp"
#include "om_msgs/msg/query.hpp"
#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"

using std::string;

/**
@namespace om_modbusRTU_node
*/
namespace om_modbusRTU_node {

class RosMessage;

/*---------------------------------------------------------------------------*/
/**
@brief ドライバと通信する機能を提供するクラス
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class Base : public ISetMessage {
 protected:
  RosMessage *ros_mes_;
  std::shared_ptr<ICheckData> *check_data_;
  std::unique_ptr<IConvertQueryAndResponse> *convert_query_response_;
  ICheckIdShareMode *check_idshare_mode_;
  IServerParamCommunicator *server_param_communicator_;

  om_msgs::msg::Response response_msg_;
  om_msgs::msg::State state_msg_;
  rclcpp::Node::SharedPtr node_;

  bool is_enabled_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_execution_time;//6/4MRI自作
  int socket_fd_;
  std::string topic_com_;
  int topic_baudrate_;
  fd_set fds_;
  struct timeval tv_;
  typedef struct {
    char slave_id = 0;
    char func_code = 0;
    int write_addr = 0;
    int read_addr = 0;
    short write_num = 0;
    short read_num = 0;
    int data[64] = {0};
  } QUERY_DATA_T;
  QUERY_DATA_T query_data_;

  typedef struct {
    char slave_id = 0;
    int data[64] = {0};
  } RESPONSE_DATA_T;
  RESPONSE_DATA_T response_data_;

  const static long TIMEOUT = 500 * 1000 * 1000;
  const static int EXCEPTION_RESPONSE_NUM = 5;

  const static int FUNCTION_CODE_READ = 0;
  const static int FUNCTION_CODE_WRITE = 1;
  const static int FUNCTION_CODE_READ_WRITE = 2;

  const static int MAX_DATA_NUM = 64;
  const static int MAX_QUERY_NUM = 10;

  const static int EXCEPTION_RESPONSE_ERROR = -1;
  const static int NO_RESPONSE = -2;
  const static int WRITE_ERROR = -3;
  const static int READ_ERROR = -4;
  const static int TIMEOUT_ERROR = -5;
  const static int SELECT_ERROR = -6;
  const static int CRC_ERROR = -7;

  const static int STATE_MES = 0;
  const static int STATE_DRIVER = 1;
  const static int STATE_ERROR = 2;

  const static int STATE_MES_NONE = 0;
  const static int STATE_MES_REACH = 1;
  const static int STATE_MES_ERROR = 2;

  const static int STATE_DRIVER_NONE = 0;
  const static int STATE_DRIVER_COMM = 1;

  const static int STATE_ERROR_NONE = 0;
  const static int STATE_ERROR_NORESPONSE = 1;
  const static int STATE_ERROR_EXCEPTION_RESPONSE = 2;

  const static int RESPONSE_SLAVE_ID = 0;
  const static int RESPONSE_FUNCTION_CODE = 1;

  // ID Share対応で追加
  const static int MIN_GLOBAL_ID = 1;  // MEXE02のShare Control Global IDと同義
  const static int MAX_GLOBAL_ID = 127;
  const static int CRC_INIT_VALUE = 0xFFFF;  // CRCの初期値

  void chkResponseLength(char *pFrm, int len);
  // void closeComm(void);
  int commRead(int fd, char *rdData, int rdLen);
  int commWrite(int fd, std::vector<char> &wrData);
  void dispErrorMessage(int error);
  void exeComm(void);
  int getBaudRate(int);
  long getCurrentTimeLinux();
  bool openComm(int, const std::string &port);
  void setState(int type, int val);
  void startThread(void);
  int transComm(int fd, std::vector<char> &pCmd, char *pRes, int resLen);
  int transCommEx(int fd, std::vector<char> &pCmd, char *pRes, int resLen);
  long transDelay(long delayNs);
  void transRead(void);
  void transReadAndWrite(void);
  void transWrite(void);

  // ID Share対応で追加
  bool chkGlobalID(int global_id);
  void informCommunicationMode(bool is_share_mode);
  std::array<char, 255> deleteAxisErc(std::array<char, 255> rxd_data, int axis_num);
  bool isIdShareMode(void);
  std::vector<int> getSlaveId(const string &str);
  std::vector<int> split(string &src, char delim = ',');
  void chkComma(const string &str);

 public:
  Base();
  ~Base();
  void init(RosMessage *ros_mes_obj, std::shared_ptr<ICheckData> check_obj[],
            std::unique_ptr<IConvertQueryAndResponse> convert_obj[], ICheckIdShareMode *check_idshare_obj,
            IServerParamCommunicator *server_param_communicator, rclcpp::Node::SharedPtr om_node);
  void setData(const om_msgs::msg::Query::SharedPtr msg) override;
  void setQueryData(int index, int value) override;
  void closeComm(void);
  // void setQueryData(int index, int value){//queryCallback
  //   if(index >= 0 && index < MAX_DATA_NUM){
  //     query_data_.data[index] = value;
  //   }
  // }
};

}  // namespace om_modbusRTU_node

#endif