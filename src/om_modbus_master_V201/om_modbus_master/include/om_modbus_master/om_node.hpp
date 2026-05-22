#ifndef OM_NODE_H
#define OM_NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using std::string;

namespace om_modbusRTU_node {

class RosMessage;
class Base;
class ICheckData;
class IConvertQueryAndResponse;
class ICheckIdShareMode;
class IServerParamCommunicator;

const static int MAX_SLAVE_NUM = 31;
const static int MIN_SLAVE_ID = 1;
const static int MAX_SLAVE_ID = 31;

// ID Shareモード関係のパラメータの仕様値
const static int UNUSE_GLOBAL_ID = -1;  // ID Shareモードを使用しないときの値
const static int MIN_GLOBAL_ID = 1;     // MEXE02のShare Control Global IDと同義
const static int MAX_GLOBAL_ID = 127;
const static int MIN_AXIS_NUM = 1;   // 軸数
const static int MAX_AXIS_NUM = 31;  // 31はModbusで設定できる軸数の最大値

std::shared_ptr<ICheckData> check_obj[MAX_SLAVE_ID + 1];
std::unique_ptr<IConvertQueryAndResponse> convert_obj[MAX_SLAVE_ID + 1];

void init(std::vector<int> &first, std::vector<int> &second, Base *base_obj, ICheckIdShareMode *check_idshare_obj,
          RosMessage *rosmes_obj, IServerParamCommunicator *idshare_obj);
void deleteSpace(string &str);
std::vector<int> split(string &src, char delim);
void chkComma(const string &str);
void chkIdShareParameter(int global_id, int axis_num);

}  // namespace om_modbusRTU_node

#endif
