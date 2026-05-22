/**
* @file om_util.hpp
* @brief
* @author
* @date

* @details
* @note
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
*/

#ifndef OM_UTIL_H
#define OM_UTIL_H

#include <string>
using std::string;

/**
@namespace om_modbusRTU_node
*/
namespace om_modbusRTU_node {

const static string FIRST_GEN = "first_gen";
const static string SECOND_GEN = "second_gen";
const static string INIT_COM = "init_com";
const static string INIT_BAUDRATE = "init_baudrate";
const static string INIT_TOPIC_ID = "init_topicID";
const static string INIT_UPDATE_RATE = "init_update_rate";
const static string GLOBAL_ID = "global_id";
const static string AXIS_NUMBER = "axis_num";

}  // namespace om_modbusRTU_node

#endif
