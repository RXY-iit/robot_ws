#ifndef ISET_RESPONSE_H
#define ISET_RESPONSE_H

#include "om_msgs/msg/response.hpp"
#include "om_msgs/msg/state.hpp"

namespace om_modbusRTU_node {

/*---------------------------------------------------------------------------*/
/**
@brief レスポンス設定インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ISetResponse {
 public:
  virtual void setResponse(const om_msgs::msg::Response res) = 0;
  virtual void setState(const om_msgs::msg::State state) = 0;
  virtual void setCommEnabled(bool val) = 0;
  virtual ~ISetResponse(){};
};

}  // namespace om_modbusRTU_node

#endif
