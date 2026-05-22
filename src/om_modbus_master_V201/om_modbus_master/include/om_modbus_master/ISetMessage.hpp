#ifndef ISET_MESSAGE_H
#define ISET_MESSAGE_H

#include "om_msgs/msg/query.hpp"

namespace om_modbusRTU_node {
struct QUERY_DATA_T;

/*---------------------------------------------------------------------------*/
/**
@brief メッセージ設定インターフェース
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
class ISetMessage {
 public:
  virtual void setData(const om_msgs::msg::Query::SharedPtr msg) = 0;
  virtual void setQueryData(int index, int value) = 0;
  virtual ~ISetMessage(){};
};

}  // namespace om_modbusRTU_node

#endif
