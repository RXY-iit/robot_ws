// #include "pub_dynamixel_data_node.hpp"
// #include "motor_param.hpp"
// /******************************************************************************/
// /* Constructor                                                                */
// /******************************************************************************/
// JointPubNode::JointPubNode()
// : Node("pub_dynamixel_data_node"), current_position_index_(0)
// {
//   publisher_ = create_publisher<SetPosition>("/set_position", 10);

//   // モータ3つのポジションデータを初期化
//   // positions_1_ = {800, home_pos[0] }; // モータ1
//   // positions_2_ = {600, home_pos[1] }; // モータ2
//   // positions_3_ = {2800, home_pos[2] }; // モータ3
//   positions_1_ = {home_pos[0] }; // モータ1
//   positions_2_ = {home_pos[1] }; // モータ2
//   positions_3_ = {home_pos[2] }; // モータ3
//   timer_ = create_wall_timer(
//     std::chrono::milliseconds(500),
//     std::bind(&JointPubNode::publishData, this)
//   );
// }

// /******************************************************************************/
// /* Function                                                                   */
// /******************************************************************************/
// void JointPubNode::publishData()
// {
//   // モータ1のデータを送信
//   SetPosition msg1;
//   msg1.id = 11;    
//   msg1.position = positions_1_[current_position_index_];
//   publisher_->publish(msg1);
//   RCLCPP_INFO(get_logger(), "Publishing ID: %d Position: %d", msg1.id, msg1.position);

//   // モータ2のデータを送信
//   SetPosition msg2;
//   msg2.id = 12;    
//   msg2.position = positions_2_[current_position_index_];
//   publisher_->publish(msg2);
//   RCLCPP_INFO(get_logger(), "Publishing ID: %d Position: %d", msg2.id, msg2.position);

//   // モータ3のデータを送信
//   SetPosition msg3;
//   msg3.id = 13;    
//   msg3.position = positions_3_[current_position_index_];
//   publisher_->publish(msg3);
//   RCLCPP_INFO(get_logger(), "Publishing ID: %d Position: %d", msg3.id, msg3.position);

//   // 次のインデックスへ
//   current_position_index_ = (current_position_index_ + 1) % positions_1_.size();
// }

// /******************************************************************************/
// /* main                                                                       */
// /******************************************************************************/
// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<JointPubNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
#include "pub_dynamixel_data_node.hpp"
#include "motor_param.hpp"

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
JointPubNode::JointPubNode()
: Node("pub_dynamixel_data_node"), current_position_index_(0)
{
  publisher_ = create_publisher<SetPosition>("/set_position", 10);

  // モータ3つのポジションデータを初期化
  positions_1_ = {home_pos[0]}; // モータ1
  positions_2_ = {home_pos[1]}; // モータ2
  positions_3_ = {home_pos[2]}; // モータ3

  timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&JointPubNode::publishData, this)
  );
}

/******************************************************************************/
/* Function                                                                   */
/******************************************************************************/
void JointPubNode::publishData()
{
  SetPosition msg;

  // 3つのモーターのデータを1つのメッセージにまとめる
  msg.id1 = 11;
  msg.position1 = positions_1_[current_position_index_];
  msg.id2 = 12;
  msg.position2 = positions_2_[current_position_index_];
  msg.id3 = 13;
  msg.position3 = positions_3_[current_position_index_];

  publisher_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "Publishing - ID1: %d Position1: %d, ID2: %d Position2: %d, ID3: %d Position3: %d",
    msg.id1, msg.position1, msg.id2, msg.position2, msg.id3, msg.position3
  );

  // 次のインデックスへ
  current_position_index_ = (current_position_index_ + 1) % positions_1_.size();
}

/******************************************************************************/
/* main                                                                       */
/******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
