#include "omni_base_driver/robot_odom_node.hpp"

PickingRobotOdom::PickingRobotOdom()
    : Node("robot_odom_node"),
    x(0.0), y(0.0), th(0.0),
    vx(0.0), vy(0.0), vth(0.0),
    current_time(this->get_clock()->now()),
    last_time(this->get_clock()->now()),
    frame_id("odom"),
    child_frame_id("base_footprint"),
    is_first_callback(true),
    has_steer_msg(false),
    has_drive_msg(false),
    size(3),
    KinemaMatrix()    
{
    steer_sub = this->create_subscription<my_messages::msg::SteerMotor>(
        "steer_odom", 10, std::bind(&PickingRobotOdom::steerCallback, this, std::placeholders::_1));
    drive_sub = this->create_subscription<my_messages::msg::DriveMotor>(
        "drive_odom", 10, std::bind(&PickingRobotOdom::driveCallback, this, std::placeholders::_1));

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    timer = this->create_wall_timer(std::chrono::milliseconds(50), 
                                    std::bind(&PickingRobotOdom::timerCallback, this));

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "PickingRobotOdom node initialized");
}

PickingRobotOdom::~PickingRobotOdom()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down PickingRobotOdom");
}

void PickingRobotOdom::steerCallback(const my_messages::msg::SteerMotor::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex);
    latest_steer_msg = *msg;
    has_steer_msg = true;
    RCLCPP_DEBUG(this->get_logger(), "steer_odom received: phi=[%.4f, %.4f, %.4f]", 
                 msg->phi1, msg->phi2, msg->phi3);
}

void PickingRobotOdom::driveCallback(const my_messages::msg::DriveMotor::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(msg_mutex);
    latest_drive_msg = *msg;
    has_drive_msg = true;
    RCLCPP_DEBUG(this->get_logger(), "drive_odom received: vel=[%.4f, %.4f, %.4f]", 
                 msg->vel1, msg->vel2, msg->vel3);
}

void PickingRobotOdom::timerCallback()
{
    std::lock_guard<std::mutex> lock(msg_mutex);
    
    static int count = 0;
    if (count % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "Timer: has_steer=%d, has_drive=%d", has_steer_msg, has_drive_msg);
    }
    count++;
    
    if (!has_steer_msg || !has_drive_msg) {
        return;
    }

    std::array<float, 3> steer_phi = {latest_steer_msg.phi1, latest_steer_msg.phi2, latest_steer_msg.phi3};
    std::array<double, 3> drive_vel = {latest_drive_msg.vel1, latest_drive_msg.vel2, latest_drive_msg.vel3};

    RCLCPP_INFO(this->get_logger(), "Input: drive_vel=[%.4f, %.4f, %.4f], steer_phi=[%.4f, %.4f, %.4f]",
                drive_vel[0], drive_vel[1], drive_vel[2],
                steer_phi[0], steer_phi[1], steer_phi[2]);

    KinemaMatrix.calcRobotVelocity(vx, vy, vth, drive_vel.data(), steer_phi.data());

    // Match ROS base_link convention and the verified cmd_vel direction:
    // +X is the robot's real forward direction.  Motor feedback currently
    // reconstructs the X component with the opposite sign, while Y is correct.
    vx = -vx;
    
    RCLCPP_INFO(this->get_logger(), "Calculated: vx=%.4f, vy=%.4f, vth=%.4f", vx, vy, vth);
    
    calcOdometry(this->get_clock()->now());
}


void PickingRobotOdom::calcOdometry(const rclcpp::Time& current_time)
{
    double dt = (current_time - last_time).seconds();

    if (is_first_callback) {
        RCLCPP_INFO(this->get_logger(), "First callback, skipping integration. Setting last_time.");
        last_time = current_time;
        is_first_callback = false;
        return;
    }

    if (dt <= 0.0 || dt > 10.0) {
        RCLCPP_INFO(this->get_logger(), "Skipping calc: dt=%.6f (out of range)", dt);
        last_time = current_time;
        return;
    }

    double dx = (vx * cos(th) - vy * sin(th)) * dt;
    double dy = (vx * sin(th) + vy * cos(th)) * dt;
    double dth = vth * dt;

    RCLCPP_INFO(this->get_logger(), "Integration: dt=%.6f, delta=[dx=%.6f, dy=%.6f, dth=%.6f]", 
                dt, dx, dy, dth);
    RCLCPP_INFO(this->get_logger(), "Before: pos=[%.6f, %.6f, %.6f], vel=[%.6f, %.6f, %.6f]",
                x, y, th, vx, vy, vth);

    x += dx;
    y += dy;
    th += dth;

    RCLCPP_INFO(this->get_logger(), "After: pos=[%.6f, %.6f, %.6f]", x, y, th);

    last_time = current_time;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub->publish(odom);
    RCLCPP_INFO(this->get_logger(), "Published wheel_odom: x=%.4f, y=%.4f, th=%.4f", x, y, th);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster->sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    // ROS 2の初期化
    rclcpp::init(argc, argv);

    // PickingRobotOdomノードを作成
    auto node = std::make_shared<PickingRobotOdom>();

    // ノードのスピン
    rclcpp::spin(node);

    // ROS 2の終了処理
    rclcpp::shutdown();
    return 0;
}
