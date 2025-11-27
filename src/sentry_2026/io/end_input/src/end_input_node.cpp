#include <cstdio>
#include "end_input/end_input.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// 1. 添加自定义消息的头文件
#include "end_input/msg/send_data.hpp"

namespace end_input {
EndInput::EndInput(const rclcpp::NodeOptions &options)
    : Node("end_intput", options) {
  RCLCPP_INFO(get_logger(), "Start EndInput!");
  action_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      std::bind(&EndInput::send_data, this, std::placeholders::_1));
  
  // 2. 使用完整的命名空间
  send_pub_ = this->create_publisher<end_input::msg::SendData>("/sentry_to_aim_data", 10);
}

void EndInput::send_data(const geometry_msgs::msg::Twist::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Start Send data from nav");

  try {
    auto sendData = std::make_unique<end_input::msg::SendData>();
    this->nav_to_aim.line_vel_x=static_cast<float>(msg->linear.x);
    this->nav_to_aim.line_vel_y=static_cast<float>(msg->linear.y);
    this->nav_to_aim.angle_vel_z=static_cast<float>(msg->angular.z);
    this->sum_Nav2Aim.push_back(this->nav_to_aim);
    // 假设你的 SendData.msg 中有这些字段
    sendData->line_vel_x = static_cast<float>(msg->linear.x);
    sendData->line_vel_y = static_cast<float>(msg->linear.y);
    sendData->angle_vel_z = static_cast<float>(msg->angular.z);
    sendData->is_use_top = true;

    send_pub_->publish(std::move(sendData));

    RCLCPP_INFO(get_logger(), "Published custom SendData packet!");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Failed to process and publish SendData: %s",
                 e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to process and publish SendData (Unknown Error)");
  }
}
} // namespace end_input