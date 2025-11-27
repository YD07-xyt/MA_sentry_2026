#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// 1. 添加自定义消息的头文件
#include "end_input/msg/send_data.hpp" 

namespace end_input{
    class EndInput: public rclcpp::Node{
        public:
            EndInput(const rclcpp::NodeOptions &options);
            ~EndInput() override;
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr action_sub_;
            rclcpp::Publisher<end_input::msg::SendData>::SharedPtr send_pub_;
            void send_data(const geometry_msgs::msg::Twist::SharedPtr msg);
    };
}