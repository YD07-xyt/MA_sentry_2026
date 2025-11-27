#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// 1. 添加自定义消息的头文件
#include "end_input/msg/send_data.hpp" 

namespace end_input{
    struct Nav2Aim{
        float line_vel_x;
        float line_vel_y;
        float angle_vel_z;

    };
    class EndInput: public rclcpp::Node{
        public:
            EndInput(const rclcpp::NodeOptions &options);
            ~EndInput() override;
            std::vector<Nav2Aim> sum_Nav2Aim;
            Nav2Aim nav_to_aim;
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr action_sub_;
            rclcpp::Publisher<end_input::msg::SendData>::SharedPtr send_pub_;
            void send_data(const geometry_msgs::msg::Twist::SharedPtr msg);
    };
}