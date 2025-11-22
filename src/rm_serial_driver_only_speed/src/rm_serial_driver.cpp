#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "crc.hpp"
#include "packet.hpp"
#include "rm_serial_driver.hpp"
namespace rm_serial_driver {
    RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions &options)
        : Node("rm_serial_driver", options),
          serial_port(io_context) {
        RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

        getParams();

        try {
            openPort();
            RCLCPP_INFO(get_logger(), "serial open OK!");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error creating serial port: %s - %s", config.device_name.c_str(), ex.what());
            throw ex;
        }

        action_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", rclcpp::SensorDataQoS(),
                std::bind(&RMSerialDriver::sendPacket, this, std::placeholders::_1));
        // serial_port.async_read_some(
        //         boost::asio::buffer(read_buffer_),
        //         std::bind(
        //                 &RMSerialDriver::receivePacket,
        //                 this,
        //                 std::placeholders::_1,// error_code
        //                 std::placeholders::_2 // bytes_transferred
        //                 ));
    }

    RMSerialDriver::~RMSerialDriver() {
        if (serial_port.is_open()) {
            closePort();
        }
        if (!io_context.stopped()) {
            boost::asio::post(io_context, [this]() {
                io_context.stop();
            });
        }
    }

    // void RMSerialDriver::receivePacket(const boost::system::error_code &ec, std::size_t bytes_transferred) {
    //     if (ec) {
    //         RCLCPP_ERROR(this->get_logger(), "Serial receive error: %s", ec.message().c_str());
    //         // 如果发生错误，尝试重新打开端口
    //         reopenPort();
    //         // 无论是否成功，都再次启动接收
    //         receivePacket();
    //         return;
    //     }

    //     RCLCPP_DEBUG(this->get_logger(), "Received %zu bytes.", bytes_transferred);

    //     // 简单的协议解析：查找帧头 'M' 'A'
    //     static std::vector<uint8_t> frame_buffer;

    //     frame_buffer.insert(frame_buffer.end(), read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);

    //     while (frame_buffer.size() >= sizeof(ReceivePacket)) {
    //         // 查找帧头
    //         auto it = std::search(frame_buffer.begin(), frame_buffer.end(), std::begin(ReceivePacket().header), std::end(ReceivePacket().header));
    //         if (it == frame_buffer.end()) {
    //             // 未找到完整帧头，清空缓冲区或保留最后一个字节以处理跨包的情况
    //             // 这里为简单起见，直接清空
    //             frame_buffer.clear();
    //             RCLCPP_WARN(this->get_logger(), "Did not find frame header 'MA', clearing buffer.");
    //             break;
    //         }

    //         // 如果帧头不在缓冲区开头，移除前面的无效数据
    //         if (it != frame_buffer.begin()) {
    //             RCLCPP_WARN(this->get_logger(), "Discarding %zu bytes of garbage data before header.", std::distance(frame_buffer.begin(), it));
    //             frame_buffer.erase(frame_buffer.begin(), it);
    //         }

    //         // 现在缓冲区开头是帧头，检查是否有足够的数据
    //         if (frame_buffer.size() < sizeof(ReceivePacket)) {
    //             // 数据不足一帧，等待下一次读取
    //             break;
    //         }

    //         // 尝试解析数据包
    //         ReceivePacket packet = fromVector(frame_buffer);

    //         // 验证CRC16
    //         // 注意：CRC计算不包含CRC字段本身
    //         uint16_t calculated_crc = get_crc16(reinterpret_cast<uint8_t *>(&receivePacket), sizeof(ReceivePacket) - sizeof(packet.crc16));
    //         if (calculated_crc != packet.crc16) {
    //             RCLCPP_ERROR(this->get_logger(), "CRC check failed! Received: 0x%04X, Calculated: 0x%04X", packet.crc16, calculated_crc);
    //             // CRC错误，移除帧头并继续查找下一个
    //             frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + sizeof(ReceivePacket::hearder));
    //             continue;
    //         }

    //         // 数据有效，处理并发布
    //         publishData(packet);

    //         // 从缓冲区中移除已处理的数据包
    //         frame_buffer.erase(frame_buffer.begin(), frame_buffer.begin() + sizeof(ReceivePacket));
    //         RCLCPP_DEBUG(this->get_logger(), "Processed a valid packet. Remaining buffer size: %zu", frame_buffer.size());
    //     }

    //     // 再次调用receivePacket以继续接收下一批数据
    //     receivePacket();
    // }

    // void publishData(const ReceivePacket &packet) {
    //     // 发布姿态信息 (PoseStamped)
    //     auto pose_msg = geometry_msgs::msg::PoseStamped();
    //     pose_msg.header.stamp = this->now();
    //     pose_msg.header.frame_id = "base_link";// 或 "imu_link"，根据你的TF树定义
    //     pose_msg.pose.orientation.w = packet.quat_w;
    //     pose_msg.pose.orientation.x = packet.quat_x;
    //     pose_msg.pose.orientation.y = packet.quat_y;
    //     pose_msg.pose.orientation.z = packet.quat_z;
    //     // 位置信息如果IMU不提供，可以设为0
    //     pose_msg.pose.position.x = 0.0;
    //     pose_msg.pose.position.y = 0.0;
    //     pose_msg.pose.position.z = 0.0;
    //     pose_publisher_->publish(pose_msg);

    //     // 发布运动信息 (TwistStamped)
    //     auto twist_msg = geometry_msgs::msg::TwistStamped();
    //     twist_msg.header.stamp = this->now();
    //     twist_msg.header.frame_id = "base_link";
    //     // 角加速度(gyro)对应Twist中的angular
    //     twist_msg.twist.angular.x = packet.gyro_x;
    //     twist_msg.twist.angular.y = packet.gyro_y;
    //     twist_msg.twist.angular.z = packet.gyro_z;
    //     // 线加速度(accel)对应Twist中的linear
    //     twist_msg.twist.linear.x = packet.accel_x;
    //     twist_msg.twist.linear.y = packet.accel_y;
    //     twist_msg.twist.linear.z = packet.accel_z;
    //     twist_publisher_->publish(twist_msg);
    // }

    void RMSerialDriver::sendPacket(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(get_logger(), "x: %.2f, y: %.2f,  rz: %.2f", msg->linear.x, msg->linear.y, msg->angular.z);
        try {
            SendPacket sendPacket;
            sendPacket.header[0] = 'M';
            sendPacket.header[1] = 'A';
            sendPacket.line_vel_x = static_cast<float>(msg->linear.x);
            sendPacket.line_vel_y = static_cast<float>(msg->linear.y);
            sendPacket.angle_vel_z = static_cast<float>(msg->angular.z);
            sendPacket.is_use_top = true;
            sendPacket.priority = 0x00;
            // 计算并填充 CRC16
            // 为了计算CRC，我们需要将结构体中除了crc16字段以外的所有数据视为一个连续的字节流
            // 注意：这里假设CRC不包含自身
            //get_crc16 函数计算校验和
            sendPacket.crc16 = get_crc16(
                    reinterpret_cast<uint8_t *>(&sendPacket), sizeof(sendPacket) - sizeof(sendPacket.crc16));

            serial_port.write_some(boost::asio::buffer(&sendPacket, sizeof(sendPacket)));
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            reopenPort();
        }
    }

    void RMSerialDriver::openPort() {
        serial_port.open(config.device_name);
        serial_port.set_option(boost::asio::serial_port_base::baud_rate(config.baud_rate));
        serial_port.set_option(boost::asio::serial_port_base::flow_control(config.flow_control));
        serial_port.set_option(boost::asio::serial_port_base::parity(config.parity));
        serial_port.set_option(boost::asio::serial_port_base::stop_bits(config.stop_bits));
    }

    void RMSerialDriver::closePort() {
        boost::system::error_code error;
        serial_port.close(error);
        if (error) {
            RCLCPP_ERROR(get_logger(), "close failed: %s", error.message().c_str());
        }
    }

    void RMSerialDriver::reopenPort() {
        RCLCPP_WARN(get_logger(), "Attempting to reopen port");
        try {
            if (serial_port.is_open()) {
                closePort();
            }
            openPort();
            RCLCPP_INFO(get_logger(), "Successfully reopened port");
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
            if (rclcpp::ok()) {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }

    void RMSerialDriver::getParams() {
        try {
            std::string device_name_ = declare_parameter<std::string>("device_name", "/tty/ACM0");
            config.device_name = device_name_;
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }

        try {
            uint32_t baud_rate = declare_parameter<long>("baud_rate", 0);
            config.baud_rate = baud_rate;
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try {
            std::string flowControlString = declare_parameter<std::string>("flow_control", "");

            if (flowControlString == "none") {
                config.flow_control = boost::asio::serial_port_base::flow_control::none;
            } else if (flowControlString == "hardware") {
                config.flow_control = boost::asio::serial_port_base::flow_control::hardware;
            } else if (flowControlString == "software") {
                config.flow_control = boost::asio::serial_port_base::flow_control::software;
            } else {
                throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try {
            std::string parityString = declare_parameter<std::string>("parity", "");

            if (parityString == "none") {
                config.parity = boost::asio::serial_port_base::parity::none;
            } else if (parityString == "odd") {
                config.parity = boost::asio::serial_port_base::parity::odd;
            } else if (parityString == "even") {
                config.parity = boost::asio::serial_port_base::parity::even;
            } else {
                throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try {
            std::string stopBitsString = declare_parameter<std::string>("stop_bits", "");

            if (stopBitsString == "1" || stopBitsString == "1.0") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::one;
            } else if (stopBitsString == "1.5") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::onepointfive;
            } else if (stopBitsString == "2" || stopBitsString == "2.0") {
                config.stop_bits = boost::asio::serial_port_base::stop_bits::two;
            } else {
                throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        } catch (rclcpp::ParameterTypeException &ex) {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }
    }

}// namespace rm_serial_driver


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
