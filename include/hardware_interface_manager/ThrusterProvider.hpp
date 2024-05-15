#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface_manager/RS485Interface.h"
#include "sonia_common_ros2/msg/motor_messages.hpp"
// #include "sonia_common_ros2/srv/dry_test.hpp"
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

namespace thruster_provider {

    class ThrusterProvider: public rclcpp::Node
    {
        public:
            ThrusterProvider();
            ~ThrusterProvider();

        private:
            // bool DryTestServiceCallback(const std::shared_ptr<sonia_common_ros2::srv::DryTest::Request> request, std::shared_ptr<sonia_common_ros2::srv::DryTest::Response> response);
            void EnableDisableMotors(const std_msgs::msg::Bool &msg);
            void PwmCallback(const std_msgs::msg::UInt16MultiArray &msg);
            
            sonia_common_ros2::msg::MotorMessages rs485Message;
            uint8_t ESC_SLAVE;

            rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _pwmPublisher;
            rclcpp::Publisher<sonia_common_ros2::msg::MotorMessages>::SharedPtr _rs485Publisher;
            // rclcpp::Service<sonia_common_ros2::srv::DryTest>::SharedPtr _dryTestServer;
            rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr _pwmSubscriber;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _motorOnOff;

            const char* auv = std::getenv("AUV");
            const uint32_t dryTestDelay = 1000;
            const uint32_t dryTestOnTime = 3000;
            const uint8_t nb_thruster = 8;
            const uint16_t default_pwm = 1500;
            const uint16_t dryTestPwm = 1550;
            
    };
}