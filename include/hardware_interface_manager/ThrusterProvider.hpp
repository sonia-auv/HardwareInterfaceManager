#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface_manager/RS485Interface.h"
#include "sonia_common_ros2/msg/motor_messages.hpp"
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

namespace thruster_provider {

    class ThrusterProvider: public rclcpp::Node
    {
        public:
            ThrusterProvider();
            ~ThrusterProvider();

        private:
            bool DryTestServiceCallback();

            void PwmCallback(const std_msgs::msg::UInt16MultiArray &msg);
            sonia_hw_interface::queueObject rs485;
            uint8_t SLAVE;

            rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr _pwm_publisher;
            
            const char* auv = std::getenv("AUV");
            const uint32_t dryTestDelay = 1;
            const uint32_t dryTestOnTime = 3;
            const uint8_t nb_thruster = 8;
            const uint16_t default_pwm = 1500;
            const uint16_t dryTestPwm = 1550;
            
    };
}