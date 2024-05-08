#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface_manager/RS485Interface.h"
#include "sonia_common_ros2/msg/motor_messages.hpp"
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/UInt16MultiArray.hpp>

namespace thruster_provider {

    class ThrusterProvider: public rclcpp::Node{
        public:
            ThrusterProvider();
            ~ThrusterProvider();

        private:
            //bool DryTestServiceCallback();

            void PwmCallback(const std_msgs::UInt16MultiArray &msg);
            sonia_hw_interface::queueObject rs485;
            uint8_t SLAVE;

            rclcpp::Publisher _publisher;

            
    };
}