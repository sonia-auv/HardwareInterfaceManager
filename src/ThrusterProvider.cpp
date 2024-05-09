#include "hardware_interface_manager/ThrusterProvider.hpp"

namespace thruster_provider{

    ThrusterProvider::ThrusterProvider()
    :Node("rs485")
    {
        _pwm_publisher = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/thruster_provider/thruster_pwm",10);

        if (strcmp(auv, "AUV8") == 0){
            SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_PWR_MANAGEMENT;
        }

        else if (strcmp(auv, "AUV7") == 0){
            SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_ESC;
        }

        else if (strcmp(auv, "LOCAL") == 0){
            SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_PWR_MANAGEMENT;
        }

        else {
            SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_ESC;
        }
    }
    ThrusterProvider::~ThrusterProvider(){}

    void ThrusterProvider::PwmCallback(const std_msgs::msg::UInt16MultiArray &msg)
    {
        rs485.slave=SLAVE;
        rs485.cmd= sonia_common_ros2::msg::MotorMessages::CMD_PWM;
        rs485.data.clear();
                
            rs485.data.push_back(msg.data[1]>>8);
            rs485.data.push_back(msg.data[1]&0xFF);
        
        _pwm_publisher->publish(msg);
    }
    bool ThrusterProvider::DryTestServiceCallback()
    {
        std_msgs::msg::UInt16MultiArray pwmsMsg;
        pwmsMsg.data.clear();

        return true;
    }
}