#include "hardware_interface_manager/ThrusterProvider.hpp"

namespace thruster_provider{

    ThrusterProvider::ThrusterProvider(){}
    ThrusterProvider::~ThrusterProvider(){}

    void ThrusterProvider::PwmCallback(const std_msgs::UInt16MultiArray &msg)
    {
        rs485.slave=SLAVE;
        rs485.cmd= sonia_common_ros2::msg::MotorMessages::CMD_PWM;
        rs485.data.clear();

        rs485.data.pop_back(msg.data[i]>>8);
        rs485.data.pop_back(msg.data[i]&0xFF);
        
    }
}