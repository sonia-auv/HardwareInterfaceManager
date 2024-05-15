#include "hardware_interface_manager/ThrusterProvider.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace thruster_provider{

    ThrusterProvider::ThrusterProvider()
    :Node("thruster")
    {
        _rs485Publisher =this->create_publisher<sonia_common_ros2::msg::MotorMessages>("/thruster_provider/rs485",10);
        _pwmPublisher = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/thruster_provider/thruster_pwm",10);
        _dryTestServer = this->create_service<sonia_common_ros2::srv::DryTest>("dry_test", std::bind(&ThrusterProvider::DryTestServiceCallback, this,_1,_2));
        _pwmSubscriber = this->create_subscription<std_msgs::msg::UInt16MultiArray>("/thruster_provider/thruster_pwm",10,std::bind(&ThrusterProvider::PwmCallback, this,_1));
        _motorOnOff=this->create_subscription<std_msgs::msg::Bool>("/thruster_provider/startMotor",10,std::bind(&ThrusterProvider::EnableDisableMotors, this,_1));

        if (!strcmp(auv, "AUV8")|| !strcmp(auv, "LOCAL")){
            ESC_SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_PWR_MANAGEMENT;
        }
        else {
            ESC_SLAVE = sonia_common_ros2::msg::MotorMessages::SLAVE_ESC;
        }
    }
    ThrusterProvider::~ThrusterProvider(){}

    void ThrusterProvider::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        
    }
    void ThrusterProvider::PwmCallback(const std_msgs::msg::UInt16MultiArray &msg)
    {
        rs485Message.slave=ESC_SLAVE;
        rs485Message.cmd= sonia_common_ros2::msg::MotorMessages::CMD_PWM;
        rs485Message.data.clear();
                
        for(uint8_t i=0; i<nb_thruster; ++i)
        {
            rs485Message.data.push_back(msg.data[i]>>8);
            rs485Message.data.push_back(msg.data[i] & 0xFF);
        }  
        _rs485Publisher->publish(rs485Message);
    }

    bool ThrusterProvider::DryTestServiceCallback(const std::shared_ptr<sonia_common_ros2::srv::DryTest::Request> request, std::shared_ptr<sonia_common_ros2::srv::DryTest::Response> response)
    {
        std::vector<uint16_t> vect(nb_thruster, default_pwm);
        std_msgs::msg::UInt16MultiArray pwmsMsg;
        pwmsMsg.data.clear();
        pwmsMsg.data.insert(pwmsMsg.data.end(), vect.begin(), vect.end());

        for(uint8_t i=0; i < nb_thruster; ++i)
        {
            pwmsMsg.data[i] = dryTestPwm;
            _pwmPublisher->publish(pwmsMsg);
            PwmCallback(pwmsMsg);
            std::this_thread::sleep_for(std::chrono::milliseconds(dryTestOnTime));
            pwmsMsg.data[i] = default_pwm;
            _pwmPublisher->publish(pwmsMsg);
            PwmCallback(pwmsMsg);
            std::this_thread::sleep_for(std::chrono::milliseconds(dryTestDelay));
        }
        return true;
    }
}