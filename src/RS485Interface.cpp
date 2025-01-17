#include "hardware_interface_manager/RS485Interface.h"
// #include "RS485Interface.h"
// #include "RS485Interface.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace sonia_hw_interface
{

    RS485Interface::RS485Interface()
        : Node("rs485_interface"), _rs485Connection("/dev/RS485", B115200, false), _thread_control(true)
    {
        try
        {
            auv = std::getenv("AUV");
            if (strcmp(auv, "AUV8") || strcmp(auv, "LOCAL"))
            {
                ESC_SLAVE = _SlaveId::SLAVE_PWR_MANAGEMENT;
                RCLCPP_INFO(this->get_logger(), "Slave on AUV8");
                // std::cerr << "Slave on AUV8" << std::endl;
            }
            else
            {
                ESC_SLAVE = _SlaveId::SLAVE_ESC;
                std::cerr << "Slave on AUV7" << std::endl;
            }
        }
        catch (...)
        {
            ESC_SLAVE = _SlaveId::SLAVE_ESC;
        }

        group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt= rclcpp::SubscriptionOptions();
        sub_opt.callback_group= group1;

        _reader = std::thread(std::bind(&RS485Interface::readData, this));
        _writer = std::thread(std::bind(&RS485Interface::writeData, this));
        _parser = std::thread(std::bind(&RS485Interface::parseData, this));
        _publisherKill = this->create_publisher<sonia_common_ros2::msg::KillStatus>("/provider_rs485/kill_status", 10);
        _publisherMission = this->create_publisher<sonia_common_ros2::msg::MissionStatus>("/provider_rs485/mission_status", 10);
        _publisherMotorVoltages = this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_voltages", 10);
        _publisherMotorCurrents = this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_currents", 10);
        _publisherMotorTemperature = this->create_publisher<sonia_common_ros2::msg::MotorPowerMessages>("/provider_power/motor_temperatures", 10);
        _publisherBatteryVoltages = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>("/provider_power/battery_voltages", 10);
        _publisherBatteryCurrents = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>("/provider_power/battery_currents", 10);
        _publisherBatteryTemperature = this->create_publisher<sonia_common_ros2::msg::BatteryPowerMessages>("/provider_power/battery_temperatures", 10);
        _publisherMotorFeedback = this->create_publisher<sonia_common_ros2::msg::MotorFeedback>("/provider_power/motor_feedback", 10);
        _dropperServer = this->create_service<sonia_common_ros2::srv::DropperService>("actuate_dropper", std::bind(&RS485Interface::processDropperRequest, this, _1, _2));
        _timerKillMission = this->create_wall_timer(500ms, std::bind(&RS485Interface::pollKillMission, this));
        _timerPowerRequest= this->create_wall_timer(500ms, std::bind(&RS485Interface::pollPower, this));

        _publisherThrusterPwm = this->create_publisher<sonia_common_ros2::msg::MotorPwm>("/provider_thruster/thruster_pwm", 10);
        _subscriberThrusterPwm = this->create_subscription<sonia_common_ros2::msg::MotorPwm>("/provider_thruster/thruster_pwm", 10, std::bind(&RS485Interface::PwmCallback, this, _1), sub_opt);
        _subscriberMotorOnOff = this->create_subscription<std_msgs::msg::Bool>("/provider_power/activate_motors", 10, std::bind(&RS485Interface::EnableDisableMotors, this, _1), sub_opt);

    }

    // node destructor
    RS485Interface::~RS485Interface()
    {
    }

    bool RS485Interface::OpenPort()
    {
        bool res = _rs485Connection.OpenPort();
        if (res)
        {
            _rs485Connection.Flush();
        }
        return res;
    }

    void RS485Interface::pollKillMission()
    {
        // Transmit request to get kill status
        mutex_.lock();
        _rs485Connection.Transmit(_GET_KILL_STATUS_MSG, 8);
        mutex_.unlock();
        // Wait for a short duration to allow for processing
        //std::this_thread::sleep_for(std::chrono::milliseconds(300));
        // Transmit request to get mission status
        mutex_.lock();
        _rs485Connection.Transmit(_GET_MISSION_STATUS_MSG, 8);
        mutex_.unlock();
        
    }
    void RS485Interface::pollPower()
    {
        mutex_.lock();
        _rs485Connection.Transmit(_GET_POWER_MSG, 15);
        mutex_.unlock();
        //std::this_thread::sleep_for(std::chrono::milliseconds(300));
        mutex_.lock();
        _rs485Connection.Transmit(_GET_FEEDBACK_MSG, 15);
        mutex_.unlock();

    }

    std::tuple<uint8_t, uint8_t> RS485Interface::checkSum(uint8_t slave, uint8_t cmd, uint8_t nbByte, std::vector<uint8_t> data)
    {
        uint16_t check = (uint16_t)(_START_BYTE + slave + cmd + nbByte + _END_BYTE);
        for (uint8_t i = 0; i < nbByte; i++)
        {
            check += (uint8_t)data[i];
        }
        return {check >> 8, check & 0XFF};
    }

    void RS485Interface::Kill()
    {
        _thread_control = false;
    }

    void RS485Interface::processDropperRequest(const std::shared_ptr<sonia_common_ros2::srv::DropperService::Request> request, std::shared_ptr<sonia_common_ros2::srv::DropperService::Response> response)
    {
        // Variables for transmission status and data vector
        ssize_t transmit_status;
        std::vector<uint8_t> data_vec;

        // Add dropper side to data vector
        data_vec.push_back(request->side);

        // Calculate checksum for the command
        std::tuple<uint8_t, uint8_t> checksum = checkSum(_SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, data_vec.size(), data_vec);

        // Construct the command packet
        const uint8_t dropper[8] = {_START_BYTE, _SlaveId::SLAVE_IO, _Cmd::CMD_IO_DROPPER_ACTION, 1, request->side, std::get<0>(checksum), std::get<1>(checksum), _END_BYTE};
        mutex_.lock();
        transmit_status = _rs485Connection.Transmit(dropper, 8);
        mutex_.unlock();
        response->result = transmit_status;
    }

    void RS485Interface::publishKill(bool status)
    {
        sonia_common_ros2::msg::KillStatus state;
        state.status = status;
        _publisherKill->publish(state);
    }

    void RS485Interface::publishMission(bool status)
    {
        sonia_common_ros2::msg::MissionStatus state;
        state.status = status;
        _publisherMission->publish(state);
    }

    void RS485Interface::publishMotor(uint8_t cmd, std::vector<float> data)
    {
        /*if (data.size() != 8)
        {
            return;
        }*/
        sonia_common_ros2::msg::MotorPowerMessages msg;
        msg.motor1 = data[0];
        msg.motor2 = data[1];
        msg.motor3 = data[2];
        msg.motor4 = data[3];
        msg.motor5 = data[4];
        msg.motor6 = data[5];
        msg.motor7 = data[6];
        msg.motor8 = data[7];

        switch (cmd)
        {
        case _Cmd::CMD_VOLTAGE:
            _publisherMotorVoltages->publish(msg);
            break;
        case _Cmd::CMD_CURRENT:
            _publisherMotorCurrents->publish(msg);
            break;
        case _Cmd::CMD_TEMPERATURE:
            _publisherMotorTemperature->publish(msg);
            break;
        default:
            break;
        }
    }

    void RS485Interface::publishBattery(uint8_t cmd, float *data)
    {
        sonia_common_ros2::msg::BatteryPowerMessages msg;
        msg.battery1 = data[0];
        msg.battery2 = data[1];

        switch (cmd)
        {
        case _Cmd::CMD_VOLTAGE:
            _publisherBatteryVoltages->publish(msg);
            break;
        case _Cmd::CMD_CURRENT:
            _publisherBatteryCurrents->publish(msg);
            break;
        case _Cmd::CMD_TEMPERATURE:
            _publisherBatteryTemperature->publish(msg);
            break;
        default:
            break;
        }
    }

    void RS485Interface::publishMotorFeedback(std::vector<uint8_t> data)
    {
        sonia_common_ros2::msg::MotorFeedback msg;
        msg.motor1 = data[0];
        msg.motor2 = data[1];
        msg.motor3 = data[2];
        msg.motor4 = data[3];
        msg.motor5 = data[4];
        msg.motor6 = data[5];
        msg.motor7 = data[6];
        msg.motor8 = data[7];
        _publisherMotorFeedback->publish(msg);
    }
    void RS485Interface::processPowerManagement(const uint8_t cmd, const std::vector<uint8_t> data)
    {
        std::vector<float> motorData;
        motorData.reserve(10);
        float batteryData[2];

        switch (cmd)
        {
        case _Cmd::CMD_VOLTAGE:
            
            if (convertBytesToFloat(data, motorData, nb_thruster+nb_battery) < 0 )
            {
                std::cerr << "ERROR in the message. Dropping VOLTAGE packet" << std::endl;
                return;
            }
            
            batteryData[0] = motorData[motorData.size()-2];
            batteryData[1] = motorData[motorData.size()-1];
            motorData.pop_back();
            motorData.pop_back();
            
            publishMotor(_Cmd::CMD_VOLTAGE, motorData);
            publishBattery(_Cmd::CMD_VOLTAGE, batteryData);

            break;
        case _Cmd::CMD_CURRENT:
            
            if (convertBytesToFloat(data, motorData,nb_thruster+nb_battery) < 0)
            {
                std::cerr << "ERROR in the message. Dropping CURRENT packet" << std::endl;
                return;
            }

            batteryData[0] = motorData[motorData.size()-2];
            batteryData[1] = motorData[motorData.size()-1];
            motorData.pop_back();
            motorData.pop_back();

            publishMotor(_Cmd::CMD_CURRENT, motorData);
            publishBattery(_Cmd::CMD_CURRENT, batteryData);
            break;
        case _Cmd::CMD_TEMPERATURE:
            
            if (convertBytesToFloat(data, motorData, nb_thruster+nb_battery) < 0)
            {
                std::cerr << "ERROR in the message. Dropping TEMPERATURE packet" << std::endl;
                return;
            }

            batteryData[0] = motorData[motorData.size() - 2];
            batteryData[1] = motorData[motorData.size() - 1];
            motorData.pop_back();
            motorData.pop_back();

            publishMotor(_Cmd::CMD_TEMPERATURE, motorData);
            publishBattery(_Cmd::CMD_TEMPERATURE, batteryData);
            break;
        case _Cmd::CMD_READ_MOTOR:

            //if (motorData.size() !=nb_thruster)
            //{
                //std::cerr << "ERROR in the message. Dropping READ MOTOR packet" << std::endl;
              //  return;
            //}
            publishMotorFeedback(data);
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "CMD Not identified");
            break;
        }
    }

    void RS485Interface::readData()
    {
        uint8_t data[_DATA_READ_CHUNCK];
        while (_thread_control)
        {
            // This sleep is needed... I DON'T KNOW WHY...
            //std::this_thread::sleep_for(std::chrono::milliseconds(300));
            mutex_.lock();
            ssize_t str_len = _rs485Connection.ReadPackets(_DATA_READ_CHUNCK, data);
            mutex_.unlock();

            if (str_len != -1)
            {
                for (ssize_t i = 0; i < str_len; i++)
                {
                    _parseQueue.push_back((uint8_t)data[i]);
                }
            }
        }
    }

    void RS485Interface::writeData()
    {
        // close the thread.
        while (_thread_control)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // pause the thread.
            while (!_writerQueue.empty())
            {
                //std::this_thread::sleep_for(std::chrono::milliseconds(300));

                queueObject msg = _writerQueue.get_n_pop_front();
                const size_t data_size = msg.data.size() + 7;
                uint8_t *data = new uint8_t[data_size];
                data[0] = _START_BYTE;
                data[1] = msg.slave;
                data[2] = msg.cmd;
                data[3] = (uint8_t)msg.data.size();

                std::vector<uint8_t> data_vec;
                for (int i = 0; i < data[3]; i++)
                {
                    data[i + 4] = msg.data[i];
                    data_vec.push_back(msg.data[i]);
                }

                std::tuple<uint8_t, uint8_t> checksum = checkSum(data[1], data[2], data[3], data_vec);

                data[data_size - 3] = std::get<0>(checksum);
                data[data_size - 2] = std::get<1>(checksum);
                data[data_size - 1] = _END_BYTE;
                mutex_.lock();
                _rs485Connection.Transmit(data, data_size);
                mutex_.unlock();
                delete data;
            }
        }
    }

    void RS485Interface::parseData()
    {
        while (_thread_control)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            // read until the start there or the queue is empty
            while (!_parseQueue.empty())
            {
                
                // check if the bit is the start bit:
                if (_parseQueue.front() != _START_BYTE)
                {
                    _parseQueue.pop_front();
                }
                else
                {
                    queueObject msg;

                    // pop the unused start data
                    _parseQueue.pop_front();

                    msg.slave = _parseQueue.get_n_pop_front();
                    msg.cmd = _parseQueue.get_n_pop_front();
                    uint8_t nbByte = _parseQueue.get_n_pop_front();

                    for (int i = 0; i < nbByte; i++)
                    {
                        msg.data.push_back(_parseQueue.get_n_pop_front());
                    }

                    std::tuple<uint8_t, uint8_t> checkResult = {(_parseQueue.get_n_pop_front()), _parseQueue.get_n_pop_front()};
                    
                    // pop the unused end data
                    _parseQueue.pop_front();

                    std::tuple<uint8_t, uint8_t> calc_checksum = checkSum(msg.slave, msg.cmd, nbByte, msg.data);
                    // if the checksum is bad, drop the packet
                    if (checkResult == calc_checksum)
                    {
                        // publisher.publish(msg);
                        switch (msg.slave)
                        {
                        case _SlaveId::SLAVE_KILLMISSION:
                            switch (msg.cmd)
                            {
                            case _Cmd::CMD_KILL:
                                // get data value
                                // publish on kill publisher
                                publishKill(msg.data[0] == 1);
                                break;
                            case _Cmd::CMD_MISSION:
                                // get data value
                                // publish on mission publisher
                                publishMission(msg.data[0] == 1);
                                break;
                            default:
                                break;
                            }
                            break;
                        case _SlaveId::SLAVE_PWR_MANAGEMENT:                         
                            processPowerManagement(msg.cmd, msg.data);
                            break;
                        default:
                            break;
                        }
                    }
                    // packet dropped
                }
            }
        }
    }

    int RS485Interface::convertBytesToFloat(const std::vector<uint8_t> &req, std::vector<float> &res, const size_t size)
    {
        uint8_t size_req = req.size();
        if (size_req % 4 != 0)
            return -1;

        _bytesToFloat converter;

        for (uint8_t i = 0; i < size; ++i) // shifting of 4 for each data
        {
            converter.bytes[0] = req[4 * i];
            converter.bytes[1] = req[4 * i + 1];
            converter.bytes[2] = req[4 * i + 2];
            converter.bytes[3] = req[4 * i + 3];
            res.push_back(converter.value);
        }
        return 0;
    }

    void RS485Interface::EnableDisableMotors(const std_msgs::msg::Bool &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_ACT_MOTOR;
        switch (ESC_SLAVE)
        {
        // AUV8 motor control
        case _SlaveId::SLAVE_PWR_MANAGEMENT:
            ser.slave = ESC_SLAVE;
            ToggleMotors(msg.data, nb_thruster, ser.data);
            _writerQueue.push_back(ser);
            break;
        // AUV7 motor control
        case _SlaveId::SLAVE_ESC:
            ser.slave = _SlaveId::SLAVE_PSU0;
            ser.data.clear();
            ToggleMotors(msg.data, nb_thruster / 4, ser.data);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU1;
            ser.data.clear();
            ToggleMotors(msg.data, nb_thruster / 4, ser.data);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU2;
            ser.data.clear();
            ToggleMotors(msg.data, nb_thruster / 4, ser.data);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU3;
            ser.data.clear();
            ToggleMotors(msg.data, nb_thruster / 4, ser.data);
            _writerQueue.push_back(ser);
            break;
        default:
            break;
        }
    }
    void RS485Interface::ToggleMotors(const bool state, uint8_t size, std::vector<uint8_t> &data)
    {
        if (state)
        {
            for (size_t i = 0; i < size; i++)
            {
                data.push_back(1);
            }
        }
        else
        {
            for (size_t i = 0; i < size; i++)
            {
                data.push_back(0);
            }
        }
    }
    void RS485Interface::PwmCallback(const sonia_common_ros2::msg::MotorPwm &msg)
    {
        queueObject ser;
        ser.cmd = _Cmd::CMD_PWM;
        switch (ESC_SLAVE)
        {
        case _SlaveId::SLAVE_PWR_MANAGEMENT:
            ser.slave = ESC_SLAVE;

            ser.data.push_back(msg.motor1 >> 8);
            ser.data.push_back(msg.motor1 & 0xFF);

            ser.data.push_back(msg.motor2 >> 8);
            ser.data.push_back(msg.motor2 & 0xFF);

            ser.data.push_back(msg.motor3 >> 8);
            ser.data.push_back(msg.motor3 & 0xFF);

            ser.data.push_back(msg.motor4 >> 8);
            ser.data.push_back(msg.motor4 & 0xFF);

            ser.data.push_back(msg.motor5 >> 8);
            ser.data.push_back(msg.motor5 & 0xFF);

            ser.data.push_back(msg.motor6 >> 8);
            ser.data.push_back(msg.motor6 & 0xFF);

            ser.data.push_back(msg.motor7 >> 8);
            ser.data.push_back(msg.motor7 & 0xFF);

            ser.data.push_back(msg.motor8 >> 8);
            ser.data.push_back(msg.motor8 & 0xFF);

            _writerQueue.push_back(ser);
            break;
        case _SlaveId::SLAVE_ESC:
            ser.slave = _SlaveId::SLAVE_PSU0;
            ser.data.clear();
            ser.data.push_back(msg.motor1 >> 8);
            ser.data.push_back(msg.motor1 & 0xFF);
            ser.data.push_back(msg.motor5 >> 8);
            ser.data.push_back(msg.motor5 & 0xFF);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU1;
            ser.data.clear();
            ser.data.push_back(msg.motor2 >> 8);
            ser.data.push_back(msg.motor2 & 0xFF);
            ser.data.push_back(msg.motor6 >> 8);
            ser.data.push_back(msg.motor6 & 0xFF);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU2;
            ser.data.clear();
            ser.data.push_back(msg.motor3 >> 8);
            ser.data.push_back(msg.motor3 & 0xFF);
            ser.data.push_back(msg.motor7 >> 8);
            ser.data.push_back(msg.motor7 & 0xFF);
            _writerQueue.push_back(ser);

            ser.slave = _SlaveId::SLAVE_PSU3;
            ser.data.clear();
            ser.data.push_back(msg.motor4 >> 8);
            ser.data.push_back(msg.motor4 & 0xFF);
            ser.data.push_back(msg.motor8 >> 8);
            ser.data.push_back(msg.motor8 & 0xFF);
            _writerQueue.push_back(ser);
            break;

        default:
            break;
        }
    }
}
