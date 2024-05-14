#include "hardware_interface_manager/RS485Interface.h"
#include "hardware_interface_manager/ThrusterProvider.hpp"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto thrust = std::make_shared<thruster_provider::ThrusterProvider>();   
    rclcpp::spin(thrust);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
