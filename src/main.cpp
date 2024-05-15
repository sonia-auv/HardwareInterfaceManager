#include "hardware_interface_manager/RS485Interface.h"
// #include "hardware_interface_manager/ThrusterProvider.hpp"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto thrust = std::make_shared<sonia_hw_interface::RS485Interface>();   
    if (!thrust->OpenPort())
    {
        printf("Could not open port...\n");
        return EXIT_FAILURE;
    }
    rclcpp::spin(thrust);

    rclcpp::shutdown();

    thrust->Kill();
    return EXIT_SUCCESS;
}
