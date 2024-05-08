#pragma once

#include <mutex>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
namespace power_provider
{
    class PowerProvider{
        public:
            PowerProvider();
            ~PowerProvider();
        private:
            std::mutex _voltageMutex;
            std::mutex _currentMutex;
            std::mutex _motorMutex;
    };
} // namespace power_provider
