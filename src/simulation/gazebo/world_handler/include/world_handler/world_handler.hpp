#pragma once

#include <signal.h>

#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
    class WorldHandler : public rclcpp::Node
    {
        public:
            WorldHandler();
            ~WorldHandler();
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            static void sigintHandler(int sig);
            void timerCallback();
    };
}
