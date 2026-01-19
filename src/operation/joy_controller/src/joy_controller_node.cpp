#include <rclcpp/rclcpp.hpp>

#include "joy_controller/joy_controller.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto joy_controller = std::make_shared<operation::JoyController>();
    rclcpp::spin(joy_controller);
    rclcpp::shutdown();
    return 0;
};
