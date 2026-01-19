#include <rclcpp/rclcpp.hpp>

#include "vel_driver/vel_driver.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto vel_driver = std::make_shared<gazebo::VelDriver>();
    rclcpp::spin(vel_driver);
    rclcpp::shutdown();
    return 0;
};
