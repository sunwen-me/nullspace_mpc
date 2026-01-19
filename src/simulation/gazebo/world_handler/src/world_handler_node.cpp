#include <rclcpp/rclcpp.hpp>

#include "world_handler/world_handler.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto world_handler = std::make_shared<gazebo::WorldHandler>();
    rclcpp::spin(world_handler);
    rclcpp::shutdown();
    return 0;
};
