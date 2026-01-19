#include <rclcpp/rclcpp.hpp>

#include "map_visualizer/map_visualizer.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto map_visualizer = std::make_shared<visualization::MapVisualizer>();
    rclcpp::spin(map_visualizer);
    rclcpp::shutdown();
    return 0;
};
