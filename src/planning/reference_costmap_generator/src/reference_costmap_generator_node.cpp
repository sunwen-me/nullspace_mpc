#include <rclcpp/rclcpp.hpp>

#include "reference_costmap_generator/reference_costmap_generator.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto reference_costmap_generator = std::make_shared<planning::ReferenceCostmapGenerator>();
    rclcpp::spin(reference_costmap_generator);
    rclcpp::shutdown();
    return 0;
};
