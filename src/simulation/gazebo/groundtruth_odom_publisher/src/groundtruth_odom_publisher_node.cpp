#include <rclcpp/rclcpp.hpp>

#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto groundtruth_odom_publisher = std::make_shared<gazebo::GroundTruthOdomPublisher>();
    rclcpp::spin(groundtruth_odom_publisher);
    rclcpp::shutdown();
    return 0;
};
