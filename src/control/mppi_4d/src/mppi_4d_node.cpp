#include <rclcpp/rclcpp.hpp>

#include "mppi_4d/mppi_4d.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mppi = std::make_shared<controller::MPPI>();
    rclcpp::spin(mppi);
    rclcpp::shutdown();
    return 0;
};
