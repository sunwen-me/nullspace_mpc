#include <rclcpp/rclcpp.hpp>

#include "mppi_h/mppi_h.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mppi = std::make_shared<controller_mppi_h::MPPI>();
    rclcpp::spin(mppi);
    rclcpp::shutdown();
    return 0;
};
