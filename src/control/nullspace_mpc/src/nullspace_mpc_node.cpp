#include <rclcpp/rclcpp.hpp>

#include "nullspace_mpc/nullspace_mpc.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto mpc = std::make_shared<controller::MPC>();
    rclcpp::spin(mpc);
    rclcpp::shutdown();
    return 0;
};
