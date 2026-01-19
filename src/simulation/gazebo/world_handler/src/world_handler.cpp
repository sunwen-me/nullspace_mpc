#include "world_handler/world_handler.hpp"

#include <chrono>

namespace gazebo
{

// constructor
WorldHandler::WorldHandler()
    : rclcpp::Node("world_handler")
{
    // register SIGINT handler
    signal(SIGINT, WorldHandler::sigintHandler);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&WorldHandler::timerCallback, this));
}

// destructor
WorldHandler::~WorldHandler()
{
    // No Contents
}

// add signal handler of Ctrl+C
void WorldHandler::sigintHandler(int sig)
{
    (void)sig;
    RCLCPP_WARN(rclcpp::get_logger("world_handler"), "Ctrl+C is pressed. kill gazebo process.");
    int result_kill_gazebo = system("killall -9 gzserver gzclient&");
    (void)result_kill_gazebo;
    rclcpp::shutdown();
}

// timer callback
void WorldHandler::timerCallback()
{
    // Not using this timer callback now...
    RCLCPP_INFO(this->get_logger(), "timer callback in world_handler node.");
}

} // namespace gazebo
