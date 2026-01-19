#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
    class VelDriver : public rclcpp::Node
    {
        public:
            VelDriver();
            ~VelDriver();
            void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
        private:
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_left_steer;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_right_steer;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_rear_left_steer;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_rear_right_steer;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_left_rotor;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_front_right_rotor;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_rear_left_rotor;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_rear_right_rotor;
            std_msgs::msg::Float64 cmd_front_left_steer, cmd_front_right_steer, cmd_rear_left_steer, cmd_rear_right_steer;
            std_msgs::msg::Float64 cmd_front_left_rotor, cmd_front_right_rotor, cmd_rear_left_rotor, cmd_rear_right_rotor;

            // vehicle parameters to be loaded from yaml file
            float l_f, l_r, d_l, d_r;
            float tire_radius;
    };
}
