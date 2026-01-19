# pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tuple>
#include <vector>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <jsk_rviz_plugins/msg/overlay_text.hpp>
#include "common_param.hpp"
#include "mode1_mppi_3d/param.hpp"
#include "mode2_mppi_4d/param.hpp"
#include "mppi_h/mppi_h_core.hpp"
#include "mpc_eval_msgs/msg/mpc_eval.hpp"

namespace controller_mppi_h
{
    class MPPI : public rclcpp::Node // MPPI execution (dependent on ROS 2)
    {
        public:
            MPPI();
            ~MPPI();
        private:
            // publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_absvel_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_vx_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_vy_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_omega_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mppi_calc_time_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mppi_optimal_traj_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mppi_sampled_traj_;
            rclcpp::Publisher<jsk_rviz_plugins::msg::OverlayText>::SharedPtr pub_mppi_overlay_text_;
            rclcpp::Publisher<mpc_eval_msgs::msg::MPCEval>::SharedPtr pub_mpc_eval_msg_;

            // subscriber
            //// odometry (global vehicle pose)
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
            void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
            bool odom_received_;
            nav_msgs::msg::Odometry latest_odom_;
            common_type::XYYaw observed_state_;

            //// reference path
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_ref_path_;
            void refPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
            bool ref_path_received_;
            nav_msgs::msg::Path latest_ref_path_;
            common_type::XYYaw goal_state_;

            //// collision costmap
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_collision_costmap_;
            void collisionCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
            bool collision_costmap_received_;
            grid_map::GridMap collision_costmap_;

            //// distance error map
            rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_distance_error_map_;
            void distanceErrorMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg);
            bool distance_error_map_received_;
            grid_map::GridMap distance_error_map_;

            //// reference yaw map
            rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_ref_yaw_map_;
            void refYawMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg);
            bool ref_yaw_map_received_;
            grid_map::GridMap ref_yaw_map_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_control_interval_;
            void calcControlCommand();

            // rviz visualization
            void publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence);
            void publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences);
            void publishOverlayText(const std::string& text);

            // mppi core instance
            MPPIHybridCore* mppi_hybrid_core_;
    };
} // namespace controller_mppi_h
