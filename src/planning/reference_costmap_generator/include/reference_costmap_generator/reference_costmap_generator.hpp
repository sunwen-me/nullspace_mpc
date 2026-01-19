#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>

namespace planning
{
    class ReferenceCostmapGenerator : public rclcpp::Node
    {
        public:
            ReferenceCostmapGenerator();
            ~ReferenceCostmapGenerator();
        private:
            // publisher
            //// goal pose marker
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_pose_marker_;
            void publishGoalPoseArrowMarker();
            void publishGoalPoseSphereMarker();

            //// distance error map
            rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_distance_error_map_;
            void publishDistanceErrorMap();
            grid_map::GridMap distance_error_map_;

            //// reference yaw angle map
            rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_ref_yaw_map_;
            void publishReferenceYawMap();
            grid_map::GridMap ref_yaw_map_;

            // subscriber
            //// reference path
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_ref_path_;
            void refPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg);
            bool ref_path_received_;
            nav_msgs::msg::Path latest_ref_path_;
            
            //// map
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
            void mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
            bool map_received_;
            grid_map::GridMap latest_map_;

            // parameters
            double map_resolution_scale_;
    };
} // namespace planning
