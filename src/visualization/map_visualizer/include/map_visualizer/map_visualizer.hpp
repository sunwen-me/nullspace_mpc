#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace visualization
{
    class MapVisualizer : public rclcpp::Node
    {
        public:
            MapVisualizer();
            ~MapVisualizer();
        private:
            // publisher
            //// rviz 3D map
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_3dmap_;
            void publishRviz3DMapOfMaze(); // for map: maze
            void publishRviz3DMapOfCylinderGarden(); // for map: cylinder_garden
    };
} // namespace visualization
