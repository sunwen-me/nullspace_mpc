#include "reference_costmap_generator/reference_costmap_generator.hpp"

namespace planning
{
    ReferenceCostmapGenerator::ReferenceCostmapGenerator()
        : rclcpp::Node("reference_costmap_generator")
    {
        // load parameters 
        map_resolution_scale_ = this->declare_parameter<double>("map_resolution_scale", 1.0);

        //// subscribing topic names
        std::string ref_path_topic =
            this->declare_parameter<std::string>("ref_path_topic", "/move_base/NavfnROS/plan");
        std::string map_topic = this->declare_parameter<std::string>("map_topic", "/map");

        //// publishing topic names
        std::string goal_pose_marker_topic =
            this->declare_parameter<std::string>("goal_pose_marker_topic", "/goal_pose_marker");
        std::string distance_error_map_topic =
            this->declare_parameter<std::string>("distance_error_map_topic", "/distance_error_map");
        std::string ref_yaw_map_topic =
            this->declare_parameter<std::string>("ref_yaw_map_topic", "/ref_yaw_map");

        // initialize subscribers
        sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(
            ref_path_topic,
            rclcpp::QoS(1),
            std::bind(&ReferenceCostmapGenerator::refPathCallback, this, std::placeholders::_1));
        sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            rclcpp::QoS(1),
            std::bind(&ReferenceCostmapGenerator::mapCallback, this, std::placeholders::_1));

        // initialize publishers
        pub_goal_pose_marker_ =
            this->create_publisher<visualization_msgs::msg::Marker>(goal_pose_marker_topic, rclcpp::QoS(10));
        pub_distance_error_map_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>(distance_error_map_topic, rclcpp::QoS(10));
        pub_ref_yaw_map_ =
            this->create_publisher<grid_map_msgs::msg::GridMap>(ref_yaw_map_topic, rclcpp::QoS(10));
    }

    ReferenceCostmapGenerator::~ReferenceCostmapGenerator()
    {
        // No Contents
    }

    void ReferenceCostmapGenerator::refPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        ref_path_received_ = true;

        // save the latest reference path as nav_msgs::Path type
        latest_ref_path_ = *msg;

        // publish goal pose marker
        publishGoalPoseArrowMarker(); // publish arrow marker
        // publishGoalPoseSphereMarker(); // publish sphere marker

        if (!map_received_ || !ref_path_received_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "[ReferenceCostmapGenerator] not all necessary data are received, map: %d, ref_path: %d",
                        map_received_,
                        ref_path_received_);
            return;
        }

        // publish distance error map
        publishDistanceErrorMap();

        // publish reference yaw map
        publishReferenceYawMap();
    }

    void ReferenceCostmapGenerator::mapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
    {
        map_received_ = true;

        // save the latest map as grid_map::GridMap type
        grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "map", latest_map_);
    }

    void ReferenceCostmapGenerator::publishGoalPoseArrowMarker()
    {
        // constant params
        //// arrow height
        double MARKER_POS_Z = 0.5; // [m]
        //// arrow scale (x, y, z)
        double arrow_scale[3] = {0.6, 0.15, 0.15};
        //// arrow color (red, green, blue, alpha)
        double arrow_color[4] = {0.0, 0.0, 1.0, 1.0};

        // publish goal pose marker
        visualization_msgs::msg::Marker goal_pose_marker;
        goal_pose_marker.header.frame_id = "map";
        goal_pose_marker.header.stamp = this->now();
        goal_pose_marker.ns = "goal_pose";
        goal_pose_marker.id = 0;
        goal_pose_marker.type = visualization_msgs::msg::Marker::ARROW;
        goal_pose_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_pose_marker.pose.position.x = latest_ref_path_.poses.back().pose.position.x;
        goal_pose_marker.pose.position.y = latest_ref_path_.poses.back().pose.position.y;
        goal_pose_marker.pose.position.z = MARKER_POS_Z;
        goal_pose_marker.pose.orientation = latest_ref_path_.poses.back().pose.orientation;
        goal_pose_marker.scale.x = arrow_scale[0];
        goal_pose_marker.scale.y = arrow_scale[1];
        goal_pose_marker.scale.z = arrow_scale[2];
        goal_pose_marker.color.r = arrow_color[0];
        goal_pose_marker.color.g = arrow_color[1];
        goal_pose_marker.color.b = arrow_color[2];
        goal_pose_marker.color.a = arrow_color[3];
        pub_goal_pose_marker_->publish(goal_pose_marker);
    }

    void ReferenceCostmapGenerator::publishGoalPoseSphereMarker()
    {
        // constant params
        double MARKER_POS_Z = 0.5; // [m]
        //// sphere scale in radius
        double sphere_scale = 0.5; // [m]
        //// arrow color (red, green, blue, alpha)
        double sphere_color[4] = {0.0, 0.0, 1.0, 1.0};
    
        // publish goal pose marker
        visualization_msgs::msg::Marker goal_pose_marker;
        goal_pose_marker.header.frame_id = "map";
        goal_pose_marker.header.stamp = this->now();
        goal_pose_marker.ns = "goal_pose";
        goal_pose_marker.id = 0;
        goal_pose_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_pose_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_pose_marker.pose.position.x = latest_ref_path_.poses.back().pose.position.x;
        goal_pose_marker.pose.position.y = latest_ref_path_.poses.back().pose.position.y;
        goal_pose_marker.pose.position.z = MARKER_POS_Z;
        goal_pose_marker.pose.orientation = latest_ref_path_.poses.back().pose.orientation; // optional for sphere
        goal_pose_marker.scale.x = sphere_scale;
        goal_pose_marker.scale.y = sphere_scale;
        goal_pose_marker.scale.z = sphere_scale;
        goal_pose_marker.color.r = sphere_color[0];
        goal_pose_marker.color.g = sphere_color[1];
        goal_pose_marker.color.b = sphere_color[2];
        goal_pose_marker.color.a = sphere_color[3];
        pub_goal_pose_marker_->publish(goal_pose_marker);
    }

    void ReferenceCostmapGenerator::publishDistanceErrorMap()
    {
        // generate distance error map
        grid_map::GridMap distance_error_map_({"distance_error"});
        distance_error_map_.setFrameId(latest_map_.getFrameId()); // usually "map"
        distance_error_map_.setGeometry(latest_map_.getLength(), latest_map_.getResolution() * map_resolution_scale_);
        rclcpp::Time time = this->now();
        distance_error_map_.setTimestamp(time.nanoseconds());

        // for each cell in the map, calculate the distance to the nearest point in the reference path
        const auto grid_size = distance_error_map_.getSize();
        const unsigned int linear_grid_size = grid_size.prod();

        // create map
        //// you can use OpenMP for parallel processing if necessary, set ON/OFF in CMakeLists.txt
        #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(1)
        for (int i = 0; i < linear_grid_size; i++){
            // get current index
            const grid_map::Index index(grid_map::getIndexFromLinearIndex(i, grid_size));

            // get the position of the cell
            grid_map::Position position;

            // get position using int i
            distance_error_map_.getPosition(index, position);

            // calculate the distance to the nearest point in the reference path
            double distance = std::numeric_limits<double>::max();
            for (int i = 0; i < latest_ref_path_.poses.size(); i++)
            {
                // calculate the distance to the point i in the reference path
                double dx = position.x() - latest_ref_path_.poses[i].pose.position.x;
                double dy = position.y() - latest_ref_path_.poses[i].pose.position.y;
                double d = sqrt(dx * dx + dy * dy);

                // if the current target point i is the nearest point so far
                if (d < distance)
                {
                    // update the minimum distance
                    distance = d;
                }
            }
            // set the distance to the distance error map
            distance_error_map_.at("distance_error", index) = distance;
        }

        // publish distance error map
        grid_map_msgs::msg::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(distance_error_map_, msg);
        pub_distance_error_map_->publish(msg);
    }

    void ReferenceCostmapGenerator::publishReferenceYawMap()
    {
        // generate reference yaw map from the reference path, yaw range is [0, 2*pi]
        grid_map::GridMap ref_yaw_map_({"ref_yaw"});
        ref_yaw_map_.setFrameId(latest_map_.getFrameId()); // usually "map"
        ref_yaw_map_.setGeometry(latest_map_.getLength(), latest_map_.getResolution() * map_resolution_scale_);
        rclcpp::Time time = this->now();
        ref_yaw_map_.setTimestamp(time.nanoseconds());

        // for each cell in the map, calculate the yaw angle of the nearest point in the reference path
        const auto grid_size = ref_yaw_map_.getSize();
        const unsigned int linear_grid_size = grid_size.prod();

        // create map
        //// you can use OpenMP for parallel processing if necessary, set ON/OFF in CMakeLists.txt
        #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(1)
        for (int i = 0; i < linear_grid_size; i++){
            // get current index
            const grid_map::Index index(grid_map::getIndexFromLinearIndex(i, grid_size));

            // get the position of the cell
            grid_map::Position position;
            ref_yaw_map_.getPosition(index, position);

            // calculate the yaw angle of the nearest point in the reference path
            double distance = std::numeric_limits<double>::max();
            double yaw = std::numeric_limits<double>::max();
            for (int i = 0; i < latest_ref_path_.poses.size(); i++)
            {
                double dx = position.x() - latest_ref_path_.poses[i].pose.position.x;
                double dy = position.y() - latest_ref_path_.poses[i].pose.position.y;
                double d = sqrt(dx * dx + dy * dy);

                // if the current target point i is the nearest point so far
                if (d < distance && 0 < i)
                {
                    // update the minimum distance
                    distance = d;

                    // calculate the yaw angle, between the pose[i] and pose[i-1], using arctan2 function
                    double n_dx = latest_ref_path_.poses[i].pose.position.x - latest_ref_path_.poses[i-1].pose.position.x;
                    double n_dy = latest_ref_path_.poses[i].pose.position.y - latest_ref_path_.poses[i-1].pose.position.y;
                    yaw = atan2(n_dy, n_dx); // range is [-pi, pi]
                    // change yaw range to [0, 2*pi]
                    if (yaw < 0)
                    {
                        yaw += 2 * M_PI;
                    }
                }
            }
            // set the yaw angle to the reference yaw map
            ref_yaw_map_.at("ref_yaw", index) = yaw;
        }

        // publish reference yaw map
        grid_map_msgs::msg::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(ref_yaw_map_, msg);
        pub_ref_yaw_map_->publish(msg);
    }


} // namespace planning
