#include "mppi_h/mppi_h.hpp"

#include <chrono>

namespace controller_mppi_h
{

// constructor
MPPI::MPPI()
    : rclcpp::Node("mppi_h")
{
    // load parameters
    param::CommonParam param_common; // common parameters managed by mppi_h
    param::MPPI3DParam param_mode1; // parameters for mode one
    param::MPPI4DParam param_mode2; // parameters for mode two

    //// navigation
    param_common.navigation.xy_goal_tolerance =
        this->declare_parameter<double>("navigation/xy_goal_tolerance", 0.5); // [m]
    param_common.navigation.yaw_goal_tolerance =
        this->declare_parameter<double>("navigation/yaw_goal_tolerance", 0.5); // [rad]
    
    //// target_system
    param_common.target_system.l_f = this->declare_parameter<double>("target_system/l_f", 0.5); // [m]
    param_common.target_system.l_r = this->declare_parameter<double>("target_system/l_r", 0.5); // [m]
    param_common.target_system.d_l = this->declare_parameter<double>("target_system/d_l", 0.5); // [m]
    param_common.target_system.d_r = this->declare_parameter<double>("target_system/d_r", 0.5); // [m]
    param_common.target_system.tire_radius = this->declare_parameter<double>("target_system/tire_radius", 0.2); // [m]

    //// mode_selector
    param_common.mode_selector.yaw_error_threshold =
        this->declare_parameter<double>("mode_selector/yaw_error_threshold", 0.3); // [rad]
    param_common.mode_selector.dist_error_threshold =
        this->declare_parameter<double>("mode_selector/dist_error_threshold", 0.3); // [m]

    //// controller [common]
    param_common.controller.control_interval =
        this->declare_parameter<double>("controller/common/control_interval", 0.05); // [s]
    param_common.controller.prediction_horizon =
        this->declare_parameter<int>("controller/common/prediction_horizon", 30); // prediction horizon steps
    param_common.controller.step_len_sec =
        this->declare_parameter<double>("controller/common/step_len_sec", 0.033); // step length [sec]

    //// controller [mode one]
    param_mode1.controller.name = this->declare_parameter<std::string>("controller/mode1/name", "mppi_3d");
    param_mode1.controller.num_samples = this->declare_parameter<int>("controller/mode1/num_samples", 3000);
    param_mode1.controller.param_exploration =
        this->declare_parameter<double>("controller/mode1/param_exploration", 0.1);
    param_mode1.controller.param_lambda = this->declare_parameter<double>("controller/mode1/param_lambda", 0.1);
    param_mode1.controller.param_alpha = this->declare_parameter<double>("controller/mode1/param_alpha", 0.1);
    param_mode1.controller.sigma =
        this->declare_parameter<std::vector<double>>("controller/mode1/sigma", {1.0, 1.0, 0.78});
    param_mode1.controller.reduce_computation =
        this->declare_parameter<bool>("controller/mode1/reduce_computation", false);
    param_mode1.controller.weight_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/mode1/weight_cmd_change", {0.0, 0.0, 0.0});
    param_mode1.controller.weight_vehicle_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/mode1/weight_vehicle_cmd_change",
                                                     {1.4, 1.4, 1.4, 1.4, 0.1, 0.1, 0.1, 0.1});
    param_mode1.controller.ref_velocity =
        this->declare_parameter<double>("controller/mode1/ref_velocity", 2.0);
    param_mode1.controller.weight_velocity_error =
        this->declare_parameter<double>("controller/mode1/weight_velocity_error", 10.0);
    param_mode1.controller.weight_angular_error =
        this->declare_parameter<double>("controller/mode1/weight_angular_error", 30.0);
    param_mode1.controller.weight_collision_penalty =
        this->declare_parameter<double>("controller/mode1/weight_collision_penalty", 50.0);
    param_mode1.controller.weight_distance_error_penalty =
        this->declare_parameter<double>("controller/mode1/weight_distance_error_penalty", 40.0);
    param_mode1.controller.weight_terminal_state_penalty =
        this->declare_parameter<double>("controller/mode1/weight_terminal_state_penalty", 50.0);
    param_mode1.controller.use_sg_filter =
        this->declare_parameter<bool>("controller/mode1/use_sg_filter", true);
    param_mode1.controller.sg_filter_half_window_size =
        this->declare_parameter<int>("controller/mode1/sg_filter_half_window_size", 10);
    param_mode1.controller.sg_filter_poly_order =
        this->declare_parameter<int>("controller/mode1/sg_filter_poly_order", 3);

    //// controller [mode two]
    param_mode2.controller.name = this->declare_parameter<std::string>("controller/mode2/name", "mppi_4d");
    param_mode2.controller.num_samples = this->declare_parameter<int>("controller/mode2/num_samples", 3000);
    param_mode2.controller.param_exploration =
        this->declare_parameter<double>("controller/mode2/param_exploration", 0.1);
    param_mode2.controller.param_lambda = this->declare_parameter<double>("controller/mode2/param_lambda", 0.1);
    param_mode2.controller.param_alpha = this->declare_parameter<double>("controller/mode2/param_alpha", 0.1);
    param_mode2.controller.sigma =
        this->declare_parameter<std::vector<double>>("controller/mode2/sigma", {1.0, 1.0, 0.78});
    param_mode2.controller.reduce_computation =
        this->declare_parameter<bool>("controller/mode2/reduce_computation", false);
    param_mode2.controller.weight_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/mode2/weight_cmd_change", {0.0, 0.0, 0.0});
    param_mode2.controller.weight_vehicle_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/mode2/weight_vehicle_cmd_change",
                                                     {1.4, 1.4, 1.4, 1.4, 0.1, 0.1, 0.1, 0.1});
    param_mode2.controller.ref_velocity =
        this->declare_parameter<double>("controller/mode2/ref_velocity", 2.0);
    param_mode2.controller.weight_velocity_error =
        this->declare_parameter<double>("controller/mode2/weight_velocity_error", 10.0);
    param_mode2.controller.weight_angular_error =
        this->declare_parameter<double>("controller/mode2/weight_angular_error", 30.0);
    param_mode2.controller.weight_collision_penalty =
        this->declare_parameter<double>("controller/mode2/weight_collision_penalty", 50.0);
    param_mode2.controller.weight_distance_error_penalty =
        this->declare_parameter<double>("controller/mode2/weight_distance_error_penalty", 40.0);
    param_mode2.controller.weight_terminal_state_penalty =
        this->declare_parameter<double>("controller/mode2/weight_terminal_state_penalty", 50.0);
    param_mode2.controller.use_sg_filter =
        this->declare_parameter<bool>("controller/mode2/use_sg_filter", true);
    param_mode2.controller.sg_filter_half_window_size =
        this->declare_parameter<int>("controller/mode2/sg_filter_half_window_size", 10);
    param_mode2.controller.sg_filter_poly_order =
        this->declare_parameter<int>("controller/mode2/sg_filter_poly_order", 3);

    //// subscribing topic names
    std::string odom_topic = this->declare_parameter<std::string>("odom_topic", "/groundtruth_odom");
    std::string ref_path_topic =
        this->declare_parameter<std::string>("ref_path_topic", "/move_base/NavfnROS/plan");
    std::string collision_costmap_topic =
        this->declare_parameter<std::string>("collision_costmap_topic", "/move_base/local_costmap/costmap");
    std::string distance_error_map_topic =
        this->declare_parameter<std::string>("distance_error_map_topic", "/distance_error_map");
    std::string ref_yaw_map_topic = this->declare_parameter<std::string>("ref_yaw_map_topic", "/ref_yaw_map");

    //// publishing topic names
    std::string control_cmd_vel_topic, mppi_absvel_topic, mppi_vx_topic, mppi_vy_topic, mppi_omega_topic, \
    calc_time_topic, mppi_optimal_traj_topic, mppi_sampled_traj_topic, mppi_overlay_text_topic, mpc_eval_msg_topic;
    control_cmd_vel_topic = this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");
    mppi_absvel_topic = this->declare_parameter<std::string>("mppi_absvel_topic", "/mpc/cmd/absvel");
    mppi_vx_topic = this->declare_parameter<std::string>("mppi_vx_topic", "/mpc/cmd/vx");
    mppi_vy_topic = this->declare_parameter<std::string>("mppi_vy_topic", "/mpc/cmd/vy");
    mppi_omega_topic = this->declare_parameter<std::string>("mppi_omega_topic", "/mpc/cmd/omega");
    calc_time_topic = this->declare_parameter<std::string>("calc_time_topic", "/mpc/calc_time");
    mppi_overlay_text_topic = this->declare_parameter<std::string>("mppi_overlay_text_topic", "/mpc/overlay_text");
    mppi_optimal_traj_topic = this->declare_parameter<std::string>("mppi_optimal_traj_topic", "/mpc/optimal_traj");
    mppi_sampled_traj_topic = this->declare_parameter<std::string>("mppi_sampled_traj_topic", "/mpc/sampled_traj");
    mpc_eval_msg_topic = this->declare_parameter<std::string>("mpc_eval_msg_topic", "/mpc/eval_info");

    // initialize subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        rclcpp::QoS(1),
        std::bind(&MPPI::odomCallback, this, std::placeholders::_1));
    odom_received_ = false;
    sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(
        ref_path_topic,
        rclcpp::QoS(1),
        std::bind(&MPPI::refPathCallback, this, std::placeholders::_1));
    ref_path_received_ = false;
    sub_collision_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        collision_costmap_topic,
        rclcpp::QoS(1),
        std::bind(&MPPI::collisionCostmapCallback, this, std::placeholders::_1));
    collision_costmap_received_ = false;
    sub_distance_error_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        distance_error_map_topic,
        rclcpp::QoS(1),
        std::bind(&MPPI::distanceErrorMapCallback, this, std::placeholders::_1));
    distance_error_map_received_ = false;
    sub_ref_yaw_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        ref_yaw_map_topic,
        rclcpp::QoS(1),
        std::bind(&MPPI::refYawMapCallback, this, std::placeholders::_1));
    ref_yaw_map_received_ = false;

    // initialize publishers
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(control_cmd_vel_topic, rclcpp::QoS(1));
    pub_cmd_absvel_ = this->create_publisher<std_msgs::msg::Float32>(mppi_absvel_topic, rclcpp::QoS(1));
    pub_cmd_vx_ = this->create_publisher<std_msgs::msg::Float32>(mppi_vx_topic, rclcpp::QoS(1));
    pub_cmd_vy_ = this->create_publisher<std_msgs::msg::Float32>(mppi_vy_topic, rclcpp::QoS(1));
    pub_cmd_omega_ = this->create_publisher<std_msgs::msg::Float32>(mppi_omega_topic, rclcpp::QoS(1));
    pub_mppi_calc_time_ = this->create_publisher<std_msgs::msg::Float32>(calc_time_topic, rclcpp::QoS(1));
    pub_mppi_overlay_text_ =
        this->create_publisher<jsk_rviz_plugins::msg::OverlayText>(mppi_overlay_text_topic, rclcpp::QoS(1));
    pub_mppi_optimal_traj_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(mppi_optimal_traj_topic, rclcpp::QoS(1));
    pub_mppi_sampled_traj_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(mppi_sampled_traj_topic, rclcpp::QoS(1));
    pub_mpc_eval_msg_ = this->create_publisher<mpc_eval_msgs::msg::MPCEval>(mpc_eval_msg_topic, rclcpp::QoS(1));

    // initialize timer
    timer_control_interval_ = this->create_wall_timer(
        std::chrono::duration<double>(param_common.controller.control_interval),
        std::bind(&MPPI::calcControlCommand, this));

    // instantiate MPPIHybridCore class
    mppi_hybrid_core_ = new MPPIHybridCore(std::make_tuple(param_common, param_mode1, param_mode2));
}

// destructor
MPPI::~MPPI()
{
    // No Contents
}

// callback to update odometry (global vehicle pose)
void MPPI::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    odom_received_ = true;
    latest_odom_ = *msg;

    // get latest vehicle pose
    double latest_x = latest_odom_.pose.pose.position.x;
    double latest_y = latest_odom_.pose.pose.position.y;
    double latest_yaw = tf2::getYaw(latest_odom_.pose.pose.orientation);

    // check if NaN is included
    if (std::isnan(latest_x) || std::isnan(latest_y) || std::isnan(latest_yaw))
    {
        RCLCPP_WARN(this->get_logger(), "NaN is included in the received odometry");
        return;
    }

    // update observed state
    observed_state_.x = latest_x;
    observed_state_.y = latest_y;
    observed_state_.yaw = latest_yaw;
    observed_state_.unwrap();
}

// callback to update reference path
void MPPI::refPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    ref_path_received_ = true;
    latest_ref_path_ = *msg;

    // update goal state
    int goal_idx = latest_ref_path_.poses.size() - 1;
    goal_state_.x = latest_ref_path_.poses[goal_idx].pose.position.x;
    goal_state_.y = latest_ref_path_.poses[goal_idx].pose.position.y;
    goal_state_.yaw = tf2::getYaw(latest_ref_path_.poses[goal_idx].pose.orientation);
}

// callback to update local costmap callback
void MPPI::collisionCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    collision_costmap_received_ = true;

    // convert subscribed OccupancyGrid to GridMap type
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "collision_cost", collision_costmap_);
}

// callback to update distance error map
void MPPI::distanceErrorMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
    distance_error_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, distance_error_map_);
}

// callback to update reference yaw map
void MPPI::refYawMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
    ref_yaw_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, ref_yaw_map_);
}

// callback to calculate control command
void MPPI::calcControlCommand()
{
    // check if all necessary data are received, and return if not.
    if (!odom_received_ || !ref_path_received_ || !collision_costmap_received_ || !distance_error_map_received_ || !ref_yaw_map_received_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "[MPPI] not all necessary data are received, odom: %d, ref_path: %d, collision_costmap: %d, distance_error_map: %d, ref_yaw_map: %d",
                    odom_received_,
                    ref_path_received_,
                    collision_costmap_received_,
                    distance_error_map_received_,
                    ref_yaw_map_received_);
        return;
    }

    // calculate optimal control command
    common_type::VxVyOmega optimal_cmd = 
        mppi_hybrid_core_->solveMPPI(
            observed_state_,
            collision_costmap_,
            distance_error_map_,
            ref_yaw_map_,
            goal_state_
    );

    // publish optimal control command as Twist message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = optimal_cmd.vx;
    cmd_vel.linear.y = optimal_cmd.vy;
    cmd_vel.angular.z = optimal_cmd.omega;
    pub_cmd_vel_->publish(cmd_vel);

    // publish rviz markers for visualization
    //// publish optimal trajectory
    publishOptimalTrajectory(mppi_hybrid_core_->getOptimalTrajectory());
    //// publish sampled trajectories
    ////// publishSampledTrajectories(mppi_hybrid_core_->getFullSampledTrajectories()); // visualize all sampled trajectories
    publishSampledTrajectories(mppi_hybrid_core_->getEliteSampledTrajectories(100)); // visualize top 100 sampled trajectories

    // publish mppi calculation time
    std_msgs::msg::Float32 calc_time;
    calc_time.data = mppi_hybrid_core_->getCalcTime();
    pub_mppi_calc_time_->publish(calc_time);

    // publish overlay text
    publishOverlayText(mppi_hybrid_core_->getControllerName());

    // publish velocity command info
    std_msgs::msg::Float32 absvel, vx, vy, omega;
    absvel.data = sqrt(pow(optimal_cmd.vx, 2) + pow(optimal_cmd.vy, 2));
    vx.data = optimal_cmd.vx;
    vy.data = optimal_cmd.vy;
    omega.data = optimal_cmd.omega;
    pub_cmd_absvel_->publish(absvel);
    pub_cmd_vx_->publish(vx);
    pub_cmd_vy_->publish(vy);
    pub_cmd_omega_->publish(omega);

    // publish mpc evaluation info
    mpc_eval_msgs::msg::MPCEval mpc_eval_msg;
    mpc_eval_msg.header.stamp = this->now();
    mpc_eval_msg.header.frame_id = mppi_hybrid_core_->getControllerName();
    mpc_eval_msg.state_cost = mppi_hybrid_core_->getStateCost();
    mpc_eval_msg.global_x = observed_state_.x;
    mpc_eval_msg.global_y = observed_state_.y;
    mpc_eval_msg.global_yaw = observed_state_.yaw;
    mpc_eval_msg.cmd_vx = optimal_cmd.vx;
    mpc_eval_msg.cmd_vy = optimal_cmd.vy;
    mpc_eval_msg.cmd_yawrate = optimal_cmd.omega;
    common_type::VehicleCommand8D optimal_vehicle_cmd = mppi_hybrid_core_->getOptimalVehicleCommand();
    mpc_eval_msg.cmd_steer_fl = optimal_vehicle_cmd.steer_fl;
    mpc_eval_msg.cmd_steer_fr = optimal_vehicle_cmd.steer_fr;
    mpc_eval_msg.cmd_steer_rl = optimal_vehicle_cmd.steer_rl;
    mpc_eval_msg.cmd_steer_rr = optimal_vehicle_cmd.steer_rr;
    mpc_eval_msg.cmd_rotor_fl = optimal_vehicle_cmd.rotor_fl;
    mpc_eval_msg.cmd_rotor_fr = optimal_vehicle_cmd.rotor_fr;
    mpc_eval_msg.cmd_rotor_rl = optimal_vehicle_cmd.rotor_rl;
    mpc_eval_msg.cmd_rotor_rr = optimal_vehicle_cmd.rotor_rr;
    mpc_eval_msg.calc_time_ms = mppi_hybrid_core_->getCalcTime();
    mpc_eval_msg.goal_reached = mppi_hybrid_core_->isGoalReached();
    pub_mpc_eval_msg_->publish(mpc_eval_msg);
}

// publish rviz markers to visualize optimal trajectory with arrow markers
void MPPI::publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.41; // [m]
    //// arrow scale (x, y, z)
    double arrow_scale[3] = {0.10, 0.03, 0.03};
    //// arrow color (red, green, blue, alpha)
    double arrow_color[4] = {1.0, 0.0, 0.0, 1.0};

    // create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    // get number of time steps
    int T = optimal_xyyaw_sequence.size();
    marker_array.markers.resize(T);

    // for each time step, add an arrow marker
    for (int t = 0; t < T; t++)
    {
        double x = optimal_xyyaw_sequence[t].x;
        double y = optimal_xyyaw_sequence[t].y;
        double yaw = optimal_xyyaw_sequence[t].yaw;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw); // Note: assume roll and pitch angles are zero

        marker_array.markers[t].header.frame_id = "map";
        marker_array.markers[t].header.stamp = this->now();
        marker_array.markers[t].ns = "optimal_trajectory";
        marker_array.markers[t].id = t;
        marker_array.markers[t].type = visualization_msgs::msg::Marker::ARROW;
        marker_array.markers[t].action = visualization_msgs::msg::Marker::ADD;
        marker_array.markers[t].pose.position.x = x;
        marker_array.markers[t].pose.position.y = y;
        marker_array.markers[t].pose.position.z = MARKER_POS_Z;
        marker_array.markers[t].pose.orientation.x = q.x();
        marker_array.markers[t].pose.orientation.y = q.y();
        marker_array.markers[t].pose.orientation.z = q.z();
        marker_array.markers[t].pose.orientation.w = q.w();
        marker_array.markers[t].scale.x = arrow_scale[0];
        marker_array.markers[t].scale.y = arrow_scale[1];
        marker_array.markers[t].scale.z = arrow_scale[2];
        marker_array.markers[t].color.r = arrow_color[0];
        marker_array.markers[t].color.g = arrow_color[1];
        marker_array.markers[t].color.b = arrow_color[2];
        marker_array.markers[t].color.a = arrow_color[3];
    }

    // publish rviz markers
    pub_mppi_optimal_traj_->publish(marker_array);
}

// publish rviz markers to visualize sampled trajectories
void MPPI::publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.39; // [m]
    //// line lifetime [s]
    double line_lifetime = 0.1;
    //// arrow scale (x, y, z)
    double line_width = 0.01;
    //// line color (red, green, blue, alpha)
    double line_color[4] = {0.0, 0.35, 1.0, 0.5};

    // get number of samples
    int K = sampled_state_sequences.size();
    int T = sampled_state_sequences[0].size();

    // create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.resize(K);

    // for each sampled state sequence, add an line strip marker
    for (int k = 0; k < K; k++)
    {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = "map";
        line.header.stamp = this->now();
        line.ns = "sampled_trajectories";
        line.id = k;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;
        line.scale.x = line_width;
        line.color.r = line_color[0];
        line.color.g = line_color[1];
        line.color.b = line_color[2];
        line.color.a = line_color[3];
        line.lifetime = rclcpp::Duration::from_seconds(line_lifetime).to_builtin_msg();
        line.points.resize(T);

        // for each time step, add a point to the line strip marker
        for (int t = 0; t < T; t++)
        {
            // add a point to the line strip marker
            line.points[t].x = sampled_state_sequences[k][t].x;
            line.points[t].y = sampled_state_sequences[k][t].y;
            line.points[t].z = MARKER_POS_Z;
        }

        marker_array.markers[k] = line;
    }

    // publish rviz markers
    pub_mppi_sampled_traj_->publish(marker_array);
}

// publish overlay text for visualization on rviz
void MPPI::publishOverlayText(const std::string& text)
{
    jsk_rviz_plugins::msg::OverlayText text_msg;
    text_msg.action = jsk_rviz_plugins::msg::OverlayText::ADD;
    text_msg.width = 500;
    text_msg.height = 50;
    text_msg.left = 0;
    text_msg.top = 50;

    std_msgs::msg::ColorRGBA color1, color2;
    color1.r = 0;
    color1.g = 0;
    color1.b = 0;
    color1.a = 0.4;
    text_msg.bg_color = color1;

    color2.r = 25.0 / 255;
    color2.g = 255.0 / 255;
    color2.b = 240.0 / 255;
    color2.a = 0.8;
    text_msg.fg_color = color2;

    text_msg.line_width = 1;
    text_msg.text_size = 22;
    text_msg.font = "Ubuntu Mono";
    text_msg.text = text;

    pub_mppi_overlay_text_->publish(text_msg);
}

} // namespace controller_mppi_h
