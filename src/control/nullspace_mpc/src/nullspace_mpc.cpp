#include "nullspace_mpc/nullspace_mpc.hpp"

#include <chrono>

namespace controller
{

// constructor
MPC::MPC()
    : rclcpp::Node("nullspace_mpc")
{
    // load parameters
    param::Param param;
    //// navigation
    param.navigation.xy_goal_tolerance = this->declare_parameter<double>("navigation/xy_goal_tolerance", 0.5); // [m]
    param.navigation.yaw_goal_tolerance = this->declare_parameter<double>("navigation/yaw_goal_tolerance", 0.5); // [rad]
    param.navigation.goal_snap_distance_for_via_pos =
        this->declare_parameter<double>("navigation/goal_snap_distance_for_via_pos", 0.1); // [m]
    param.navigation.goal_snap_distance_for_via_angle =
        this->declare_parameter<double>("navigation/goal_snap_distance_for_via_angle", 0.5); // [m]

    //// target_system
    param.target_system.l_f = this->declare_parameter<double>("target_system/l_f", 0.5); // [m]
    param.target_system.l_r = this->declare_parameter<double>("target_system/l_r", 0.5); // [m]
    param.target_system.d_l = this->declare_parameter<double>("target_system/d_l", 0.5); // [m]
    param.target_system.d_r = this->declare_parameter<double>("target_system/d_r", 0.5); // [m]
    param.target_system.tire_radius = this->declare_parameter<double>("target_system/tire_radius", 0.2); // [m]

    //// controller
    param.controller.name = this->declare_parameter<std::string>("controller/name", "nullspace_mpc");
    param.controller.control_interval = this->declare_parameter<double>("controller/control_interval", 0.05); // [s]
    param.controller.num_samples = this->declare_parameter<int>("controller/num_samples", 3000); // number of samples
    param.controller.prediction_horizon =
        this->declare_parameter<int>("controller/prediction_horizon", 30); // prediction horizon steps
    param.controller.step_len_sec = this->declare_parameter<double>("controller/step_len_sec", 0.033); // step length [sec]
    param.controller.param_exploration = this->declare_parameter<double>("controller/param_exploration", 0.1); // 0.0 ~ 1.0
    param.controller.param_lambda = this->declare_parameter<double>("controller/param_lambda", 0.1);
    param.controller.param_alpha = this->declare_parameter<double>("controller/param_alpha", 0.1);
    param.controller.sigma =
        this->declare_parameter<std::vector<double>>("controller/sigma", {1.0, 1.0, 0.78}); // for {vx, vy, yawrate}
    param.controller.idx_via_states = this->declare_parameter<std::vector<int>>("controller/idx_via_states",
                                                                               {5, 10, 15, 20, 25}); // index of via states
    param.controller.reduce_computation = this->declare_parameter<bool>("controller/reduce_computation", false);
    param.controller.weight_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/weight_cmd_change", {0.0, 0.0, 0.0});
    param.controller.weight_vehicle_cmd_change =
        this->declare_parameter<std::vector<double>>("controller/weight_vehicle_cmd_change",
                                                     {1.4, 1.4, 1.4, 1.4, 0.1, 0.1, 0.1, 0.1});
    param.controller.ref_velocity = this->declare_parameter<double>("controller/ref_velocity", 2.0); // [m/s]
    param.controller.weight_velocity_error = this->declare_parameter<double>("controller/weight_velocity_error", 10.0);
    param.controller.weight_angular_error = this->declare_parameter<double>("controller/weight_angular_error", 30.0);
    param.controller.weight_collision_penalty =
        this->declare_parameter<double>("controller/weight_collision_penalty", 50.0);
    param.controller.weight_distance_error_penalty =
        this->declare_parameter<double>("controller/weight_distance_error_penalty", 40.0);
    param.controller.weight_terminal_state_penalty =
        this->declare_parameter<double>("controller/weight_terminal_state_penalty", 50.0);
    param.controller.use_sg_filter = this->declare_parameter<bool>("controller/use_sg_filter", true);
    param.controller.sg_filter_half_window_size =
        this->declare_parameter<int>("controller/sg_filter_half_window_size", 10);
    param.controller.sg_filter_poly_order = this->declare_parameter<int>("controller/sg_filter_poly_order", 3);

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
    calc_time_topic, mppi_optimal_traj_topic, mppi_sampled_traj_topic, mppi_via_state_seq_topic, mppi_overlay_text_topic, mpc_eval_msg_topic;
    control_cmd_vel_topic = this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");
    mppi_absvel_topic = this->declare_parameter<std::string>("mppi_absvel_topic", "/mpc/cmd/absvel");
    mppi_vx_topic = this->declare_parameter<std::string>("mppi_vx_topic", "/mpc/cmd/vx");
    mppi_vy_topic = this->declare_parameter<std::string>("mppi_vy_topic", "/mpc/cmd/vy");
    mppi_omega_topic = this->declare_parameter<std::string>("mppi_omega_topic", "/mpc/cmd/omega");
    calc_time_topic = this->declare_parameter<std::string>("calc_time_topic", "/mpc/calc_time");
    mppi_overlay_text_topic = this->declare_parameter<std::string>("mppi_overlay_text_topic", "/mpc/overlay_text");
    mppi_optimal_traj_topic = this->declare_parameter<std::string>("mppi_optimal_traj_topic", "/mpc/optimal_traj");
    mppi_sampled_traj_topic = this->declare_parameter<std::string>("mppi_sampled_traj_topic", "/mpc/sampled_traj");
    mppi_via_state_seq_topic = this->declare_parameter<std::string>("mppi_via_state_seq_topic", "/mpc/via_state_seq");
    mpc_eval_msg_topic = this->declare_parameter<std::string>("mpc_eval_msg_topic", "/mpc/eval_info");

    // initialize subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        rclcpp::QoS(1),
        std::bind(&MPC::odomCallback, this, std::placeholders::_1));
    odom_received_ = false;
    sub_ref_path_ = this->create_subscription<nav_msgs::msg::Path>(
        ref_path_topic,
        rclcpp::QoS(1),
        std::bind(&MPC::refPathCallback, this, std::placeholders::_1));
    ref_path_received_ = false;
    sub_collision_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        collision_costmap_topic,
        rclcpp::QoS(1),
        std::bind(&MPC::collisionCostmapCallback, this, std::placeholders::_1));
    collision_costmap_received_ = false;
    sub_distance_error_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        distance_error_map_topic,
        rclcpp::QoS(1),
        std::bind(&MPC::distanceErrorMapCallback, this, std::placeholders::_1));
    distance_error_map_received_ = false;
    sub_ref_yaw_map_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        ref_yaw_map_topic,
        rclcpp::QoS(1),
        std::bind(&MPC::refYawMapCallback, this, std::placeholders::_1));
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
    pub_mppi_via_state_sequence_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(mppi_via_state_seq_topic, rclcpp::QoS(1));
    pub_mpc_eval_msg_ = this->create_publisher<mpc_eval_msgs::msg::MPCEval>(mpc_eval_msg_topic, rclcpp::QoS(1));

    // initialize timer
    timer_control_interval_ = this->create_wall_timer(
        std::chrono::duration<double>(param.controller.control_interval),
        std::bind(&MPC::calcControlCommand, this));

    // set parameters
    lookahead_distance_ = param.controller.ref_velocity * param.controller.step_len_sec * param.controller.prediction_horizon; // [m]
    idx_via_states_ = param.controller.idx_via_states; // index of via states
    goal_snap_distance_for_via_pos_ = param.navigation.goal_snap_distance_for_via_pos; // [m]
    goal_snap_distance_for_via_angle_ = param.navigation.goal_snap_distance_for_via_angle; // [rad]

    // instantiate MPCCore class
    mpc_core_ = new controller::MPCCore(param);
}

// destructor
MPC::~MPC()
{
    // No Contents
}

// callback to update odometry (global vehicle pose)
void MPC::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    odom_received_ = true;
    latest_odom_ = *msg;

    // get latest vehicle pose
    double latest_x = latest_odom_.pose.pose.position.x;
    double latest_y = latest_odom_.pose.pose.position.y;
    double latest_yaw = tf2::getYaw(latest_odom_.pose.pose.orientation);
    
    // get latest vehicle velocity
    double latest_vx = latest_odom_.twist.twist.linear.x;
    double latest_vy = latest_odom_.twist.twist.linear.y;
    double latest_omega = latest_odom_.twist.twist.angular.z;

    // check if NaN is included
    if (std::isnan(latest_x) || std::isnan(latest_y) || std::isnan(latest_yaw) ||
        std::isnan(latest_vx) || std::isnan(latest_vy) || std::isnan(latest_omega))
    {
        // log warning and return
        RCLCPP_WARN(this->get_logger(), "NaN is included in the received odometry");
        RCLCPP_WARN(this->get_logger(), "latest_x: %f, latest_y: %f, latest_yaw: %f", latest_x, latest_y, latest_yaw);
        RCLCPP_WARN(this->get_logger(), "latest_vx: %f, latest_vy: %f, latest_omega: %f", latest_vx, latest_vy, latest_omega);
        return;
    }

    // update observed state
    observed_state_.x = latest_x;
    observed_state_.y = latest_y;
    observed_state_.yaw = latest_yaw;
    observed_state_.unwrap();

    // update observed velocity
    observed_velocity_.vx = latest_vx;
    observed_velocity_.vy = latest_vy;
    observed_velocity_.omega = latest_omega;
}

// callback to update reference path
void MPC::refPathCallback(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
    ref_path_received_ = true;
    latest_ref_path_ = *msg;

    // update goal state
    int goal_idx = latest_ref_path_.poses.size() - 1;
    global_goal_state_.x = latest_ref_path_.poses[goal_idx].pose.position.x;
    global_goal_state_.y = latest_ref_path_.poses[goal_idx].pose.position.y;
    global_goal_state_.yaw = tf2::getYaw(latest_ref_path_.poses[goal_idx].pose.orientation);
}

// callback to update local costmap callback
void MPC::collisionCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    collision_costmap_received_ = true;

    // convert subscribed OccupancyGrid to GridMap type
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "collision_cost", collision_costmap_);
}

// callback to update distance error map
void MPC::distanceErrorMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
    distance_error_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, distance_error_map_);
}

// callback to update reference yaw map
void MPC::refYawMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr msg)
{
    ref_yaw_map_received_ = true;

    // convert subscribed GridMap to GridMap type
    grid_map::GridMapRosConverter::fromMessage(*msg, ref_yaw_map_);
}

// calculate via state sequence
std::vector<common_type::XYYaw> MPC::calcViaStateSequence(
    const common_type::XYYaw& observed_state,
    const nav_msgs::msg::Path& ref_path,
    const double lookahead_distance,
    const std::vector<int>& idx_via_states
)
{
    std::vector<common_type::XYYaw> via_states;

    if (ref_path.poses.empty() || idx_via_states.empty()) {
        throw std::invalid_argument("Empty reference path or via state index list.");
    }

    // 1. find the closest point on the reference path
    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = -1;
    for (size_t i = 0; i < ref_path.poses.size(); ++i)
    {
        double dx = ref_path.poses[i].pose.position.x - observed_state.x;
        double dy = ref_path.poses[i].pose.position.y - observed_state.y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_idx = static_cast<int>(i);
        }
    }

    if (closest_idx == -1) {
        throw std::runtime_error("No closest point found on the reference path.");
    }

    // 2. efficiently find each via state using cached progress
    int max_idx = idx_via_states.back();
    double accumulated_dist = 0.0;
    size_t current_idx = closest_idx;

    for (int step : idx_via_states)
    {
        double target_distance = (static_cast<double>(step) / max_idx) * lookahead_distance;

        // resume scan from where we left off
        while (current_idx < ref_path.poses.size() - 1)
        {
            const auto& p1 = ref_path.poses[current_idx].pose.position;
            const auto& p2 = ref_path.poses[current_idx + 1].pose.position;
            double segment_length = std::hypot(p2.x - p1.x, p2.y - p1.y);

            if (accumulated_dist + segment_length >= target_distance)
                break;

            accumulated_dist += segment_length;
            ++current_idx;
        }

        // use direction from previous to current point
        double via_x   = 0.0;
        double via_y   = 0.0;
        double via_yaw = 0.0;

        // helper: squared distance from (x, y) to the global goal
        auto dist2_to_goal = [&](double x, double y) {
            const double dx = global_goal_state_.x - x;
            const double dy = global_goal_state_.y - y;
            return dx * dx + dy * dy;
        };

        if (current_idx < ref_path.poses.size() - 1) {
            const auto& prev = ref_path.poses[current_idx - 1].pose.position;
            const auto& curr = ref_path.poses[current_idx].pose.position;

            // decide via position
            if (dist2_to_goal(via_x, via_y) < goal_snap_distance_for_via_pos_ * goal_snap_distance_for_via_pos_) {
                // use goal position when close to goal
                via_x = global_goal_state_.x;
                via_y = global_goal_state_.y;
            } else {
                via_x = curr.x;
                via_y = curr.y;
            }

            // decide via yaw
            if (dist2_to_goal(via_x, via_y) < goal_snap_distance_for_via_angle_ * goal_snap_distance_for_via_angle_) {
                // use goal angle when close to goal
                via_yaw = global_goal_state_.yaw;
            } else {
                via_yaw = std::atan2(curr.y - prev.y, curr.x - prev.x);
            }
        }
        else // when the end of the path is reached
        {
            via_x   = global_goal_state_.x;
            via_y   = global_goal_state_.y;
            via_yaw = global_goal_state_.yaw;
        }

        common_type::XYYaw via;
        via.x   = via_x;
        via.y   = via_y;
        via.yaw = via_yaw;
        via_states.push_back(via);
    }

    return via_states;
}

// callback to calculate control command
void MPC::calcControlCommand()
{

    // check if all necessary data are received, and return if not.
    if (!odom_received_ || !ref_path_received_ || !collision_costmap_received_ || !distance_error_map_received_ || !ref_yaw_map_received_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "[MPC] not all necessary data are received, odom: %d, ref_path: %d, collision_costmap: %d, distance_error_map: %d, ref_yaw_map: %d",
                    odom_received_,
                    ref_path_received_,
                    collision_costmap_received_,
                    distance_error_map_received_,
                    ref_yaw_map_received_);
        return;
    }

    // calculate the via state sequence
    std::vector<common_type::XYYaw> via_state_sequence = calcViaStateSequence(
        observed_state_,
        latest_ref_path_,
        lookahead_distance_,
        idx_via_states_
    );

    // calculate optimal control command
    common_type::VxVyOmega optimal_cmd = 
        mpc_core_->solveMPC(
            observed_state_,
            observed_velocity_,
            collision_costmap_,
            distance_error_map_,
            ref_yaw_map_,
            via_state_sequence,
            global_goal_state_
    );

    // publish optimal control command as Twist message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = optimal_cmd.vx;
    cmd_vel.linear.y = optimal_cmd.vy;
    cmd_vel.angular.z = optimal_cmd.omega;
    pub_cmd_vel_->publish(cmd_vel);

    // publish rviz markers for visualization
    //// publish via state sequence
    publishViaStateSequence(mpc_core_->getViaStateSequence());
    //// publish optimal trajectory
    publishOptimalTrajectory(mpc_core_->getOptimalTrajectory());
    //// publish sampled trajectories
    publishSampledTrajectories(mpc_core_->getFullSampledTrajectories()); // visualize all sampled trajectories
    //// publishSampledTrajectories(mpc_core_->getEliteSampledTrajectories(100)); // visualize top 100 sampled trajectories

    // publish mppi calculation time
    std_msgs::msg::Float32 calc_time;
    calc_time.data = mpc_core_->getCalcTime();
    pub_mppi_calc_time_->publish(calc_time);

    // publish overlay text
    publishOverlayText(mpc_core_->getControllerName());

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
    mpc_eval_msg.header.frame_id = mpc_core_->getControllerName();
    mpc_eval_msg.state_cost = mpc_core_->getStateCost();
    mpc_eval_msg.global_x = observed_state_.x;
    mpc_eval_msg.global_y = observed_state_.y;
    mpc_eval_msg.global_yaw = observed_state_.yaw;
    mpc_eval_msg.cmd_vx = optimal_cmd.vx;
    mpc_eval_msg.cmd_vy = optimal_cmd.vy;
    mpc_eval_msg.cmd_yawrate = optimal_cmd.omega;
    common_type::VehicleCommand8D optimal_vehicle_cmd = mpc_core_->getOptimalVehicleCommand();
    mpc_eval_msg.cmd_steer_fl = optimal_vehicle_cmd.steer_fl;
    mpc_eval_msg.cmd_steer_fr = optimal_vehicle_cmd.steer_fr;
    mpc_eval_msg.cmd_steer_rl = optimal_vehicle_cmd.steer_rl;
    mpc_eval_msg.cmd_steer_rr = optimal_vehicle_cmd.steer_rr;
    mpc_eval_msg.cmd_rotor_fl = optimal_vehicle_cmd.rotor_fl;
    mpc_eval_msg.cmd_rotor_fr = optimal_vehicle_cmd.rotor_fr;
    mpc_eval_msg.cmd_rotor_rl = optimal_vehicle_cmd.rotor_rl;
    mpc_eval_msg.cmd_rotor_rr = optimal_vehicle_cmd.rotor_rr;
    mpc_eval_msg.calc_time_ms = mpc_core_->getCalcTime();
    mpc_eval_msg.goal_reached = mpc_core_->isGoalReached();
    pub_mpc_eval_msg_->publish(mpc_eval_msg);
}

// publish rviz markers to visualize via state sequence as arrows
void MPC::publishViaStateSequence(const std::vector<common_type::XYYaw>& via_state_sequence)
{
    // constant params
    double ARROW_SCALE[3] = {0.225, 0.045, 0.045}; // smaller arrow: {0.10, 0.03, 0.03};
    double ARROW_COLOR[4] = {0.0, 1.0, 0.0, 1.0}; // (r, g, b, a)
    double ARROW_POS_Z = 0.4; // [m]
    double marker_lifetime = 1.5; // [s]

    // create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    int VT = via_state_sequence.size();
    marker_array.markers.resize(VT);

    for (int vt = 0; vt < VT; vt++)
    {
        double x = via_state_sequence[vt].x;
        double y = via_state_sequence[vt].y;
        double yaw = via_state_sequence[vt].yaw;

        marker_array.markers[vt].header.frame_id = "map";
        marker_array.markers[vt].header.stamp = this->now();
        marker_array.markers[vt].ns = "via_states";
        marker_array.markers[vt].id = vt;
        marker_array.markers[vt].type = visualization_msgs::msg::Marker::ARROW;
        marker_array.markers[vt].action = visualization_msgs::msg::Marker::ADD;

        // Set the pose
        marker_array.markers[vt].pose.position.x = x;
        marker_array.markers[vt].pose.position.y = y;
        marker_array.markers[vt].pose.position.z = ARROW_POS_Z;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        marker_array.markers[vt].pose.orientation.x = quat.x();
        marker_array.markers[vt].pose.orientation.y = quat.y();
        marker_array.markers[vt].pose.orientation.z = quat.z();
        marker_array.markers[vt].pose.orientation.w = quat.w();

        // Set the scale: x = length, y = shaft diameter, z = head diameter
        marker_array.markers[vt].scale.x = ARROW_SCALE[0];
        marker_array.markers[vt].scale.y = ARROW_SCALE[1];
        marker_array.markers[vt].scale.z = ARROW_SCALE[2];

        // Set color
        marker_array.markers[vt].color.r = ARROW_COLOR[0];
        marker_array.markers[vt].color.g = ARROW_COLOR[1];
        marker_array.markers[vt].color.b = ARROW_COLOR[2];
        marker_array.markers[vt].color.a = ARROW_COLOR[3];

        marker_array.markers[vt].lifetime = rclcpp::Duration::from_seconds(marker_lifetime).to_builtin_msg();
    }

    // publish rviz markers
    pub_mppi_via_state_sequence_->publish(marker_array);
}

// publish rviz markers to visualize optimal trajectory with arrow markers
void MPC::publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.41; // [m]
    //// arrow scale (x, y, z)
    double arrow_scale[3] = {0.225, 0.045, 0.045}; // smaller arrow: {0.10, 0.03, 0.03};
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
void MPC::publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences)
{
    // constant params
    //// arrow height
    double MARKER_POS_Z = 0.39; // [m]
    //// line lifetime [s]
    double line_lifetime = 1.5; // [s]
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
void MPC::publishOverlayText(const std::string& text)
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

} // namespace controller
