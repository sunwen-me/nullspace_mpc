#include "vel_driver/vel_driver.hpp"

namespace gazebo
{

// constructor
VelDriver::VelDriver()
    : rclcpp::Node("vel_driver")
{
    // load parameters 

    //// vehicle parameters
    l_f = this->declare_parameter<float>("l_f", 0.5f);
    l_r = this->declare_parameter<float>("l_r", 0.5f);
    d_l = this->declare_parameter<float>("d_l", 0.5f);
    d_r = this->declare_parameter<float>("d_r", 0.5f);
    tire_radius = this->declare_parameter<float>("tire_radius", 0.2f);

    //// subscribing topic names
    std::string control_cmd_vel_topic =
        this->declare_parameter<std::string>("control_cmd_vel_topic", "/cmd_vel");

    //// publishing topic names
    std::string front_left_steer_cmd_topic, front_right_steer_cmd_topic, rear_left_steer_cmd_topic, rear_right_steer_cmd_topic;
    std::string front_left_rotor_cmd_topic, front_right_rotor_cmd_topic, rear_left_rotor_cmd_topic, rear_right_rotor_cmd_topic;
    front_left_steer_cmd_topic =
        this->declare_parameter<std::string>("front_left_steer_cmd_topic", "/fwids/front_left_steer_rad/command");
    front_right_steer_cmd_topic =
        this->declare_parameter<std::string>("front_right_steer_cmd_topic", "/fwids/front_right_steer_rad/command");
    rear_left_steer_cmd_topic =
        this->declare_parameter<std::string>("rear_left_steer_cmd_topic", "/fwids/rear_left_steer_rad/command");
    rear_right_steer_cmd_topic =
        this->declare_parameter<std::string>("rear_right_steer_cmd_topic", "/fwids/rear_right_steer_rad/command");
    front_left_rotor_cmd_topic =
        this->declare_parameter<std::string>("front_left_rotor_cmd_topic", "/fwids/front_left_rotor_radpersec/command");
    front_right_rotor_cmd_topic =
        this->declare_parameter<std::string>("front_right_rotor_cmd_topic", "/fwids/front_right_rotor_radpersec/command");
    rear_left_rotor_cmd_topic =
        this->declare_parameter<std::string>("rear_left_rotor_cmd_topic", "/fwids/rear_left_rotor_radpersec/command");
    rear_right_rotor_cmd_topic =
        this->declare_parameter<std::string>("rear_right_rotor_cmd_topic", "/fwids/rear_right_rotor_radpersec/command");

    // initialize subscribers
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        control_cmd_vel_topic,
        rclcpp::QoS(1),
        std::bind(&VelDriver::cmdVelCallback, this, std::placeholders::_1));

    // initialize publishers
    pub_cmd_front_left_steer =
        this->create_publisher<std_msgs::msg::Float64>(front_left_steer_cmd_topic, rclcpp::QoS(10));
    pub_cmd_front_right_steer =
        this->create_publisher<std_msgs::msg::Float64>(front_right_steer_cmd_topic, rclcpp::QoS(10));
    pub_cmd_rear_left_steer =
        this->create_publisher<std_msgs::msg::Float64>(rear_left_steer_cmd_topic, rclcpp::QoS(10));
    pub_cmd_rear_right_steer =
        this->create_publisher<std_msgs::msg::Float64>(rear_right_steer_cmd_topic, rclcpp::QoS(10));
    pub_cmd_front_left_rotor =
        this->create_publisher<std_msgs::msg::Float64>(front_left_rotor_cmd_topic, rclcpp::QoS(10));
    pub_cmd_front_right_rotor =
        this->create_publisher<std_msgs::msg::Float64>(front_right_rotor_cmd_topic, rclcpp::QoS(10));
    pub_cmd_rear_left_rotor =
        this->create_publisher<std_msgs::msg::Float64>(rear_left_rotor_cmd_topic, rclcpp::QoS(10));
    pub_cmd_rear_right_rotor =
        this->create_publisher<std_msgs::msg::Float64>(rear_right_rotor_cmd_topic, rclcpp::QoS(10));
}

// destructor
VelDriver::~VelDriver()
{
    // No Contents
}

// /cmd_vel topic callback
void VelDriver::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    // if msg contains NaN, return without publishing commands
    if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) || std::isnan(msg->angular.z))
    {
        RCLCPP_WARN(this->get_logger(), "Received NaN in Twist Command. Skip this message.");
    }

    // parse received twist message
    // Note : urdf model axis (forward : x, left : y) and vehicle local axis (right : x, forward : y) are different.
    float v_r    = -msg->linear.y;  // velocity in the vehicle right direction
    float v_f    =  msg->linear.x;  // velocity in the vehicle forward direction
    float omega =  msg->angular.z; // angular velocity around the z-axis, counter-clockwise is positive

    // [for debug] annouce received Twist message
    RCLCPP_DEBUG(this->get_logger(),
                 "Received Twist Command: v_right = %+5.1f, v_forward = %+5.1f, yaw_rate = %+5.1f",
                 v_r,
                 v_f,
                 omega);

    // convert from Vx, Vy, Omega to 8 DoF vehicle control commands
    float vx_fl = v_r - omega * l_f;
    float vx_fr = v_r - omega * l_f;
    float vx_rl = v_r + omega * l_r;
    float vx_rr = v_r + omega * l_r;

    float vy_fl = v_f - omega * d_l;
    float vy_fr = v_f + omega * d_r;
    float vy_rl = v_f - omega * d_l;
    float vy_rr = v_f + omega * d_r;

    cmd_front_left_steer.data  = atan2(-vx_fl, vy_fl);
    cmd_front_right_steer.data = atan2(-vx_fr, vy_fr);
    cmd_rear_left_steer.data   = atan2(-vx_rl, vy_rl);
    cmd_rear_right_steer.data  = atan2(-vx_rr, vy_rr);

    cmd_front_left_rotor.data  = sqrt(vx_fl*vx_fl + vy_fl*vy_fl) / tire_radius; // [rad/s]
    cmd_front_right_rotor.data = sqrt(vx_fr*vx_fr + vy_fr*vy_fr) / tire_radius; // [rad/s]
    cmd_rear_left_rotor.data   = sqrt(vx_rl*vx_rl + vy_rl*vy_rl) / tire_radius; // [rad/s]
    cmd_rear_right_rotor.data  = sqrt(vx_rr*vx_rr + vy_rr*vy_rr) / tire_radius; // [rad/s]

    // [for debug] annouce publishing Twist message
    RCLCPP_DEBUG(this->get_logger(),
                 "FWIDS Steer Commands: FL_steer = %+5.1f, FR_steer = %+5.1f, RL_steer = %+5.1f, RR_steer = %+5.1f",
                 cmd_front_left_steer.data,
                 cmd_front_right_steer.data,
                 cmd_rear_left_steer.data,
                 cmd_rear_right_steer.data);
    RCLCPP_DEBUG(this->get_logger(),
                 "FWIDS Rotor Commands: FL_rotor = %+5.1f, FR_rotor = %+5.1f, RL_rotor = %+5.1f, RR_rotor = %+5.1f",
                 cmd_front_left_rotor.data,
                 cmd_front_right_rotor.data,
                 cmd_rear_left_rotor.data,
                 cmd_rear_right_rotor.data);

    // publish 8 DoF vehicle commands to gazebo ros_control plugin
    pub_cmd_front_left_steer->publish(cmd_front_left_steer);
    pub_cmd_front_right_steer->publish(cmd_front_right_steer);
    pub_cmd_rear_left_steer->publish(cmd_rear_left_steer);
    pub_cmd_rear_right_steer->publish(cmd_rear_right_steer);

    pub_cmd_front_left_rotor->publish(cmd_front_left_rotor);
    pub_cmd_front_right_rotor->publish(cmd_front_right_rotor);
    pub_cmd_rear_left_rotor->publish(cmd_rear_left_rotor);
    pub_cmd_rear_right_rotor->publish(cmd_rear_right_rotor);
}

} // namespace gazebo
