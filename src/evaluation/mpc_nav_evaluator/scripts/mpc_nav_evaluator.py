#!/usr/bin/env python3
import csv
import os
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from mpc_eval_msgs.msg import MPCEval
from rclpy.node import Node
import yaml

class MPCEvaluator(Node):
    def __init__(self):
        super().__init__("mpc_nav_evaluator")

        # load param
        ## load initial_pose_x
        self.initial_pose_x = self.declare_parameter("initial_pose_x", 0.0).value
        ## load initial_pose_y
        self.initial_pose_y = self.declare_parameter("initial_pose_y", 0.0).value

        # initialize subscriber
        mpc_eval_topic = self.declare_parameter("mpc_eval_topic", "").value
        self.sub = self.create_subscription(MPCEval, mpc_eval_topic, self.callback, 10)

        # initialize publisher
        goal_pose_topic = self.declare_parameter("goal_pose_topic", "").value
        self.pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)

        # initialize timer
        self.timer = self.create_timer(0.5, self.timer_callback)

        # load rosparam
        self.scenario_config_path = self.declare_parameter("scenario_config_path", "./default.yaml").value
        self.eval_result_dir = self.declare_parameter("eval_result_dir", "./default").value
        self.csv_filepath = os.path.join(
            os.path.expanduser(self.eval_result_dir),
            os.path.splitext(os.path.basename(self.scenario_config_path))[0] + ".csv",
        )

        # load scenario config
        if not os.path.exists(os.path.expanduser(self.scenario_config_path)):
            self.get_logger().error(f"[mpc_nav_evaluator] {self.scenario_config_path} does not exist")
            sys.exit(1)
        with open(os.path.expanduser(self.scenario_config_path), 'r') as file:
            self.config = yaml.safe_load(file)

        # write header to csv file
        with open(self.csv_filepath, "w") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "time_stamp",
                    "state_cost",
                    "global_x",
                    "global_y",
                    "global_yaw",
                    "cmd_vx",
                    "cmd_vy",
                    "cmd_yawrate",
                    "cmd_steer_fl",
                    "cmd_steer_fr",
                    "cmd_steer_rl",
                    "cmd_steer_rr",
                    "cmd_rotor_fl",
                    "cmd_rotor_fr",
                    "cmd_rotor_rl",
                    "cmd_rotor_rr",
                    "calc_time_ms",
                    "goal_reached",
                ]
            )

        # initialize variables
        self.calc_time_ms = 0.0 #[ms]
        self.node_launch_time = self.get_clock().now() # get roslaunch time
        self.launch_waiting_time = 5.0 # [s] after launching this node
        self.cooling_time = 0.1 # [s] after publishing goal pose
        self.goal_publishing_time = self.get_clock().now() # get goal publishing time
        self.published_first_goal = False
        self.reached_goal_count = 0
        self.total_goal_count = len(self.config['goals'])
        self.latest_goal_reached_pos_x = self.initial_pose_x
        self.latest_goal_reached_pos_y = self.initial_pose_y

    def kill_simulation(self):
        # kill simulation
        WORKSPACE_DIR = '~/nullspace_mpc'
        KILL_ALL_CMD = 'make killall'
        subprocess.Popen(KILL_ALL_CMD, shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE, cwd=os.path.expanduser(WORKSPACE_DIR))

    def callback(self, mpc_eval_msg):
        # parse and save mpc evaluation message
        goal_reached = mpc_eval_msg.goal_reached
        DISTANCE_THRESHOLD = 0.5
        is_remaining_last_goal = True if (mpc_eval_msg.global_x - self.latest_goal_reached_pos_x) ** 2 + (mpc_eval_msg.global_y - self.latest_goal_reached_pos_y) ** 2 < DISTANCE_THRESHOLD ** 2 else False
        is_final_goal_reached = False

        # at the timing of reaching the goal
        if goal_reached and not is_remaining_last_goal:
            self.get_logger().warning("[mpc_nav_evaluator] publish next goal pose.")
            if (self.get_clock().now() - self.goal_publishing_time).nanoseconds * 1e-9 > self.cooling_time:

                # add goal reached count
                self.reached_goal_count += 1
                self.latest_goal_reached_pos_x = mpc_eval_msg.global_x
                self.latest_goal_reached_pos_y = mpc_eval_msg.global_y
                self.get_logger().info(f"[mpc_nav_evaluator] goal reached count: {self.reached_goal_count}")
                self.get_logger().info(
                    f"[mpc_nav_evaluator] latest goal reached position: ({self.latest_goal_reached_pos_x}, {self.latest_goal_reached_pos_y})"
                )

                # check if it is the last goal in this episode scenario
                if self.reached_goal_count < self.total_goal_count:
                    next_goal_x = self.config['goals'][self.reached_goal_count]['goal_x']
                    next_goal_y = self.config['goals'][self.reached_goal_count]['goal_y']
                    self.publish_goal_pose(next_goal_x, next_goal_y)
                else:
                    is_final_goal_reached = True

        # save data to csv file
        with open(self.csv_filepath, "a") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    mpc_eval_msg.header.stamp.sec + mpc_eval_msg.header.stamp.nanosec * 1e-9,
                    mpc_eval_msg.state_cost,
                    mpc_eval_msg.global_x,
                    mpc_eval_msg.global_y,
                    mpc_eval_msg.global_yaw,
                    mpc_eval_msg.cmd_vx,
                    mpc_eval_msg.cmd_vy,
                    mpc_eval_msg.cmd_yawrate,
                    mpc_eval_msg.cmd_steer_fl,
                    mpc_eval_msg.cmd_steer_fr,
                    mpc_eval_msg.cmd_steer_rl,
                    mpc_eval_msg.cmd_steer_rr,
                    mpc_eval_msg.cmd_rotor_fl,
                    mpc_eval_msg.cmd_rotor_fr,
                    mpc_eval_msg.cmd_rotor_rl,
                    mpc_eval_msg.cmd_rotor_rr,
                    mpc_eval_msg.calc_time_ms,
                    self.reached_goal_count,
                ]
            )

        # kill simulation if all goals are reached
        if is_final_goal_reached:
            self.get_logger().warning("[mpc_nav_evaluator] all goals are reached.")
            self.kill_simulation()
            return

    def timer_callback(self):
        # get elapsed time since this node is launched.
        elapsed_time = (self.get_clock().now() - self.node_launch_time).nanoseconds * 1e-9

        # announce elapsed time as rosinfo. 
        self.get_logger().info(f"[mpc_nav_evaluator] elapsed time: {elapsed_time} [s]")

        # publish first goal pose after waiting for launch_waiting_time [s]
        if elapsed_time > self.launch_waiting_time and not self.published_first_goal:
            self.get_logger().warning("[mpc_nav_evaluator] publish first goal pose.")
            self.publish_goal_pose(self.config['goals'][0]['goal_x'], self.config['goals'][0]['goal_y'])
            self.published_first_goal = True

    def publish_goal_pose(self, goal_x: float, goal_y: float):
        """publish goal pose to (goal_x, goal_y)
        Args:
            goal_x (float): x position of goal pose
            goal_y (float): y position of goal pose
        """
        # publish goal pose as PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.pub.publish(goal_pose)
        self.goal_publishing_time = self.get_clock().now()

        # update previous goal position
        self.get_logger().info(f"[mpc_nav_evaluator] publish goal pose: ({goal_x}, {goal_y})")

if __name__ == "__main__":
    # initialize node
    rclpy.init()
    evaluator = MPCEvaluator()
    rclpy.spin(evaluator)
    evaluator.destroy_node()
    rclpy.shutdown()
