#include "dwa_planner/dwa_planner_node.hpp"
#include "dwa_planner/dwa_planner_component.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <limits>

namespace dwa_planner
{

DWAPlannerNode::DWAPlannerNode()
: Node("dwa_planner"),
  x_{0.0, 0.0, 0.0, 0.0, 0.0},
  goal_{0.0, 0.0},
  robot_radius_(0.3),
  obstacle_radius_(0.3),
  received_obstacles_(false),
  received_goal_(false),
  received_odom_(false)
{
  // デフォルトパラメータ
//   kinematic_ = {1.0, toRadian(20.0), 0.2, toRadian(50.0), 0.01, toRadian(1.0)};
  kinematic_ = {0.5, toRadian(20.0), 0.1, toRadian(50.0), 0.01, toRadian(1.0)};
  eval_param_ = {0.1, 0.08, 0.1, 3.0};

  // Subscriber
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&DWAPlannerNode::odomCallback, this, std::placeholders::_1));

  local_obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "local_obstacle_markers", 10,
    std::bind(&DWAPlannerNode::local_obstacle_callback, this, std::placeholders::_1));

  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "waypoint", 10,
    std::bind(&DWAPlannerNode::target_callback, this, std::placeholders::_1));

  // Publisher
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DWAPlannerNode::timerCallback, this));

  // TF
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  send_static_transform();
}

void DWAPlannerNode::timerCallback()
{
  if (!received_odom_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for /odom...");
    return;
  }
  if (!received_obstacles_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for local_obstacle_markers...");
    return;
  }
  if (!received_goal_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for /waypoint...");
    return;
  }

  // DWA計算
  auto u = DWA::DynamicWindowApproach(
    x_,
    kinematic_,
    goal_,
    eval_param_,
    obstacle_,
    obstacle_radius_,
    robot_radius_);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = u[0];
  cmd.angular.z = u[1];
  cmd_vel_pub_->publish(cmd);
}

void DWAPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x_[0] = msg->pose.pose.position.x;
  x_[1] = msg->pose.pose.position.y;

  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  x_[2] = yaw;
  x_[3] = msg->twist.twist.linear.x;
  x_[4] = msg->twist.twist.angular.z;

  if (!x_.empty()) {
    received_odom_ = true;
  }
}

void DWAPlannerNode::local_obstacle_callback(
  const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  obstacle_.clear();
  for (const auto & marker : msg->markers) {
    if (marker.type == visualization_msgs::msg::Marker::CYLINDER) {
      double ox = marker.pose.position.x;
      double oy = marker.pose.position.y;
      obstacle_.push_back({ox, oy});
    }
  }

  if (!obstacle_.empty()) {
    received_obstacles_ = true;
  }
}

void DWAPlannerNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_[0] = msg->pose.position.x;
  goal_[1] = msg->pose.position.y;

  RCLCPP_INFO(get_logger(), "New goal set: (%.2f, %.2f)", goal_[0], goal_[1]);

  if (!goal_.empty()) {
    received_goal_ = true;
  }
}

void DWAPlannerNode::send_static_transform()
{
  geometry_msgs::msg::TransformStamped static_transform_stamped;
  static_transform_stamped.header.stamp = now();
  static_transform_stamped.header.frame_id = "map";
  static_transform_stamped.child_frame_id = "odom";
  static_transform_stamped.transform.translation.x = 0.0;
  static_transform_stamped.transform.translation.y = 0.0;
  static_transform_stamped.transform.translation.z = 0.0;
  static_transform_stamped.transform.rotation.x = 0.0;
  static_transform_stamped.transform.rotation.y = 0.0;
  static_transform_stamped.transform.rotation.z = 0.0;
  static_transform_stamped.transform.rotation.w = 1.0;

  static_broadcaster_->sendTransform(static_transform_stamped);
}

} // namespace dwa_planner
