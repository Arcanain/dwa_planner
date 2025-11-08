#pragma once

#include <array>
#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace dwa_planner
{

class DWAPlannerNode : public rclcpp::Node
{
public:
  DWAPlannerNode();
  ~DWAPlannerNode() = default;

private:
  void timerCallback();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void send_static_transform();

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  //rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // States
  std::array<double, 5> x_;
  std::array<double, 2> goal_;
  std::vector<std::array<double, 2>> obstacle_;

  // DWA param
  std::array<double, 6> kinematic_;
  std::array<double, 4> eval_param_;

  double robot_radius_;
  double obstacle_radius_;

  bool received_obstacles_;
  bool received_goal_;
  bool received_odom_;
};

} // namespace dwa_planner
