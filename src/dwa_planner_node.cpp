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

// constructor
DWAPlannerNode::DWAPlannerNode()
: rclcpp_lifecycle::LifecycleNode("dwa_planner"),
  x_{0.0, 0.0, 0.0, 0.0, 0.0},
  goal_{0.0, 0.0},
  robot_radius_(0.0),
  obstacle_radius_(0.0),
  received_obstacles_(false),
  received_goal_(false),
  received_odom_(false)
{
  // パラメータ宣言のみ。実体生成は on_configure() で行う
  this->declare_parameter<std::vector<double>>("kinematic", {0.5, 20.0, 0.1, 50.0, 0.01, 1.0});
  this->declare_parameter<std::vector<double>>("eval_param", {0.1, 0.08, 0.1, 3.0});
  this->declare_parameter<double>("robot_radius", 0.3);
  this->declare_parameter<double>("obstacle_radius", 0.3);
}

CallbackReturn DWAPlannerNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_configure: configuring DWA planner");

  // YAML から取得
  std::vector<double> kin_vec, eval_vec;
  this->get_parameter("kinematic", kin_vec);
  this->get_parameter("eval_param", eval_vec);
  this->get_parameter("robot_radius", robot_radius_);
  this->get_parameter("obstacle_radius", obstacle_radius_);

  if (kin_vec.size() < 6 || eval_vec.size() < 4) {
    RCLCPP_ERROR(get_logger(), "Parameter size mismatch: kinematic=%zu (need 6), eval_param=%zu (need 4)",
                 kin_vec.size(), eval_vec.size());
    return CallbackReturn::FAILURE;
  }

  // 配列に詰め替え（角度はマクロで変換）
  kinematic_[0] = kin_vec[0];
  kinematic_[1] = TO_RADIAN(kin_vec[1]);
  kinematic_[2] = kin_vec[2];
  kinematic_[3] = TO_RADIAN(kin_vec[3]);
  kinematic_[4] = kin_vec[4];
  kinematic_[5] = TO_RADIAN(kin_vec[5]);

  for (size_t i = 0; i < eval_param_.size(); ++i) {
    eval_param_[i] = eval_vec[i];
  }

  // Subscriber: Inactive 中も状態更新を続けるため on_configure で作る
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&DWAPlannerNode::odomCallback, this, std::placeholders::_1));

  local_obstacle_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "global_obstacle_markers", 10,
    std::bind(&DWAPlannerNode::local_obstacle_callback, this, std::placeholders::_1));

  target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "waypoint", 10,
    std::bind(&DWAPlannerNode::target_callback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/filtered_scan", 10,
    std::bind(&DWAPlannerNode::scanCallback, this, std::placeholders::_1));

  // Publisher (Lifecycle 版)
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  predict_path_pub = create_publisher<nav_msgs::msg::Path>("predict_path", 50);
  bool_pub_ = create_publisher<std_msgs::msg::Bool>("dwa_active", 10);
  goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("dwa_goal_marker", 10);

  // Timer: 生成後すぐに停止しておき、on_activate で再開する
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&DWAPlannerNode::timerCallback, this));
  timer_->cancel();

  // 静的 TF はラッチされるので configure 時に一度だけ送る
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  send_static_transform();

  return CallbackReturn::SUCCESS;
}

CallbackReturn DWAPlannerNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_activate: activating DWA planner");

  // LifecyclePublisher を有効化（publish() が実際に送信されるようになる）
  cmd_vel_pub_->on_activate();
  predict_path_pub->on_activate();
  bool_pub_->on_activate();
  goal_marker_pub_->on_activate();

  // 稼働中フラグを発信
  std_msgs::msg::Bool flag_msg;
  flag_msg.data = true;
  bool_pub_->publish(flag_msg);

  // タイマ再開（10Hz で DWA 計算を開始）
  timer_->reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn DWAPlannerNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_deactivate: deactivating DWA planner");

  // まずタイマを止めて DWA 計算を停止
  if (timer_) {
    timer_->cancel();
  }

  // 安全停止: ゼロ速度と非稼働フラグを送ってから publisher を deactivate する
  publishStopCommand();

  std_msgs::msg::Bool flag_msg;
  flag_msg.data = false;
  bool_pub_->publish(flag_msg);

  cmd_vel_pub_->on_deactivate();
  predict_path_pub->on_deactivate();
  bool_pub_->on_deactivate();
  goal_marker_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn DWAPlannerNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "on_cleanup: cleaning up DWA planner");

  timer_.reset();
  cmd_vel_pub_.reset();
  predict_path_pub.reset();
  bool_pub_.reset();
  goal_marker_pub_.reset();
  odom_sub_.reset();
  local_obstacle_sub_.reset();
  target_sub_.reset();
  scan_sub_.reset();
  static_broadcaster_.reset();

  obstacle_.clear();
  x_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  goal_ = {0.0, 0.0};
  received_obstacles_ = false;
  received_goal_ = false;
  received_odom_ = false;

  return CallbackReturn::SUCCESS;
}

CallbackReturn DWAPlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "on_shutdown: shutting down DWA planner");

  // Active 状態からの shutdown では deactivate 相当の処理を行う
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (timer_) {
      timer_->cancel();
    }
    publishStopCommand();
    if (cmd_vel_pub_) cmd_vel_pub_->on_deactivate();
    if (predict_path_pub) predict_path_pub->on_deactivate();
    if (bool_pub_) bool_pub_->on_deactivate();
  }

  return on_cleanup(state);
}

void DWAPlannerNode::publishStopCommand()
{
  if (!cmd_vel_pub_) return;
  geometry_msgs::msg::Twist stop_cmd;  // 全ゼロ
  cmd_vel_pub_->publish(stop_cmd);
}

void DWAPlannerNode::timerCallback()
{
  if (!received_odom_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for /odom...");
    return;
  }
  /*
  if (!received_obstacles_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for local_obstacle_markers...");
    return;
  }*/
  if (!received_goal_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for /waypoint...");
    return;
  }

  std_msgs::msg::Bool flag_msg;
  flag_msg.data = true;
  bool_pub_->publish(flag_msg);


  // DWA計算
  auto result = DWA::DynamicWindowApproach(
    x_,
    kinematic_,
    goal_,
    eval_param_,
    obstacle_,
    obstacle_radius_,
    robot_radius_);

  // --- 最適制御コマンドを取得 ---
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = result.control[0];
  cmd.angular.z = result.control[1];
  //RCLCPP_INFO(get_logger(), "cmd_vel: (%.2f, %.2f)", cmd.linear.x, cmd.angular.z);
  cmd_vel_pub_->publish(cmd);

  // 目標点を赤いSphereマーカーで可視化
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = now();
    marker.ns = "dwa_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_[0];
    marker.pose.position.y = goal_[1];
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0;
    goal_marker_pub_->publish(marker);
  }

  // --- 軌跡をパスとして可視化 ---
  nav_msgs::msg::Path all_traj_path;
  all_traj_path.header.stamp = now();
  all_traj_path.header.frame_id = "odom";

  // すべての GenerateTrajectory() 結果を Path に追加
  for (const auto &xt : result.trajectories) {
    for (const auto &state : xt) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = "odom";
      pose.pose.position.x = state[0];
      pose.pose.position.y = state[1];

      // θ（yaw）を姿勢に変換
      tf2::Quaternion q;
      q.setRPY(0, 0, state[2]);
      pose.pose.orientation = tf2::toMsg(q);

      all_traj_path.poses.push_back(pose);
    }
  }

  // --- 軌跡群を publish ---
  predict_path_pub->publish(all_traj_path);
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

void DWAPlannerNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  obstacle_.clear();
  double angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
      float r = msg->ranges[i];
      if (std::isfinite(r) && (r >= msg->range_min && r <= msg->range_max))
      {
          double ox = r * std::cos(angle) + x_[0];
          double oy = r * std::sin(angle)+ x_[1];
          obstacle_.push_back({ox, oy});
      }
      angle += msg->angle_increment;
  }

  if (!obstacle_.empty()) {
    received_obstacles_ = true;
  }
}

void DWAPlannerNode::local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  obstacle_.clear();
  for (const auto & marker : msg->markers) {
    obstacle_.push_back({marker.pose.position.x, marker.pose.position.y});
  }
  if (!obstacle_.empty()) {
    received_obstacles_ = true;
  }
}

void DWAPlannerNode::target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_[0] = msg->pose.position.x;
  goal_[1] = msg->pose.position.y;

  //RCLCPP_INFO(get_logger(), "New goal set: (%.2f, %.2f)", goal_[0], goal_[1]);

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
