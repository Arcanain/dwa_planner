#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <iostream>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

static const double dt = 0.1; // [s]

inline double toRadian(double degree) {
  return degree / 180.0 * M_PI;
}

inline double toDegree(double radian) {
  return radian / M_PI * 180.0;
}

// DWAプランナーをC++で実装
class DWA {
public:
  // DynamicWindowApproach
  //   x         : [x, y, yaw, v, w] (現在状態)
  //   model     : [max_vel, max_omega, accel, accel_omega, v_reso, w_reso]
  //   goal      : [goal_x, goal_y]
  //   evalParam : [heading_gain, dist_gain, vel_gain, predict_dt]
  //   ob        : 障害物座標群 ([x, y])
  //   R, robotR : 障害物半径, ロボット半径
  // 戻り値： [v, w]
  static std::vector<double> DynamicWindowApproach(
    const std::array<double,5> &x,
    const std::array<double,6> &model,
    const std::array<double,2> &goal,
    const std::array<double,4> &evalParam,
    const std::vector<std::array<double,2>> &ob,
    double R, double robotR)
  {
    auto Vr = CalcDynamicWindow(x, model);
    std::vector<std::array<double,5>> evalDB;

    for (double vt = Vr[0]; vt <= Vr[1]; vt += model[4]) {
      for (double ot = Vr[2]; ot <= Vr[3]; ot += model[5]) {
        auto xt = GenerateTrajectory(x, vt, ot, evalParam[3]);

        double heading = CalcHeadingEval(xt, goal);
        double dist = CalcDistEval(xt, ob, R, robotR);
        double vel = std::fabs(vt);

        // 衝突していればスキップ
        if (dist < 0) continue;

        evalDB.push_back({vt, ot, heading, dist, vel});
      }
    }

    if (evalDB.empty()) {
      std::cout << "No path to goal!" << std::endl;
      return {0.0, 0.0};
    }

    NormalizeEval(evalDB);
    return SelectBestControl(evalDB, evalParam);
  }

private:
  static std::array<double,4> CalcDynamicWindow(const std::array<double,5> &x, const std::array<double,6> &model) {
    return {
      std::max(0.0, x[3] - model[2] * dt),
      std::min(model[0], x[3] + model[2] * dt),
      std::max(-model[1], x[4] - model[3] * dt),
      std::min(model[1], x[4] + model[3] * dt)
    };
  }

  static std::array<double,5> GenerateTrajectory(const std::array<double,5> &x, double vt, double ot, double evaldt) {
    std::array<double,5> xt = x;
    for (double t = 0; t <= evaldt; t += dt) {
      xt[0] += dt * std::cos(xt[2]) * vt;
      xt[1] += dt * std::sin(xt[2]) * vt;
      xt[2] += dt * ot;
      xt[3] = vt;
      xt[4] = ot;
    }
    return xt;
  }

  static double CalcHeadingEval(const std::array<double,5> &x, const std::array<double,2> &goal) {
    double targetTheta = toDegree(std::atan2(goal[1] - x[1], goal[0] - x[0]));
    double diff = std::fabs(toDegree(x[2]) - targetTheta);
    return 180.0 - std::min(diff, 360.0 - diff);
  }

  static double CalcDistEval(const std::array<double,5> &x, const std::vector<std::array<double,2>> &ob, double R, double robotR) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto &o : ob) {
      double dist = std::hypot(o[0] - x[0], o[1] - x[1]) - (R + robotR);
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  static void NormalizeEval(std::vector<std::array<double,5>> &evalDB) {
    double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0;
    for (auto &e : evalDB) {
      sum_heading += e[2];
      sum_dist += e[3];
      sum_vel += e[4];
    }
    for (auto &e : evalDB) {
      e[2] /= sum_heading;
      e[3] /= sum_dist;
      e[4] /= sum_vel;
    }
  }

  static std::vector<double> SelectBestControl(const std::vector<std::array<double,5>> &evalDB, const std::array<double,4> &evalParam) {
    double max_score = -std::numeric_limits<double>::infinity();
    std::vector<double> best_u = {0.0, 0.0};
    for (const auto &e : evalDB) {
      double score = evalParam[0] * e[2] + evalParam[1] * e[3] + evalParam[2] * e[4];
      if (score > max_score) {
        max_score = score;
        best_u = {e[0], e[1]};
      }
    }
    return best_u;
  }
};

class DWAPlannerNode : public rclcpp::Node {
public:
  DWAPlannerNode() : Node("dwa_planner"), robot_radius_(0.3), obstacle_radius_(0.3) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&DWAPlannerNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // local_obstacle_markersをsubscribe
    local_obstacle_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "local_obstacle_markers", 10,
      std::bind(&DWAPlannerNode::local_obstacle_callback, this, std::placeholders::_1));

    // 目標位置(waypoint)をSubscribe (PoseStamped)
    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "waypoint", 10, 
      std::bind(&DWAPlannerNode::target_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DWAPlannerNode::timerCallback, this));
    
    // 静的な変換を送信するタイマー
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    send_static_transform();
  }

private:
  void timerCallback() {
    // --- (1) 障害物が来ていない or ゴールが来ていない場合はスキップ
    if(!received_obstacles_){
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         "No obstacle info yet. Waiting for local_obstacle_markers...");
      return;
    }
    if(!received_goal_){
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         "No goal info yet. Waiting for /waypoint...");
      return;
    }
    if(!received_odom_){
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
         "No goal info yet. Waiting for /odom...");
      return;
    }

    auto u = DWA::DynamicWindowApproach(x_, kinematic_, goal_, eval_param_, obstacle_, obstacle_radius_, robot_radius_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = u[0];    // forward velocity
    cmd.angular.z = u[1];   // yaw rate
    cmd_vel_pub_->publish(cmd);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // position
    x_[0] = msg->pose.pose.position.x;
    x_[1] = msg->pose.pose.position.y;

    // orientation(クォータニオン→yaw)
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    x_[2]= yaw_tmp;

    // velocity
    x_[3] = msg->twist.twist.linear.x;
    x_[4] = msg->twist.twist.angular.z;

    // received_odom_ = true;
    if(!x_.empty()) {
      received_odom_ = true;
    }
  }

  void local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // 受信した障害物リストを一旦クリア
    obstacle_.clear();

    for (const auto & marker : msg->markers) {
      // CYLINDERマーカーだけを対象とする例
      if (marker.type == visualization_msgs::msg::Marker::CYLINDER) {
        // (x,y)を取り出してobstacle_に追加
        double obs_x = marker.pose.position.x;
        double obs_y = marker.pose.position.y;
        obstacle_.push_back({obs_x, obs_y});
      }
    }

    // デバッグ出力
    for (auto & obs : obstacle_) {
      RCLCPP_INFO(this->get_logger(), "Obstacle position: (%.2f, %.2f)", obs[0], obs[1]);
    }

    if(!obstacle_.empty()) {
      received_obstacles_ = true; // 初めて障害物を取得
    }
  }

  // ★ 目標位置(PoseStamped)コールバック
  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // goal_に代入
    goal_[0] = msg->pose.position.x;
    goal_[1] = msg->pose.position.y;

    // ログ出力など必要なら
    RCLCPP_INFO(this->get_logger(), "New goal set: (%.2f, %.2f)", goal_[0], goal_[1]);

    // received_goal_ = true;
    if(!goal_.empty()) {
      received_goal_ = true;
    }
  }

  void send_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = this->get_clock()->now();
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

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::array<double,5> x_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double,2> goal_ = {0.0, 0.0};
  std::vector<std::array<double,2>> obstacle_;
  std::array<double,6> kinematic_ = {1.0, toRadian(20), 0.2, toRadian(50), 0.01, toRadian(1)};
  std::array<double,4> eval_param_ = {0.1, 0.08, 0.1, 3.0};
  double robot_radius_;
  double obstacle_radius_;
  bool received_obstacles_ = false;
  bool received_goal_ = false;
  bool received_odom_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DWAPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
