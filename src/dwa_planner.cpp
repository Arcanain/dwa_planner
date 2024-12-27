#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

// ===========================
// データ構造定義
// ===========================
struct RobotState {
  double x;
  double y;
  double yaw;
  double v;
  double w;
};

struct Obstacle {
  double x;
  double y;
  double radius;
};

struct KinematicParam {
  double max_vel;
  double max_omega;
  double accel;
  double accel_omega;
  double v_reso;
  double w_reso;
};

struct EvalParam {
  double heading;
  double dist;
  double velocity;
  double predict_dt;
};

// ===========================
// DWAプランナークラス
// ===========================
class DWAPlanner {
public:
  DWAPlanner(double dt,
             const KinematicParam &kin_param,
             const EvalParam &eval_param,
             double robot_radius,
             double obstacle_inflation_radius)
    : dt_(dt),
      Kinematic_(kin_param),
      evalParam_(eval_param),
      robot_radius_(robot_radius),
      obstacle_radius_(obstacle_inflation_radius){}

  std::array<double,2> computeControl(
    const RobotState &state,
    const std::array<double,2> &goal,
    const std::vector<Obstacle> &obstacles)
  {
    auto Vr = calcDynamicWindow(state);
    std::vector<std::array<double,5>> evalDB; 
    // evalDB: [vt, ot, heading, dist, vel]

    for (double vt = Vr[0]; vt <= Vr[1]; vt += Kinematic_.v_reso) {
      for (double ot = Vr[2]; ot <= Vr[3]; ot += Kinematic_.w_reso) {
        auto [xt, traj] = generateTrajectory(state, vt, ot, evalParam_.predict_dt);

        double heading = calcHeadingEval(xt, goal);
        double dist = calcDistEval(xt, obstacles);
        double vel = std::fabs(vt);

        if (dist < 0) {
          continue;
        }

        evalDB.push_back({vt, ot, heading, dist, vel});
      }
    }

    if (evalDB.empty()) {
      // no feasible path
      return {0.0, 0.0};
    }

    normalizeEval(evalDB);

    double max_score = -std::numeric_limits<double>::infinity();
    int best_i = -1;
    for (int i = 0; i < (int)evalDB.size(); i++) {
      double score = evalParam_.heading*evalDB[i][2] +
                     evalParam_.dist*evalDB[i][3] +
                     evalParam_.velocity*evalDB[i][4];
      if(score > max_score) {
        max_score = score;
        best_i = i;
      }
    }

    return {evalDB[best_i][0], evalDB[best_i][1]};
  }

private:
  double dt_;
  KinematicParam Kinematic_;
  EvalParam evalParam_;
  double robot_radius_;
  double obstacle_radius_;

  std::array<double,4> calcDynamicWindow(const RobotState &x) const {
    double Vs_min_v = 0.0;
    double Vs_max_v = Kinematic_.max_vel;
    double Vs_min_w = -Kinematic_.max_omega;
    double Vs_max_w = Kinematic_.max_omega;

    double Vd_min_v = x.v - Kinematic_.accel * dt_;
    double Vd_max_v = x.v + Kinematic_.accel * dt_;
    double Vd_min_w = x.w - Kinematic_.accel_omega * dt_;
    double Vd_max_w = x.w + Kinematic_.accel_omega * dt_;

    double v_min = std::max(Vs_min_v, Vd_min_v);
    double v_max = std::min(Vs_max_v, Vd_max_v);
    double w_min = std::max(Vs_min_w, Vd_min_w);
    double w_max = std::min(Vs_max_w, Vd_max_w);

    return {v_min, v_max, w_min, w_max};
  }

  void normalizeEval(std::vector<std::array<double,5>> &EvalDB) const {
    double sum_heading=0.0, sum_dist=0.0, sum_vel=0.0;
    for (auto &e: EvalDB) {
      sum_heading += e[2];
      sum_dist += e[3];
      sum_vel += e[4];
    }

    if (sum_heading != 0) {
      for (auto &e : EvalDB) e[2] /= sum_heading;
    }
    if (sum_dist != 0) {
      for (auto &e : EvalDB) e[3] /= sum_dist;
    }
    if (sum_vel != 0) {
      for (auto &e : EvalDB) e[4] /= sum_vel;
    }
  }

  double calcHeadingEval(const RobotState &x, const std::array<double,2> &goal) const {
    double goalTheta = std::atan2(goal[1]-x.y, goal[0]-x.x);
    double dtheta = goalTheta - x.yaw;
    dtheta = std::fmod(dtheta + M_PI, 2*M_PI) - M_PI;
    double targetTheta = std::fabs(toDegree(dtheta));
    if (targetTheta > 180.0) targetTheta = 360.0 - targetTheta;
    double heading = 180.0 - targetTheta;
    return heading;
  }

  // double calcDistEval(const RobotState &x, const std::vector<Obstacle> &ob) const {
  //   double dist = std::numeric_limits<double>::infinity();
  //   for (auto &o: ob) {
  //     double disttmp = std::hypot(o.x - x.x, o.y - x.y) - (obstacle_radius_ + robot_radius_ + o.radius);
  //     if (dist > disttmp) {
  //       dist = disttmp;
  //     }
  //   }
  //   if(dist < 0) return dist;
  //   return dist;
  // }

  double calcDistEval(const RobotState &x, const std::vector<Obstacle> &ob) const {
    double dist = std::numeric_limits<double>::infinity();
    for (auto &o: ob) {
      // MATLABと同様にR + robotR = 0.3 + 0.3 = 0.6固定とする
      double total_radius = obstacle_radius_ + robot_radius_; 
      double disttmp = std::hypot(o.x - x.x, o.y - x.y) - total_radius;

      // std::cout << total_radius << std::endl;
      // std::cout << disttmp << std::endl;
      if (dist > disttmp) {
        dist = disttmp;
      }
    }
    if(dist < 0) return dist;
    return dist;
  }

  std::pair<RobotState,std::vector<double>> generateTrajectory(RobotState x, double vt, double ot, double evaldt) const {
    std::vector<double> traj;
    double time=0.0;
    while(time <= evaldt) {
      time += dt_;
      x = motion(x, vt, ot);
    }
    return {x,traj};
  }

  RobotState motion(const RobotState &x, double v, double w) const {
    RobotState x_new = x;
    x_new.x += v*std::cos(x.yaw)*dt_;
    x_new.y += v*std::sin(x.yaw)*dt_;
    x_new.yaw += w*dt_;
    x_new.v = v;
    x_new.w = w;
    return x_new;
  }

  double toDegree(double rad) const { return rad * 180.0 / M_PI; }
};

// ===========================
// ROSノードクラス
// 元のDWANodeをMPCNodeにし、サブスク/コールバックを修正
// ===========================
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : Node("dwa_planner"), received_goal_(false), received_odom_(false)
  {
    double dt = 0.1; 
    KinematicParam kin = {
      1.0,
      toRadian(20.0),
      0.2,
      toRadian(50.0),
      0.01,
      toRadian(1.0)
    };
    // EvalParam eval = {
    //   0.1, // heading
    //   0.08, // dist
    //   0.1, // velocity
    //   3.0 // predict_dt
    // };
    EvalParam eval = {
      0.1, // heading
      0.08, // dist
      0.1, // velocity
      3.0 // predict_dt
    };
    double robot_r = 0.25;
    double obstacle_r = 0.25;

    planner_ = std::make_unique<DWAPlanner>(dt, kin, eval, robot_r, obstacle_r);

    // 修正後のSubscriber
    using std::placeholders::_1;

    // odometoryのSubscribe (トピック名 "odom", コールバック odometry_callback)
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MPCNode::odometry_callback, this, _1));

    // obstacleのSubscribe (トピック名 "local_obstacle_markers", コールバック local_obstacle_callback)
    local_obstacle_subscriber = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "local_obstacle_markers", 10, std::bind(&MPCNode::local_obstacle_callback, this, _1));

    // waypoint（目標位置）をSubscribe (トピック名 "waypoint", コールバック target_callback)
    target_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "waypoint", 10, std::bind(&MPCNode::target_callback, this, _1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      100ms, std::bind(&MPCNode::timerCallback, this));
  }

private:
  // void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //   double qw = msg->pose.pose.orientation.w;
  //   double qx = msg->pose.pose.orientation.x;
  //   double qy = msg->pose.pose.orientation.y;
  //   double qz = msg->pose.pose.orientation.z;

  //   double siny_cosp = 2.0*(qw*qz + qx*qy);
  //   double cosy_cosp = 1.0 - 2.0*(qy*qy + qz*qz);
  //   double yaw = std::atan2(siny_cosp, cosy_cosp);

  //   state_.x = msg->pose.pose.position.x;
  //   state_.y = msg->pose.pose.position.y;
  //   state_.yaw = yaw;
  //   state_.v = msg->twist.twist.linear.x;
  //   state_.w = msg->twist.twist.angular.z;
  //   received_odom_ = true;
  // }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // x, y 座標を取得
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // クォータニオンからYaw角を取得
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // RobotState に代入
    state_.x = x;
    state_.y = y;
    state_.yaw = yaw;  // RPYのyaw角
    state_.v = msg->twist.twist.linear.x;
    state_.w = msg->twist.twist.angular.z;

    // フラグを設定
    received_odom_ = true;

    RCLCPP_INFO(this->get_logger(), "Odom: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
  }


  void target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_[0] = msg->pose.position.x;
    goal_[1] = msg->pose.position.y;
    received_goal_ = true;
  }

  void local_obstacle_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    obstacles_.clear();
    for (auto &marker : msg->markers) {
      Obstacle o;
      o.x = marker.pose.position.x;
      o.y = marker.pose.position.y;
      double r = (marker.scale.x * 0.5);
      o.radius = r;
      obstacles_.push_back(o);
    }
  }

  void timerCallback() {
    if(!received_odom_ || !received_goal_) {
      return;
    }

    auto u = planner_->computeControl(state_, goal_, obstacles_);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = u[0];
    cmd.angular.z = u[1];
    cmd_vel_pub_->publish(cmd);
  }

  double toRadian(double deg) {return deg*M_PI/180.0;}

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr local_obstacle_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  RobotState state_;
  std::array<double,2> goal_;
  bool received_goal_;
  bool received_odom_;
  std::vector<Obstacle> obstacles_;

  std::unique_ptr<DWAPlanner> planner_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
