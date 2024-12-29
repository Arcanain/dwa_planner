#pragma once

#include <array>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>

namespace dwa_planner
{

static const double DT = 0.1; // 刻み時間[s]

// ユーティリティ関数（インライン化）
inline double toRadian(double degree)
{
  return degree * M_PI / 180.0;
}
inline double toDegree(double radian)
{
  return radian * 180.0 / M_PI;
}

// DWA計算を行うクラス (静的メソッドのみ)
class DWA
{
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
    const std::array<double, 5> & x,
    const std::array<double, 6> & model,
    const std::array<double, 2> & goal,
    const std::array<double, 4> & evalParam,
    const std::vector<std::array<double, 2>> & ob,
    double R,
    double robotR
  );

private:
  // Dynamic Windowの計算
  static std::array<double, 4> CalcDynamicWindow(
    const std::array<double, 5> & x,
    const std::array<double, 6> & model);

  // 軌跡生成
  static std::array<double, 5> GenerateTrajectory(
    const std::array<double, 5> & x,
    double vt, double ot,
    double evaldt);

  // Heading評価
  static double CalcHeadingEval(
    const std::array<double, 5> & x,
    const std::array<double, 2> & goal);

  // 障害物との距離評価
  static double CalcDistEval(
    const std::array<double, 5> & x,
    const std::vector<std::array<double, 2>> & ob,
    double R,
    double robotR);

  // 正規化
  static void NormalizeEval(std::vector<std::array<double, 5>> & evalDB);

  // 最適コントロール選択
  static std::vector<double> SelectBestControl(
    const std::vector<std::array<double, 5>> & evalDB,
    const std::array<double, 4> & evalParam);
};

} // namespace dwa_planner
