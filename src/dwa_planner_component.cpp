#include "dwa_planner/dwa_planner_component.hpp"
#include <iostream>

// --- 返り値用の構造体を定義 ---
struct DWAResult {
  std::vector<double> control;                      // [v, ω]
  std::vector<std::vector<std::array<double, 5>>> trajectories;  // 各軌跡 xt を格納
};

namespace dwa_planner
{

static inline double normalizeAngle(double a) {
  // normalize to [-pi, pi)
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

DWAResult DWA::DynamicWindowApproach(
  const std::array<double, 5> & x,
  const std::array<double, 6> & model,
  const std::array<double, 2> & goal,
  const std::array<double, 4> & evalParam,
  const std::vector<std::array<double, 2>> & ob,
  double R,
  double robotR
)
{
  auto Vr = CalcDynamicWindow(x, model);
  std::vector<std::array<double, 5>> evalDB;
  std::vector<std::vector<std::array<double, 5>>> trajectory_list;

  for (double vt = Vr[0]; vt <= Vr[1]; vt += model[4]) {
    for (double ot = Vr[2]; ot <= Vr[3]; ot += model[5]) {
      std::vector<std::array<double, 5>> xt = GenerateTrajectory(x, vt, ot, evalParam[3]);

      double heading = CalcHeadingEval(xt.back(), goal);
      double dist = CalcDistEval(xt.back(), ob, R, robotR);
      double vel = std::fabs(vt);

      if (dist < 0.05) {
        continue;                // 衝突→skip
      }
      /*std::cout << "score: "
            << "heading=" << heading << ", "
            << "dist=" << dist << ", "
            << "vel=" << vel << std::endl;*/

      //std::cout << "ot: " << ot << std::endl;
      evalDB.push_back({vt, ot, heading, dist, vel});
      trajectory_list.push_back(xt);
    }
  }

  if (evalDB.empty()) {
    std::cout << "No path to goal!" << std::endl;
    return {{0.0, 0.0}, trajectory_list};
  }

  NormalizeEval(evalDB);
  auto bestControl = SelectBestControl(evalDB, evalParam);
  return {bestControl, trajectory_list};
}

std::array<double, 4> DWA::CalcDynamicWindow(
  const std::array<double, 5> & x,
  const std::array<double, 6> & model)
{
  double v_min = std::max(0.00, x[3] - model[2] * DT);
  double v_max = std::min(model[0], x[3] + model[2] * DT);
  double w_min = std::max(-model[1], x[4] - model[3] * DT);
  double w_max = std::min(model[1], x[4] + model[3] * DT);
  /*std::cout << "DT: "
            << DT << std::endl;
  std::cout << "Dynamic Window: "
            << "v_min=" << v_min << ", "
            << "v_max=" << v_max << ", "
            << "w_min=" << w_min << ", "
            << "w_max=" << w_max << std::endl;*/
  return {v_min, v_max, w_min, w_max};
}

std::vector<std::array<double, 5>> DWA::GenerateTrajectory(
  const std::array<double, 5> & x,
  double vt, double ot, double evaldt)
{
  std::vector<std::array<double, 5>> trajectory;
  auto xt = x;
  //std::cout << "evaldt: " << evaldt << std::endl;
  for (double t = 0.0; t <= evaldt; t += DT) {
    xt[2] = normalizeAngle(xt[2]);
    xt[0] += DT * std::cos(xt[2]) * vt;
    xt[1] += DT * std::sin(xt[2]) * vt;
    xt[2] += normalizeAngle(DT * ot);
    xt[3] = vt;
    xt[4] = ot;
    trajectory.push_back(xt);
  }
  return trajectory;
}

double DWA::CalcHeadingEval(
  const std::array<double, 5> & x,
  const std::array<double, 2> & goal)
{
  double targetTheta = TO_DEGREE(std::atan2(goal[1] - x[1], goal[0] - x[0]));
  double diff = std::fabs(TO_DEGREE(x[2]) - targetTheta);
  return 180.0 - std::min(diff, 360.0 - diff);
}

double DWA::CalcDistEval(
  const std::array<double, 5> & x,
  const std::vector<std::array<double, 2>> & ob,
  double R,
  double robotR)
{
  // 障害物が存在しない場合
  if (ob.empty()) {
    std::cout << "no dist" << std::endl;
    return 1000.0;  // 安全な大きな値を返す
  }
  // double min_dist = std::numeric_limits<double>::infinity();
  double min_dist = 1000.0;

  for (const auto & o : ob) {
    double dist = std::hypot(o[0] - x[0], o[1] - x[1]) - (R + robotR);
    if (dist < min_dist) {
      min_dist = dist;
      std::cout << "dist: " << dist << std::endl;
    }
  }

  return min_dist;
}

void DWA::NormalizeEval(std::vector<std::array<double, 5>> & evalDB)
{
  double sum_heading = 0.0, sum_dist = 0.0, sum_vel = 0.0;
  for (auto & e: evalDB) {
    sum_heading += e[2];
    sum_dist += e[3];
    sum_vel += e[4];
  }
  for (auto & e: evalDB) {
    if (std::fabs(sum_heading) > 1e-9) {e[2] /= sum_heading;}
    if (std::fabs(sum_dist) > 1e-9) {e[3] /= sum_dist;}
    if (std::fabs(sum_vel) > 1e-9) {e[4] /= sum_vel;}
  }
}

std::vector<double> DWA::SelectBestControl(
  const std::vector<std::array<double, 5>> & evalDB,
  const std::array<double, 4> & evalParam)
{
  double max_score = -100000;
  std::vector<double> best_u{0.0, 0.0};
  
  
  for (auto & e : evalDB) {
    /*std::cout << "score: "
            << "heading=" << e[2] << ", "
            << "dist=" << e[3] << ", "
            << "vel=" << e[4] << ", "
            << "v=" << e[0] << ", "
            << "w=" << e[1] << ", " << std::endl;*/
    //std::cout << "e: " << e[1] << std::endl;
    double score = evalParam[0] * e[2] +
      evalParam[1] * e[3] +
      evalParam[2] * e[4];
      //std::cout << "score: " << score << std::endl;
    if (score > max_score) {
      max_score = score;
      best_u[0] = e[0];
      best_u[1] = e[1];
    }
    //std::cout << "best_u: " << best_u[1] << std::endl;
  }
  //std::cout << "best_u: " << best_u[0] << best_u[1] << std::endl;
  return best_u;
}

} // namespace dwa_planner
