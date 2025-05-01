#include "dwa_planner/dwa_planner_component.hpp"

namespace dwa_planner
{

std::vector<double> DWA::DynamicWindowApproach(
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

  for (double vt = Vr[0]; vt <= Vr[1]; vt += model[4]) {
    for (double ot = Vr[2]; ot <= Vr[3]; ot += model[5]) {
      auto xt = GenerateTrajectory(x, vt, ot, evalParam[3]);

      double heading = CalcHeadingEval(xt, goal);
      double dist = CalcDistEval(xt, ob, R, robotR);
      double vel = std::fabs(vt);

      if (dist < 0.0) {
        continue;                // 衝突→skip

      }
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

std::array<double, 4> DWA::CalcDynamicWindow(
  const std::array<double, 5> & x,
  const std::array<double, 6> & model)
{
  double v_min = std::max(0.0, x[3] - model[2] * DT);
  double v_max = std::min(model[0], x[3] + model[2] * DT);
  double w_min = std::max(-model[1], x[4] - model[3] * DT);
  double w_max = std::min(model[1], x[4] + model[3] * DT);

  return {v_min, v_max, w_min, w_max};
}

std::array<double, 5> DWA::GenerateTrajectory(
  const std::array<double, 5> & x,
  double vt, double ot, double evaldt)
{
  auto xt = x;
  for (double t = 0.0; t <= evaldt; t += DT) {
    xt[0] += DT * std::cos(xt[2]) * vt;
    xt[1] += DT * std::sin(xt[2]) * vt;
    xt[2] += DT * ot;
    xt[3] = vt;
    xt[4] = ot;
  }
  return xt;
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
  // double min_dist = std::numeric_limits<double>::infinity();
  double min_dist = 1000;

  for (const auto & o : ob) {
    double dist = std::hypot(o[0] - x[0], o[1] - x[1]) - (R + robotR);
    if (dist < min_dist) {
      min_dist = dist;
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
  double max_score = -std::numeric_limits<double>::infinity();
  std::vector<double> best_u{0.0, 0.0};

  for (auto & e : evalDB) {
    double score = evalParam[0] * e[2] +
      evalParam[1] * e[3] +
      evalParam[2] * e[4];
    if (score > max_score) {
      max_score = score;
      best_u[0] = e[0];
      best_u[1] = e[1];
    }
  }
  return best_u;
}

} // namespace dwa_planner
