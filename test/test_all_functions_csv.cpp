#include <gtest/gtest.h>
#include "dwa_planner/dwa_planner_component.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <cmath>
#include <stdexcept>

using namespace dwa_planner;

// --- CSV 読み込みユーティリティ --------------------------------
static std::vector<std::map<std::string, double>> readCsv(const std::string & path) {
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("Cannot open CSV: " + path);
  std::string line;
  std::vector<std::string> headers;
  // ヘッダ行
  if (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string col;
    while (std::getline(ss, col, ',')) {
      headers.push_back(col);
    }
  }
  // データ行
  std::vector<std::map<std::string,double>> rows;
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::map<std::string,double> row;
    for (size_t i = 0; i < headers.size() && std::getline(ss, cell, ','); ++i) {
      row[headers[i]] = std::stod(cell);
    }
    rows.push_back(row);
  }
  return rows;
}
// ----------------------------------------------------------------

// テスト対象関数を抽象化するための構造体
struct FuncTest {
  std::string name;
  std::vector<std::string> inCols;   // 入力カラム名
  std::vector<std::string> outCols;  // 期待値カラム名
  // CSV の１行 map<string,double> → 関数呼び出し → vector<double>（実際の戻り値）
  std::function<std::vector<double>(const std::map<std::string,double>&)> invoke;
};

TEST(DWA_AllFunctions, CsvBased) {
  const double tol = 1e-6;

  // 関数ごとの設定
  std::vector<FuncTest> tests = {
    {
      "DynamicWindowApproach",
      {"x0","x1","x2","x3","x4",
       "model0","model1","model2","model3","model4","model5",
       "goal0","goal1","eval0","eval1","eval2","eval3",
       "R","robotR"},
      {"exp_v","exp_w"},
      [](const auto &r){
        std::array<double,5> x     = {r.at("x0"),r.at("x1"),r.at("x2"),r.at("x3"),r.at("x4")};
        std::array<double,6> model = {
          r.at("model0"),r.at("model1"),r.at("model2"),
          r.at("model3"),r.at("model4"),r.at("model5")
        };
        std::array<double,2> goal  = {r.at("goal0"), r.at("goal1")};
        std::array<double,4> eval  = {
          r.at("eval0"),r.at("eval1"),r.at("eval2"),r.at("eval3")
        };
        std::vector<std::array<double,2>> ob; // 障害物なし
        double R      = r.at("R");
        double robotR = r.at("robotR");
        return DWA::DynamicWindowApproach(x, model, goal, eval, ob, R, robotR);
      }
    },
    {
      "CalcDynamicWindow",
      {"x3","x4","model0","model1","model2","model3","model4","model5"},
      {"exp_min_v","exp_max_v","exp_min_w","exp_max_w"},
      [](const auto &r){
        std::array<double,5> x     = {0,0,0,r.at("x3"),r.at("x4")};
        std::array<double,6> model = {
          r.at("model0"),r.at("model1"),r.at("model2"),
          r.at("model3"),r.at("model4"),r.at("model5")
        };
        auto out = DWA::CalcDynamicWindow(x, model);
        return std::vector<double>{out[0], out[1], out[2], out[3]};
      }
    },
    {
      "GenerateTrajectory",
      {"x0","x1","x2","x3","x4","vt","ot","evaldt"},
      {"exp_x0","exp_x1","exp_x2","exp_x3","exp_x4"},
      [](const auto &r){
        std::array<double,5> x     = {r.at("x0"),r.at("x1"),r.at("x2"),r.at("x3"),r.at("x4")};
        double vt   = r.at("vt");
        double ot   = r.at("ot");
        double evaldt = r.at("evaldt");
        auto out = DWA::GenerateTrajectory(x, vt, ot, evaldt);
        return std::vector<double>{out[0], out[1], out[2], out[3], out[4]};
      }
    },
    {
      "CalcHeadingEval",
      {"x0","x1","x2","goal0","goal1"},
      {"exp_heading"},
      [](const auto &r){
        std::array<double,5> x     = {r.at("x0"),r.at("x1"),r.at("x2"),0,0};
        std::array<double,2> goal  = {r.at("goal0"), r.at("goal1")};
        double h = DWA::CalcHeadingEval(x, goal);
        return std::vector<double>{h};
      }
    },
    {
      "CalcDistEval",
      {"x0","x1","x2","x3","x4","ob0x","ob0y","R","robotR"},
      {"exp_dist"},
      [](const auto &r){
        std::array<double,5> x     = {r.at("x0"),r.at("x1"),r.at("x2"),r.at("x3"),r.at("x4")};
        std::vector<std::array<double,2>> ob = {{r.at("ob0x"), r.at("ob0y")}};
        double R      = r.at("R");
        double robotR = r.at("robotR");
        double d = DWA::CalcDistEval(x, ob, R, robotR);
        return std::vector<double>{d};
      }
    },
    {
      "NormalizeEval",
      {"h0","d0","v0","h1","d1","v1"},
      {"exp_h0","exp_d0","exp_v0","exp_h1","exp_d1","exp_v1"},
      [](const auto &r){
        std::vector<std::array<double,5>> evalDB = {
          {0,0,r.at("h0"),r.at("d0"),r.at("v0")},
          {0,0,r.at("h1"),r.at("d1"),r.at("v1")}
        };
        DWA::NormalizeEval(evalDB);
        return std::vector<double>{
          evalDB[0][2], evalDB[0][3], evalDB[0][4],
          evalDB[1][2], evalDB[1][3], evalDB[1][4]
        };
      }
    },
    {
      "SelectBestControl",
      {"h0","d0","v0","h1","d1","v1","eval0","eval1","eval2","eval3"},
      {"exp_v","exp_w"},
      [](const auto &r){
        std::vector<std::array<double,5>> evalDB = {
          {0,0,r.at("h0"),r.at("d0"),r.at("v0")},
          {0,0,r.at("h1"),r.at("d1"),r.at("v1")}
        };
        std::array<double,4> evalParam = {
          r.at("eval0"), r.at("eval1"), r.at("eval2"), r.at("eval3")
        };
        auto u = DWA::SelectBestControl(evalDB, evalParam);
        return std::vector<double>{u[0], u[1]};
      }
    }
  };

  // 各関数ごとにテスト実行
  for (auto &ft : tests) {
    // 1) 入力 CSV 読み込み
    // auto rows = readCsv("test_data/" + ft.name + ".csv");
    auto rows = readCsv(ft.name + ".csv");

    // 2) 結果出力用 CSV
    std::ofstream ofs("test_results_" + ft.name + ".csv");
    ofs << "row";
    for (auto &c : ft.inCols)  ofs << "," << c;
    for (auto &c : ft.outCols) ofs << ",act_" << c << ",OK";
    ofs << "\n";

    // 3) 各行チェック
    for (size_t i = 0; i < rows.size(); ++i) {
      const auto &r = rows[i];
      auto result = ft.invoke(r);

      bool ok = true;
      for (size_t k = 0; k < ft.outCols.size(); ++k) {
        double expv = r.at(ft.outCols[k]);
        if (std::fabs(result[k] - expv) > tol) ok = false;
      }

      // CSV 書き出し
      ofs << i;
      for (auto &c : ft.inCols)  ofs << "," << r.at(c);
      for (size_t k = 0; k < ft.outCols.size(); ++k) {
        ofs << "," << result[k] << "," << (ok ? "OK" : "NG");
      }
      ofs << "\n";

      // GTest レポート
      EXPECT_TRUE(ok) << "[" << ft.name << "] row " << i;
    }
    ofs.close();
  }
}
