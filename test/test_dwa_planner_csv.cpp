// // test/test_dwa_planner_csv.cpp

// #include <gtest/gtest.h>
// #include "dwa_planner/dwa_planner_component.hpp"

// #include <fstream>
// #include <sstream>
// #include <string>
// #include <vector>
// #include <map>
// #include <stdexcept>

// // --- CSV 読み込みユーティリティをここにインライン定義 ----------------
// static std::vector<std::map<std::string, double>> readCsv(const std::string & path) {
//   std::ifstream ifs(path);
//   if (!ifs) {
//     throw std::runtime_error("Cannot open CSV file: " + path);
//   }
//   std::string line;
//   // ヘッダ行の読み込み
//   std::vector<std::string> headers;
//   if (std::getline(ifs, line)) {
//     std::stringstream ss(line);
//     std::string col;
//     while (std::getline(ss, col, ',')) {
//       headers.push_back(col);
//     }
//   }
//   // データ行の読み込み
//   std::vector<std::map<std::string, double>> rows;
//   while (std::getline(ifs, line)) {
//     std::stringstream ss(line);
//     std::string cell;
//     std::map<std::string, double> row;
//     size_t idx = 0;
//     while (std::getline(ss, cell, ',')) {
//       if (idx < headers.size()) {
//         row[headers[idx]] = std::stod(cell);
//       }
//       ++idx;
//     }
//     rows.push_back(row);
//   }
//   return rows;
// }
// // ----------------------------------------------------------------------


// using namespace dwa_planner;

// TEST(TestDWA_CsvBased, AllRows) {
//   const std::string csv_path = "test_data.csv";
//   auto rows = readCsv(csv_path);

//   for (size_t i = 0; i < rows.size(); ++i) {
//     const auto &r = rows[i];

//     std::array<double,5> x = {
//       r.at("x0"), r.at("x1"), r.at("x2"), r.at("x3"), r.at("x4")
//     };
//     std::array<double,6> model = {
//       r.at("model0"), r.at("model1"), r.at("model2"),
//       r.at("model3"), r.at("model4"), r.at("model5")
//     };
//     std::array<double,2> goal = { r.at("goal0"), r.at("goal1") };
//     std::array<double,4> eval = {
//       r.at("eval0"), r.at("eval1"), r.at("eval2"), r.at("eval3")
//     };
//     // この例では障害物なし
//     std::vector<std::array<double,2>> ob;
//     double R      = r.at("R");
//     double robotR = r.at("robotR");

//     auto result = DWA::DynamicWindowApproach(x, model, goal, eval, ob, R, robotR);

//     double exp_v = r.at("exp_v");
//     double exp_w = r.at("exp_w");

//     EXPECT_EQ(result[0], exp_v);
//     EXPECT_NEAR(result[0], exp_v, 1e-6) << "row " << i;
//     EXPECT_NEAR(result[1], exp_w, 1e-6) << "row " << i;
//   }
// }

// test/test_dwa_planner_csv.cpp

#include <gtest/gtest.h>
#include "dwa_planner/dwa_planner_component.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <stdexcept>

// --- CSV 読み込みユーティリティ --------------------------------
static std::vector<std::map<std::string, double>> readCsv(const std::string & path) {
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("Cannot open CSV file: " + path);
  std::string line;
  std::vector<std::string> headers;
  // ヘッダ行
  if (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string col;
    while (std::getline(ss, col, ',')) headers.push_back(col);
  }
  // データ行
  std::vector<std::map<std::string, double>> rows;
  while (std::getline(ifs, line)) {
    std::stringstream ss(line);
    std::string cell;
    std::map<std::string, double> row;
    for (size_t i = 0; std::getline(ss, cell, ',') && i < headers.size(); ++i) {
      row[headers[i]] = std::stod(cell);
    }
    rows.push_back(row);
  }
  return rows;
}
// ----------------------------------------------------------------

using namespace dwa_planner;

TEST(TestDWA_CsvBased, AllRows) {
  const auto rows = readCsv("test_data.csv");

  // テスト結果を出力する CSV
  std::ofstream ofs("test_results.csv");
  ofs << "row,exp_v,act_v,exp_w,act_w,OK\n";

  const double tol = 1e-6;
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto &r = rows[i];

    // 入力＆期待値をセット
    std::array<double,5> x     = {r.at("x0"), r.at("x1"), r.at("x2"), r.at("x3"), r.at("x4")};
    std::array<double,6> model = {r.at("model0"), r.at("model1"), r.at("model2"),
                                  r.at("model3"), r.at("model4"), r.at("model5")};
    std::array<double,2> goal  = {r.at("goal0"), r.at("goal1")};
    std::array<double,4> eval  = {r.at("eval0"), r.at("eval1"), r.at("eval2"), r.at("eval3")};
    std::vector<std::array<double,2>> ob;               // 障害物なし
    double R      = r.at("R");
    double robotR = r.at("robotR");
    double exp_v  = r.at("exp_v");
    double exp_w  = r.at("exp_w");

    // 実行
    auto result = DWA::DynamicWindowApproach(x, model, goal, eval, ob, R, robotR);

    // 判定
    bool ok_v = std::fabs(result[0] - exp_v) <= tol;
    bool ok_w = std::fabs(result[1] - exp_w) <= tol;
    bool ok   = ok_v && ok_w;

    // CSV に書き出し
    ofs
      << i << ","
      << exp_v  << "," << result[0] << ","
      << exp_w  << "," << result[1] << ","
      << (ok ? "OK" : "NG")
      << "\n";

    // GTest 上も失敗を報告するが、EXPECT_* のため次ループに進む
    EXPECT_TRUE(ok) << "Row " << i
      << "  v:" << result[0] << "(exp " << exp_v << ")"
      << "  w:" << result[1] << "(exp " << exp_w << ")";
  }

  ofs.close();
}
