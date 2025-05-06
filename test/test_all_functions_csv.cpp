#include <gtest/gtest.h>
#include "test_utils.hpp"                       // test utility（必ずインクルード）
#include "dwa_planner/dwa_planner_component.hpp"    // テスト対象

using namespace dwa_planner;
using test_utils::FuncTest;
using test_utils::runCsvTests;

const double tol = 1e-3;

//-------------------------
// DynamicWindowApproach
//-------------------------
TEST(DynamicWindowApproach, CsvBased) {
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
        }
    };

    runCsvTests(tests, tol);
}

//-------------------------
// CalcDynamicWindow
//-------------------------
TEST(CalcDynamicWindow, CsvBased) {
    std::vector<FuncTest> tests = {
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
        }
    };
  
    runCsvTests(tests, tol);
}

//-------------------------
// GenerateTrajectory
//-------------------------
TEST(GenerateTrajectory, CsvBased) {
    std::vector<FuncTest> tests = {
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
        }
    };
  
    runCsvTests(tests, tol);
}

//-------------------------
// CalcHeadingEval
//-------------------------
TEST(CalcHeadingEval, CsvBased) {
    std::vector<FuncTest> tests = {
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
        }
    };
  
    runCsvTests(tests, tol);
}

//-------------------------
// CalcDistEval
//-------------------------
TEST(CalcDistEval, CsvBased) {
    std::vector<FuncTest> tests = {
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
        }
    };
  
    runCsvTests(tests, tol);
}

//-------------------------
// NormalizeEval
//-------------------------
TEST(NormalizeEval, CsvBased) {
    std::vector<FuncTest> tests = {
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
        }
    };
  
    runCsvTests(tests, tol);
}

//-------------------------
// SelectBestControl
//-------------------------
TEST(SelectBestControl, CsvBased) {
    std::vector<FuncTest> tests = {
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
  
    runCsvTests(tests, tol);
}