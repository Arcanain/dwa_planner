cmake_minimum_required(VERSION 3.8)
project(dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage")

#==========================================================
# 必要な依存関係の検索
#==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

#==========================================================
# ライブラリターゲットの定義
#==========================================================
# DWA Plannerのメインライブラリ
add_library(dwa_planner_library
  src/dwa_planner_node.cpp
  src/dwa_planner_component.cpp
)
ament_target_dependencies(dwa_planner_library
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)
target_include_directories(dwa_planner_library PUBLIC
  include
)

#==========================================================
# 実行可能ターゲットの定義
#==========================================================
# DWA Plannerのメインノード
add_executable(dwa_planner src/dwa_planner.cpp)
target_link_libraries(dwa_planner PRIVATE dwa_planner_library)



#==========================================================
# ファイルのインストール設定
#==========================================================
# 実行可能ファイルのインストール
install(TARGETS
  dwa_planner
  dwa_planner_library
  DESTINATION lib/${PROJECT_NAME}
)

# includeディレクトリのインストール
install(DIRECTORY include/
  DESTINATION include/
)

# YAML 設定ファイルをインストール
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# その他のファイルのインストール
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

#==========================================================
# Lintやカバレッジ設定 (オプション)
#==========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#==========================================================
# パッケージ設定
#==========================================================
ament_package()
