cmake_minimum_required(VERSION 3.8)
project(dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# pure_pursuit_planner
add_executable(dwa_planner src/dwa_planner_node.cpp src/dwa_planner_component.cpp src/dwa_planner.cpp)
ament_target_dependencies(dwa_planner
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)
target_include_directories(dwa_planner PUBLIC
  include
)

# Install Cpp executables
install(TARGETS
  dwa_planner
  DESTINATION lib/${PROJECT_NAME}
)

# # include/ ディレクトリをインクルードパスに追加
# include_directories(
#   include
# )

# # ----------------------------------------------------------------------------
# # 1) DWAプランナー本体のコンポーネント（dwa_planner_component.cpp）をライブラリ化
# # ----------------------------------------------------------------------------
# add_library(dwa_planner_component SHARED
#   src/dwa_planner_component.cpp
# )
# ament_target_dependencies(dwa_planner_component
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   std_msgs
# )

# # ----------------------------------------------------------------------------
# # 2) ROSノード部分（dwa_planner_node.cpp）を別ライブラリとして定義
# #    → Node本体を実装し、上記ライブラリを利用してDWA計算を行う
# # ----------------------------------------------------------------------------
# add_library(dwa_planner_node SHARED
#   src/dwa_planner_node.cpp
# )
# ament_target_dependencies(dwa_planner_node
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   std_msgs
# )

# # このノードライブラリは、DWAコンポーネントライブラリにリンク
# target_link_libraries(dwa_planner_node
#   dwa_planner_component
# )

# # ----------------------------------------------------------------------------
# # 3) メイン実行ファイル（dwa_planner.cpp）をビルド
# #    → ノードを立ち上げてspinするエントリーポイント
# # ----------------------------------------------------------------------------
# add_executable(dwa_planner
#   src/dwa_planner.cpp
# )
# ament_target_dependencies(dwa_planner
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   std_msgs
# )
# # メイン実行ファイルは、上記2つのライブラリをリンク
# target_link_libraries(dwa_planner
#   dwa_planner_component
#   dwa_planner_node
# )

# # ----------------------------------------------------------------------------
# # インストール設定
# # ----------------------------------------------------------------------------
# install(TARGETS
#   dwa_planner_component
#   dwa_planner_node
#   dwa_planner
#   DESTINATION lib/${PROJECT_NAME}
# )

# # add_executable(dwa_planner src/dwa_planner.cpp)
# # ament_target_dependencies(dwa_planner 
# #   rclcpp geometry_msgs nav_msgs visualization_msgs std_msgs tf2_geometry_msgs tf2_ros
# # )

# # add_executable(dwa_node src/dwa_node.cpp)
# # ament_target_dependencies(dwa_node rclcpp geometry_msgs std_msgs nav_msgs tf2_geometry_msgs tf2_ros visualization_msgs)

# # install(TARGETS
# #   dwa_planner
# #   dwa_node
# #   DESTINATION lib/${PROJECT_NAME}
# # )

# Install other files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# # ヘッダファイルをインストール
# install(DIRECTORY include/
#   DESTINATION include/
# )

# # ライブラリをエクスポート
# ament_export_libraries(dwa_planner_component dwa_planner_node)
# ament_export_include_directories(include)
# ament_export_dependencies(
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   std_msgs
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
