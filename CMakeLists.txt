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
# テストターゲットの定義
#==========================================================
if(BUILD_TESTING)
  ament_add_gtest(test_dwa_component
    test/test_dwa_component.cpp
  )
  if(TARGET test_dwa_component)
    target_link_libraries(test_dwa_component dwa_planner_library)
    target_compile_options(test_dwa_component PUBLIC -fprofile-arcs -ftest-coverage)
    target_link_options(test_dwa_component PUBLIC -fprofile-arcs -ftest-coverage)

    target_include_directories(test_dwa_component PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    ament_target_dependencies(test_dwa_component
      rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
    )
  endif()

  # ── CSV ベースのテストターゲットを追加 ───────────────────
  # 1) テストユーティリティヘッダを 'test/test_csv_utils.hpp' に置くこと
  # 2) テストコードを 'test/test_dwa_planner_csv.cpp' として作成すること
  ament_add_gtest(test_dwa_csv
    test/test_dwa_planner_csv.cpp
  )
  if(TARGET test_dwa_csv)
    # ライブラリ本体をリンク
    target_link_libraries(test_dwa_csv dwa_planner_library)
    #target_compile_options(test_dwa_csv PUBLIC -fprofile-arcs -ftest-coverage)
    #target_link_options(test_dwa_csv PUBLIC -fprofile-arcs -ftest-coverage)

    # include パスに <project_root>/include と <project_root>/test を追加
    target_include_directories(test_dwa_csv PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  
    # 依存パッケージを追加
    ament_target_dependencies(test_dwa_csv
      rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
    )

    # ビルド後、自動で CSV ファイルをテスト実行ディレクトリにコピー
    add_custom_command(TARGET test_dwa_csv POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_data.csv
        $<TARGET_FILE_DIR:test_dwa_csv>/test_data.csv
      ) 
  endif()

  # ── CSV ベースの全関数対応 CSV テストを追加 ───────────────────
  ament_add_gtest(test_dwa_all_functions
    test/test_all_functions_csv.cpp
  )
  if (TARGET test_dwa_all_functions)
    target_link_libraries(test_dwa_all_functions dwa_planner_library)
    target_include_directories(test_dwa_all_functions PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
    ament_target_dependencies(test_dwa_all_functions
      rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
    )

    # # テスト用 CSV データをコピー
    # add_custom_command(TARGET test_dwa_all_functions POST_BUILD
    #   COMMAND ${CMAKE_COMMAND} -E copy_directory
    #     ${CMAKE_CURRENT_SOURCE_DIR}/test/test_data
    #     ${CMAKE_CURRENT_BINARY_DIR}/test_data
    # )
    # # 実行時カレントディレクトリをバイナリルートに設定
    # set_tests_properties(test_dwa_all_functions PROPERTIES
    #   WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    #   #WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data"
    # )

    set_tests_properties(test_dwa_all_functions PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data"
    )
  endif()

  # カバレッジ生成ターゲットの追加
  find_program(LCOV_EXEC lcov)
  find_program(GENHTML_EXEC genhtml)
  if(LCOV_EXEC AND GENHTML_EXEC)
    add_custom_target(coverage
      COMMAND ${LCOV_EXEC} --directory . --capture --output-file coverage.info
      COMMAND ${LCOV_EXEC} --remove coverage.info '/usr/*' '/opt/ros/*' --output-file coverage.filtered.info
      COMMAND ${GENHTML_EXEC} coverage.filtered.info --output-directory coverage_html
      COMMENT "Generating coverage report..."
    )
  endif()
endif()

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
# if(BUILD_TESTING)
#   find_program(LCOV_EXEC lcov)
#   find_program(GENHTML_EXEC genhtml)
#   if(LCOV_EXEC AND GENHTML_EXEC)
#     add_custom_target(coverage
#       COMMAND ${LCOV_EXEC} --directory . --capture --output-file coverage.info
#       COMMAND ${GENHTML_EXEC} coverage.info --output-directory coverage_html
#       COMMENT "Generating coverage report..."
#     )
#   endif()
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#==========================================================
# パッケージ設定
#==========================================================
ament_package()
