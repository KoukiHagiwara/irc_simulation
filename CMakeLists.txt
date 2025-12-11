cmake_minimum_required(VERSION 3.8)
project(irc_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# =========================================
# 1. C++ノードのビルド設定 
# =========================================
add_executable(ball_chaser src/ball_chaser.cpp)
ament_target_dependencies(ball_chaser rclcpp geometry_msgs std_msgs sensor_msgs)

# =========================================
# 2. 実行ファイル(TARGETS)のインストール
# =========================================
install(TARGETS
  ball_chaser
  DESTINATION lib/${PROJECT_NAME}
)

# Pythonスクリプトを実行ファイルとしてインストールする命令
install(PROGRAMS
  src/ball_color_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================
# 3. ディレクトリ(DIRECTORY)のインストール
# =========================================
install(DIRECTORY
  launch
  worlds
  models
  config
  weights
  DESTINATION share/${PROJECT_NAME}
)

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
