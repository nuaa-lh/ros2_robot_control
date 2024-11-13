cmake_minimum_required(VERSION 3.8)
project(ur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robot_math REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(rcm_omega_control src/rcm_omega_control.cpp)

target_include_directories(rcm_omega_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ur_control>
  $<BUILD_INTERFACE:/home/eric/Desktop/ros2_robot_control/install/robot_math/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
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

ament_target_dependencies(
    rcm_omega_control PUBLIC
    rclcpp
    ur_rtde
    Eigen3
    robot_math
    jsoncpp
)

install(TARGETS rcm_omega_control
        DESTINATION lib/${PROJECT_NAME})

ament_package()
