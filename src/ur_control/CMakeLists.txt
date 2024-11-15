cmake_minimum_required(VERSION 3.8)
project(ur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 节点所需的依赖
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED) # get_package_share_directory方法
find_package(rclcpp REQUIRED)
find_package(ur_rtde REQUIRED) # RTDE
find_package(Eigen3 REQUIRED)
find_package(robot_math REQUIRED) # 数学包
find_package(jsoncpp REQUIRED)

# 添加可执行文件
add_executable(rcm_omega_control src/rcm_omega_control.cpp)

# 设置头文件的包含路径
target_include_directories(rcm_omega_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ur_control>
    $<BUILD_INTERFACE:/home/eric/Desktop/ros2_robot_control/install/robot_math/include> # 必须使用绝对路径指定 否则找不到
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

# 将外部依赖库链接到 rcm_omega_control 节点
target_link_libraries(rcm_omega_control PRIVATE ur_rtde::rtde)

# 使用 ament_target_dependencies 宏链接依赖
ament_target_dependencies(
    rcm_omega_control PUBLIC
    rclcpp
    ament_index_cpp
    Eigen3
    robot_math
    jsoncpp
)

# 安装可执行文件到 install/ur_control/lib 目录
install(TARGETS rcm_omega_control
        DESTINATION lib/${PROJECT_NAME})

# 安装 config 目录中的文件到 install/ur_control/share/ur_control/config 目录
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()