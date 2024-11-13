cmake_minimum_required(VERSION 3.8)
project(rcm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# HW_IF_INCLUDE_DEPENDS 包含了用于硬件接口的依赖库
set(HW_IF_INCLUDE_DEPENDS
  pluginlib              # ROS插件库，用于加载和管理插件
  rcpputils              # 提供了一些ROS C++的辅助工具和功能
  hardware_interface     # ROS 2硬件接口库，用于定义机器人硬件接口
)

# REF_GEN_INCLUDE_DEPENDS 包含了用于参考轨迹生成的依赖库
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser             # KDL解析器库，用于将URDF模型解析为KDL树结构
  rclcpp                 # ROS 2 C++客户端库，用于创建和管理ROS 2节点
  trajectory_msgs        # ROS消息类型库，包含轨迹消息格式，用于轨迹生成和控制
)

# CONTROLLER_INCLUDE_DEPENDS 包含了用于控制器的依赖库
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib              # ROS插件库，用于加载和管理插件（用于控制器）
  rcpputils              # 提供了一些ROS C++的辅助工具和功能（用于控制器）
  controller_interface   # ROS 2控制器接口库，用于定义和实现控制器接口
  realtime_tools         # 实时工具库，提供控制实时性能的辅助工具
  trajectory_msgs        # ROS消息类型库，包含轨迹消息格式，用于发送控制轨迹
)

find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

# add_library(rcm_controller SHARED
#             src/rcm_controller.cpp
#           )

# target_compile_features(rcm_controller PUBLIC cxx_std_17)
# target_include_directories(rcm_controller PUBLIC
#     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include/rcm_controller>
# )

# ament_target_dependencies(
#     rcm_controller PUBLIC
#     ${HW_IF_INCLUDE_DEPENDS}
#     ${CONTROLLER_INCLUDE_DEPENDS}
# )

# # Export controller plugins
# pluginlib_export_plugin_description_file(rcm_controller resource/rcm_controller.xml)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# # INSTALL
# install(TARGETS rcm_controller
#         DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/${PROJECT_NAME}
#         DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY resource launch config
#         DESTINATION share/${PROJECT_NAME})

# install(TARGETS rcm_controller
#   EXPORT export_rcm_controller
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ## EXPORTS
# ament_export_targets(export_rcm_controller HAS_LIBRARY_TARGET)
# ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
