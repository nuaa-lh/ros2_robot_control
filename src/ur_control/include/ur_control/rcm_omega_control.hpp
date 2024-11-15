#ifndef RCM_OMEGA_CONTROL_HPP
#define RCM_OMEGA_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Dense>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "robot_math/matlab_code/exp_twist.h"
#include "robot_math/matlab_code/generateRCM.h"
#include "robot_math/robot_math/MovingFilter.h"
#include "robot_math/robot_math/DataComm.h"
#include "robot_math/robot_math/robot_math.hpp"
#include "robot_math/devices/omega.h"
#include "robot_math/robot_math/UR5e.h"
#include "robot_math/robot_math/UR10e.h"

class RcmOmegaControl : public rclcpp::Node
{
public:
    RcmOmegaControl();
    void performRcmControl();
};

#endif // RCM_OMEGA_CONTROL_HPP
