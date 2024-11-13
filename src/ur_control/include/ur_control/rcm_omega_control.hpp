#ifndef RCM_OMEGA_CONTROL_HPP
#define RCM_OMEGA_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

#include "robot_math/matlab_code/exp_twist.h"
#include "robot_math/robot_math/MovingFilter.h"

class RcmOmegaControl : public rclcpp::Node
{
public:
    RcmOmegaControl();
    void performRcmControl();
};

#endif // RCM_OMEGA_CONTROL_HPP
