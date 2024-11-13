#include "ur_control/rcm_omega_control.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>

using namespace Eigen;
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
using namespace robot_math;

RcmOmegaControl::RcmOmegaControl() : Node("rcm_omega_control_node")
{
    // 启动RCM控制
    performRcmControl();
    MovingFilter<double> pFilter(4);
    RCLCPP_INFO(this->get_logger(), "RCM Omega Control Node Initialized.");
}

void RcmOmegaControl::performRcmControl()
{
    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RcmOmegaControl>());
    rclcpp::shutdown();
    return 0;
}
