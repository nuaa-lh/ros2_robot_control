#include "ur_control/rcm_omega_control.hpp"

using namespace Eigen;
using namespace ur_rtde;
using namespace std::chrono;
using namespace std;
using namespace robot_math;

RcmOmegaControl::RcmOmegaControl() : Node("rcm_omega_control_node")
{
    // 启动RCM控制
    RCLCPP_INFO(this->get_logger(), "RCM Omega Control Node Initialized.");
    performRcmControl();
}

void RcmOmegaControl::performRcmControl()
{
    string package_share_directory = ament_index_cpp::get_package_share_directory("ur_control");
    string config_file_path = package_share_directory + "/config/calibrationUR10.conf";
    Robot robot = UR10e(config_file_path.c_str());
    RTDEReceiveInterface rtde_state("192.168.31.201");
    RTDEControlInterface rtde_control("192.168.31.201");
    std::vector<double> angleStart = {-1, -1.12, 1.63, -0.47, 1.43, -1.82};
    rtde_control.moveJ(angleStart, 0.5);
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
    
    Matrix4d Tcp = Matrix4d::Identity();
    double dt = 0.002;
   
    bool flag = false;
    double od, oa, ob, oc;
    double xyz[3], abc[3], gap;
    openHaptics();
    dhdEnableForce(DHD_OFF);
    auto pose = rtde_state.getActualTCPPose();
    Matrix4d T_init = pose2T(pose);
    Matrix4d dT, T;
    double p[4] = {0}, v[4];
    double rcm[] = {0.198,0,0.096};  // rcm精度实验的rcm位置
    Tcp(2, 3) = rcm[2];
    double tol[] = {1e-7, 1e-5};
    double flag_inv;

    std::vector<double> qt(6);

    Eigen::Matrix4d Td = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_corrected = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_corrected;
    R_corrected = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
    T_corrected.block(0, 0, 3, 3) = R_corrected;
    Eigen::Vector3d rcm_vector(rcm[0], rcm[1], rcm[2]);
    Eigen::Vector3d rcm_corrected = R_corrected.transpose() * rcm_vector;
    rcm[0] = rcm_corrected(0);
    rcm[1] = rcm_corrected(1);
    rcm[2] = rcm_corrected(2);

    MovingFilter<double> pFilter(4);

    Matrix4d Td_pre = T_init;
    RobotData robotData = {0};
    
    auto t_zero = high_resolution_clock::now();
    while (!rtde_state.getDigitalOutState(0))
    {
        auto t_start = high_resolution_clock::now();
        robotData.t = std::chrono::duration<double>(t_start - t_zero).count();
        auto q = rtde_state.getActualQ();

        auto pose = rtde_state.getActualTCPPose();

        dhdGetPositionAndOrientationRad(xyz, xyz + 1, xyz + 2, abc, abc + 1, abc + 2);
        log2Channel(robotData, 0, xyz, 3);
        log2Channel(robotData, 1, q.data(), 6);
        if (flag == false)
        {
            flag = true;
            od = xyz[0];
            oa = abc[0];
            ob = abc[1];
            oc = abc[2];
        }
        else
        {
            double dabc[] = {od - xyz[0], abc[0] - oa, abc[1] - ob, abc[2] - oc};
            memcpy(p, dabc, 4 * sizeof(double));
        }
        pFilter.filtering(p, p);
        generateRCM(p[0], p[1], -p[3], p[2], rcm, dT.data());

        //Td = T_init * dT;
        Td = T_init * T_corrected * dT * invertT(T_corrected);

        inverse_kin_general(&robot, Td, q, tol, qt, &flag_inv);
        if (flag_inv == 1.0)
        {
            rtde_control.servoJ(qt, 0.5, 0.5, dt, 0.1, 500);
        }
        else
        {
            std::cout << "failure!!!\n";
            break;
        }

        DataComm::getInstance()->sendRobotStatus(robotData);
        auto t_stop = high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);
        // std::cout << "time: " << t_duration.count() << "\n";
        if (t_duration.count() < dt)
            std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
    
    closeHaptics();
    // rtde_control.speedStop();
    rtde_control.servoStop();
    rtde_control.stopScript();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RcmOmegaControl>());
    rclcpp::shutdown();
    return 0;
}
