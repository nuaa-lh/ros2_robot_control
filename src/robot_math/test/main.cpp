#include "robot_math/robot_math.hpp"
#include "robot_math/MovingFilter.h"
#include "robot_math/OnlineTrajPlanner.h"
#include "matlab_code/logR.h"

#include <iostream>
using namespace robot_math;
using namespace std;
using namespace Eigen;

int main()
{
    Robot r;
    const double R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double w[3] = {0,0,0};
    logR(R, w);
    return 0;
}