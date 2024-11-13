//
// File: forward_kin_general.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 23:12:51
//
#ifndef FORWARD_KIN_GENERAL_H
#define FORWARD_KIN_GENERAL_H

// Include Files
#include "robot_math/robot_math.hpp"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void forward_kin_general(const robot_math::Robot *robot, const coder::array<
  double, 2U> &q, double T[16]);

#endif

//
// File trailer for forward_kin_general.h
//
// [EOF]
//
