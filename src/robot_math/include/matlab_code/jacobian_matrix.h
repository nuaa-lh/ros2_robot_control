//
// File: jacobian_matrix.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 21:33:31
//
#ifndef JACOBIAN_MATRIX_H
#define JACOBIAN_MATRIX_H

// Include Files
#include "robot_math/robot_math.hpp"
#include <cstddef>
#include <cstdlib>

// Function Declarations, NO TCP 
extern void jacobian_matrix(const robot_math::Robot *robot, const coder::array<double,
  2U> &q, coder::array<double, 2U> &Jb, double T[16]);

#endif

//
// File trailer for jacobian_matrix.h
//
// [EOF]
//
