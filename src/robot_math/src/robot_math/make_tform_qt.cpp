//
// File: make_tform_qt.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//

// Include Files
#include "make_tform_qt.h"

// Function Definitions
//
// Arguments    : const double qt_data[]
//                double T[16]
// Return Type  : void
//
void make_tform_qt(const double qt_data[], double T[16])
{
  double R[9];
  double R_tmp;
  double b_R_tmp;
  double c_R_tmp;
  double d_R_tmp;
  double e_R_tmp;
  double f_R_tmp;
  double g_R_tmp;
  double h_R_tmp;
  R_tmp = qt_data[0] * qt_data[0];
  b_R_tmp = qt_data[1] * qt_data[1];
  c_R_tmp = qt_data[2] * qt_data[2];
  d_R_tmp = qt_data[3] * qt_data[3];
  R[0] = ((R_tmp + b_R_tmp) - c_R_tmp) - d_R_tmp;
  e_R_tmp = qt_data[1] * qt_data[2];
  f_R_tmp = qt_data[0] * qt_data[3];
  R[3] = 2.0 * (e_R_tmp - f_R_tmp);
  g_R_tmp = qt_data[1] * qt_data[3];
  h_R_tmp = qt_data[0] * qt_data[2];
  R[6] = 2.0 * (g_R_tmp + h_R_tmp);
  R[1] = 2.0 * (e_R_tmp + f_R_tmp);
  R_tmp -= b_R_tmp;
  R[4] = (R_tmp + c_R_tmp) - d_R_tmp;
  b_R_tmp = qt_data[2] * qt_data[3];
  e_R_tmp = qt_data[0] * qt_data[1];
  R[7] = 2.0 * (b_R_tmp - e_R_tmp);
  R[2] = 2.0 * (g_R_tmp - h_R_tmp);
  R[5] = 2.0 * (b_R_tmp + e_R_tmp);
  R[8] = (R_tmp - c_R_tmp) + d_R_tmp;
  for (int i = 0; i < 3; i++) {
    int T_tmp;
    T_tmp = i << 2;
    T[T_tmp] = R[3 * i];
    T[T_tmp + 1] = R[3 * i + 1];
    T[T_tmp + 2] = R[3 * i + 2];
  }

  T[12] = qt_data[4];
  T[13] = qt_data[5];
  T[14] = qt_data[6];
  T[3] = 0.0;
  T[7] = 0.0;
  T[11] = 0.0;
  T[15] = 1.0;
}

//
// File trailer for make_tform_qt.cpp
//
// [EOF]
//
