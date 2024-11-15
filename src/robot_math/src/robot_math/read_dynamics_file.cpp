//
// File: read_dynamics_file.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//

// Include Files
#include "read_dynamics_file.h"
#include "fileManager.h"
#include "fscanf.h"
#include "make_tform_qt.h"
#include "read_dynamics_file_data.h"
#include "read_dynamics_file_initialize.h"
#include "read_dynamics_file_types.h"
#include "coder_array.h"
#include "stdio.h"
#include <cstring>
#include <stdio.h>

// Function Definitions
//
// Arguments    : const coder::array<char, 2U> &filename
//                struct0_T *robot
// Return Type  : void
//
using namespace robot_math;
void read_dynamics_file(const coder::array<char, 2U> &filename, Robot *robot)
{
  static const signed char b_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const char cv[3] = { 'a', 'l', 'l' };

  FILE * filestar;
  double b_a[36];
  double T[16];
  double b[16];
  double b_T[16];
  double d_T[16];
  double a[9];
  double b_y_tmp[9];
  double c_T[9];
  double c_y_tmp[9];
  double y_tmp[9];
  double b_tmp_data[7];
  double inert_data[7];
  double tmp_data[7];
  double A_data[6];
  double b_bigA[6];
  double b_q[3];
  double c_bigA[3];
  double bigA;
  double d;
  double n_data_idx_0;
  double q;
  double tmpOut_f1;
  int tmp_size[2];
  int b_robot;
  int fid;
  int i;
  int numRead;
  int numWrittenTotal;
  int tmpOut_f2;
  char cfmt[4];
  signed char invT_tmp_tmp[3];
  boolean_T exitg2;
  boolean_T incompleteRead;
  if (!isInitialized_read_dynamics_file) {
    read_dynamics_file_initialize();
  }

  incompleteRead = false;
  if (filename.size(1) == 3) {
    numWrittenTotal = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (numWrittenTotal < 3) {
        if (filename[numWrittenTotal] != cv[numWrittenTotal]) {
          exitg1 = 1;
        } else {
          numWrittenTotal++;
        }
      } else {
        incompleteRead = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  if (incompleteRead) {
    fid = 0;
  } else {
    signed char fileid;
    fileid = coder::cfopen(filename, "rb");
    fid = fileid;
  }

  tmpOut_f1 = 0.0;
  bigA = 0.0;
  numWrittenTotal = 0;
  filestar = coder::fileManager(static_cast<double>(fid));
  numRead = 0;
  exitg2 = false;
  while ((!exitg2) && (numRead > 0)) {
    tmpOut_f2 = -1;
    numRead = fscanf(filestar, "%lf%n", &tmpOut_f1, &tmpOut_f2);
    if (tmpOut_f2 != -1) {
      numRead++;
    }

    incompleteRead = (numRead == 0);
    if (numRead > 0) {
      bigA = tmpOut_f1;
      numWrittenTotal++;
      incompleteRead = ((2 > numRead) || incompleteRead);
    }

    if (incompleteRead || (1 >= 1 - numWrittenTotal)) {
      exitg2 = true;
    }
  }

  if (numWrittenTotal < 1) {
    tmpOut_f2 = -1;
    cfmt[0] = '%';
    cfmt[1] = 'l';
    cfmt[2] = 'f';
    cfmt[3] = '\x00';
    numRead = fscanf(filestar, &cfmt[0], &tmpOut_f1, &tmpOut_f2);
    if (tmpOut_f2 != -1) {
      numRead++;
    }

    if (1 <= numRead) {
      bigA = tmpOut_f1;
      numWrittenTotal++;
    }
  }

  if (numWrittenTotal > 0) {
    n_data_idx_0 = bigA;
  }

  b_robot = static_cast<int>(n_data_idx_0);
  robot->mass.set_size(b_robot);
  robot->inertia.set_size(3, 3, b_robot);
  robot->A.set_size(b_robot, 6);
  numWrittenTotal = static_cast<int>(n_data_idx_0) * 6;
  for (i = 0; i < numWrittenTotal; i++) {
    robot->A[i] = 0.0;
  }

  robot->M.set_size(4, 4, b_robot);
  robot->jtMechanics.set_size(b_robot, 3);
  numWrittenTotal = static_cast<int>(n_data_idx_0) * 3;
  for (i = 0; i < numWrittenTotal; i++) {
    robot->jtMechanics[i] = 0.0;
  }

  for (int b_i = 0; b_i < b_robot; b_i++) {
    boolean_T ranOut;
    coder::b_fscanf(static_cast<double>(fid), tmp_data, tmp_size);
    numWrittenTotal = tmp_size[0];
    if (0 <= numWrittenTotal - 1) {
      std::memcpy(&inert_data[0], &tmp_data[0], numWrittenTotal * sizeof(double));
    }

    robot->mass[b_i] = inert_data[0];
    robot->inertia[9 * b_i] = inert_data[1];
    robot->inertia[9 * b_i + 3] = inert_data[2];
    robot->inertia[9 * b_i + 6] = inert_data[3];
    robot->inertia[9 * b_i + 1] = inert_data[2];
    robot->inertia[9 * b_i + 4] = inert_data[4];
    robot->inertia[9 * b_i + 7] = inert_data[5];
    robot->inertia[9 * b_i + 2] = inert_data[3];
    robot->inertia[9 * b_i + 5] = inert_data[5];
    robot->inertia[9 * b_i + 8] = inert_data[6];
    coder::b_fscanf(static_cast<double>(fid), tmp_data, tmp_size);
    numWrittenTotal = tmp_size[0];
    if (0 <= numWrittenTotal - 1) {
      std::memcpy(&inert_data[0], &tmp_data[0], numWrittenTotal * sizeof(double));
    }

    make_tform_qt(inert_data, *(double (*)[16])&robot->M[16 * b_i]);
    tmpOut_f1 = 0.0;
    for (numWrittenTotal = 0; numWrittenTotal < 6; numWrittenTotal++) {
      b_bigA[numWrittenTotal] = 0.0;
    }

    numRead = 1;
    numWrittenTotal = 0;
    ranOut = true;
    filestar = coder::fileManager(static_cast<double>(fid));
    exitg2 = false;
    while ((!exitg2) && (numRead > 0)) {
      tmpOut_f2 = -1;
      numRead = fscanf(filestar, "%lf%n", &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        numRead++;
      }

      incompleteRead = (numRead == 0);
      if (numRead > 0) {
        b_bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
        incompleteRead = ((2 > numRead) || incompleteRead);
      }

      if (incompleteRead) {
        exitg2 = true;
      } else {
        if (1 >= 6 - numWrittenTotal) {
          ranOut = false;
          exitg2 = true;
        }
      }
    }

    if ((numWrittenTotal < 6) && (!ranOut)) {
      tmpOut_f2 = -1;
      cfmt[0] = '%';
      cfmt[1] = 'l';
      cfmt[2] = 'f';
      cfmt[3] = '\x00';
      numRead = fscanf(filestar, &cfmt[0], &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        numRead++;
      }

      if (1 <= numRead) {
        b_bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
      }
    }

    if (numWrittenTotal > 0) {
      if (numWrittenTotal > 6) {
        if (numWrittenTotal - numWrittenTotal / 6 * 6 == 0) {
          for (i = 0; i < 6; i++) {
            A_data[i] = b_bigA[i];
          }
        }
      } else {
        if (0 <= numWrittenTotal - 1) {
          std::memcpy(&A_data[0], &b_bigA[0], numWrittenTotal * sizeof(double));
        }
      }
    }

    for (i = 0; i < 6; i++) {
      robot->A[b_i + robot->A.size(0) * i] = A_data[i];
    }

    tmpOut_f1 = 0.0;
    c_bigA[0] = 0.0;
    c_bigA[1] = 0.0;
    c_bigA[2] = 0.0;
    numRead = 1;
    numWrittenTotal = 0;
    ranOut = true;
    filestar = coder::fileManager(static_cast<double>(fid));
    exitg2 = false;
    while ((!exitg2) && (numRead > 0)) {
      tmpOut_f2 = -1;
      numRead = fscanf(filestar, "%lf%n", &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        numRead++;
      }

      incompleteRead = (numRead == 0);
      if (numRead > 0) {
        c_bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
        incompleteRead = ((2 > numRead) || incompleteRead);
      }

      if (incompleteRead) {
        exitg2 = true;
      } else {
        if (1 >= 3 - numWrittenTotal) {
          ranOut = false;
          exitg2 = true;
        }
      }
    }

    if ((numWrittenTotal < 3) && (!ranOut)) {
      tmpOut_f2 = -1;
      cfmt[0] = '%';
      cfmt[1] = 'l';
      cfmt[2] = 'f';
      cfmt[3] = '\x00';
      numRead = fscanf(filestar, &cfmt[0], &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        numRead++;
      }

      if (1 <= numRead) {
        c_bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
      }
    }

    if (numWrittenTotal > 0) {
      if (numWrittenTotal > 3) {
        if (numWrittenTotal - numWrittenTotal / 3 * 3 == 0) {
          b_q[0] = c_bigA[0];
          b_q[1] = c_bigA[1];
          b_q[2] = c_bigA[2];
        }
      } else {
        if (0 <= numWrittenTotal - 1) {
          std::memcpy(&b_q[0], &c_bigA[0], numWrittenTotal * sizeof(double));
        }
      }
    }

    robot->jtMechanics[b_i] = b_q[0];
    robot->jtMechanics[b_i + robot->jtMechanics.size(0)] = b_q[1];
    robot->jtMechanics[b_i + robot->jtMechanics.size(0) * 2] = b_q[2];
  }

  coder::b_fscanf(static_cast<double>(fid), tmp_data, tmp_size);
  numWrittenTotal = tmp_size[0];
  if (0 <= numWrittenTotal - 1) {
    std::memcpy(&inert_data[0], &tmp_data[0], numWrittenTotal * sizeof(double));
  }

  if (tmp_size[0] != 0) {
    double a_tmp;
    double d1;
    double d2;
    coder::b_fscanf(static_cast<double>(fid), tmp_data, tmp_size);
    numWrittenTotal = tmp_size[0];
    if (0 <= numWrittenTotal - 1) {
      std::memcpy(&b_tmp_data[0], &tmp_data[0], numWrittenTotal * sizeof(double));
    }

    make_tform_qt(b_tmp_data, T);
    coder::b_fscanf(static_cast<double>(fid), tmp_data, tmp_size);
    numWrittenTotal = tmp_size[0];
    if (0 <= numWrittenTotal - 1) {
      std::memcpy(&b_tmp_data[0], &tmp_data[0], numWrittenTotal * sizeof(double));
    }

    make_tform_qt(b_tmp_data, b);

    //  combine I1 I2 to T, T is also relative to T1
    a_tmp = robot->mass[static_cast<int>(n_data_idx_0) - 1];
    tmpOut_f1 = inert_data[0] / (a_tmp + inert_data[0]);
    for (i = 0; i < 3; i++) {
      numWrittenTotal = i << 2;
      b_T[numWrittenTotal] = b_b[3 * i];
      b_T[numWrittenTotal + 1] = b_b[3 * i + 1];
      b_T[numWrittenTotal + 2] = b_b[3 * i + 2];
      b_T[i + 12] = tmpOut_f1 * T[i + 12];
    }

    b_T[3] = 0.0;
    b_T[7] = 0.0;
    b_T[11] = 0.0;
    b_T[15] = 1.0;

    //  T: 4x4xn
    for (i = 0; i < 3; i++) {
      invT_tmp_tmp[i] = static_cast<signed char>(i + 1);
      c_T[3 * i] = -T[i];
      c_T[3 * i + 1] = -T[i + 4];
      c_T[3 * i + 2] = -T[i + 8];
    }

    for (i = 0; i < 3; i++) {
      numWrittenTotal = i << 2;
      d_T[numWrittenTotal] = T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[0] - 1) << 2))
        - 1];
      d_T[numWrittenTotal + 1] = T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[1] - 1) <<
        2)) - 1];
      d_T[numWrittenTotal + 2] = T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[2] - 1) <<
        2)) - 1];
      d_T[i + 12] = (c_T[i] * T[12] + c_T[i + 3] * T[13]) + c_T[i + 6] * T[14];
    }

    d_T[3] = 0.0;
    d_T[7] = 0.0;
    d_T[11] = 0.0;
    d_T[15] = 1.0;
    for (i = 0; i < 4; i++) {
      d = d_T[i];
      d1 = d_T[i + 4];
      d2 = d_T[i + 8];
      tmpOut_f1 = d_T[i + 12];
      for (b_robot = 0; b_robot < 4; b_robot++) {
        numWrittenTotal = b_robot << 2;
        T[i + numWrittenTotal] = ((d * b_T[numWrittenTotal] + d1 *
          b_T[numWrittenTotal + 1]) + d2 * b_T[numWrittenTotal + 2]) + tmpOut_f1
          * b_T[numWrittenTotal + 3];
      }
    }

    //  Ib defined in a center mass frame
    for (i = 0; i < 3; i++) {
      y_tmp[3 * i] = b_T[i];
      y_tmp[3 * i + 1] = b_T[i + 4];
      y_tmp[3 * i + 2] = b_T[i + 8];
    }

    bigA = 0.0;

    //  Ib defined in a center mass frame
    d = b_T[12];
    d1 = b_T[13];
    d2 = b_T[14];
    for (i = 0; i < 3; i++) {
      tmpOut_f1 = (y_tmp[i] * d + y_tmp[i + 3] * d1) + y_tmp[i + 6] * d2;
      c_bigA[i] = tmpOut_f1;
      bigA += tmpOut_f1 * tmpOut_f1;
      b_y_tmp[3 * i] = T[i];
      b_y_tmp[3 * i + 1] = T[i + 4];
      b_y_tmp[3 * i + 2] = T[i + 8];
    }

    q = 0.0;
    c_T[0] = inert_data[1];
    c_T[3] = inert_data[2];
    c_T[6] = inert_data[3];
    c_T[1] = inert_data[2];
    c_T[4] = inert_data[4];
    c_T[7] = inert_data[5];
    c_T[2] = inert_data[3];
    c_T[5] = inert_data[5];
    c_T[8] = inert_data[6];
    for (i = 0; i < 3; i++) {
      double d3;
      double d4;
      d = b_y_tmp[i];
      d1 = b_y_tmp[i + 3];
      d2 = b_y_tmp[i + 6];
      tmpOut_f1 = (d * T[12] + d1 * T[13]) + d2 * T[14];
      b_q[i] = tmpOut_f1;
      q += tmpOut_f1 * tmpOut_f1;
      tmpOut_f1 = y_tmp[i];
      d3 = y_tmp[i + 3];
      d4 = y_tmp[i + 6];
      for (b_robot = 0; b_robot < 3; b_robot++) {
        c_y_tmp[i + 3 * b_robot] = (tmpOut_f1 * robot->inertia[3 * b_robot + 9 *
          (static_cast<int>(n_data_idx_0) - 1)] + d3 * robot->inertia[(3 *
          b_robot + 9 * (static_cast<int>(n_data_idx_0) - 1)) + 1]) + d4 *
          robot->inertia[(3 * b_robot + 9 * (static_cast<int>(n_data_idx_0) - 1))
          + 2];
        tmpOut_f2 = b_robot + 3 * i;
        a[tmpOut_f2] = bigA * static_cast<double>(b_b[tmpOut_f2]) -
          c_bigA[b_robot] * c_bigA[i];
      }

      for (b_robot = 0; b_robot < 3; b_robot++) {
        y_tmp[i + 3 * b_robot] = (d * c_T[3 * b_robot] + d1 * c_T[3 * b_robot +
          1]) + d2 * c_T[3 * b_robot + 2];
      }
    }

    tmpOut_f1 = inert_data[0];
    for (i = 0; i < 3; i++) {
      d = c_y_tmp[i];
      d1 = c_y_tmp[i + 3];
      d2 = c_y_tmp[i + 6];
      for (b_robot = 0; b_robot < 3; b_robot++) {
        numWrittenTotal = b_robot + 3 * i;
        c_T[numWrittenTotal] = q * static_cast<double>(b_b[numWrittenTotal]) -
          b_q[b_robot] * b_q[i];
        numWrittenTotal = b_robot << 2;
        numRead = i + 3 * b_robot;
        b_y_tmp[numRead] = ((d * b_T[numWrittenTotal] + d1 * b_T[numWrittenTotal
                             + 1]) + d2 * b_T[numWrittenTotal + 2]) + a_tmp *
          a[numRead];
      }
    }

    for (i = 0; i < 3; i++) {
      d = y_tmp[i];
      d1 = y_tmp[i + 3];
      d2 = y_tmp[i + 6];
      for (b_robot = 0; b_robot < 3; b_robot++) {
        numWrittenTotal = b_robot << 2;
        numRead = i + 3 * b_robot;
        c_y_tmp[numRead] = ((d * T[numWrittenTotal] + d1 * T[numWrittenTotal + 1])
                            + d2 * T[numWrittenTotal + 2]) + tmpOut_f1 *
          c_T[numRead];
      }
    }

    for (i = 0; i < 3; i++) {
      robot->inertia[3 * i + 9 * (static_cast<int>(n_data_idx_0) - 1)] =
        b_y_tmp[3 * i] + c_y_tmp[3 * i];
      b_robot = 3 * i + 1;
      robot->inertia[(3 * i + 9 * (static_cast<int>(n_data_idx_0) - 1)) + 1] =
        b_y_tmp[b_robot] + c_y_tmp[b_robot];
      b_robot = 3 * i + 2;
      robot->inertia[(3 * i + 9 * (static_cast<int>(n_data_idx_0) - 1)) + 2] =
        b_y_tmp[b_robot] + c_y_tmp[b_robot];
    }

    for (i = 0; i < 4; i++) {
      for (b_robot = 0; b_robot < 4; b_robot++) {
        numWrittenTotal = b_robot << 2;
        T[i + numWrittenTotal] = ((robot->M[i + 16 * (static_cast<int>
          (n_data_idx_0) - 1)] * b_T[numWrittenTotal] + robot->M[(i + 16 * (
          static_cast<int>(n_data_idx_0) - 1)) + 4] * b_T[numWrittenTotal + 1])
          + robot->M[(i + 16 * (static_cast<int>(n_data_idx_0) - 1)) + 8] *
          b_T[numWrittenTotal + 2]) + robot->M[(i + 16 * (static_cast<int>
          (n_data_idx_0) - 1)) + 12] * b_T[numWrittenTotal + 3];
      }
    }

    for (i = 0; i < 4; i++) {
      b_robot = i << 2;
      robot->M[4 * i + 16 * (static_cast<int>(n_data_idx_0) - 1)] = T[b_robot];
      robot->M[(4 * i + 16 * (static_cast<int>(n_data_idx_0) - 1)) + 1] =
        T[b_robot + 1];
      robot->M[(4 * i + 16 * (static_cast<int>(n_data_idx_0) - 1)) + 2] =
        T[b_robot + 2];
      robot->M[(4 * i + 16 * (static_cast<int>(n_data_idx_0) - 1)) + 3] =
        T[b_robot + 3];
    }

    //  T: 4x4xn
    for (i = 0; i < 3; i++) {
      c_T[3 * i] = -b_T[i];
      c_T[3 * i + 1] = -b_T[i + 4];
      c_T[3 * i + 2] = -b_T[i + 8];
    }

    for (i = 0; i < 3; i++) {
      numWrittenTotal = i << 2;
      T[numWrittenTotal] = b_T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[0] - 1) << 2))
        - 1];
      T[numWrittenTotal + 1] = b_T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[1] - 1) <<
        2)) - 1];
      T[numWrittenTotal + 2] = b_T[(invT_tmp_tmp[i] + ((invT_tmp_tmp[2] - 1) <<
        2)) - 1];
      T[i + 12] = (c_T[i] * b_T[12] + c_T[i + 3] * b_T[13]) + c_T[i + 6] * b_T
        [14];
    }

    T[3] = 0.0;
    T[7] = 0.0;
    T[11] = 0.0;
    T[15] = 1.0;
    for (i = 0; i < 4; i++) {
      d = T[i];
      d1 = T[i + 4];
      d2 = T[i + 8];
      tmpOut_f1 = T[i + 12];
      for (b_robot = 0; b_robot < 4; b_robot++) {
        numWrittenTotal = b_robot << 2;
        robot->ME[i + numWrittenTotal] = ((d * b[numWrittenTotal] + d1 *
          b[numWrittenTotal + 1]) + d2 * b[numWrittenTotal + 2]) + tmpOut_f1 *
          b[numWrittenTotal + 3];
      }
    }

    //  AdT operator T -> 6x6 mapping
    std::memset(&b_a[0], 0, 36U * sizeof(double));
    for (i = 0; i < 3; i++) {
      tmpOut_f2 = (invT_tmp_tmp[i] - 1) << 2;
      b_a[6 * i] = T[(invT_tmp_tmp[0] + tmpOut_f2) - 1];
      numWrittenTotal = i << 2;
      numRead = 6 * (i + 3);
      b_a[numRead + 3] = T[numWrittenTotal];
      b_a[6 * i + 1] = T[(invT_tmp_tmp[1] + tmpOut_f2) - 1];
      b_a[numRead + 4] = T[numWrittenTotal + 1];
      b_a[6 * i + 2] = T[(invT_tmp_tmp[2] + tmpOut_f2) - 1];
      b_a[numRead + 5] = T[numWrittenTotal + 2];
    }

    //  w - > [w]
    //  x: vector or m by 3 matrix
    //  X: 3 by 3 by m matrix
    c_T[0] = 0.0;
    c_T[3] = -T[14];
    c_T[6] = T[13];
    c_T[1] = T[14];
    c_T[4] = 0.0;
    c_T[7] = -T[12];
    c_T[2] = -T[13];
    c_T[5] = T[12];
    c_T[8] = 0.0;
    for (i = 0; i < 3; i++) {
      d = c_T[i];
      d1 = c_T[i + 3];
      d2 = c_T[i + 6];
      for (b_robot = 0; b_robot < 3; b_robot++) {
        numWrittenTotal = b_robot << 2;
        b_a[(i + 6 * b_robot) + 3] = (d * T[numWrittenTotal] + d1 *
          T[numWrittenTotal + 1]) + d2 * T[numWrittenTotal + 2];
      }
    }

    for (i = 0; i < 6; i++) {
      d = 0.0;
      for (b_robot = 0; b_robot < 6; b_robot++) {
        d += b_a[i + 6 * b_robot] * robot->A[(static_cast<int>(n_data_idx_0) +
          robot->A.size(0) * b_robot) - 1];
      }

      b_bigA[i] = d;
    }

    for (i = 0; i < 6; i++) {
      robot->A[(static_cast<int>(n_data_idx_0) + robot->A.size(0) * i) - 1] =
        b_bigA[i];
    }
  } else {
    std::memset(&robot->ME[0], 0, 16U * sizeof(double));
    robot->ME[0] = 1.0;
    robot->ME[5] = 1.0;
    robot->ME[10] = 1.0;
    robot->ME[15] = 1.0;
  }

  coder::cfclose(static_cast<double>(fid));
  robot->dof = n_data_idx_0;

  // ���ɶ�
  // ����
  //  ���Ծ���
  //  screw axis
  //  relation at zero position
  //  end-effector frame
  //  joint damping
  robot->gravity[0] = 0.0;
  robot->gravity[1] = 0.0;
  robot->gravity[2] = -9.8;

  std::memset(&robot->TCP[0], 0, 16U * sizeof(double));
  robot->TCP[0] = 1.0;
  robot->TCP[5] = 1.0;
  robot->TCP[10] = 1.0;
  robot->TCP[15] = 1.0;
  //  gravity acceleration in base frame
}

//
// File trailer for read_dynamics_file.cpp
//
// [EOF]
//
