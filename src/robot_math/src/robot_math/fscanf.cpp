//
// File: fscanf.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//

// Include Files
#include "fscanf.h"
#include "fileManager.h"
#include "stdio.h"
#include <cstring>
#include <stdio.h>

// Function Definitions
//
// Arguments    : double fileID
//                double A_data[]
//                int A_size[2]
// Return Type  : void
//
namespace coder
{
  void b_fscanf(double fileID, double A_data[], int A_size[2])
  {
    FILE * filestar;
    double bigA[7];
    double tmpOut_f1;
    int i;
    int numWrittenTotal;
    int tmpOut_f2;
    char cfmt[4];
    boolean_T exitg1;
    boolean_T ranOut;
    tmpOut_f1 = 0.0;
    for (i = 0; i < 7; i++) {
      bigA[i] = 0.0;
    }

    i = 1;
    numWrittenTotal = 0;
    ranOut = true;
    filestar = fileManager(fileID);
    exitg1 = false;
    while ((!exitg1) && (i > 0)) {
      boolean_T incompleteRead;
      tmpOut_f2 = -1;
      i = fscanf(filestar, "%lf%n", &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        i++;
      }

      incompleteRead = (i == 0);
      if (i > 0) {
        bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
        incompleteRead = ((2 > i) || incompleteRead);
      }

      if (incompleteRead) {
        exitg1 = true;
      } else {
        if (1 >= 7 - numWrittenTotal) {
          ranOut = false;
          exitg1 = true;
        }
      }
    }

    if ((numWrittenTotal < 7) && (!ranOut)) {
      tmpOut_f2 = -1;
      cfmt[0] = '%';
      cfmt[1] = 'l';
      cfmt[2] = 'f';
      cfmt[3] = '\x00';
      i = fscanf(filestar, &cfmt[0], &tmpOut_f1, &tmpOut_f2);
      if (tmpOut_f2 != -1) {
        i++;
      }

      if (1 <= i) {
        bigA[numWrittenTotal] = tmpOut_f1;
        numWrittenTotal++;
      }
    }

    if (numWrittenTotal > 0) {
      if (numWrittenTotal > 7) {
        if (numWrittenTotal - numWrittenTotal / 7 * 7 == 0) {
          A_size[0] = 7;
          A_size[1] = 1;
          for (i = 0; i < 7; i++) {
            A_data[i] = bigA[i];
          }
        }
      } else {
        A_size[0] = numWrittenTotal;
        A_size[1] = 1;
        if (0 <= numWrittenTotal - 1) {
          std::memcpy(&A_data[0], &bigA[0], numWrittenTotal * sizeof(double));
        }
      }
    } else {
      A_size[0] = 0;
      A_size[1] = 1;
    }
  }
}

//
// File trailer for fscanf.cpp
//
// [EOF]
//
