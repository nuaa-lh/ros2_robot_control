//
// File: fileManager.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//

// Include Files
#include "fileManager.h"
#include "coder_array.h"
#include <cmath>
#include <stdio.h>

// Variable Definitions
static FILE * eml_openfiles[20];
static boolean_T eml_autoflush[20];

// Function Declarations
namespace coder
{
  static signed char filedata();
}

static double rt_roundd_snf(double u);

// Function Definitions
//
// Arguments    : void
// Return Type  : signed char
//
namespace coder
{
  static signed char filedata()
  {
    int k;
    signed char f;
    boolean_T exitg1;
    f = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 20)) {
      if (eml_openfiles[k] == NULL) {
        f = static_cast<signed char>(k + 1);
        exitg1 = true;
      } else {
        k++;
      }
    }

    return f;
  }

  //
  // Arguments    : double u
  // Return Type  : double
  //
}

static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : double fid
// Return Type  : int
//
namespace coder
{
  int cfclose(double fid)
  {
    FILE * filestar;
    int st;
    signed char b_fileid;
    signed char fileid;
    st = -1;
    fileid = static_cast<signed char>(rt_roundd_snf(fid));
    if ((fileid < 0) || (fid != fileid)) {
      fileid = -1;
    }

    b_fileid = fileid;
    if (fileid < 0) {
      b_fileid = -1;
    }

    if (b_fileid >= 3) {
      filestar = eml_openfiles[b_fileid - 3];
    } else if (b_fileid == 0) {
      filestar = stdin;
    } else if (b_fileid == 1) {
      filestar = stdout;
    } else if (b_fileid == 2) {
      filestar = stderr;
    } else {
      filestar = NULL;
    }

    if ((filestar != NULL) && (fileid >= 3)) {
      int cst;
      cst = fclose(filestar);
      if (cst == 0) {
        st = 0;
        eml_openfiles[fileid - 3] = NULL;
        eml_autoflush[fileid - 3] = true;
      }
    }

    return st;
  }

  //
  // Arguments    : const ::coder::array<char, 2U> &cfilename
  //                const char *cpermission
  // Return Type  : signed char
  //
  signed char cfopen(const ::coder::array<char, 2U> &cfilename, const char
                     *cpermission)
  {
    array<char, 2U> ccfilename;
    signed char fileid;
    signed char j;
    fileid = -1;
    j = filedata();
    if (j >= 1) {
      FILE * filestar;
      int i;
      int loop_ub;
      ccfilename.set_size(1, (cfilename.size(1) + 1));
      loop_ub = cfilename.size(1);
      for (i = 0; i < loop_ub; i++) {
        ccfilename[i] = cfilename[i];
      }

      ccfilename[cfilename.size(1)] = '\x00';
      filestar = fopen(&ccfilename[0], cpermission);
      if (filestar != NULL) {
        eml_openfiles[j - 1] = filestar;
        eml_autoflush[j - 1] = true;
        i = j + 2;
        if (j + 2 > 127) {
          i = 127;
        }

        fileid = static_cast<signed char>(i);
      }
    }

    return fileid;
  }

  //
  // Arguments    : double varargin_1
  // Return Type  : FILE *
  //
  FILE * fileManager(double varargin_1)
  {
    FILE * f;
    signed char fileid;
    fileid = static_cast<signed char>(rt_roundd_snf(varargin_1));
    if ((fileid < 0) || (varargin_1 != fileid)) {
      fileid = -1;
    }

    if (fileid >= 3) {
      f = eml_openfiles[fileid - 3];
    } else if (fileid == 0) {
      f = stdin;
    } else if (fileid == 1) {
      f = stdout;
    } else if (fileid == 2) {
      f = stderr;
    } else {
      f = NULL;
    }

    return f;
  }

  //
  // Arguments    : void
  // Return Type  : void
  //
}

void filedata_init()
{
  FILE * a;
  a = NULL;
  for (int i = 0; i < 20; i++) {
    eml_autoflush[i] = false;
    eml_openfiles[i] = a;
  }
}

//
// File trailer for fileManager.cpp
//
// [EOF]
//
