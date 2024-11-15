//
// File: fileManager.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 23-Mar-2022 14:59:48
//
#ifndef FILEMANAGER_H
#define FILEMANAGER_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>

// Function Declarations
namespace coder
{
  int cfclose(double fid);
  signed char cfopen(const ::coder::array<char, 2U> &cfilename, const char
                     *cpermission);
  FILE * fileManager(double varargin_1);
}

void filedata_init();

#endif

//
// File trailer for fileManager.h
//
// [EOF]
//
