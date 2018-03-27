#include "projects/stereo_traversability_experiments/mlr/tMLR.h"

using namespace finroc::stereo_traversability_experiments::mlr;
void tMLR::LOG_PRINT(/*INPUT*/const Mat& mat, /*INPUT*/const std::string& name, /*INPUT*/const eLOG_PRINT_STATE& state)
{

  if (state == eLOG_PRINT_STATE::eIS_ACTIVE)
  {
    cout << "startttttttttttttttttttttttttttttt============================================================================================= " << endl;
    cout << name << endl;
    cout << "Mat::::::::::::::::::: " << endl
         << "Mat size: (width, height) =  (cols, rows) =  (X, Y)=   (j, i) = (w, h) -------------> " << endl
         << "Mat size: (width, height) || (cols, rows) || (X, Y) || (j, i) ------------> " << endl
         << "size: " << mat.size() << "[cols, rows]" << endl
         << "rows: " << mat.rows << "  height, Y, i" << endl
         << "cols: " << mat.cols << "  width, X, j" << endl
         << "type: " << mat.type() << endl
         << "depth: " << mat.depth() << endl
         << "channels; " << mat.channels() << endl
         << "elemSize: " << mat.elemSize() << endl
         << "step1: " << mat.step1() << endl
         << "isContinuous: " << mat.isContinuous() << endl
         << "total: " << mat.total() << endl;
    cout << "endddddddddddddddddddddddd============================================================================================= " << endl;

  }
}
