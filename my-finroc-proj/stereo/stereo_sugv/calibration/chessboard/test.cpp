/*
 * test.cpp
 *
 *  Created on: May 13, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/stereo_sugv/calibration/chessboard/tStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::stereo_sugv::calibration;




int main(int argc, char** argv)
{
  tStereoProcessing stereo;
  stereo.run();

  return 0;
}





