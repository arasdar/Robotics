/*
 * mStereoProc_pointPickingCallback.cpp
 *
 *  Created on: May 19, 2014
 *      Author: aras
 */

#include "projects/stereo_traversability_experiments/openTraverse/mauro/test_finroc/UPD.h"

//using namespace finroc::icarus::sensor_processing::pointCloudProcessing;
using namespace finroc::mauro::test;

void UPD::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp())
  {
    switch (event.getKeyCode())
    {
    case ' ': //forward
      trigger = true;
      break;
    case 'b': //backward
      bwd = true;
      break;
    case 'c':
      continuous = !continuous;
      break;
    }
  }
}//keyboard callback
