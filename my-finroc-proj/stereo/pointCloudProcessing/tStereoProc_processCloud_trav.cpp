/*
 * tStereoProc_processCloud_trav.cpp
 *
 *  Created on: Aug 4, 2014
 *      Author: aras
 */


#include "projects/icarus/sensor_processing/pointCloudProcessing/tStereoProcessing.h"

using namespace finroc::icarus::sensor_processing::pointCloudProcessing;

void tStereoProcessing::processCloud_trav()
{

  processCloud_trav_segm2plane();
  processCloud_trav_slopeAnalysis();
  processCloud_trav_dominGroundPlane();
  processCloud_trav_stepAnalysis();
}



